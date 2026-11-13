"""One explicit vision task at a time, sharing a serial session and OCR model.

No navigation or arm commands. Run directly for an interactive task test, or
use VisionTaskSession as a context manager from the future controller.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from queue import Queue
import re
from threading import Event, Lock, Thread
import time
import uuid


PROJECT_DIR = Path(__file__).resolve().parent


def utc_now():
    return datetime.now(timezone.utc).isoformat()


class VisionTaskSession:
    def __init__(self, port, target_id, images=None, results=None, catalog=None,
                 baudrate=115200, boot_wait=5.0, min_name_score=0.8,
                 receiver=None, runner=None):
        if not isinstance(port, str) or not port.strip():
            raise ValueError("port 必须是已确定的K210端口。")
        if not isinstance(target_id, str) or not target_id.strip():
            raise ValueError("target_id 必须是目录中的目标编号。")
        if type(baudrate) is not int or not 0 < baudrate <= 5000000:
            raise ValueError("baudrate 无效。")
        if (isinstance(boot_wait, bool) or not isinstance(boot_wait, (int, float))
                or not math.isfinite(boot_wait) or not 0 <= boot_wait <= 30):
            raise ValueError("boot_wait 必须在0到30秒之间。")
        if runner is None:
            if __package__:
                from .ocr_photo import OCRRunner
            else:
                from ocr_photo import OCRRunner
            runner = OCRRunner(catalog_path=catalog, target_id=target_id,
                               min_name_score=min_name_score)
        if not isinstance(runner.target, dict) or runner.target.get("id") != target_id:
            raise ValueError("OCR目标与任务会话目标不一致。")
        self.port, self.baudrate, self.boot_wait = port, baudrate, boot_wait
        self.target = dict(runner.target)
        self.images = Path(images or PROJECT_DIR / "images").expanduser().resolve()
        self.results = Path(results or PROJECT_DIR / "ocr_results").expanduser().resolve()
        self.runner, self._receiver = runner, receiver
        self._requests = Queue(maxsize=1)
        self._stop, self._ready = Event(), Event()
        self._lock, self._one_task = Lock(), Lock()
        self._thread = None
        self._active = None
        self._fatal = None
        self._used_ids = set()

    def __enter__(self):
        if self._thread is not None or self._stop.is_set():
            raise RuntimeError("此会话不能重复启动。")
        self._thread = Thread(target=self._receive, name="vision-task-receiver", daemon=False)
        self._thread.start()
        try:
            if not self._ready.wait(self.boot_wait + 18):
                raise TimeoutError("相机任务会话启动超时。")
            with self._lock:
                if self._fatal is not None:
                    raise RuntimeError("相机任务会话启动失败：" + str(self._fatal))
            return self
        except BaseException:
            self.close()
            raise

    def __exit__(self, *_):
        self.close()

    def close(self):
        self._stop.set()
        with self._lock:
            if self._active is not None:
                self._active["error"] = "会话已关闭，本次任务取消。"
                self._active["done"].set()
        if self._thread is not None:
            self._thread.join()

    @property
    def receiver_error(self):
        """Fatal receiver error, if any; requires a new explicit session."""
        with self._lock:
            return None if self._fatal is None else str(self._fatal)

    def _on_state(self, phase, message):
        with self._lock:
            active = self._active
            if active is not None and phase == "capturing":
                active["dispatched"] = True
            if phase == "ready":
                self._ready.set()
                if (active is not None and active["dispatched"]
                        and active["photo"] is None and active["error"] is None):
                    active["error"] = message
                    active["done"].set()

    def _on_requested_photo(self, path, metadata):
        # This callback is only made for a complete, validated software request.
        # Physical K1 photos continue to save but cannot satisfy a task.
        with self._lock:
            active = self._active
            if (active is None or not active["dispatched"] or self._stop.is_set()
                    or active["photo"] is not None or active["error"] is not None):
                return
            active["photo"] = (Path(path).resolve(), dict(metadata))
            active["done"].set()

    def _receive(self):
        try:
            receiver = self._receiver
            if receiver is None:
                if __package__:
                    from .receive_photo import receive_photos
                else:
                    from receive_photo import receive_photos
                receiver = receive_photos
            receiver(self.port, self.baudrate, self.images,
                     stop_event=self._stop, capture_requests=self._requests,
                     on_state=self._on_state, on_requested_photo=self._on_requested_photo,
                     boot_wait=self.boot_wait)
            if not self._stop.is_set():
                raise RuntimeError("摄像头接收意外结束。")
        except Exception as error:
            with self._lock:
                self._fatal = error
                if self._active is not None:
                    self._active["error"] = str(error)
                    self._active["done"].set()
        finally:
            self._ready.set()

    def recognize(self, task_id, location_id=None):
        return self._run_task(task_id, location_id, with_ocr=True)

    def capture(self, task_id, location_id=None):
        """Capture a fresh requested photo without OCR or identity confirmation."""
        return self._run_task(task_id, location_id, with_ocr=False)

    def _run_task(self, task_id, location_id=None, with_ocr=True):
        """Return this task's result; transport/OCR failures have status=error.

        The target is fixed for this session. Validation errors, a duplicate ID
        or a concurrent call raise before another capture can be submitted.
        """
        if (not isinstance(task_id, str)
                or re.fullmatch(r"[A-Za-z0-9_.-]{1,128}", task_id) is None):
            raise ValueError("task_id须为1到128位字母、数字、下划线、点或短横线。")
        if location_id is not None and (not isinstance(location_id, str) or not location_id.strip()):
            raise ValueError("location_id须为非空字符串或None。")
        if not self._one_task.acquire(blocking=False):
            raise RuntimeError("已有识别任务正在运行。")
        started = time.perf_counter()
        result = {"type": "vision_task_result" if with_ocr else "vision_capture_result",
                  "version": 1, "task_id": task_id,
                  "location_id": location_id, "target_id": self.target["id"],
                  "target_name": self.target["name"], "status": "error",
                  "recognized_id": None, "recognized_name": None,
                  "capture": None, "image_path": None, "ocr_result_path": None,
                  "started_at": utc_now(), "completed_at": None,
                  "elapsed_s": None, "error": None}
        stage = "capture"
        active = {"done": Event(), "dispatched": False, "photo": None, "error": None}
        try:
            with self._lock:
                if self._thread is None or not self._ready.is_set() or self._stop.is_set():
                    raise RuntimeError("请先用with启动可用的任务会话。")
                if task_id in self._used_ids:
                    raise ValueError("本会话已使用此task_id，不会重复拍照。")
                self._used_ids.add(task_id)
                self._active = active
                fatal = self._fatal
                if fatal is None:
                    self._requests.put_nowait(True)
            try:
                if fatal is not None:
                    raise RuntimeError(str(fatal))
                # Receiver deadlines bound an actual request; there is no retry.
                active["done"].wait()
                with self._lock:
                    error, photo = active["error"], active["photo"]
                if error is not None:
                    raise RuntimeError(error)
                if photo is None:
                    raise RuntimeError("没有收到本次完整照片。")
                path, metadata = photo
                result["capture"], result["image_path"] = metadata, str(path)
                if not with_ocr:
                    if self._stop.is_set():
                        raise RuntimeError("会话在拍照期间关闭，本次结果不提交。")
                    result.update(status="captured", identity_checked=False,
                                  completed_at=utc_now(), elapsed_s=time.perf_counter()-started)
                    return result
                stage = "ocr"
                report = self.runner.run(path, self.results)
                selection = report.get("target_selection")
                if (Path(report.get("image_path", "")).resolve() != path
                        or report.get("image_width") != metadata["image_width"]
                        or report.get("image_height") != metadata["image_height"]
                        or not isinstance(selection, dict)
                        or selection.get("target_id") != self.target["id"]):
                    raise ValueError("OCR报告与本次照片或目标不一致。")
                status = selection.get("status")
                rid, name = selection.get("recognized_id"), selection.get("recognized_name")
                if status not in {"target_found", "other_medicine", "unconfirmed"}:
                    raise ValueError("OCR报告的筛选状态无效。")
                if status == "unconfirmed":
                    if rid is not None or name is not None:
                        raise ValueError("未确认结果携带了旧药品信息。")
                elif (not isinstance(rid, str) or not rid
                      or not isinstance(name, str) or not name
                      or (rid == self.target["id"]) != (status == "target_found")):
                    raise ValueError("识别药品与筛选状态不一致。")
                if self._stop.is_set():
                    raise RuntimeError("会话在OCR期间关闭，本次结果不提交。")
                result.update(status=status, recognized_id=rid, recognized_name=name,
                              reason=selection.get("reason"),
                              ocr_result_path=str(self.results / (path.stem + "_ocr.json")))
            except Exception as error:
                result["error"] = {"stage": stage, "message": str(error)}
            except BaseException:
                # An interrupted call must not leave a capture queued for a
                # later task if the controller catches KeyboardInterrupt.
                self.close()
                raise
            result["completed_at"] = utc_now()
            result["elapsed_s"] = time.perf_counter() - started
            return result
        finally:
            with self._lock:
                if self._active is active:
                    self._active = None
            self._one_task.release()


def main():
    parser = argparse.ArgumentParser(description="独立测试主控可调用的一次识别任务；不控制机械臂")
    parser.add_argument("--port", help="当前K210端口；省略时列出供选择")
    parser.add_argument("--target", required=True, help="本次会话固定的目标药品编号")
    parser.add_argument("--task-results", type=Path, default=PROJECT_DIR / "task_results")
    args = parser.parse_args()
    if __package__:
        from .receive import choose_port
    else:
        from receive import choose_port
    try:
        with VisionTaskSession(args.port or choose_port(), args.target) as session:
            print("任务接口已就绪。按回车发起一次识别，输入q退出；无需打开照片窗口。", flush=True)
            while input("准备好药品后按回车拍照（q退出）：").strip().lower() != "q":
                task_id = "vision_" + uuid.uuid4().hex
                print("开始识别任务：", task_id, flush=True)
                result = session.recognize(task_id)
                args.task_results.mkdir(parents=True, exist_ok=True)
                output = args.task_results / (task_id + "_vision.json")
                with output.open("x", encoding="utf-8") as stream:
                    stream.write(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
                print("任务结果：", result["status"], flush=True)
                print("任务编号：", result["task_id"], flush=True)
                print("识别药品：", result["recognized_name"] or "暂未确认", flush=True)
                if result["error"] is not None:
                    print("错误：", result["error"], flush=True)
                print("任务结果文件：", output.resolve(), flush=True)
    except (KeyboardInterrupt, EOFError):
        print("已停止识别任务并关闭串口。")
    except Exception as error:
        print("任务程序停止：", error, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
