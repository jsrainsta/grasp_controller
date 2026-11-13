"""Process saved photos without blocking the serial receiver."""

from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread


class OCRWorker:
    def __init__(self, output_dir, catalog_path=None, min_name_score=0.8,
                 runner=None, max_pending=2, target_id=None):
        if type(max_pending) is not int or max_pending < 1:
            raise ValueError("max_pending must be a positive integer")
        if runner is None:
            if __package__:
                from .ocr_photo import OCRRunner
            else:
                from ocr_photo import OCRRunner
            runner = OCRRunner(catalog_path=catalog_path,
                               min_name_score=min_name_score, target_id=target_id)
        self.runner = runner
        self.output_dir = Path(output_dir).expanduser().resolve()
        self._queue = Queue(maxsize=max_pending)
        self._stopping = Event()
        self._state_lock = Lock()
        self._closed = False
        self._thread = Thread(target=self._run, name="photo-ocr", daemon=True)
        self._thread.start()

    def submit(self, path):
        """Queue this exact saved file; never wait for a model or free space."""
        path = Path(path).expanduser().resolve()
        with self._state_lock:
            if self._closed:
                return False
            try:
                self._queue.put_nowait(path)
            except Full:
                print("自动 OCR 队列已满，这张照片仍已保存，请稍后手动识别：", path)
                return False
        print("已加入自动 OCR 队列：", path.name)
        return True

    def _run(self):
        while True:
            try:
                path = self._queue.get(timeout=0.1)
            except Empty:
                # submit() may have queued the final photo immediately after
                # get() timed out and just before close() stopped submissions.
                if self._stopping.is_set() and self._queue.empty():
                    return
                continue
            try:
                print("\n开始自动 OCR：", path.name)
                self.runner.run(path, self.output_dir)
                print("自动 OCR 完成：", path.name, flush=True)
            except Exception as error:
                # A bad photo or inference error must not stop serial reception
                # or prevent recognition of the next successfully saved photo.
                print("自动 OCR 失败，原照片仍保留：{}；原因：{}".format(path, error),
                      flush=True)
            finally:
                self._queue.task_done()

    def close(self):
        """Stop accepting photos and finish every already accepted job."""
        with self._state_lock:
            self._closed = True
            self._stopping.set()
        if self._queue.unfinished_tasks:
            print("正在等待已排队的照片识别完成……", flush=True)
        try:
            self._thread.join()
        except KeyboardInterrupt:
            print("已中断等待，未完成 OCR 的原照片仍保留，可稍后手动识别。", flush=True)
            raise
