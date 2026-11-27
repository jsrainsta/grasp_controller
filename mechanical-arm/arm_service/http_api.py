"""仅本机的 HTTP＋JSON 接口，使用 Python 标准库，无框架依赖。"""

from __future__ import annotations

import json
import logging
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

from . import __version__
from .errors import ArmError
from .validation import integer, parse_command

LOG = logging.getLogger(__name__)
PREFIX = "/v1/arm"
MAX_BODY_BYTES = 256 * 1024


def route(service, method: str, path: str, query: dict, data=None):
    """可由离线测试直接调用；网络处理和业务控制分离。"""
    refresh_routes = {"/device", "/positions", "/pose"}
    suffix = path[len(PREFIX):] if path.startswith(PREFIX + "/") else None
    if suffix is None:
        raise ArmError("not_found", "接口不存在", 404)
    if method == "GET" and suffix in refresh_routes:
        if set(query) - {"refresh"} or any(len(values) != 1 for values in query.values()):
            raise ArmError("invalid_request", "查询参数仅支持一个 refresh", 422)
        value = query.get("refresh", ["false" if suffix == "/device" else "true"])[0]
        if value not in {"true", "false"}:
            raise ArmError("invalid_request", "refresh 必须为 true 或 false", 422)
        return 200, getattr(service, suffix[1:])(refresh=value == "true")
    if query:
        raise ArmError("invalid_request", "此接口不接受查询参数", 422)
    if method == "GET":
        if suffix == "/status":
            return 200, service.status()
        if suffix == "/groups":
            return 200, service.groups()
        if suffix == "/offsets":
            return 200, service.offsets()
        match = re.fullmatch(r"/tasks/([a-f0-9]{32})", suffix)
        if match:
            return 200, service.task(match[1])
    if method == "POST":
        names = {"/move": "move", "/reset": "reset", "/stop": "stop",
                 "/gripper/close": "gripper_close",
                 "/connection/reconnect": "reconnect", "/coordinate/increment": "coordinate",
                 "/coordinate/preview": "coordinate_preview",
                 "/groups/erase": "group_erase", "/offsets/set": "offset_set",
                 "/offsets/save": "offset_save"}
        name = names.get(suffix)
        task_match = re.fullmatch(r"/tasks/([a-f0-9]{32})/release", suffix)
        if task_match:
            name = "task_release"
        match = re.fullmatch(r"/groups/(\d{1,3})/(upload|run)", suffix)
        group_id = None
        if match:
            group_id = integer(int(match[1]), 0, 254, "group_id")
            name = "group_upload" if match[2] == "upload" else "group_run"
        if name is not None:
            command = parse_command(name, data)
            rid = command["request_id"]
            if name == "move":
                result = service.move(rid, command["targets"], command["duration_ms"], command["position_tolerance"])
            elif name == "task_release":
                result = service.release_task(rid, task_match[1], command["confirm_stationary"])
            elif name == "gripper_close":
                result = service.close_gripper(rid, command["close_target"], command["expected_contact_range"])
            elif name == "coordinate":
                result = service.coordinate(rid, command["delta_ticks"])
            elif name == "coordinate_preview":
                result = service.preview_coordinate(rid, command["delta_ticks"])
            elif name == "group_upload":
                result = service.upload_group(rid, group_id, command["frames"])
            elif name == "group_run":
                result = service.run_group(rid, group_id, command["repeat"])
            elif name == "group_erase":
                result = service.erase_groups(rid)
            elif name == "offset_set":
                result = service.set_offsets(rid, command["offsets"])
            elif name == "offset_save":
                result = service.save_offsets(rid)
            else:
                result = getattr(service, name)(rid)
            return (202 if result["status"] in {"sent", "stop_requested"} else 200), result
    raise ArmError("not_found", "接口或 HTTP 方法不存在", 404)


def json_object(raw: bytes):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError("JSON 字段不能重复")
            result[key] = value
        return result

    return json.loads(raw.decode("utf-8"), object_pairs_hook=pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError("JSON 不支持 NaN 或 Infinity")))


class ArmHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, service):
        if address[0] != "127.0.0.1":
            raise ValueError("本机接口仅支持监听 127.0.0.1")
        self.service = service
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    server_version = f"LeArmHTTP/{__version__}"
    sys_version = ""
    protocol_version = "HTTP/1.0"

    def setup(self):
        super().setup()
        self.connection.settimeout(5)

    def log_message(self, fmt, *args):
        LOG.info("HTTP %s", fmt % args)

    def _reply(self, status: int, body: dict):
        encoded = json.dumps(body, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(encoded)
        self.close_connection = True

    def _handle(self, method: str):
        service = self.server.service
        try:
            url = urlsplit(self.path)
            if url.scheme or url.netloc or url.fragment or "%" in url.path or "//" in url.path:
                raise ArmError("invalid_request", "接口路径格式无效", 400)
            query = parse_qs(url.query, keep_blank_values=True, max_num_fields=4)
            data = None
            if method == "POST":
                if self.headers.get("Transfer-Encoding"):
                    raise ArmError("invalid_request", "请使用 Content-Length 发送 JSON", 400)
                lengths = self.headers.get_all("Content-Length", [])
                if len(lengths) != 1 or not lengths[0].isdigit():
                    raise ArmError("invalid_request", "需要唯一且有效的 Content-Length", 411)
                length = int(lengths[0])
                if length > MAX_BODY_BYTES:
                    raise ArmError("request_too_large", "请求体最多 256 KiB", 413)
                if self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower() != "application/json":
                    raise ArmError("unsupported_media_type", "请求体必须为 application/json", 415)
                raw = self.rfile.read(length)
                if len(raw) != length:
                    raise ArmError("invalid_request", "请求体长度不足", 400)
                try:
                    data = json_object(raw)
                except (ValueError, UnicodeError, RecursionError):
                    raise ArmError("invalid_json", "请求体不是有效的 UTF-8 JSON，或包含重复字段", 400)
            status, result = route(service, method, url.path, query, data)
            self._reply(status, {"ok": True, "mode": service.settings.mode, "data": result})
        except ArmError as error:
            self._reply(error.status, {"ok": False, "mode": service.settings.mode, "error": error.as_dict()})
        except (ValueError, TimeoutError):
            self._reply(400, {"ok": False, "mode": service.settings.mode,
                              "error": ArmError("invalid_request", "请求格式无效或接收超时", 400).as_dict()})
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception:
            LOG.exception("处理 HTTP 请求失败")
            self._reply(500, {"ok": False, "mode": service.settings.mode,
                              "error": ArmError("internal_error", "服务内部错误，请查询状态；不要自动重发动作", 500, True).as_dict()})

    def do_GET(self):
        self._handle("GET")

    def do_POST(self):
        self._handle("POST")

    def do_PUT(self):
        self._reply(405, {"ok": False, "mode": self.server.service.settings.mode,
                          "error": ArmError("method_not_allowed", "使用 GET 或 POST", 405).as_dict()})

    do_DELETE = do_PUT
    do_PATCH = do_PUT
    do_HEAD = do_PUT
