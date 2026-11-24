"""总控程序可使用的标准库客户端；控制请求不自动重试。"""

from __future__ import annotations

import json
import uuid
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class ArmClientError(Exception):
    def __init__(self, message: str, response: dict | None = None):
        super().__init__(message)
        self.response = response


class ArmClient:
    def __init__(self, base_url: str = "http://127.0.0.1:8000/v1/arm", timeout: float = 10):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    def get(self, path: str) -> dict:
        return self._request("GET", path)

    def post(self, path: str, body: dict | None = None, request_id: str | None = None) -> dict:
        payload = dict(body or {})
        payload["request_id"] = request_id or uuid.uuid4().hex
        return self._request("POST", path, payload)

    def _request(self, method: str, path: str, data: dict | None = None) -> dict:
        body = json.dumps(data, ensure_ascii=False, allow_nan=False).encode("utf-8") if data is not None else None
        request = Request(self.base_url + "/" + path.lstrip("/"), data=body, method=method,
                          headers={"Content-Type": "application/json"} if body is not None else {})
        try:
            with urlopen(request, timeout=self.timeout) as response:
                result = json.load(response)
        except HTTPError as error:
            try:
                result = json.load(error)
            except (ValueError, UnicodeError):
                raise ArmClientError(f"HTTP {error.code}：服务响应无法解析") from error
            raise ArmClientError(result.get("error", {}).get("message", str(error)), result) from error
        except (URLError, TimeoutError, OSError) as error:
            raise ArmClientError("连接失败或等待超时；控制结果可能未知，请查询状态，不要自动重发") from error
        if not result.get("ok"):
            raise ArmClientError("服务返回错误", result)
        return result
