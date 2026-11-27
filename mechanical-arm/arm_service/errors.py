"""统一的调用错误，包含命令结果是否未知。"""


class ArmError(Exception):
    def __init__(self, code: str, message: str, status: int = 409,
                 outcome_unknown: bool = False, details: dict | None = None):
        super().__init__(message)
        self.code = code
        self.message = message
        self.status = status
        self.outcome_unknown = outcome_unknown
        self.details = details or {}

    def as_dict(self) -> dict:
        return {"code": self.code, "message": self.message,
                "outcome_unknown": self.outcome_unknown, "details": self.details}
