"""SDK 错误 → 工具返回的 envelope。

RobotSdkError 在 Python 侧只有消息字符串，码嵌在 "<CODE>: <message>" 前缀里，故需解析。
ok 是通信层，动作自己的 data["success"] 才是业务层。失败返回 envelope 而不抛 ——
Walle 的 MCP 客户端在 isError 时抛 RuntimeError，模型就读不到稳定错误码。
"""

from __future__ import annotations

from typing import Any

import robot_sdk

# 与 error.hpp 的 toString() 对应。
KNOWN_CODES = frozenset({
    "OK", "TIMEOUT", "CANCELLED", "UNAVAILABLE", "NOT_INITIALIZED",
    "INVALID_ARGUMENT", "OUT_OF_RANGE", "NOT_SUPPORTED", "INTERNAL", "UNKNOWN",
})


def ok(data: Any, message: str = "") -> dict:
    """成功，业务层结果在 data 里。"""
    return {"ok": True, "code": "OK", "message": message, "data": data}


def fail(error: robot_sdk.RobotSdkError) -> dict:
    """失败：RobotSdkError → envelope，不抛。"""
    code, _, message = str(error).partition(": ")
    if code not in KNOWN_CODES:
        return {"ok": False, "code": "INTERNAL", "message": str(error), "data": {}}
    return {"ok": False, "code": code, "message": message, "data": {}}


def invalid(message: str) -> dict:
    """参数不合法：不调 SDK 就能判定，直接返回 envelope。"""
    return {"ok": False, "code": "INVALID_ARGUMENT", "message": message, "data": {}}
