"""envelope 翻译：成功 / 各错误码 / 无前缀 / 参数非法。"""

import robot_sdk
from robot_sdk_mcp import errors


def test_ok():
    assert errors.ok({"success": True}, "arrived") == {
        "ok": True,
        "code": "OK",
        "message": "arrived",
        "data": {"success": True},
    }


def test_fail_parses_code_prefix():
    got = errors.fail(robot_sdk.RobotSdkError("TIMEOUT: no robot status received"))
    assert got["ok"] is False
    assert got["code"] == "TIMEOUT"
    assert got["message"] == "no robot status received"


def test_fail_code_without_message():
    got = errors.fail(robot_sdk.RobotSdkError("UNAVAILABLE"))
    assert got["code"] == "UNAVAILABLE"
    assert got["message"] == ""


def test_fail_unknown_prefix_falls_back_to_internal():
    got = errors.fail(robot_sdk.RobotSdkError("something odd"))
    assert got["code"] == "INTERNAL"
    assert got["message"] == "something odd"  # 正文不丢


def test_invalid_needs_no_sdk_call():
    got = errors.invalid("bad arg")
    assert got == {
        "ok": False,
        "code": "INVALID_ARGUMENT",
        "message": "bad arg",
        "data": {},
    }
