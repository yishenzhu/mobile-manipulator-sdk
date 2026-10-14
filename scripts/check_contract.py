#!/usr/bin/env python3
"""契约检查（CI 与本地都跑）：① 版本一致 ② 接口只增不删。

① `VERSION` 是**声明**，14 个文件 / 15 处（11 个 package.xml + 2 个 setup.py + version.hpp 的
   数字与字符串两项）必须与它一致 —— 手写这么多处，最容易悄悄漂。
② 把 .msg/.action 摊平成"声明清单"，和 docs/contract-baseline.txt 比：**基线必须是当前的**前缀**，
   而且按接口块比**（CDR 只有"末尾追加"兼容；中间插字段、删字段、改名、换类型都破坏；新增一个 msg
   文件属于"加块"，允许）。加了字段之后用 `--update` 把基线更新进 review。

⚠️ 机械检查只覆盖"接口声明 + 版本号"；C++ 头文件 / Python 绑定 / 工具语义的人工把关见 docs/contract.md。
"""

import argparse
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
INTERFACES = ROOT / "src" / "robot_sdk_interfaces"
BASELINE = ROOT / "docs" / "contract-baseline.txt"
VERSION_HEADER = ROOT / "src" / "robot_sdk_core" / "include" / "robot_sdk" / "version.hpp"
SKIP = ("/.venv/", "/build/", "/install/")


def interface_blocks():
    """{块名: [声明…]}。块名如 `msg RobotStatus` / `action MoveToPose [Goal]`。"""
    blocks = {}
    for path in sorted(INTERFACES.rglob("*")):
        if path.suffix not in (".msg", ".action") or any(s in str(path) for s in SKIP):
            continue
        kind = "action" if path.suffix == ".action" else "msg"
        section = "Goal" if kind == "action" else ""
        for raw in path.read_text().splitlines():
            line = raw.split("#", 1)[0].strip()          # 注释是给人看的，不进契约
            if not line:
                continue
            if line == "---":                            # action 的分段线
                section = "Result" if section in ("Goal", "") else "Feedback"
                continue
            name = f"{kind} {path.stem}" + (f" [{section}]" if section else "")
            blocks.setdefault(name, []).append(" ".join(line.split()))
    return blocks


def load_baseline():
    if not BASELINE.exists():
        raise SystemExit(f"缺少基线文件 {BASELINE} —— 先跑一次 --update")
    blocks, current = {}, None
    for raw in BASELINE.read_text().splitlines():
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        if raw.startswith("  "):
            blocks[current].append(raw.strip())
        else:
            current = raw.strip()
            blocks[current] = []
    return blocks


def dump_baseline(blocks):
    lines = ["# 接口基线：只增不删。改动请用 `scripts/check_contract.py --update`，让 diff 进 review。"]
    for name, items in blocks.items():
        lines.append(name)
        lines.extend(f"  {item}" for item in items)
    BASELINE.write_text("\n".join(lines) + "\n")


def compare(baseline, current):
    """返回 (破坏性问题, 允许的追加)。只比声明的**顺序与内容**，不比格式。"""
    problems, additions = [], []
    for name, base_items in baseline.items():
        if name not in current:
            problems.append(f"整块没了：{name}")
            continue
        now = current[name]
        if now[:len(base_items)] != base_items:
            for index, item in enumerate(base_items):
                if index >= len(now) or now[index] != item:
                    was = now[index] if index < len(now) else "（没了）"
                    problems.append(f"{name} 第 {index + 1} 项起变了：基线 `{item}` → 现在 `{was}`")
                    break
        elif len(now) > len(base_items):
            additions.append(f"{name} 追加：{', '.join(now[len(base_items):])}")
    additions.extend(f"新增接口块：{name}" for name in current if name not in baseline)
    return problems, additions


def check_versions():
    declared = (ROOT / "VERSION").read_text().strip()
    places = {}
    if not re.fullmatch(r"\d+\.\d+\.\d+", declared):
        return declared, places, {"VERSION": f"不是语义化版本：{declared!r}"}
    for path in sorted((ROOT / "src").rglob("package.xml")):
        found = re.search(r"<version>([^<]+)</version>", path.read_text())
        if found:
            places[str(path.relative_to(ROOT))] = found.group(1)
    for path in sorted((ROOT / "src").rglob("setup.py")):
        if any(s in str(path) for s in SKIP):
            continue
        found = re.search(r'^\s*version="([^"]+)"', path.read_text(), re.M)
        if found:
            places[str(path.relative_to(ROOT))] = found.group(1)
    # 最危险的一处：它编进公开头，漂了就是直接发给客户。
    header = VERSION_HEADER.read_text()

    def grab(pattern):
        found = re.search(pattern, header)
        return found.group(1) if found else "?"

    numbers = ".".join(grab(rf"kVersion{part} = (\d+);") for part in ("Major", "Minor", "Patch"))
    places[str(VERSION_HEADER.relative_to(ROOT)) + "（数字）"] = numbers
    places[str(VERSION_HEADER.relative_to(ROOT)) + '（kVersionString）'] = grab(
        r'kVersionString = "([^"]+)"')
    bad = {name: value for name, value in places.items() if value != declared}
    return declared, places, bad


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--update", action="store_true", help="把当前接口声明写成新基线")
    args = parser.parse_args()

    if args.update:
        blocks = interface_blocks()
        dump_baseline(blocks)
        print(f"已更新基线：{BASELINE.relative_to(ROOT)} —— "
              f"{len(blocks)} 个块、{sum(len(v) for v in blocks.values())} 条声明")
        return 0

    failed = False
    declared, places, bad = check_versions()
    print(f"版本：声明 {declared}（VERSION）")
    if bad:
        failed = True
        for name, value in bad.items():
            print(f"  ✗ {name}: {value}")
    else:
        print(f"  ✓ {len(places)} 处版本声明全部一致（package.xml / setup.py / version.hpp）")

    blocks = load_baseline()
    problems, additions = compare(blocks, interface_blocks())
    print(f"接口：基线 {len(blocks)} 个块 / 当前 {len(interface_blocks())} 个块")
    for item in additions:
        print(f"  + 允许的追加：{item}")
    for item in problems:
        print(f"  ✗ 破坏性改动：{item}")
    if problems:
        failed = True
    else:
        print("  ✓ 只增不删")

    if failed:
        print("\n结论：契约检查**未通过**。破坏性改动要么改回去，要么升 major 版本并更新基线。")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
