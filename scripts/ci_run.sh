#!/usr/bin/env bash
# 在 CI 里跑一条命令：失败时把输出尾部打出来，并做成 ::error:: annotation。
# 为什么要它：公开仓库下载 job 日志需要 token，annotation 是唯一"公开可读"的失败现场。
# 收敛说明：以前这段在 3 个步骤里各抄一遍（还有过 PIPESTATUS 之类只属于 bash 的写法，容器里翻车）。
#
#   ./scripts/ci_run.sh <标签> <命令...>
#   ./scripts/ci_run.sh ASan bash -c './scripts/build_x.sh && ./scripts/run_y.sh'
set -uo pipefail

label="${1:?用法: ci_run.sh <标签> <命令...>}"; shift

if ! out="$( "$@" 2>&1 )"; then
  printf '%s\n' "$out" | tail -30
  printf '%s\n' "$out" | tail -8 | while IFS= read -r line; do
    printf '::error::[%s] %s\n' "$label" "$(printf '%s' "$line" | tr -d '\r')"
  done
  exit 1
fi
printf '%s\n' "$out" | tail -5
