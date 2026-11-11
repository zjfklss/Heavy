#!/usr/bin/env bash
# 把上游 sp_jlcv_aimer 的 rm_simulation/ 同步到本仓库的 rm_simulation/。
#
# 用法：
#   scripts/sync_rm_simulation.sh --check     只看差异，不改任何文件（先跑这个）
#   scripts/sync_rm_simulation.sh             同步，但不提交；改完自己 git diff 审一遍再提交
#   scripts/sync_rm_simulation.sh --ref=<分支/标签/提交号>   同步指定版本，默认 main
#
# 环境变量 RM_SIM_UPSTREAM 可以换上游地址（默认走 SSH，需要本机配好 GitHub 密钥）。
#
# 本仓库自己加的文件不会被覆盖或删除，见下面的 LOCAL_ONLY。以后再加本地文件，把相对路径加进去。
# 同步后 rm_simulation/UPSTREAM_COMMIT 会记下来源提交，方便追溯。
set -euo pipefail

UPSTREAM="${RM_SIM_UPSTREAM:-git@github.com:ASLianEFY/sp_jlcv_aimer.git}"
REF=main
CHECK=0
for arg in "$@"; do
  case "$arg" in
    --check) CHECK=1 ;;
    --ref=*) REF="${arg#--ref=}" ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "未知参数：$arg（见 --help）" >&2; exit 2 ;;
  esac
done

# 本仓库独有、上游没有的文件（相对 rm_simulation/）。
LOCAL_ONLY=(
  "configs/scenarios/sensors/hero.json"   # 英雄传感器预设，参数取自 configs/hero.yaml
  "UPSTREAM_COMMIT"                       # 本脚本写入的来源记录
)

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
dest="$root/rm_simulation"
[[ -d "$dest" ]] || { echo "找不到 $dest" >&2; exit 1; }

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

echo "从 $UPSTREAM ($REF) 拉取 rm_simulation/ ..."
# 只取这一个目录，避免下载上游整个 119MB 的历史
GIT_TERMINAL_PROMPT=0 git clone -q --depth 1 --filter=blob:none --sparse --branch "$REF" "$UPSTREAM" "$work/up" 2>/dev/null ||
  { # --branch 不接受提交号：退回完整稀疏克隆再 checkout
    rm -rf "$work/up"
    GIT_TERMINAL_PROMPT=0 git clone -q --filter=blob:none --sparse "$UPSTREAM" "$work/up"
    git -C "$work/up" checkout -q "$REF"
  }
git -C "$work/up" sparse-checkout set rm_simulation
commit="$(git -C "$work/up" rev-parse HEAD)"
subject="$(git -C "$work/up" log -1 --format=%s)"
[[ -d "$work/up/rm_simulation" ]] || { echo "上游 $REF 里没有 rm_simulation/" >&2; exit 1; }

excludes=(--exclude=/build/ --exclude=/UPSTREAM_COMMIT)
for f in "${LOCAL_ONLY[@]}"; do excludes+=("--exclude=/$f"); done

# 只按内容(校验和)和可执行位比较：时间戳、组写权限每次克隆都不同，git 也不记录它们
mode=(-rl --checksum --executability --delete --itemize-changes)
(( CHECK )) && mode+=(--dry-run)

echo "上游提交：${commit:0:7} $subject"
echo "---- 将要变化的文件（空表示已是最新）----"
changes="$(rsync "${mode[@]}" "${excludes[@]}" "$work/up/rm_simulation/" "$dest/" | grep -v '/$' || true)"
[[ -n "$changes" ]] && echo "$changes" || echo "(无)"

if (( CHECK )); then
  echo "---- 仅检查，未改动任何文件 ----"
  exit 0
fi

{
  echo "upstream: $UPSTREAM"
  echo "commit:   $commit"
  echo "subject:  $subject"
  echo "synced:   $(date -Iseconds)"
} > "$dest/UPSTREAM_COMMIT"
echo "已同步到 ${commit:0:7}。请 git diff rm_simulation 审阅、重新构建并确认 hero --simulation 能用，再提交。"
