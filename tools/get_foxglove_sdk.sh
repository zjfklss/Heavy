#!/usr/bin/env bash
# 下载 Foxglove C++ SDK 预编译包到 third_party/foxglove（约 34MB，不进 git）。
# 用法：bash tools/get_foxglove_sdk.sh [版本]    国内慢可先 export https_proxy=...
# 下完重新 cmake，日志出现 "Foxglove SDK: ON" 即生效。
set -euo pipefail
VER="${1:-0.29.1}"
ARCH="$(uname -m)"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DST="$ROOT/third_party/foxglove"
URL="https://github.com/foxglove/foxglove-sdk/releases/download/sdk/v${VER}/foxglove-v${VER}-cpp-${ARCH}-unknown-linux-gnu.zip"

if [ -f "$DST/lib/cmake/foxglove-sdk/foxglove-sdkConfig.cmake" ]; then
  echo "已存在 $DST，跳过（删掉目录可重下）"
  exit 0
fi
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
echo "下载 $URL"
curl -fL --retry 3 -o "$TMP/fox.zip" "$URL"
unzip -q "$TMP/fox.zip" -d "$TMP"
mkdir -p "$ROOT/third_party"
mv "$TMP/foxglove" "$DST"
echo "完成：$DST"
