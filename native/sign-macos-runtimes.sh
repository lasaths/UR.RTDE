#!/usr/bin/env bash
# Re-sign macOS native dylibs after install_name_tool edits and drop generic Boost copies.
set -euo pipefail

ROOT="${1:-$(cd "$(dirname "$0")/.." && pwd)/src/UR.RTDE/runtimes}"

sign_dir() {
  local dir="$1"
  [[ -d "$dir" ]] || return 0
  find "$dir" -maxdepth 1 -name '*.dylib' -print0 | while IFS= read -r -d '' lib; do
    codesign --force --sign - "$lib"
    xattr -d com.apple.quarantine "$lib" 2>/dev/null || true
  done
}

for rid in osx-arm64 osx-x64; do
  native="${ROOT}/${rid}/native"
  [[ -d "$native" ]] || continue
  rm -f "${native}/libboost_system-mt.dylib" \
        "${native}/libboost_thread-mt.dylib" \
        "${native}/librtde.1.6.dylib" \
        "${native}/librtde.dylib"
  sign_dir "$native"
done

echo "Signed macOS runtimes under ${ROOT}"
