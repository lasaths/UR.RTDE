#!/usr/bin/env bash
# Build ur_rtde + C API facade for macOS arm64 with Boost/ur_rtde linked into one C ABI dylib.
# This keeps Boost/Asio C++ symbols hidden from Rhino's process.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
UR_RTDE_SRC="${ROOT}/build-native/ur_rtde"
FACADE_SRC="${ROOT}/native/facade"
RUNTIME_DIR="${ROOT}/src/UR.RTDE/runtimes/osx-arm64/native"
INSTALL_PREFIX="${UR_RTDE_SRC}/install-static"

# Boost 1.85 provides boost_system CMake config; 1.89+ is header-only for system.
BOOST_ROOT="${BOOST_ROOT:-/opt/homebrew/Cellar/boost@1.85/1.85.0_3}"
if [[ ! -d "${BOOST_ROOT}" ]]; then
  BOOST_ROOT="$(brew --prefix boost 2>/dev/null || true)"
fi
if [[ ! -d "${BOOST_ROOT}/include/boost" ]]; then
  echo "Boost not found. Install with: brew install boost" >&2
  exit 1
fi

JOBS="$(sysctl -n hw.ncpu)"

echo "==> Boost: ${BOOST_ROOT}"
echo "==> ur_rtde source: ${UR_RTDE_SRC}"

rm -rf "${UR_RTDE_SRC}/build-static" "${INSTALL_PREFIX}"
mkdir -p "${UR_RTDE_SRC}/build-static"
cmake -S "${UR_RTDE_SRC}" -B "${UR_RTDE_SRC}/build-static" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DPYTHON_BINDINGS=OFF \
  -DEXAMPLES=OFF \
  -DBUILD_STATIC=ON \
  -DBoost_USE_STATIC_LIBS=ON \
  -DBoost_ROOT="${BOOST_ROOT}"

cmake --build "${UR_RTDE_SRC}/build-static" --parallel "${JOBS}"
cmake --install "${UR_RTDE_SRC}/build-static"

RTDE_ARCHIVE="$(find "${INSTALL_PREFIX}" -maxdepth 4 -name 'librtde*.a' | head -1)"
if [[ -z "${RTDE_ARCHIVE}" ]]; then
  echo "librtde static archive not found under ${INSTALL_PREFIX}" >&2
  exit 1
fi

echo "==> Static ur_rtde archive: ${RTDE_ARCHIVE}"

rm -rf "${FACADE_SRC}/build-static"
cmake -S "${FACADE_SRC}" -B "${FACADE_SRC}/build-static" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${FACADE_SRC}/build-static/install" \
  -Dur_rtde_DIR="${INSTALL_PREFIX}/lib/cmake/ur_rtde" \
  -DUR_RTDE_FACADE_STATIC_RTDE=ON \
  -DBoost_USE_STATIC_LIBS=ON \
  -DBoost_ROOT="${BOOST_ROOT}"

cmake --build "${FACADE_SRC}/build-static" --parallel "${JOBS}"

FACADE_DYLIB="${FACADE_SRC}/build-static/libur_rtde_c_api.dylib"
if [[ ! -f "${FACADE_DYLIB}" ]]; then
  FACADE_DYLIB="$(find "${FACADE_SRC}/build-static" -name 'libur_rtde_c_api.dylib' | head -1)"
fi

echo "==> Checking facade has no dynamic ur_rtde/Boost dependencies:"
otool -L "${FACADE_DYLIB}" | tee /dev/stderr
if otool -L "${FACADE_DYLIB}" | grep -Eqi 'boost|librtde'; then
  echo "ERROR: libur_rtde_c_api still links Boost or librtde dynamically." >&2
  exit 1
fi

echo "==> Checking facade exports only the C API:"
if nm -gU "${FACADE_DYLIB}" | c++filt | grep -Eiq 'boost::|std::__|ur_rtde::'; then
  echo "ERROR: libur_rtde_c_api exports C++ symbols." >&2
  nm -gU "${FACADE_DYLIB}" | c++filt | grep -Ei 'boost::|std::__|ur_rtde::' | head -50 >&2
  exit 1
fi

mkdir -p "${RUNTIME_DIR}"
cp -f "${FACADE_DYLIB}" "${RUNTIME_DIR}/libur_rtde_c_api.dylib"

rm -f "${RUNTIME_DIR}"/librtde*.dylib \
      "${RUNTIME_DIR}"/libboost_*_ur_rtde.dylib \
      "${RUNTIME_DIR}"/libboost_system*.dylib \
      "${RUNTIME_DIR}"/libboost_thread*.dylib

cd "${RUNTIME_DIR}"

install_name_tool -id "@rpath/libur_rtde_c_api.dylib" libur_rtde_c_api.dylib 2>/dev/null || true

bash "${ROOT}/native/sign-macos-runtimes.sh" "$(dirname "${RUNTIME_DIR}")/.."

echo "==> Installed to ${RUNTIME_DIR}:"
ls -la "${RUNTIME_DIR}"
