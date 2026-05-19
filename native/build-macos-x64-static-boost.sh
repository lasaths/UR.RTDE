#!/usr/bin/env bash
# Build ur_rtde + C API facade for macOS x86_64 (Intel / Rosetta Rhino) with static Boost.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
UR_RTDE_SRC="${ROOT}/build-native/ur_rtde"
FACADE_SRC="${ROOT}/native/facade"
RUNTIME_DIR="${ROOT}/src/UR.RTDE/runtimes/osx-x64/native"
INSTALL_PREFIX="${UR_RTDE_SRC}/install-static-x64"
UR_RTDE_BUILD_DIR="${UR_RTDE_SRC}/build-static-x64"
FACADE_BUILD_DIR="${FACADE_SRC}/build-static-x64"
OSX_ARCH="x86_64"

# x86_64 Boost: Intel Homebrew (/usr/local) or explicit BOOST_ROOT.
if [[ -z "${BOOST_ROOT:-}" ]]; then
  for candidate in \
    /usr/local/opt/boost@1.85 \
    /usr/local/Cellar/boost@1.85/*/ \
    /usr/local/opt/boost \
    /usr/local/Cellar/boost/*/; do
    if [[ -d "${candidate}/include/boost" ]]; then
      BOOST_ROOT="${candidate}"
      break
    fi
  done
fi
if [[ -z "${BOOST_ROOT:-}" || ! -d "${BOOST_ROOT}/include/boost" ]]; then
  echo "x86_64 Boost not found. On Apple Silicon install with:" >&2
  echo "  arch -x86_64 /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\"" >&2
  echo "  arch -x86_64 brew install boost@1.85" >&2
  echo "Or set BOOST_ROOT to an x86_64 Boost prefix." >&2
  exit 1
fi

JOBS="$(sysctl -n hw.ncpu)"

echo "==> Target arch: ${OSX_ARCH}"
echo "==> Boost: ${BOOST_ROOT}"
echo "==> ur_rtde source: ${UR_RTDE_SRC}"

if [[ ! -d "${UR_RTDE_SRC}" ]]; then
  echo "Missing ${UR_RTDE_SRC}. Clone ur_rtde into build-native/ur_rtde first (see native/BUILD.md)." >&2
  exit 1
fi

CMAKE_ARCH_FLAGS=(-DCMAKE_OSX_ARCHITECTURES="${OSX_ARCH}")

rm -rf "${UR_RTDE_BUILD_DIR}" "${INSTALL_PREFIX}"
mkdir -p "${UR_RTDE_BUILD_DIR}"
cmake -S "${UR_RTDE_SRC}" -B "${UR_RTDE_BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DPYTHON_BINDINGS=OFF \
  -DEXAMPLES=OFF \
  -DBUILD_STATIC=ON \
  -DBoost_USE_STATIC_LIBS=ON \
  -DBoost_ROOT="${BOOST_ROOT}" \
  "${CMAKE_ARCH_FLAGS[@]}"

cmake --build "${UR_RTDE_BUILD_DIR}" --parallel "${JOBS}"
cmake --install "${UR_RTDE_BUILD_DIR}"

RTDE_ARCHIVE="$(find "${INSTALL_PREFIX}" -maxdepth 4 -name 'librtde*.a' | head -1)"
if [[ -z "${RTDE_ARCHIVE}" ]]; then
  echo "librtde static archive not found under ${INSTALL_PREFIX}" >&2
  exit 1
fi

echo "==> Static ur_rtde archive: ${RTDE_ARCHIVE}"

rm -rf "${FACADE_BUILD_DIR}"
cmake -S "${FACADE_SRC}" -B "${FACADE_BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${FACADE_BUILD_DIR}/install" \
  -Dur_rtde_DIR="${INSTALL_PREFIX}/lib/cmake/ur_rtde" \
  -DUR_RTDE_FACADE_STATIC_RTDE=ON \
  -DBoost_USE_STATIC_LIBS=ON \
  -DBoost_ROOT="${BOOST_ROOT}" \
  "${CMAKE_ARCH_FLAGS[@]}"

cmake --build "${FACADE_BUILD_DIR}" --parallel "${JOBS}"

FACADE_DYLIB="${FACADE_BUILD_DIR}/libur_rtde_c_api.dylib"
if [[ ! -f "${FACADE_DYLIB}" ]]; then
  FACADE_DYLIB="$(find "${FACADE_BUILD_DIR}" -name 'libur_rtde_c_api.dylib' | head -1)"
fi

echo "==> Checking facade architecture:"
file "${FACADE_DYLIB}" | tee /dev/stderr
if ! file "${FACADE_DYLIB}" | grep -q 'x86_64'; then
  echo "ERROR: expected x86_64 dylib." >&2
  exit 1
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
