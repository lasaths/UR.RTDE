# Build Instructions for Native Facade

## Prerequisites

### Windows (x64)
- Visual Studio 2019 or later (with C++ tools)
- CMake 3.15+
- Boost 1.70+ (install via vcpkg recommended)
- Git

### macOS (arm64)
- Xcode Command Line Tools
- CMake 3.15+ (`brew install cmake`)
- Boost 1.85 preferred for Rhino 8 package builds (`brew install boost@1.85` if available; newer Homebrew Boost may require CMake target adjustments)
- Git

## Step 1: Clone and Build ur_rtde

```bash
# Clone ur_rtde
git clone --recursive https://gitlab.com/sdurobotics/ur_rtde.git
cd ur_rtde

# Pin to known-good commit for reproducible builds (ur_rtde 1.6.3)
git checkout 68ac4e18f357f8e9361bfc5eef344acd9aa241be

# Build ur_rtde
mkdir build && cd build

# Windows
cmake .. -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDINGS=OFF
cmake --build . --config Release
cmake --install . --prefix ../install

# macOS dynamic developer build
cmake .. -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDINGS=OFF
make -j$(sysctl -n hw.ncpu)
make install DESTDIR=../install
```

For the packaged macOS arm64 runtime used in Rhino 8, prefer the static-Boost build script:

```bash
native/build-macos-arm64-static-boost.sh
```

That script builds upstream `ur_rtde` as a static archive, links it into `libur_rtde_c_api.dylib`, hides C++/Boost symbols, verifies the facade does not link separate `librtde` or Boost dylibs, then installs only `libur_rtde_c_api.dylib` into `src/UR.RTDE/runtimes/osx-arm64/native/`.

## Step 2: Build C API Facade

```bash
cd ../../native/facade
mkdir build && cd build

# Windows
cmake .. -DCMAKE_BUILD_TYPE=Release -Dur_rtde_DIR=../../../ur_rtde/install/lib/cmake/ur_rtde
cmake --build . --config Release

# macOS
cmake .. -DCMAKE_BUILD_TYPE=Release -Dur_rtde_DIR=../../../ur_rtde/install/lib/cmake/ur_rtde
make -j$(sysctl -n hw.ncpu)
```

## Output Locations

After build:
- **Windows**: `build/Release/ur_rtde_c_api.dll`
- **macOS**: `build/libur_rtde_c_api.dylib`

Copy these to:
- **Windows**: `../../src/UR.RTDE/runtimes/win-x64/native/`
- **macOS arm64**: `../../src/UR.RTDE/runtimes/osx-arm64/native/`

## Troubleshooting

### Boost not found
**Windows**: Install via vcpkg
```bash
vcpkg install boost-system:x64-windows boost-thread:x64-windows
cmake .. -DCMAKE_TOOLCHAIN_FILE=[vcpkg root]/scripts/buildsystems/vcpkg.cmake
```

**macOS**: Install via Homebrew
```bash
brew install boost
```

### ur_rtde not found
Ensure `-Dur_rtde_DIR` points to the correct CMake config directory.
