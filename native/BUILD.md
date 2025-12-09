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
- Boost 1.70+ (`brew install boost`)
- Git

## Step 1: Clone and Build ur_rtde

```bash
# Clone ur_rtde
git clone --recursive https://gitlab.com/sdurobotics/ur_rtde.git
cd ur_rtde
git checkout v1.6.2

# Build ur_rtde
mkdir build && cd build

# Windows
cmake .. -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDINGS=OFF
cmake --build . --config Release
cmake --install . --prefix ../install

# macOS
cmake .. -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDINGS=OFF
make -j$(sysctl -n hw.ncpu)
make install DESTDIR=../install
```

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
