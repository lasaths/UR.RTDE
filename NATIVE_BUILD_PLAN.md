# Native C++ Wrapper Build Plan

## 🎯 Goal
Build production-ready native C++ wrapper for NuGet distribution

## 📋 Prerequisites Check

Run these commands to check your environment:

```powershell
# 1. Check CMake
cmake --version

# 2. Check Visual Studio C++ (cl.exe)
cl.exe

# 3. Check vcpkg (if installed)
vcpkg version

# 4. Check git
git --version
```

## 🛠️ Required Tools

### 1. Visual Studio 2022 (or 2019)
- **Workload**: "Desktop development with C++"
- **Components**: MSVC v143, Windows SDK
- **Download**: https://visualstudio.microsoft.com/downloads/

### 2. CMake 3.20+
- **Download**: https://cmake.org/download/
- **Alternative**: Comes with Visual Studio

### 3. vcpkg (Package Manager)
```powershell
# Install vcpkg
cd C:\
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
.\vcpkg integrate install
```

### 4. Install Boost via vcpkg
```powershell
# Install Boost (required by ur_rtde)
vcpkg install boost-system:x64-windows
vcpkg install boost-thread:x64-windows
vcpkg install boost-chrono:x64-windows
```

## 📦 Build Steps

### Step 1: Clone and Build ur_rtde
```powershell
cd C:\Users\lasaths\Github\UR.RTDE
mkdir build-native
cd build-native

# Clone ur_rtde
git clone --recursive https://gitlab.com/sdurobotics/ur_rtde.git
cd ur_rtde
git checkout v1.6.2

# Build ur_rtde
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDINGS=OFF -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build . --config Release
cmake --install . --prefix ../install
```

### Step 2: Build C API Facade
```powershell
cd C:\Users\lasaths\Github\UR.RTDE\native\facade
mkdir build
cd build

# Configure
cmake .. -DCMAKE_BUILD_TYPE=Release -Dur_rtde_DIR=C:\Users\lasaths\Github\UR.RTDE\build-native\ur_rtde\install

# Build
cmake --build . --config Release

# Output will be in: build/Release/ur_rtde_c_api.dll
```

### Step 3: Copy to NuGet Structure
```powershell
# Create runtime folders
cd C:\Users\lasaths\Github\UR.RTDE
mkdir -Force src\UR.RTDE\runtimes\win-x64\native

# Copy DLLs
Copy-Item native\facade\build\Release\ur_rtde_c_api.dll src\UR.RTDE\runtimes\win-x64\native\
Copy-Item build-native\ur_rtde\install\bin\*.dll src\UR.RTDE\runtimes\win-x64\native\
```

### Step 4: Switch from Python.NET to Native P/Invoke
- Remove `pythonnet` NuGet dependency
- Enable native `RTDEControl.cs` and `RTDEReceive.cs`
- Disable `PythonBridge` namespace

### Step 5: Test & Package
```powershell
# Build NuGet
cd C:\Users\lasaths\Github\UR.RTDE
dotnet pack src/UR.RTDE -c Release

# Test console app
cd samples/Console
dotnet run --configuration Release
```

## ⏱️ Time Estimate

| Step | Time |
|------|------|
| Install Visual Studio | 30 min |
| Install vcpkg + Boost | 30 min |
| Build ur_rtde | 15 min |
| Build C API facade | 10 min |
| Test & package | 15 min |
| **Total** | **~2 hours** |

## 🚨 Common Issues

### Issue: "CMake not found"
**Solution**: Install CMake or use VS CMake
```powershell
# Add CMake to PATH (if installed with VS)
$env:PATH += ";C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin"
```

### Issue: "Boost not found"
**Solution**: Install via vcpkg
```powershell
vcpkg install boost-system:x64-windows boost-thread:x64-windows
```

### Issue: "cl.exe not found"
**Solution**: Run from "Developer Command Prompt for VS 2022"
- Start Menu → Visual Studio 2022 → Developer Command Prompt

## 🎯 Next Steps

**Tell me:**
1. Do you have Visual Studio with C++ installed?
2. Do you have vcpkg installed?
3. Should I guide you through the setup, or can you run it yourself?

I can provide step-by-step commands for each part!
