# Manual Build Instructions for UR.RTDE Native Wrapper

## Prerequisites

1. **Visual Studio 2022** with "Desktop development with C++"
2. **Boost 1.84.0** pre-built binaries installed to `C:\Boost`
3. **CMake 3.20+** (included with VS 2022)

## Step 1: Install Boost

1. Download: [Boost 1.84.0 MSVC 14.3 x64](https://sourceforge.net/projects/boost/files/boost-binaries/1.84.0/boost_1_84_0-msvc-14.3-64.exe/download)
2. Run installer
3. Install to: `C:\Boost`
4. Wait for installation (~5 minutes)

## Step 2: Open Developer Command Prompt

**Important:** You MUST use Developer Command Prompt, not regular PowerShell!

- Start Menu → Visual Studio 2022 → Developer Command Prompt for VS 2022

## Step 3: Run Build Script

```bat
cd C:\Users\lasaths\Github\UR.RTDE
build-native.bat
```

This script will:
1. ✅ Check Boost installation
2. ✅ Build ur_rtde library (5-10 minutes)
3. ✅ Build C API facade (2 minutes)
4. ✅ Copy DLLs to NuGet structure

## Step 4: Test

```bat
cd samples\Console
dotnet run --configuration Release
```

Expected output:
```
✅ Control: True
✅ Receive: True
✅ Joint positions read
✅ Streaming @ 500 Hz
✅ MoveJ executed
✅ StopJ executed
```

## Step 5: Package NuGet

```bat
cd C:\Users\lasaths\Github\UR.RTDE
dotnet pack src\UR.RTDE -c Release -o nupkgs
```

Output: `nupkgs\UR.RTDE.1.0.0.nupkg`

## Troubleshooting

### Error: "cl.exe not found"
**Solution:** Use Developer Command Prompt, not regular PowerShell

### Error: "Boost not found"
**Solution:** Install Boost to `C:\Boost` (exact path required)

### Error: "CMake not found"
**Solution:** Add to PATH:
```bat
set PATH=%PATH%;C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin
```

### Error: "ur_rtde build failed"
**Solution:** Check that Boost installed correctly to `C:\Boost`
- Should have `C:\Boost\include\boost-1_84\boost\`
- Should have `C:\Boost\lib\boost_system-vc143-mt-x64-1_84.lib`

## Build Output

After successful build:

```
src/UR.RTDE/runtimes/win-x64/native/
├── ur_rtde_c_api.dll    (Your C API wrapper)
└── rtde.dll             (ur_rtde core library)
```

These DLLs will be automatically included in the NuGet package.

## Next: macOS Build

For macOS arm64 support (later):
1. Install Xcode
2. Install Homebrew
3. `brew install boost cmake`
4. Similar CMake commands with `-DCMAKE_OSX_ARCHITECTURES=arm64`
