# Updating to Newer Versions of ur_rtde

**Purpose**: This guide provides step-by-step instructions for updating the UR.RTDE C# wrapper to use a newer version of the upstream [ur_rtde C++ library](https://gitlab.com/sdurobotics/ur_rtde).

**Current Version**: ur_rtde v1.6.0 (pinned)  
**Last Updated**: 2025-10-28

---

## Overview

The UR.RTDE wrapper is built on top of the SDU Robotics `ur_rtde` C++ library. When a new version of `ur_rtde` is released, you may want to update to get:

- Bug fixes
- Performance improvements  
- New features
- Support for newer UR controller versions

This guide covers the complete process: checking compatibility → updating source → patching → building → testing → packaging.

---

## Table of Contents

1. [Before You Start](#before-you-start)
2. [Step 1: Check New Version](#step-1-check-new-version)
3. [Step 2: Update Source Code](#step-2-update-source-code)
4. [Step 3: Apply Compatibility Patches](#step-3-apply-compatibility-patches)
5. [Step 4: Update C API Facade](#step-4-update-c-api-facade)
6. [Step 5: Rebuild Native Libraries](#step-5-rebuild-native-libraries)
7. [Step 6: Update C# Wrapper](#step-6-update-c-wrapper)
8. [Step 7: Test with URSim](#step-7-test-with-ursim)
9. [Step 8: Update Documentation](#step-8-update-documentation)
10. [Step 9: Package and Release](#step-9-package-and-release)
11. [Troubleshooting](#troubleshooting)
12. [Rollback Instructions](#rollback-instructions)

---

## Before You Start

### Prerequisites

- ✅ Clean git working directory (commit or stash changes)
- ✅ VS 2022 with C++ workload installed
- ✅ vcpkg installed and configured
- ✅ URSim container running (for testing)
- ✅ Backup of current working build

### Backup Current Version

```powershell
# Create backup branch
git checkout -b backup/ur_rtde-v1.6.0
git push origin backup/ur_rtde-v1.6.0

# Return to main branch
git checkout main
```

---

## Step 1: Check New Version

### 1.1 Check ur_rtde Releases

Visit: https://gitlab.com/sdurobotics/ur_rtde/-/tags

**Look for**:
- Latest stable tag (e.g., `v1.7.0`)
- Release notes/changelog
- Breaking changes
- New dependencies

### 1.2 Review Changelog

Read the release notes carefully:

```bash
# Clone latest ur_rtde to temporary location
cd C:\Temp
git clone https://gitlab.com/sdurobotics/ur_rtde.git ur_rtde-latest
cd ur_rtde-latest
git checkout v1.7.0  # Replace with target version

# View changelog
git log v1.6.0..v1.7.0 --oneline
git diff v1.6.0..v1.7.0 -- include/
```

### 1.3 Check Breaking Changes

**Key areas to check**:

| Area | What to Look For | Impact |
|------|------------------|--------|
| **Public API** | Added/removed/renamed methods | C API facade needs updates |
| **Dependencies** | Boost version changes | vcpkg updates required |
| **Build system** | CMake changes | Build scripts need updates |
| **Data types** | Changed structs/enums | P/Invoke bindings need updates |
| **Threading model** | Async behavior changes | C# wrapper needs review |

### 1.4 Document Decision

Create a tracking issue:

```markdown
# Update ur_rtde to v1.7.0

**Reason**: [Bug fixes / New features / Performance]
**Breaking Changes**: [Yes/No - list if yes]
**Dependencies**: [Any new requirements]
**Risk**: [Low/Medium/High]
**Estimated Effort**: [Hours/Days]
```

---

## Step 2: Update Source Code

### 2.1 Update Git Submodule (if using)

If `ur_rtde` is a git submodule:

```powershell
cd C:\Users\lasaths\Github\UR.RTDE\build-native\ur_rtde
git fetch --all --tags
git checkout v1.7.0  # Replace with target version
cd ..\..
git add build-native/ur_rtde
git commit -m "Update ur_rtde to v1.7.0"
```

### 2.2 Manual Source Update (current method)

Our current setup uses a manual copy of ur_rtde source:

```powershell
# Backup current source
cd C:\Users\lasaths\Github\UR.RTDE\build-native
Rename-Item ur_rtde ur_rtde-v1.6.0-backup

# Clone new version
git clone https://gitlab.com/sdurobotics/ur_rtde.git
cd ur_rtde
git checkout v1.7.0

# Document the version
git rev-parse HEAD > VERSION.txt
```

### 2.3 Verify Source Structure

Check that key files are present:

```powershell
# Should exist:
Test-Path include\ur_rtde\rtde_control_interface.h  # Should be True
Test-Path include\ur_rtde\rtde_receive_interface.h  # Should be True
Test-Path include\ur_rtde\rtde_io_interface.h       # Should be True
Test-Path src\rtde_control_interface.cpp            # Should be True
Test-Path CMakeLists.txt                            # Should be True
```

---

## Step 3: Apply Compatibility Patches

### 3.1 Known Patches (from v1.6.0)

We had to patch ur_rtde v1.6.0 for Boost 1.89.0 compatibility:

**Original Issue**: `boost::asio::io_service` deprecated → use `boost::asio::io_context`

**Files Patched**:
- `include/ur_rtde/rtde.h`
- `src/rtde.cpp`

**Changes**:
```cpp
// OLD (Boost < 1.70):
boost::asio::io_service io_service_;

// NEW (Boost >= 1.70):
boost::asio::io_context io_service_;
```

### 3.2 Check if Patches Still Needed

```powershell
cd C:\Users\lasaths\Github\UR.RTDE\build-native\ur_rtde

# Search for io_service usage
Select-String -Path "include\ur_rtde\*.h" -Pattern "io_service"
Select-String -Path "src\*.cpp" -Pattern "io_service"
```

**If found**: Apply patches (see 3.3)  
**If not found**: Patches already upstream! Skip to Step 4

### 3.3 Apply Patches

**Method 1: Automated Patch File** (recommended for future)

Create `patches/boost-1.89-compat.patch`:

```diff
--- a/include/ur_rtde/rtde.h
+++ b/include/ur_rtde/rtde.h
@@ -200,7 +200,7 @@ class RTDE
   std::string hostname_;
   int port_;
   bool verbose_;
-  boost::asio::io_service io_service_;
+  boost::asio::io_context io_service_;
   tcp::socket socket_;
   std::atomic<bool> conn_state_;
   
--- a/src/rtde.cpp
+++ b/src/rtde.cpp
@@ -50,7 +50,7 @@ RTDE::RTDE(const std::string &hostname, int port, bool verbose)
     : hostname_(hostname),
       port_(port),
       verbose_(verbose),
-      io_service_(),
+      io_service_(),  // Type changed in header
       socket_(io_service_),
       conn_state_(false)
 {
```

Apply patch:

```powershell
cd C:\Users\lasaths\Github\UR.RTDE\build-native\ur_rtde
git apply ..\..\patches\boost-1.89-compat.patch
```

**Method 2: Manual Edit**

```powershell
# Edit files manually
code include\ur_rtde\rtde.h
# Replace: io_service_ → io_context_ (or use your preferred editor)
```

### 3.4 Document Patches Applied

Update `build-native/PATCHES_APPLIED.md`:

```markdown
# Patches Applied to ur_rtde v1.7.0

## Boost 1.89 Compatibility

**Status**: [APPLIED / NOT NEEDED]
**Files**: rtde.h, rtde.cpp
**Reason**: io_service deprecated in Boost 1.70+
**Applied**: 2025-10-28
```

---

## Step 4: Update C API Facade

### 4.1 Check for New ur_rtde Methods

Compare headers:

```powershell
# Compare old vs new
diff C:\Users\lasaths\Github\UR.RTDE\build-native\ur_rtde-v1.6.0-backup\include\ur_rtde\rtde_control_interface.h `
     C:\Users\lasaths\Github\UR.RTDE\build-native\ur_rtde\include\ur_rtde\rtde_control_interface.h
```

**Look for**:
- New public methods
- Removed methods (breaking!)
- Changed signatures (breaking!)

### 4.2 Add New Methods to C API

If new methods exist in ur_rtde, add them to the C API facade:

**Example**: New method `setGravity` added in ur_rtde v1.7.0

**Step 1**: Update `native/facade/ur_rtde_c_api.h`

```c
// Add new method
UR_RTDE_C_API int ur_rtde_control_set_gravity(
    ur_rtde_control_t* control,
    double* gravity_vector,  // [x, y, z]
    int* error_code
);
```

**Step 2**: Update `native/facade/ur_rtde_c_api.cpp`

```cpp
int ur_rtde_control_set_gravity(
    ur_rtde_control_t* control,
    double* gravity_vector,
    int* error_code
) {
    if (!control || !control->instance || !gravity_vector) {
        if (error_code) *error_code = -1;
        return 0;
    }
    
    try {
        std::vector<double> gravity(gravity_vector, gravity_vector + 3);
        bool result = control->instance->setGravity(gravity);
        if (error_code) *error_code = 0;
        return result ? 1 : 0;
    }
    catch (const std::exception& e) {
        if (error_code) *error_code = -2;
        return 0;
    }
}
```

### 4.3 Rebuild Facade CMake

Update `native/facade/CMakeLists.txt` if needed:

```cmake
# Check ur_rtde version
find_package(ur_rtde 1.7.0 REQUIRED)  # Update version
```

---

## Step 5: Rebuild Native Libraries

### 5.1 Clean Previous Build

```powershell
cd C:\Users\lasaths\Github\UR.RTDE

# Remove old build artifacts
Remove-Item -Recurse -Force build-native\ur_rtde\build -ErrorAction SilentlyContinue
Remove-Item -Recurse -Force native\facade\build -ErrorAction SilentlyContinue
Remove-Item src\UR.RTDE\runtimes\win-x64\native\*.dll -ErrorAction SilentlyContinue
```

### 5.2 Build ur_rtde

```powershell
cd build-native\ur_rtde
mkdir build
cd build

# Configure with vcpkg
cmake .. -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE=C:\vcpkg\scripts\buildsystems\vcpkg.cmake `
  -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build . --config Release

# Verify output
Test-Path Release\rtde.dll  # Should be True
```

**Expected output**: `rtde.dll` (~800 KB)

### 5.3 Build C API Facade

```powershell
cd ..\..\..\native\facade
mkdir build
cd build

# Configure
cmake .. -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE=C:\vcpkg\scripts\buildsystems\vcpkg.cmake `
  -DCMAKE_BUILD_TYPE=Release `
  -DUR_RTDE_DIR=..\..\..\build-native\ur_rtde\build

# Build
cmake --build . --config Release

# Verify output
Test-Path Release\ur_rtde_c_api.dll  # Should be True
```

**Expected output**: `ur_rtde_c_api.dll` (~50 KB)

### 5.4 Copy to NuGet Structure

```powershell
cd ..\..\..

# Copy DLLs to NuGet runtimes folder
Copy-Item build-native\ur_rtde\build\Release\rtde.dll `
          src\UR.RTDE\runtimes\win-x64\native\

Copy-Item native\facade\build\Release\ur_rtde_c_api.dll `
          src\UR.RTDE\runtimes\win-x64\native\
```

### 5.5 Verify DLL Dependencies

```powershell
# Check what DLLs depend on
dumpbin /dependents src\UR.RTDE\runtimes\win-x64\native\ur_rtde_c_api.dll
dumpbin /dependents src\UR.RTDE\runtimes\win-x64\native\rtde.dll
```

**Should NOT see** external dependencies besides Windows system DLLs (kernel32.dll, etc.)

---

## Step 6: Update C# Wrapper

### 6.1 Add P/Invoke Bindings for New Methods

If new C API methods were added, update `src/UR.RTDE/NativeMethods.cs`:

```csharp
// Add new P/Invoke declaration
[DllImport("ur_rtde_c_api", CallingConvention = CallingConvention.Cdecl)]
internal static extern int ur_rtde_control_set_gravity(
    IntPtr control,
    [In] double[] gravityVector,
    out int errorCode
);
```

### 6.2 Add C# Wrapper Methods

Update `src/UR.RTDE/RTDEControl.cs` (or appropriate class):

```csharp
/// <summary>
/// Sets the gravity vector for payload calculation.
/// </summary>
/// <param name="gravityVector">Gravity vector [x, y, z] in m/s²</param>
/// <returns>True if successful</returns>
public bool SetGravity(double[] gravityVector)
{
    if (gravityVector == null || gravityVector.Length != 3)
        throw new ArgumentException("Gravity vector must have 3 components", nameof(gravityVector));
    
    ThrowIfDisposed();
    
    int result = NativeMethods.ur_rtde_control_set_gravity(
        _handle,
        gravityVector,
        out int errorCode
    );
    
    if (errorCode != 0)
        throw new RTDEException($"Failed to set gravity (error code: {errorCode})");
    
    return result != 0;
}
```

### 6.3 Rebuild C# Wrapper

```powershell
cd C:\Users\lasaths\Github\UR.RTDE

# Clean
dotnet clean src\UR.RTDE

# Build both TFMs
dotnet build src\UR.RTDE -c Release -f net48
dotnet build src\UR.RTDE -c Release -f net8.0
```

**Expected output**:
- `src/UR.RTDE/bin/Release/net48/UR.RTDE.dll`
- `src/UR.RTDE/bin/Release/net8.0/UR.RTDE.dll`

### 6.4 Update Version Numbers

**File**: `src/UR.RTDE/UR.RTDE.csproj`

```xml
<PropertyGroup>
  <TargetFrameworks>net48;net8.0</TargetFrameworks>
  <Version>2.1.0</Version>  <!-- Increment version -->
  <AssemblyVersion>2.1.0.0</AssemblyVersion>
  <FileVersion>2.1.0.0</FileVersion>
  <PackageReleaseNotes>
    - Updated to ur_rtde v1.7.0
    - Added SetGravity method
    - Performance improvements
  </PackageReleaseNotes>
</PropertyGroup>
```

---

## Step 7: Test with URSim

### 7.1 Start URSim

```powershell
# Start URSim container (if not running)
cd C:\Users\lasaths\Github\Multi-Actor-Interface-Library\docker\ursim
docker-compose up -d

# Verify running
docker ps | Select-String "ursim"

# Check IP (should be 172.18.0.2)
```

### 7.2 Run Existing Tests

```powershell
cd C:\Users\lasaths\Github\UR.RTDE\samples\Console

# Run test suite
dotnet run --configuration Release
```

**Expected output**: All tests pass (connection, streaming, movement, etc.)

### 7.3 Test New Features

If new methods were added, create a test:

**File**: `samples/Console/Program.cs` (add test)

```csharp
static void TestSetGravity(RTDEControl control)
{
    Console.WriteLine("\n=== Test 8: Set Gravity ===");
    
    try
    {
        double[] earthGravity = { 0.0, 0.0, -9.82 };
        bool result = control.SetGravity(earthGravity);
        
        Console.WriteLine(result ? "✅ Gravity set successfully" : "❌ Failed to set gravity");
    }
    catch (Exception ex)
    {
        Console.WriteLine($"❌ Exception: {ex.Message}");
    }
}
```

Run the new test:

```powershell
dotnet run --configuration Release
```

### 7.4 Long-Duration Test

Test stability over extended period:

```powershell
# Run 5-minute streaming test
dotnet run --configuration Release -- --test streaming --duration 300
```

**Monitor for**:
- Memory leaks
- Dropped packets
- Connection stability
- CPU usage

---

## Step 8: Update Documentation

### 8.1 Update README.md

```markdown
Built on [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) C++ library (v1.7.0) by SDU Robotics.

## 🎯 Features

### Core Capabilities (v2.1.0)
- ✅ Updated to ur_rtde v1.7.0
- ✅ New: Gravity vector configuration
- ✅ Performance improvements
```

### 8.2 Update AGENTS.md

```markdown
**Version**: 2.1.0 (Updated to ur_rtde v1.7.0)  
**Validation**: URSim e-Series 5.23.0 @ 172.18.0.2

1. ✅ A thin **native C ABI façade** over `ur_rtde` v1.7.0
```

### 8.3 Update FEATURE_COVERAGE.md

Add new features to the coverage matrix.

### 8.4 Create CHANGELOG.md Entry

```markdown
# Changelog

## [2.1.0] - 2025-10-28

### Changed
- Updated ur_rtde from v1.6.0 to v1.7.0
- Performance improvements in RTDE receive loop

### Added
- RTDEControl.SetGravity() method
- Support for new gravity vector configuration

### Fixed
- [Any bug fixes from upstream]

### Breaking Changes
- [List any breaking changes, if present]
```

### 8.5 Update Build Documentation

Update `BUILD_INSTRUCTIONS.md` and `BUILD_SUCCESS.md` with new version numbers.

---

## Step 9: Package and Release

### 9.1 Create NuGet Package

```powershell
cd C:\Users\lasaths\Github\UR.RTDE

# Pack with new version
dotnet pack src\UR.RTDE -c Release -o nupkgs

# Verify package
Test-Path nupkgs\UR.RTDE.2.1.0.nupkg  # Should be True
```

### 9.2 Test NuGet Package

Create a test project:

```powershell
mkdir test-nuget
cd test-nuget
dotnet new console
dotnet add package UR.RTDE --source ..\nupkgs --version 2.1.0

# Verify DLLs copied
Test-Path bin\Debug\net8.0\runtimes\win-x64\native\ur_rtde_c_api.dll  # Should be True
```

### 9.3 Git Tag and Commit

```powershell
cd C:\Users\lasaths\Github\UR.RTDE

# Commit all changes
git add -A
git commit -m "Update to ur_rtde v1.7.0 - Version 2.1.0

- Updated ur_rtde from v1.6.0 to v1.7.0
- Added SetGravity method
- Updated documentation
- All tests passing with URSim 5.23.0"

# Tag release
git tag -a v2.1.0 -m "Release v2.1.0 - ur_rtde v1.7.0"
git push origin main --tags
```

### 9.4 Publish to NuGet.org (Optional)

```powershell
# Get API key from nuget.org
$apiKey = "YOUR_NUGET_API_KEY"

# Push package
dotnet nuget push nupkgs\UR.RTDE.2.1.0.nupkg `
  --api-key $apiKey `
  --source https://api.nuget.org/v3/index.json
```

### 9.5 Create GitHub Release

1. Go to: https://github.com/lasaths/UR.RTDE/releases
2. Click "Draft a new release"
3. Tag: `v2.1.0`
4. Title: `UR.RTDE v2.1.0 - ur_rtde v1.7.0 Update`
5. Description: Copy from CHANGELOG.md
6. Attach: `UR.RTDE.2.1.0.nupkg`
7. Publish release

---

## Troubleshooting

### Issue: Build Fails with Boost Errors

**Symptoms**:
```
error: undefined reference to 'boost::system::...'
```

**Solution**:
```powershell
# Update Boost
cd C:\vcpkg
vcpkg update
vcpkg upgrade boost:x64-windows
```

### Issue: New Methods Cause Runtime Errors

**Symptoms**:
```
DllNotFoundException: Unable to load DLL 'ur_rtde_c_api'
```

**Solution**:
1. Check C API exports: `dumpbin /exports native\facade\build\Release\ur_rtde_c_api.dll`
2. Verify P/Invoke signatures match exactly
3. Check calling convention (should be Cdecl)

### Issue: Tests Fail After Update

**Symptoms**: Previous tests now fail

**Solution**:
1. Check ur_rtde changelog for behavioral changes
2. Review test expectations
3. Update tests to match new behavior
4. If breaking change, document in CHANGELOG

### Issue: Memory Leaks Detected

**Symptoms**: Memory usage grows over time

**Solution**:
1. Check Dispose() implementations
2. Verify native handles are freed
3. Use memory profiler (dotMemory, PerfView)
4. Check ur_rtde for upstream memory leaks

---

## Rollback Instructions

If the update fails or causes issues:

### Quick Rollback

```powershell
cd C:\Users\lasaths\Github\UR.RTDE

# Restore from backup branch
git checkout backup/ur_rtde-v1.6.0
git checkout -b main-rollback

# Restore ur_rtde source
cd build-native
Remove-Item -Recurse ur_rtde
Rename-Item ur_rtde-v1.6.0-backup ur_rtde

# Rebuild
cd ..
.\build-complete.bat

# Test
cd samples\Console
dotnet run --configuration Release
```

### Clean Rollback

```powershell
# Reset to previous tag
git checkout v1.1.0.0
git checkout -b rollback-to-v1.1.0.0

# Verify tests
cd samples\Console
dotnet run --configuration Release

# If good, merge
git checkout main
git reset --hard v1.1.0.0
git push --force  # Use with caution!
```

---

## Checklist

Use this checklist when updating ur_rtde:

- [ ] **Step 1**: Check new version and changelog
- [ ] **Step 2**: Backup current version (git branch)
- [ ] **Step 3**: Update ur_rtde source code
- [ ] **Step 4**: Apply/verify compatibility patches
- [ ] **Step 5**: Check for new/removed methods
- [ ] **Step 6**: Update C API facade (if needed)
- [ ] **Step 7**: Rebuild native libraries
- [ ] **Step 8**: Update P/Invoke bindings (if needed)
- [ ] **Step 9**: Update C# wrapper classes (if needed)
- [ ] **Step 10**: Rebuild C# wrapper (both TFMs)
- [ ] **Step 11**: Run existing test suite
- [ ] **Step 12**: Test new features (if added)
- [ ] **Step 13**: Long-duration stability test
- [ ] **Step 14**: Update README.md
- [ ] **Step 15**: Update AGENTS.md
- [ ] **Step 16**: Update FEATURE_COVERAGE.md
- [ ] **Step 17**: Create CHANGELOG.md entry
- [ ] **Step 18**: Update version numbers
- [ ] **Step 19**: Create NuGet package
- [ ] **Step 20**: Test NuGet package
- [ ] **Step 21**: Git commit and tag
- [ ] **Step 22**: Push to GitHub
- [ ] **Step 23**: Create GitHub release
- [ ] **Step 24**: Publish to NuGet.org (optional)

---

## Automation (Future Enhancement)

Consider creating a PowerShell script `Update-UrRtde.ps1`:

```powershell
param(
    [Parameter(Mandatory=$true)]
    [string]$NewVersion,
    
    [switch]$SkipTests,
    [switch]$AutoCommit
)

# Automated update script
# - Clone new version
# - Apply patches
# - Build
# - Test
# - Package
# - Commit
```

This would reduce manual steps and ensure consistency across updates.

---

**Questions?** Contact the maintainer or file an issue at: https://github.com/lasaths/UR.RTDE/issues
