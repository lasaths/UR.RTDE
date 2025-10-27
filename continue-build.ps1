# Automated Native Build Continuation Script
# Run this after Boost installation completes

Write-Output "=================================================================  
UR.RTDE NATIVE BUILD - AUTOMATED CONTINUATION
=================================================================`n"

# Check Boost installed
Write-Output "[1/5] Verifying Boost installation..."
if (!(Test-Path "C:\vcpkg\installed\x64-windows\lib\boost_system*.lib")) {
    Write-Error "Boost not installed! Run Boost install first."
    exit 1
}
Write-Output "✅ Boost libraries found`n"

# Build ur_rtde
Write-Output "[2/5] Building ur_rtde C++ library (5-10 minutes)..."
cd "C:\Users\lasaths\Github\UR.RTDE\build-native\ur_rtde"
if (!(Test-Path "build")) { mkdir build | Out-Null }
cd build

cmake .. `
    -G "Visual Studio 17 2022" `
    -A x64 `
    -DCMAKE_BUILD_TYPE=Release `
    -DPYTHON_BINDINGS=OFF `
    -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"

if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake configuration failed!"
    exit 1
}

cmake --build . --config Release --parallel
if ($LASTEXITCODE -ne 0) {
    Write-Error "ur_rtde build failed!"
    exit 1
}

cmake --install . --prefix ../install --config Release
Write-Output "✅ ur_rtde built and installed`n"

# Build C API facade
Write-Output "[3/5] Building C API facade (2 minutes)..."
cd "C:\Users\lasaths\Github\UR.RTDE\native\facade"
if (!(Test-Path "build")) { mkdir build | Out-Null }
cd build

cmake .. `
    -G "Visual Studio 17 2022" `
    -A x64 `
    -DCMAKE_BUILD_TYPE=Release `
    -Dur_rtde_DIR="C:/Users/lasaths/Github/UR.RTDE/build-native/ur_rtde/install/lib/cmake/ur_rtde" `
    -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"

if ($LASTEXITCODE -ne 0) {
    Write-Error "C API CMake configuration failed!"
    exit 1
}

cmake --build . --config Release --parallel
if ($LASTEXITCODE -ne 0) {
    Write-Error "C API build failed!"
    exit 1
}
Write-Output "✅ C API facade built`n"

# Copy DLLs
Write-Output "[4/5] Copying DLLs to NuGet structure..."
cd "C:\Users\lasaths\Github\UR.RTDE"
if (!(Test-Path "src\UR.RTDE\runtimes\win-x64\native")) {
    mkdir -Force "src\UR.RTDE\runtimes\win-x64\native" | Out-Null
}

Copy-Item "native\facade\build\Release\ur_rtde_c_api.dll" "src\UR.RTDE\runtimes\win-x64\native\" -Force
Copy-Item "build-native\ur_rtde\build\Release\rtde.dll" "src\UR.RTDE\runtimes\win-x64\native\" -Force

if ((Test-Path "src\UR.RTDE\runtimes\win-x64\native\ur_rtde_c_api.dll") -and 
    (Test-Path "src\UR.RTDE\runtimes\win-x64\native\rtde.dll")) {
    Write-Output "✅ DLLs copied successfully"
    $apiSize = (Get-Item "src\UR.RTDE\runtimes\win-x64\native\ur_rtde_c_api.dll").Length / 1KB
    $rtdeSize = (Get-Item "src\UR.RTDE\runtimes\win-x64\native\rtde.dll").Length / 1KB
    Write-Output "   ur_rtde_c_api.dll: $([math]::Round($apiSize, 2)) KB"
    Write-Output "   rtde.dll: $([math]::Round($rtdeSize, 2)) KB`n"
} else {
    Write-Error "DLL copy failed!"
    exit 1
}

# Test with URSim
Write-Output "[5/5] Testing with URSim..."
Write-Output "Running Console demo against URSim at 172.18.0.2...`n"
cd "C:\Users\lasaths\Github\UR.RTDE\samples\Console"
dotnet run --configuration Release -- 172.18.0.2

if ($LASTEXITCODE -eq 0) {
    Write-Output "`n✅ Test PASSED!`n"
} else {
    Write-Warning "Test failed or robot not reachable. Check URSim is running."
}

# Package NuGet
Write-Output "[BONUS] Packaging NuGet..."
cd "C:\Users\lasaths\Github\UR.RTDE"
if (!(Test-Path "nupkgs")) { mkdir nupkgs | Out-Null }
dotnet pack src\UR.RTDE -c Release -o nupkgs

if (Test-Path "nupkgs\UR.RTDE.*.nupkg") {
    $nupkg = Get-Item "nupkgs\UR.RTDE.*.nupkg" | Select-Object -First 1
    $nupkgSize = $nupkg.Length / 1KB
    Write-Output "✅ NuGet package created: $($nupkg.Name) ($([math]::Round($nupkgSize, 2)) KB)`n"
}

Write-Output "=================================================================  
✅ NATIVE BUILD COMPLETE!
=================================================================`n"
Write-Output "Next steps:"
Write-Output "  1. git add -A"
Write-Output "  2. git commit -m 'Native build complete - working NuGet package'"
Write-Output "  3. git push"
Write-Output "  4. Publish to NuGet.org (if ready)`n"
Write-Output "Package location: nupkgs\UR.RTDE.*.nupkg"
