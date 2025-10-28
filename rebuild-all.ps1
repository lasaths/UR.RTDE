# rebuild-all.ps1 - Rebuild native and managed code
Set-Location "C:\Users\lasaths\Github\UR.RTDE"

Write-Host "==================================" -ForegroundColor Cyan
Write-Host "UR.RTDE Complete Rebuild" -ForegroundColor Cyan
Write-Host "==================================" -ForegroundColor Cyan

# Find Visual Studio installation
$vsWhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vsWhere)) {
    Write-Error "Visual Studio not found!"
    exit 1
}

$vsPath = & $vsWhere -latest -property installationPath
$vcvarsPath = Join-Path $vsPath "VC\Auxiliary\Build\vcvarsall.bat"

if (-not (Test-Path $vcvarsPath)) {
    Write-Error "vcvarsall.bat not found at: $vcvarsPath"
    exit 1
}

Write-Host "`n[1/3] Setting up Visual Studio environment..." -ForegroundColor Yellow
$env:VSCMD_ARG_TGT_ARCH = "x64"

# Import VS environment variables
cmd /c "`"$vcvarsPath`" x64 && set" | ForEach-Object {
    if ($_ -match "^(.*?)=(.*)$") {
        Set-Item -Force -Path "ENV:\$($matches[1])" -Value $matches[2]
    }
}

Write-Host "`n[2/3] Building native C++ library..." -ForegroundColor Yellow
$buildDir = "build-native"

if (-not (Test-Path $buildDir)) {
    Write-Error "Build directory not found! Run build-native.bat first."
    exit 1
}

# Build directly - CMake should already be configured
Write-Host "Building ur_rtde_c_api..." -ForegroundColor Gray
& cmake --build $buildDir --config Release --target ur_rtde_c_api --parallel

if ($LASTEXITCODE -ne 0) {
    Write-Error "Native build failed!"
    exit 1
}

Write-Host "`n[3/3] Building C# wrapper..." -ForegroundColor Yellow
dotnet build src\UR.RTDE\UR.RTDE.csproj -c Release

if ($LASTEXITCODE -ne 0) {
    Write-Error "C# build failed!"
    exit 1
}

Write-Host "`n[DONE] Building URSimTests..." -ForegroundColor Yellow
dotnet build samples\URSimTests\URSimTests.csproj -c Release

if ($LASTEXITCODE -ne 0) {
    Write-Error "Test project build failed!"
    exit 1
}

Write-Host "`n=================================="-ForegroundColor Green
Write-Host "Build completed successfully!" -ForegroundColor Green
Write-Host "=================================="-ForegroundColor Green
Write-Host "`nNative DLL: $buildDir\Release\ur_rtde_c_api.dll" -ForegroundColor Cyan
Write-Host "Managed DLL: src\UR.RTDE\bin\Release\" -ForegroundColor Cyan
Write-Host "`nRun tests: dotnet run --project samples\URSimTests\URSimTests.csproj -c Release" -ForegroundColor Yellow
