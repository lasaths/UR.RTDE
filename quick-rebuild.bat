@echo off
REM Quick rebuild of C API facade only (assumes ur_rtde already built)

echo ========================================
echo Quick Rebuild: C API Facade + C# Wrapper
echo ========================================

REM Check VS environment
where cl.exe >nul 2>&1
if errorlevel 1 (
    echo ERROR: Run from "Developer Command Prompt for VS 2022"
    exit /b 1
)

set BOOST_ROOT=C:\Boost
set BOOST_INCLUDEDIR=C:\Boost\include\boost-1_84
set BOOST_LIBRARYDIR=C:\Boost\lib

echo [1/3] Rebuilding C API facade...
cd /d "%~dp0native\facade\build"

cmake --build . --config Release --target ur_rtde_c_api --parallel
if errorlevel 1 (
    echo ERROR: C API facade build failed
    exit /b 1
)

echo [2/3] Copying DLL...
copy /Y "Release\ur_rtde_c_api.dll" "%~dp0src\UR.RTDE\runtimes\win-x64\native\"

echo [3/3] Building C# wrapper...
cd /d "%~dp0"
dotnet build src\UR.RTDE\UR.RTDE.csproj -c Release
dotnet build samples\URSimTests\URSimTests.csproj -c Release

echo.
echo ========================================
echo ✓ REBUILD COMPLETE!
echo ========================================
echo Run tests: dotnet run --project samples\URSimTests -c Release
