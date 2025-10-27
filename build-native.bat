@echo off
REM Build script for UR.RTDE native wrapper
REM Run this from Developer Command Prompt for VS 2022

echo ========================================
echo UR.RTDE Native C++ Build Script
echo ========================================
echo.

REM Check if running in VS Developer Command Prompt
where cl.exe >nul 2>&1
if errorlevel 1 (
    echo ERROR: cl.exe not found!
    echo Please run this from "Developer Command Prompt for VS 2022"
    echo Start Menu ^> Visual Studio 2022 ^> Developer Command Prompt
    exit /b 1
)

echo [1/5] Checking Boost installation...
if not exist "C:\Boost" (
    echo ERROR: Boost not found at C:\Boost
    echo Please install Boost 1.84.0 to C:\Boost first
    echo Run: boost_1_84_0-msvc-14.3-64.exe
    exit /b 1
)
set BOOST_ROOT=C:\Boost
set BOOST_INCLUDEDIR=C:\Boost\include\boost-1_84
set BOOST_LIBRARYDIR=C:\Boost\lib
echo ✓ Boost found

echo.
echo [2/5] Building ur_rtde library...
cd /d "%~dp0build-native\ur_rtde"
if not exist build mkdir build
cd build

cmake .. ^
    -G "Visual Studio 17 2022" ^
    -A x64 ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DPYTHON_BINDINGS=OFF ^
    -DBOOST_ROOT=%BOOST_ROOT% ^
    -DBoost_INCLUDE_DIR=%BOOST_INCLUDEDIR% ^
    -DBoost_LIBRARY_DIRS=%BOOST_LIBRARYDIR% ^
    -DBoost_USE_STATIC_LIBS=ON

if errorlevel 1 (
    echo ERROR: CMake configuration failed
    exit /b 1
)

cmake --build . --config Release --parallel
if errorlevel 1 (
    echo ERROR: ur_rtde build failed
    exit /b 1
)

echo ✓ ur_rtde built successfully

echo.
echo [3/5] Installing ur_rtde...
cmake --install . --prefix ../install --config Release
echo ✓ ur_rtde installed

echo.
echo [4/5] Building C API facade...
cd /d "%~dp0native\facade"
if not exist build mkdir build
cd build

cmake .. ^
    -G "Visual Studio 17 2022" ^
    -A x64 ^
    -DCMAKE_BUILD_TYPE=Release ^
    -Dur_rtde_DIR="%~dp0build-native\ur_rtde\install\lib\cmake\ur_rtde" ^
    -DBOOST_ROOT=%BOOST_ROOT%

if errorlevel 1 (
    echo ERROR: C API facade CMake configuration failed
    exit /b 1
)

cmake --build . --config Release --parallel
if errorlevel 1 (
    echo ERROR: C API facade build failed
    exit /b 1
)

echo ✓ C API facade built successfully

echo.
echo [5/5] Copying DLLs to NuGet structure...
cd /d "%~dp0"
if not exist "src\UR.RTDE\runtimes\win-x64\native" mkdir "src\UR.RTDE\runtimes\win-x64\native"

copy /Y "native\facade\build\Release\ur_rtde_c_api.dll" "src\UR.RTDE\runtimes\win-x64\native\"
copy /Y "build-native\ur_rtde\build\Release\rtde.dll" "src\UR.RTDE\runtimes\win-x64\native\"

echo ✓ DLLs copied to NuGet structure

echo.
echo ========================================
echo ✓ BUILD COMPLETE!
echo ========================================
echo.
echo Native DLLs ready at: src\UR.RTDE\runtimes\win-x64\native\
echo.
echo Next steps:
echo   1. Test: dotnet run --project samples\Console
echo   2. Pack: dotnet pack src\UR.RTDE -c Release
echo.
