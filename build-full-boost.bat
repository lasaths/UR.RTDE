@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

echo ========================================
echo Installing Full Boost Library
echo ========================================
cd /d C:\vcpkg
vcpkg install boost:x64-windows

if errorlevel 1 exit /b 1

echo.
echo ========================================
echo Rebuilding ur_rtde with full Boost
echo ========================================
cd /d C:\Users\lasaths\Github\UR.RTDE\build-native\ur_rtde
rd /s /q build
mkdir build
cd build

cmake .. ^
    -G "Visual Studio 17 2022" ^
    -A x64 ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DPYTHON_BINDINGS=OFF ^
    -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake ^
    -DVCPKG_TARGET_TRIPLET=x64-windows

if errorlevel 1 exit /b 1

cmake --build . --config Release --parallel
if errorlevel 1 exit /b 1

cmake --install . --prefix ../install --config Release
echo ✓ ur_rtde installed

echo.
echo ========================================
echo Building C API facade
echo ========================================
cd /d C:\Users\lasaths\Github\UR.RTDE\native\facade
if not exist build mkdir build
cd build

cmake .. ^
    -G "Visual Studio 17 2022" ^
    -A x64 ^
    -DCMAKE_BUILD_TYPE=Release ^
    -Dur_rtde_DIR=C:/Users/lasaths/Github/UR.RTDE/build-native/ur_rtde/install/lib/cmake/ur_rtde ^
    -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake ^
    -DVCPKG_TARGET_TRIPLET=x64-windows

if errorlevel 1 exit /b 1

cmake --build . --config Release --parallel
if errorlevel 1 exit /b 1

echo.
echo ========================================
echo Copying DLLs
echo ========================================
cd /d C:\Users\lasaths\Github\UR.RTDE
if not exist "src\UR.RTDE\runtimes\win-x64\native" mkdir "src\UR.RTDE\runtimes\win-x64\native"

copy /Y "native\facade\build\Release\ur_rtde_c_api.dll" "src\UR.RTDE\runtimes\win-x64\native\"
copy /Y "build-native\ur_rtde\build\Release\rtde.dll" "src\UR.RTDE\runtimes\win-x64\native\"

dir "src\UR.RTDE\runtimes\win-x64\native\"

echo.
echo ========================================
echo ✓ BUILD COMPLETE!
echo ========================================
