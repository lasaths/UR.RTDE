@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

echo ========================================
echo Building C API facade (fixed)
echo ========================================
cd /d C:\Users\lasaths\Github\UR.RTDE\native\facade\build

cmake --build . --config Release --parallel
if errorlevel 1 exit /b 1

echo.
echo ========================================
echo Copying DLLs to NuGet structure
echo ========================================
cd /d C:\Users\lasaths\Github\UR.RTDE
if not exist "src\UR.RTDE\runtimes\win-x64\native" mkdir "src\UR.RTDE\runtimes\win-x64\native"

copy /Y "native\facade\build\Release\ur_rtde_c_api.dll" "src\UR.RTDE\runtimes\win-x64\native\"
copy /Y "build-native\ur_rtde\build\Release\rtde.dll" "src\UR.RTDE\runtimes\win-x64\native\"

echo.
dir "src\UR.RTDE\runtimes\win-x64\native\"

echo.
echo ========================================
echo ✓✓✓ BUILD COMPLETE SUCCESS! ✓✓✓
echo ========================================
echo.
echo Native DLLs ready for NuGet packaging
echo.
