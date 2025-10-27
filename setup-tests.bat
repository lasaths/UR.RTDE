@echo off
REM Setup script to prepare UR.RTDE for testing with URSim

echo ========================================
echo UR.RTDE Testing Setup
echo ========================================
echo.

echo [1/3] Copying Boost DLL to runtimes folder...
if exist "C:\vcpkg\installed\x64-windows\bin\boost_thread-vc143-mt-x64-1_89.dll" (
    copy /Y "C:\vcpkg\installed\x64-windows\bin\boost_thread-vc143-mt-x64-1_89.dll" "src\UR.RTDE\runtimes\win-x64\native\"
    echo   - Copied boost_thread DLL
) else (
    echo   ERROR: Boost DLL not found in vcpkg
    echo   Please ensure vcpkg Boost is installed
    exit /b 1
)

echo.
echo [2/3] Copying DLLs to Console sample...
copy /Y "src\UR.RTDE\runtimes\win-x64\native\*.dll" "samples\Console\"
echo   - Copied all DLLs to Console sample

echo.
echo [3/3] Building URSimTests...
cd samples\URSimTests
dotnet build -c Release
if errorlevel 1 (
    echo   ERROR: Build failed
    exit /b 1
)

cd ..\..
copy /Y "src\UR.RTDE\runtimes\win-x64\native\*.dll" "samples\URSimTests\bin\Release\net8.0\"
echo   - Copied DLLs to URSimTests

echo.
echo ========================================
echo Setup Complete!
echo ========================================
echo.
echo Next steps:
echo   1. Ensure URSim is running at 172.18.0.2
echo   2. Access VNC: http://172.18.0.2:6080/vnc.html
echo   3. Power on robot and release brakes
echo   4. Run tests: cd samples\URSimTests ^&^& dotnet run -c Release
echo.
