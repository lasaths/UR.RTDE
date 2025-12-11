@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "C:\Users\lasaths\GitHub\UR.RTDE\native\facade"

cl.exe /LD /std:c++17 /EHsc /O2 /MD /DUR_RTDE_C_API_EXPORTS ^
    /I"C:\Users\lasaths\GitHub\UR.RTDE\build-native\ur_rtde\include" ^
    /I"C:\vcpkg\installed\x64-windows\include" ^
    ur_rtde_c_api.cpp ^
    /link ^
    /LIBPATH:"C:\Users\lasaths\GitHub\UR.RTDE\build-native\ur_rtde\build\Release" ^
    /LIBPATH:"C:\vcpkg\installed\x64-windows\lib" ^
    rtde.lib ^
    /OUT:ur_rtde_c_api.dll

echo Build complete!
endlocal
