@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
cd /d "C:\Users\lasaths\Github\UR.RTDE\native\facade"

cl.exe /LD /std:c++17 /EHsc /O2 /MD /DUR_RTDE_C_API_EXPORTS ^
    /I"C:\Users\lasaths\Github\UR.RTDE\build-native\ur_rtde\install\include" ^
    /I"C:\vcpkg\installed\x64-windows\include" ^
    ur_rtde_c_api.cpp ^
    /link ^
    /LIBPATH:"C:\Users\lasaths\Github\UR.RTDE\build-native\ur_rtde\install\lib" ^
    /LIBPATH:"C:\vcpkg\installed\x64-windows\lib" ^
    rtde.lib ^
    /OUT:ur_rtde_c_api.dll

echo Build complete!
