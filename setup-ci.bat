@echo off
rem CI / AppVeyor: vcpkg only (Python z obrazu lub -DPython3_ROOT_DIR)
setlocal
cd /d "%~dp0"

echo Preparing vcpkg packages
pushd ref\vcpkg
call bootstrap-vcpkg.bat
if errorlevel 1 popd & exit /b 1
vcpkg install directx-dxc:x64-windows
if errorlevel 1 popd & exit /b 1
popd

endlocal
