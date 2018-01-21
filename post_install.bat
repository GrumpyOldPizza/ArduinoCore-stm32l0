@echo off
set ARGS=/SE /SW /SA
if "%PROCESSOR_ARCHITECTURE%" == "AMD64" (
  drivers\windows\dpinst-amd64.exe %ARGS%
) ELSE IF "%PROCESSOR_ARCHITEW6432%" == "AMD64" (
  drivers\windows\dpinst-amd64.exe %ARGS%
) ELSE (
  drivers\windows\dpinst-x86.exe %ARGS%
)
exit /b 0
