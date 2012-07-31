@set CONFIG_DIR=%CD%\config
@set INCLUDE_DIR=%CD%\..\include
@if not defined WORK_DIR set WORK_DIR=.
@set OUTPUT_DIR=%WORK_DIR%\bin\test\%1
@set CONFIG_NAME=%1
@shift
@xcopy /i /r /y /z ".\src" "%OUTPUT_DIR%" >nul &&^
cd /D %OUTPUT_DIR% &&^
make %1 %2 %3 %4 %5 %6 %7 %8 %9
