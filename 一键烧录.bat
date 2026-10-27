@echo off
chcp 936 >nul
setlocal enabledelayedexpansion
title 一键烧录 - 2024bizhang

set "HEX=%~dp0Debug\2024bizhang.hex"
set "FLASHER=C:\Infineon\AURIX-Studio-1.10.10\tools\AurixFlasherSoftwareTool_v3.0.4\AURIXFlasher.exe"

echo ============================================================
echo   2024bizhang 一键烧录
echo ============================================================
echo.

if not exist "%FLASHER%" (
    echo [错误] 找不到烧录工具:
    echo        %FLASHER%
    echo.
    pause
    exit /b 1
)

if not exist "%HEX%" (
    echo [错误] 找不到固件:
    echo        %HEX%
    echo.
    echo 请先在 IDE 里编译一次（Project - Build）
    pause
    exit /b 1
)

for %%F in ("%HEX%") do (
    echo   固件路径: %%~fF
    echo   编译时间: %%~tF
    echo   文件大小: %%~zF 字节
)
echo.
echo   [核对] 正确的大小应该是 233554 字节左右
echo          如果是 210000 左右，说明烧的是 2024 年的旧固件
echo.
echo ------------------------------------------------------------
echo   开始烧录，请不要拔线...
echo ------------------------------------------------------------
echo.

"%FLASHER%" -hex "%HEX%" -erase on -prog on -ver on -start on

set "RC=%ERRORLEVEL%"

echo.
if "%RC%"=="0" (
    echo ============================================================
    echo   [完成] 烧录成功，已自动复位，车可以跑了
    echo ============================================================
) else (
    echo ============================================================
    echo   [失败] 返回码 %RC%
    echo   检查：1^) 板子是否上电  2^) USB 线是否插好
    echo         3^) 按键是否按下（部分板子需要）
    echo ============================================================
)
echo.
pause
