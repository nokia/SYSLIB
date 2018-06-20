@echo off

if %0%1==%0  goto usage

set MACROS_FILE=%1
REM =========================================================================
if not exist %SYSLIB_INSTALL_PATH:/=\%\ti (
echo SYSLIB_INSTALL_PATH = %SYSLIB_INSTALL_PATH%/packages > %MACROS_FILE%
) else (
echo SYSLIB_INSTALL_PATH = %SYSLIB_INSTALL_PATH% > %MACROS_FILE%
)
if defined PDK_INSTALL_PATH (
echo PDK_INSTALL_PATH    = %PDK_INSTALL_PATH% >> %MACROS_FILE%
)
if defined BIOS_INSTALL_PATH (
echo BIOS_INSTALL_PATH   = %BIOS_INSTALL_PATH% >> %MACROS_FILE%
)
if defined IPC_INSTALL_PATH (
echo IPC_INSTALL_PATH    = %IPC_INSTALL_PATH% >> %MACROS_FILE%
)
if defined UIA_INSTALL_PATH (
echo UIA_INSTALL_PATH  = %UIA_INSTALL_PATH% >> %MACROS_FILE%
)
if defined SNOW_INSTALL_PATH (
echo SNOW_INSTALL_PATH  = %SNOW_INSTALL_PATH% >> %MACROS_FILE%
)
if defined CPPIDEV_INSTALL_PATH (
echo CPPIDEV_INSTALL_PATH  = %CPPIDEV_INSTALL_PATH% >> %MACROS_FILE%
)
if defined QMSSDEV_INSTALL_PATH (
echo QMSSDEV_INSTALL_PATH  = %QMSSDEV_INSTALL_PATH% >> %MACROS_FILE%
)
REM =========================================================================

goto :eof

:usage
echo.
echo "  Usage: %0 <File with path>"
echo "  Example:- %0 macros_tmdxevm6638lxe.ini"
echo.
