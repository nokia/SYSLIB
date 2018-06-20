@REM ******************************************************************************
@REM * FILE PURPOSE: Syslib Unit Test Project Creator
@REM ******************************************************************************
@REM * FILE NAME: syslib_ProjectCreate.bat
@REM *
@REM * DESCRIPTION: 
@REM *  The script file is used to create the unit test projects of all
@REM *  components under Syslib. These projects will then be available 
@REM *  in the specified workspace.
@REM *
@REM * USAGE:
@REM *  syslib_ProjectCreate.bat [arg1]
@REM *
@REM *  arg1:
@REM *     k2h  -   to build projects for platform k2h
@REM *     k2k  -   to build projects for platform k2k
@REM * Copyright (C) 2012, Texas Instruments, Inc.
@REM *****************************************************************************
@echo OFF

if "%~1" equ "" (
@echo Error: Device Type is not passed as first argument. 
@echo Supported devices are k2h and k2k.
goto end
)
if "%1%" == "k2h" goto ResultTrue
if "%1%" == "k2k" goto ResultTrue (
) else (
@echo Error: Device Type is not k2h or k2k.
goto end
)

:ResultTrue
REM arguments have been validated

set TARGET_NAME=tmdxevm6638lxe


echo.   ************************************************************************
echo.   Detected TARGET_NAME is set to "%TARGET_NAME%"
echo.   To use different TARGET_NAME, call the file with the appropiate argument
echo.   ************************************************************************

REM This is Endianess of the Projects being created.
REM Valid Values are 'little' and 'big'
if "%2" == "big" (
set ENDIAN=big
) else (
set ENDIAN=little
)
echo.   ************************************************************************
echo.   Detected ENDIAN is set to "%ENDIAN%"
echo.   To use different ENDIAN, call the file with the appropiate argument
echo.   ************************************************************************

REM *****************************************************************************
REM * Version Information of the various tools etc required to build the test
REM * projects. Customers are free to modify these to meet their requirements.
REM *****************************************************************************

REM setup environment

if "%1%"=="k2h" (
echo "Select for K2H"
call syslib_setupenv_tmdxevm6638lxe.bat k2h
) 
if "%1%"=="k2k" (
echo "Select for K2K"
call syslib_setupenv_tmdxevm6638lxe.bat k2k
)

call create_macros_file.bat macros_tmdxevm6638lxe.ini

REM Macros file
set MACROS_FILE=macros_tmdxevm6638lxe.ini

REM Version of CG-Tools
set CGT_VERSION=7.4.8

REM Version of XDC
set XDC_VERSION=3.30.04.52

REM Version of BIOS
set BIOS_VERSION=6.40.04.47

REM RTSC Platform Name depending upon the target.
set RTSC_PLATFORM_NAME=ti.runtime.platforms.tmdxevm6638lxe

REM *****************************************************************************
REM *****************************************************************************
REM                 Please do NOT change anything below this
REM *****************************************************************************
REM *****************************************************************************

REM Enable Delayed Expansion
setlocal ENABLEDELAYEDEXPANSION 

REM Batch file execution location
set WORKDIR_SHORT=%~sdp0

REM Set auto create command by default for use with CCSv5
set AUTO_CREATE_COMMAND=eclipse\eclipsec -noSplash

REM This is the format of the executable being created
REM Valid Values are 'ELF' and 'COFF'
set OUTPUT_FORMAT=ELF

REM RTSC Target 
REM - Please ensure that you select this taking into account the
REM   OUTPUT_FORMAT and the RTSC_PLATFORM_NAME 
if "%ENDIAN%" == "big" (
set RTSC_TARGET=ti.targets.elf.C66_big_endian
) else (
set RTSC_TARGET=ti.targets.elf.C66
)

REM Goto the Syslib Installation Path.
pushd %SYSLIB_INSTALL_PATH:/=\%

echo *****************************************************************************
echo Detecting UnitTest Projects in Syslib and importing them in the workspace %CCS_WORKSPACE%_%TARGET_NAME%

set listfile=testpjtlist.txt
if exist %listfile% (del /f %listfile%)
dir /b /s *testProject.txt | findstr "%TARGET_NAME%" >> %listfile% 

for /F %%I IN (%listfile%) do (
echo Detected Test Project: %%~nI

dir /b /s %%I > tmpFile.txt
if "%1%"=="k2h" (
echo "setting xdc options to 1 = K2H"
set XDC_OPTIONS="-DBUILDENV=1"
)
if "%1%"=="k2k" (
echo "setting xdc options to 2 = K2K"
set XDC_OPTIONS="-DBUILDENV=2"
)

REM Goto each directory where the test project file is located and create the projects.
pushd %%~dI%%~pI

if "%1%"=="k2h" (
echo "setting CCS options to K2H "
REM Execute the command to create the project using the parameters specified above.
%CCS_DIR%\%AUTO_CREATE_COMMAND% -data %CCS_WORKSPACE%_%TARGET_NAME% -application com.ti.ccstudio.apps.projectCreate -ccs.name %%~nI_%ENDIAN% -ccs.outputFormat %OUTPUT_FORMAT% -ccs.device com.ti.ccstudio.deviceModel.C6000.GenericC64xPlusDevice -ccs.endianness %ENDIAN% -ccs.kind executable -ccs.cgtVersion %CGT_VERSION% -rtsc.xdcVersion %XDC_VERSION% -rtsc.enableDspBios -rtsc.biosVersion %BIOS_VERSION% -rtsc.buildProfile "debug" -rtsc.platform "%RTSC_PLATFORM_NAME%" -rtsc.target %RTSC_TARGET% -rtsc.products "com.ti.rtsc.SYSBIOS:%BIOS_VERSION%" -rtsc.setConfiguroOptions !XDC_OPTIONS! -ccs.rts libc.a -ccs.args %%~nI%%~xI %SIMULATOR_SUPPORT_DEFINE% -ccs.overwrite full -ccs.defineBuildVariable DEVICE_DEFINE  DEVICE_K2H @type string @scope project 
)

if "%1%"=="k2k" (
echo "setting CCS options to K2K "
REM Execute the command to create the project using the parameters specified above.
%CCS_DIR%\%AUTO_CREATE_COMMAND% -data %CCS_WORKSPACE%_%TARGET_NAME% -application com.ti.ccstudio.apps.projectCreate -ccs.name %%~nI_%ENDIAN% -ccs.outputFormat %OUTPUT_FORMAT% -ccs.device com.ti.ccstudio.deviceModel.C6000.GenericC64xPlusDevice -ccs.endianness %ENDIAN% -ccs.kind executable -ccs.cgtVersion %CGT_VERSION% -rtsc.xdcVersion %XDC_VERSION% -rtsc.enableDspBios -rtsc.biosVersion %BIOS_VERSION% -rtsc.buildProfile "debug" -rtsc.platform "%RTSC_PLATFORM_NAME%" -rtsc.target %RTSC_TARGET% -rtsc.products "com.ti.rtsc.SYSBIOS:%BIOS_VERSION%" -rtsc.setConfiguroOptions !XDC_OPTIONS! -ccs.rts libc.a -ccs.args %%~nI%%~xI %SIMULATOR_SUPPORT_DEFINE% -ccs.overwrite full -ccs.defineBuildVariable DEVICE_DEFINE  DEVICE_K2K @type string @scope project
)

REM Add syslib path into XDC tools
xs -f %WORKDIR_SHORT%fixccspjt.js %CCS_WORKSPACE%_%TARGET_NAME%\%%~nI_%ENDIAN%\.cproject 

REM copy the macros.ini to project location
copy %WORKDIR_SHORT%%MACROS_FILE% %CCS_WORKSPACE%_%TARGET_NAME%\%%~nI_%ENDIAN%\macros.ini

popd
)

del /f %listfile%

popd

:end

