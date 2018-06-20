@REM ******************************************************************
@REM * FILE PURPOSE: Environment Setup 
@REM ******************************************************************
@REM * FILE NAME: syslib_setupenv.bat
@REM *
@REM * DESCRIPTION: 
@REM *  Configures and sets up the Build Environment for small cell.
@REM *
@REM *
@REM * USAGE:
@REM *  syslib_setupenv.bat [Device Type]
@REM *
@REM *  arg1:
@REM *     Device Type to build projects for platform 
@REM *     valid values are k2h or k2k
@REM * Copyright (C) 2012, Texas Instruments, Inc.
@REM ******************************************************************
@echo off

REM ******************************************************************
REM Tools & Components Configuration.
REM ******************************************************************

REM Set/Get SYSLIB_INSTALL_PATH
REM ============================================================================
REM Check input argument

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

set DEVICE=%1

REM Syslib path not given, derive from working directory
pushd ..
set SYSLIB_INSTALL_PATH="%CD%"
popd
goto argcheckdone

:arggiven
REM Syslib path given exclusively, use it
set SYSLIB_INSTALL_PATH="%1"

:argcheckdone
REM Print message about the Syslib path detected
echo.  ************************************************************************
echo.    Detected SYSLIB_INSTALL_PATH is set to %SYSLIB_INSTALL_PATH%
echo.    To use different path, call the file with the appropiate argument
echo.  ************************************************************************

REM ============================================================================
REM Set CCS Installation Root directory
REM ============================================================================
if exist %CCS_ROOT% goto ccsrootdone
if exist "c:\ti" (
set CCS_ROOT="c:/ti"
) else if exist "C:\Program Files (x86)" (
set CCS_ROOT="C:/Program Files (x86)/Texas Instruments"
) else if exist "C:\Program Files" (
set CCS_ROOT="C:/Program Files/Texas Instruments"
) else (
echo.  ********************************************
echo.    CCS_ROOT is not defined, check the script
echo.  ********************************************
)
:ccsrootdone
REM Print message about the CCS base path detected
echo.  ********************************************
echo.    Detected CCS_ROOT is set to %CCS_ROOT%
echo.  ********************************************
REM ============================================================================

REM Convert the paths to short path without spaces
REM ============================================================================
REM Get XDC utililty path and set to path to use 'path2dos'
for /f "tokens=1* delims=" %%a in ('dir /b %CCS_ROOT:/=\%\xdctools_3_25*') do (
set XDC_UTIL_PATH=%CCS_ROOT:/=\%\%%a\packages\xdc\services\io\release
)
set PATH=%PATH%;%XDC_UTIL_PATH%
set XDC_UTIL_PATH=

REM Covert variables for short path
for /f "tokens=1* delims=" %%a in ('cmd /q/c path2dos %CCS_ROOT%') do (set CCS_ROOT=%%a)
for /f "tokens=1* delims=" %%a in ('cmd /q/c path2dos %SYSLIB_INSTALL_PATH%') do (set SYSLIB_INSTALL_PATH=%%a)
REM ============================================================================

REM ----------------------------------------------------------
REM Component version configuration:
REM  This needs to be modified if the various components 
REM  which are required are located in a different directory 
REM ----------------------------------------------------------

REM Variables primarily used in PATH setup
set CCS_DIR=%CCS_ROOT:/=\%\ccsv5
set CCS_WORKSPACE=%SYSLIB_INSTALL_PATH%\ccs_ws
set CGXML_DIR=%CCS_ROOT:/=\%\cg_xml
set XDC_INSTALL_PATH=%CCS_ROOT:/=\%\xdctools_3_30_04_52

REM Variables primarily used in XDC setup
set CGT_DIR=%CCS_ROOT%/cgt_7.4.4
set BIOS_INSTALL_PATH=%CCS_ROOT%/bios_6_40_04_47/packages
set IPC_INSTALL_PATH=%CCS_ROOT%/ipc_3_30_01_12/packages
set PDK_INSTALL_PATH=%CCS_ROOT%/pdk_keystone2_3_01_02_05/packages
set CPPIDEV_INSTALL_PATH=%CCS_ROOT%/pdk_keystone2_3_01_02_05/packages/ti/drv/cppi/device/%DEVICE%/
set QMSSDEV_INSTALL_PATH=%CCS_ROOT%/pdk_keystone2_3_01_02_05/packages/ti/drv/qmss/device/%DEVICE%/
set UIA_INSTALL_PATH=%CCS_ROOT%/uia_2_00_03_43/packages
set SNOW_INSTALL_PATH=%CCS_ROOT%/snow3g_1_0_0_2/packages
set SYSLIB_INSTALL_PATH=%SYSLIB_INSTALL_PATH:\=/%
set C6X_GEN_INSTALL_PATH=%CGT_DIR%

REM ----------------------------------------------------------
REM Setup the PATH configuration: 
REM  This can be modified if the tools are installed in a
REM  different directory.
REM ----------------------------------------------------------

REM Clear all environment paths first.
set PATH=
set INCLUDE=
set LIB=

REM Test requirements
set PATH=%PATH%;C:\Windows;C:\Windows\system32

REM ----------------------------------------------------------
REM Please do not change anything below this 
REM ----------------------------------------------------------

REM Get the DSS Base Location
for /f "tokens=1* delims=" %%a in ('dir /b %CCS_DIR%\ccs_base*') do (set DSS_ROOT=%CCS_DIR%\%%a)

REM XDC Tools including gmake
set PATH=%PATH%;%XDC_INSTALL_PATH%

REM Set XDC environment variables
set XDCCGROOT=%CGT_DIR%
set XDCBUILDCFG=%SYSLIB_INSTALL_PATH%/config.bld
set XDCARGS=

REM Clear all XDCPATH values first.
set XDCPATH=
REM Set the XDC PATH to point to the location of all the dependent packages
REM - BIOS
REM - IPC
REM - PDK
REM - NDK
REM - SYSLIB
REM - XDAIS
set XDCPATH=%XDCPATH%;%BIOS_INSTALL_PATH%
set XDCPATH=%XDCPATH%;%IPC_INSTALL_PATH%
set XDCPATH=%XDCPATH%;%PDK_INSTALL_PATH%
set XDCPATH=%XDCPATH%;%SYSLIB_INSTALL_PATH%;%SYSLIB_INSTALL_PATH%/packages
set XDCPATH=%XDCPATH%;%XDAIS_INSTALL_PATH%
set XDCPATH=%XDCPATH%;%UIA_INSTALL_PATH%

REM TI Code Generation Tools
set PATH=%PATH%;%CGT_DIR%\bin;

REM Test requirements
set PATH=%PATH%;%CCS_DIR%\eclipse\jre\bin

REM Setup the Execution Environment 
set PATH=%PATH%;%XDC_INSTALL_PATH%\bin
set PATH=%PATH%;%CGXML_DIR%\bin
set PATH=%PATH%;%DSS_ROOT%\DebugServer\packages\ti\dss\java
set PATH=%PATH%;%DSS_ROOT%\DebugServer\bin\win32
set PATH=%PATH%;%DSS_ROOT%\common\bin
set PATH=%PATH%;%DSS_ROOT%\common\uscif
set PATH=%PATH%;%DSS_ROOT%\scripting\bin

@echo -----------------------------------------------
@echo Syslib Build Environment Configured 
@echo -----------------------------------------------

REM Set the Title Window appropiately.
Title Syslib Build Environment

:end
