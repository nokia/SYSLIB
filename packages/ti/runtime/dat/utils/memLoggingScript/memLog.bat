@REM This script file is used to convert DAT memory dump to a bin file for System Anaylyzer
@REM It converts all the files in the given file list and combine them into one *.bin file
@REM Syntax: memLog.bat <file contains files to be converted>
@REM         memLog.bat filelist

@echo off

echo     Convert UIA log to SA bin file, syntax:
echo         "memlog.bat <file that contains filename for the files to be converted> "
echo

@REM clean up *.bin file in
del *.bin

@REM Read file list from a giIven file
echo "Reading file from %1"
for /f "delims=" %%a in (%1) do  (
    echo "input file %%a"

    @REM run perl script to convert UIA memory dump to bin file
    echo "Calling perl script for memdump to bin"
    perl mem2bin.pl %%a %%a.bin
)

@REM put all bin files into one
copy /b *.bin %2


