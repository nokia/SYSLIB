#! /bin/bash
#

if [[ -z ${SYSLIB_INSTALL_PATH} ]]; then
    echo "[ERROR] SYSLIB_INSTALL_PATH is not defined!";
    return 1;
fi

source ${SYSLIB_INSTALL_PATH}/scripts/bldFunctions.sh

function usage ()
{
    printErrorMsg "DO THE FOLLOWING STEPS AND TRY AGAIN: This is just an example of the various ENVIRONMENT variables to be configured"
    printErrorMsg "   The INSTALL_JAMMER_INSTALL_PATH is only required to make the release. This is not required to rebuilt"
    printErrorMsg "   the SYSLIB libraries. "
    printErrorMsg "export ARMTOOLS_INSTALL_PATH=/opt/ti/linaro-2013.03"
    printErrorMsg "export ARAGO_INSTALL_PATH=~/myWork/linux-devkit/sysroots/armv7ahf-vfp-neon-oe-linux-gnueabi"
    printErrorMsg "export CGT_INSTALL_PATH=~/ti/cgt_7.4.2"
    printErrorMsg "export XDC_INSTALL_PATH=~/ti/xdctools_3_25_02_70"
    printErrorMsg "export PDK_INSTALL_PATH=~/ti/pdk_keystone2_3_00_02_13/packages"
    printErrorMsg "export SYSLIB_INSTALL_PATH=~/work/k2_dev/syslib"
    printErrorMsg "export SNOW3G_INSTALL_PATH=~/ti/snow3g_1_0_0_2"
    printErrorMsg "export BIOS_INSTALL_PATH=~/ti/bios_6_40_04_47/packages"
    printErrorMsg "export IPC_INSTALL_PATH=~/ti/ipc_3_30_01_12/packages"
    printErrorMsg "export UIA_INSTALL_PATH=~/ti/uia_1_03_00_02/packages"
    printErrorMsg "export CUIA_INSTALL_PATH=~/tools/cuia_1_01_00_03Custom"
    printErrorMsg "export INSTALL_JAMMER_INSTALL_PATH=~/tools/installjammer-1.2.15"
}

# Check if necessary variables are defined
###########################################################
if [ -z $ARMTOOLS_INSTALL_PATH ]; then
        printErrorMsg "ARMTOOLS_INSTALL_PATH not defined. Define the variable and run again"
        usage
        return 1
fi
if [ -z $ARAGO_INSTALL_PATH ]; then
        printErrorMsg "ARAGO_INSTALL_PATH not defined. Define the variable and run again"
        usage
        return 1
fi
if [ -z $CGT_INSTALL_PATH ]; then
        printErrorMsg "CGT_INSTALL_PATH not defined. Define the variable and run again"
        usage
        return 1
fi
if [ -z $XDC_INSTALL_PATH ]; then
        printErrorMsg "XDC_INSTALL_PATH not defined. Define the variable and run again"
        usage
        return 1
fi
if [ -z $PDK_INSTALL_PATH ]; then
        printErrorMsg "PDK_INSTALL_PATH not defined. Define the variable and run again"
        usage
        return 1
fi
if [ -z $SYSLIB_INSTALL_PATH ]; then
        printErrorMsg "SYSLIB_INSTALL_PATH not defined. Define the variable and run again"
        usage
        return 1
fi
if [ -z $SNOW3G_INSTALL_PATH ]; then
        printErrorMsg "SNOW3G_INSTALL_PATH not defined. Define the variable and run again"
        usage
        return 1
fi
if [ -z $CUIA_INSTALL_PATH ]; then
        printErrorMsg "CUIA_INSTALL_PATH not defined. Define the variable and run again"
        usage
        return 1
fi
if [ -z $BIOS_INSTALL_PATH ]; then
        printErrorMsg "BIOS_INSTALL_PATH not defined. Define the variable and run again"
        usage
        return 1
fi
if [ -z $IPC_INSTALL_PATH ]; then
        printErrorMsg "IPC_INSTALL_PATH not defined. Define the variable and run again"
        usage
        return 1
fi
#if [ -z $INSTALL_JAMMER_INSTALL_PATH ]; then
#        printErrorMsg "INSTALL_JAMMER_INSTALL_PATH not defined. Define the variable and run again"
#        usage
#        return 1
#fi


export ARCH=arm
export arch=arm
export CROSS_COMPILE=arm-linux-gnueabihf-
export CC=${CROSS_COMPILE}gcc
export AR=${CROSS_COMPILE}ar
export XDCCGROOT=${CGT_INSTALL_PATH}
export PATH=${ARMTOOLS_INSTALL_PATH}/bin:${CGT_INSTALL_PATH}/bin:${XDC_INSTALL_PATH}:${INSTALL_JAMMER_INSTALL_PATH}:$PATH
export ARAGODIR=${ARAGO_INSTALL_PATH}/usr

# Set the XDCPATH to ensure that all the packages are in the path.
export XDCPATH=
export XDCPATH="${SYSLIB_INSTALL_PATH};${SYSLIB_INSTALL_PATH}/packages;${PDK_INSTALL_PATH};${SNOW3G_INSTALL_PATH};${UIA_INSTALL_PATH};${CUIA_INSTALL_PATH};${BIOS_INSTALL_PATH};${IPC_INSTALL_PATH}"

printInfoMsg  "Build Environment configured"
