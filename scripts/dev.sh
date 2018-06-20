#! /bin/bash
#
# The script can be invoked as follows:
#   source ./dev.sh <DEV_NAME> <ARM_BUILD> <DSP_BUILD> <DEMO_BUILD> <ARM_UNIT_TEST> <DSP_UNIT_TEST>
#  Arguments:
#   DEV_NAME=k2x Device for which the builds are to be done
#   ARM_BUILD=1 This will cause the ARM libraries and daemons to get built.
#   DSP_BUILD=1 This will cause the DSP libraries to get built and the release
#   package to be generated.
#   DEMO_BUILD=1 This will cause the LTE Demo application to be built
#   ARM_TEST=1 This will build all the ARM Unit tests
#   DSP_TEST=1 This will build all the DSP Unit tests
#
# Developers can use the above options to rebuild only the DSP or ARM to reduce
# build times
#
# Releases should get built with all the arguments set to 1.
#
# Prerequiste: Ensure that the BUILD environment has been setup properly before calling
# the script.
# #####################################################################################

source ${SYSLIB_INSTALL_PATH}/scripts/bldFunctions.sh

function devusage ()
{
    printErrorMsg "ERROR: Usage of the script is invalid"
    printErrorMsg "source ./dev.sh <DEV_NAME> <ARM_BUILD> <DSP_BUILD> <DEMO_BUILD> <ARM_UNIT_TEST> <DSP_UNIT_TEST>"
}

########################################################################################
# Sanity Check: Validate the arguments to the release script
########################################################################################
if [ $# -ne 6 ]; then
    devusage
    return 1
fi

# Initialize the build variables.
BUILD_DEVICE=$1
BUILD_ARM=$2
BUILD_DSP=$3
BUILD_DEMO=$4
BUILD_ARM_UNIT=$5
BUILD_DSP_UNIT=$6

########################################################################################
# Build the SYSLIB ARM package for all the devices.
########################################################################################
if [ $BUILD_ARM -eq 1 ]; then
    buildARM $1
fi

########################################################################################
# Build the SYSLIB DSP package for all the devices.
# The SYSLIB top level DSP config.bld has a list of all the devices which are supported
########################################################################################
if [ $BUILD_DSP -eq 1 ]; then
    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg  "Building the SYSLIB libraries for DSP"
    xdc clean -PD .
    xdc release -PD .
    printInfoMsg "DSP Builds completed"
    popd
fi

########################################################################################
# Build the LTE Demo:
########################################################################################
if [ $BUILD_DEMO -eq 1 ]; then
    buildDemo $1
fi

########################################################################################
# Build the ARM Unit Tests
########################################################################################
if [ $BUILD_ARM_UNIT -eq 1 ]; then
    buildARMUnitTest $1
fi

########################################################################################
# Build the DSP Unit Tests
########################################################################################
if [ $BUILD_DSP_UNIT -eq 1 ]; then
    buildDSPUnitTest $1
fi

