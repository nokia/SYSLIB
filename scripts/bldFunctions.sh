#! /bin/bash
#
# The script is invoked from the development & release scripts. Standard build function
# which are invoked from the development/release scripts.
#
# Prerequiste: Ensure that the BUILD environment has been setup properly before calling
# the script.
# #####################################################################################

if [[ -z ${MAKECMD} ]]; then
    MAKECMD=make
fi

########################################################################################
# Utility function which is used to print an error message on the build console
########################################################################################
function printErrorMsg ()
{
  echo -e "\033[31m"${1}${2}
  echo -e -n "\033[0m"
}

########################################################################################
# Utility function which is used to print a successful message on the build console
########################################################################################
function printSuccessMsg ()
{
  echo -e "\033[32m"${1}${2}
  echo -e -n "\033[0m"
}

########################################################################################
# Utility function which is used to print an informational message on the build console
########################################################################################
function printInfoMsg ()
{
  echo -e "\033[33m"${1}${2}
  echo -e -n "\033[0m"
}

########################################################################################
# The function is used to display the proper build environment which is required to
# build the SYSLIB modules
########################################################################################
function envUsageError ()
{
    printErrorMsg "ERROR:Please ensure that the setupenv shell script is executed and the following are exported"
    printErrorMsg " - ARMTOOLS_INSTALL_PATH"
    printErrorMsg " - ARAGO_INSTALL_PATH"
    printErrorMsg " - CGT_INSTALL_PATH"
    printErrorMsg " - XDC_INSTALL_PATH"
    printErrorMsg " - PDK_INSTALL_PATH"
    printErrorMsg " - SYSLIB_INSTALL_PATH"
    printErrorMsg " - SNOW3G_INSTALL_PATH"
    printErrorMsg " - SYSLIB_DEVICE"
}

########################################################################################
# The function is used to build all the SYSLIB ARM components for a specific device.
# These include libraries and the SYSLIB Servers.
########################################################################################
function buildARM ()
{
    export SYSLIB_DEVICE=${1}

    printInfoMsg "**********************************************************"
    printInfoMsg "Building the ARM SYSLIB components ${1}"
    printInfoMsg "**********************************************************"

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building RESMGR"
    cd ti/runtime/resmgr
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} libClean
    fi
    ${MAKECMD} lib
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building PKTLIB"
    cd ti/runtime/pktlib
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} libClean
    fi
    ${MAKECMD} lib
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building MSGCOM"
    cd ti/runtime/msgcom
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} libClean
    fi
    ${MAKECMD} lib
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building NETFP"
    cd ti/runtime/netfp
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} libClean
    fi
    ${MAKECMD} lib
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building JOSH"
    cd ti/runtime/josh
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} libClean
    fi
    ${MAKECMD} lib
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building Name"
    cd ti/runtime/name
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} libClean
    fi
    ${MAKECMD} lib
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building UINTC"
    cd ti/runtime/uintc
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} libClean
    fi
    ${MAKECMD} lib
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building DAT"
    cd ti/runtime/dat
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} libClean
    fi
    ${MAKECMD} lib
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building MEMLOG"
    cd ti/runtime/memlog
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} libClean
    fi
    ${MAKECMD} lib
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building ROOT"
    cd ti/runtime/root
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} clean
    fi
    ${MAKECMD} all
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building DOMAIN"
    cd ti/runtime/domain
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} clean
    fi
    ${MAKECMD} all
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building FAPI Tracing"
    cd ti/runtime/fapi_tracing
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} clean
    fi
    ${MAKECMD} all
    popd

    printInfoMsg "Building SYSLIB ARM Daemons"

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building RESMGR Server Daemon"
    cd ti/apps/resmgr_server
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} clean
    fi
    ${MAKECMD} all
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building Name Proxy Daemon"
    cd ti/apps/name_proxy
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} clean
    fi
    ${MAKECMD} all
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building DAT Server Daemon"
    cd ti/apps/dat_server
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} clean
    fi
    ${MAKECMD} all
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building NETFP Proxy"
    cd ti/apps/netfp_proxy
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} clean
    fi
    ${MAKECMD} all
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building NETFP Server Daemon"
    cd ti/apps/netfp_server
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} clean
    fi
    ${MAKECMD} all
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building NETFP Master Daemon"
    cd ti/apps/netfp_master
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} clean
    fi
    ${MAKECMD} all
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building SYSLIB SOC Init"
    cd ti/apps/soc_init
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} clean
    fi
    ${MAKECMD} all
    popd

    # Switch to the top level directory
    pushd $SYSLIB_INSTALL_PATH

    #
    # Validating the SYSLIB applications:
    #
    printInfoMsg "Validating daemons"
    SYSLIB_DAEMONS="ti/apps/resmgr_server/resmgr_server_$SYSLIB_DEVICE.out
    ti/apps/name_proxy/name_proxy_$SYSLIB_DEVICE.out
    ti/apps/netfp_proxy/netfpproxy_$SYSLIB_DEVICE.out
    ti/apps/dat_server/dat_server_$SYSLIB_DEVICE.out
    ti/apps/netfp_master/netfp_master_$SYSLIB_DEVICE.out
    ti/apps/soc_init/soc_init_$SYSLIB_DEVICE.out
    ti/apps/netfp_server/netfp_server_$SYSLIB_DEVICE.out"
    for file in $SYSLIB_DAEMONS
    do
      if [ ! -e "$file" ]       # Check if file exists.
      then
        printErrorMsg "$file"  "... [Failed]"
        return 1
       fi
       printSuccessMsg "$file" "... [Successful]"
    done

    #
    # Even though the NETFP Proxy Plugin and command shell are technically
    # sample applications; these are still used heavily during system integration
    # so here we package these applications.
    #
    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building NETFP Proxy default plugin"
    cd ti/apps/netfp_proxy
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} pluginClean
    fi
    ${MAKECMD} plugin
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building NETFP Proxy command shell"
    cd ti/apps/netfp_proxy
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} armTestClean
    fi
    ${MAKECMD} armTest
    popd

    #
    # Sanity Check: Ensure that the ARM applications are built
    #
    SYSLIB_ARM_UNIT_TEST="ti/apps/netfp_proxy/test/cmd_shell_app/cmd_shell_$SYSLIB_DEVICE.out
    ti/apps/netfp_proxy/test/netfp_proxy_plugin/netfpproxy_plugin_$SYSLIB_DEVICE.so"
    for file in $SYSLIB_ARM_UNIT_TEST
    do
      if [ ! -e "$file" ]       # Check if file exists.
      then
        printErrorMsg "$file"  "... [Failed]"
        return 1
       fi
       printSuccessMsg "$file" "... [Successful]"
    done

    #
    # The Named resource manager is a very useful tool for debugging the contents
    # of the database on ARM. So we package this as a part of the standard release.
    #
    printInfoMsg "Building SYSLIB Utilities"
    pushd $SYSLIB_INSTALL_PATH
    cd ti/runtime/name
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} utilsClean
    fi
    ${MAKECMD} utils
    popd

    #
    # Validating the SYSLIB Utilities
    #
    printInfoMsg "Validating SYSLIB Utilities"
    SYSLIB_UTILS="ti/runtime/name/utils/nr_mgr_$SYSLIB_DEVICE.out"
    for file in $SYSLIB_UTILS
    do
      if [ ! -e "$file" ]       # Check if file exists.
      then
        printErrorMsg "$file"  "... [Failed]"
        return 1
       fi
       printSuccessMsg "$file" "... [Successful]"
    done

    popd
    printInfoMsg "ARM Builds completed."
}

########################################################################################
# The function is used to build the LTE Demo application for a specific device. This
# will include building the Master application.
########################################################################################
function buildDemo()
{
    printInfoMsg "**********************************************************"
    printInfoMsg "Building the DEMO application for ${1}"
    printInfoMsg "**********************************************************"

    export SYSLIB_DEVICE=${1}
    pushd $SYSLIB_INSTALL_PATH
    cd ti/demo/lte
    ${MAKECMD} all
    popd

    pushd $SYSLIB_INSTALL_PATH

    if [ "$SYSLIB_DEVICE" == "k2l" ]
    then
        SYSLIB_DEMO_OUTS="ti/demo/lte/demo_master_$SYSLIB_DEVICE.out
        ti/demo/lte/rel10-d1/sim_phy/core0_configPkg_$SYSLIB_DEVICE/rel10D1_core0.out
        ti/demo/lte/rel10-d1/sim_phy/core1_configPkg_$SYSLIB_DEVICE/rel10D1_core1.out
        ti/demo/lte/rel10-d1/sim_phy/core2_configPkg_$SYSLIB_DEVICE/rel10D1_core2.out
        ti/demo/lte/rel10-d1/sim_phy/core3_configPkg_$SYSLIB_DEVICE/rel10D1_core3.out
        ti/demo/lte/demo_rel10d1_l2_$SYSLIB_DEVICE.out"
    fi
    if [ "$SYSLIB_DEVICE" == "k2h" ]
    then
        SYSLIB_DEMO_OUTS="ti/demo/lte/demo_master_$SYSLIB_DEVICE.out
        ti/demo/lte/rel10-d1/sim_phy/core0_configPkg_$SYSLIB_DEVICE/rel10D1_core0.out
        ti/demo/lte/rel10-d1/sim_phy/core1_configPkg_$SYSLIB_DEVICE/rel10D1_core1.out
        ti/demo/lte/rel10-d1/sim_phy/core2_configPkg_$SYSLIB_DEVICE/rel10D1_core2.out
        ti/demo/lte/rel10-d1/sim_phy/core3_configPkg_$SYSLIB_DEVICE/rel10D1_core3.out
        ti/demo/lte/demo_rel10d1_l2_$SYSLIB_DEVICE.out
        ti/demo/lte/rel9/l2/core0_configPkg_$SYSLIB_DEVICE/rel9_core0.out
        ti/demo/lte/rel9/l2/core3_configPkg_$SYSLIB_DEVICE/rel9_core3.out
        ti/demo/lte/rel9/l2/core4_configPkg_$SYSLIB_DEVICE/rel9_core4.out
        ti/demo/lte/rel9/l2/core7_configPkg_$SYSLIB_DEVICE/rel9_core7.out
        ti/demo/lte/rel9/sim_phy/core1_configPkg_$SYSLIB_DEVICE/rel9_core1.out
        ti/demo/lte/rel9/sim_phy/core2_configPkg_$SYSLIB_DEVICE/rel9_core2.out
        ti/demo/lte/rel9/sim_phy/core5_configPkg_$SYSLIB_DEVICE/rel9_core5.out
        ti/demo/lte/rel9/sim_phy/core6_configPkg_$SYSLIB_DEVICE/rel9_core6.out
        ti/demo/lte/rel10-d2/l2/core0_configPkg_$SYSLIB_DEVICE/rel10D2_core0.out
        ti/demo/lte/rel10-d2/l2/core3_configPkg_$SYSLIB_DEVICE/rel10D2_core3.out
        ti/demo/lte/rel10-d2/l2/core4_configPkg_$SYSLIB_DEVICE/rel10D2_core4.out
        ti/demo/lte/rel10-d2/l2/core7_configPkg_$SYSLIB_DEVICE/rel10D2_core7.out
        ti/demo/lte/rel10-d2/sim_phy/core1_configPkg_$SYSLIB_DEVICE/rel10D2_core1.out
        ti/demo/lte/rel10-d2/sim_phy/core2_configPkg_$SYSLIB_DEVICE/rel10D2_core2.out
        ti/demo/lte/rel10-d2/sim_phy/core5_configPkg_$SYSLIB_DEVICE/rel10D2_core5.out
        ti/demo/lte/rel10-d2/sim_phy/core6_configPkg_$SYSLIB_DEVICE/rel10D2_core6.out"
    fi

    for file in $SYSLIB_DEMO_OUTS
    do
      if [ ! -e "$file" ]       # Check if file exists.
      then
        printErrorMsg "$file"  "... [Failed]"
        return 1
       fi
       printSuccessMsg "$file" "... [Successful]"
    done
    popd
}

########################################################################################
# The function is used to build the SYSLIB ARM unit test applications for all the
# modules
########################################################################################
function buildARMUnitTest ()
{
    export SYSLIB_DEVICE=${1}

    printInfoMsg "**********************************************************"
    printInfoMsg "Building ARM Unit Test Applications for ${1}"
    printInfoMsg "**********************************************************"

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building PKTLIB Test"
    cd ti/runtime/pktlib
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} armTestClean
    fi
    ${MAKECMD} armTest
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building MSGCOM Test"
    cd ti/runtime/msgcom
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} armTestClean dspArmClean stressClean
    fi
    ${MAKECMD} armTest dspArm stress
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building NETFP Test"
    cd ti/runtime/netfp
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} armTestClean dspArmClean
    fi
    ${MAKECMD} armTest dspArm
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building Name Test"
    cd ti/runtime/name
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} armTestClean dspArmClean
    fi
    ${MAKECMD} armTest dspArm
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building UINTC Test"
    cd ti/runtime/uintc
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} armTestClean
    fi
    ${MAKECMD} armTest
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building DAT Test"
    cd ti/runtime/dat
    if [[ ${FORCE_CLEAN} -eq 1 ]]; then
        ${MAKECMD} dspArmClean
    fi
    ${MAKECMD} dspArm
    popd

    # Move to the top level directory
    pushd $SYSLIB_INSTALL_PATH

    # Validating the unit tests. This is done to make sure that there are no build
    # errors while making the release.
    printInfoMsg "Validating ARM unit tests"

    SYSLIB_ARM_UNIT_TEST="ti/runtime/name/test/arm/test_name_$SYSLIB_DEVICE.out
    ti/runtime/name/test/dsp_arm/test_dsp_arm_name_$SYSLIB_DEVICE.out
    ti/apps/netfp_master/test/test_netfp_master_$SYSLIB_DEVICE.out
    ti/runtime/dat/test/arm_dsp_consumer/test_dat_$SYSLIB_DEVICE.out
    ti/runtime/msgcom/test/arm/test_msgcom_$SYSLIB_DEVICE.out
    ti/runtime/msgcom/test/dsp_arm/test_dsp_arm_msgcom_$SYSLIB_DEVICE.out
    ti/runtime/msgcom/test/stress/test_dsp_arm_stress_msgcom_$SYSLIB_DEVICE.out
    ti/runtime/netfp/test/arm/test_netfp_$SYSLIB_DEVICE.out
    ti/runtime/netfp/test/dsp-arm/test_dsp_arm_netfp_$SYSLIB_DEVICE.out
    ti/runtime/uintc/test/test_uintc_$SYSLIB_DEVICE.out
    ti/runtime/pktlib/test/arm/test_pktlib_$SYSLIB_DEVICE.out"

    for file in $SYSLIB_ARM_UNIT_TEST
    do
      if [ ! -e "$file" ]       # Check if file exists.
      then
        printErrorMsg "$file"  "... [Failed]"
        return 1
       fi
       printSuccessMsg "$file" "... [Successful]"
    done
    popd
}

########################################################################################
# The function is used to build the SYSLIB DSP unit test applications for all the
# modules
########################################################################################
function buildDSPUnitTest ()
{
    export SYSLIB_DEVICE=${1}

    printInfoMsg "**********************************************************"
    printInfoMsg "Building DSP Test Applications ${1}"
    printInfoMsg "**********************************************************"

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building PKTLIB Test"
    cd ti/runtime/pktlib
    ${MAKECMD} dspCore0 dspCore1
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building MSGCOM Test"
    cd ti/runtime/msgcom
    ${MAKECMD} dspCore0 dspCore1 stressCore0 stressCore1 virtualCore0 virtualCore1
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building JOSH Test"
    cd ti/runtime/josh
    ${MAKECMD} dspCore0 dspCore1
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building NETFP Test"
    cd ti/runtime/netfp
    ${MAKECMD} dspCore0 dspCore1 dspArmCore0 dspArmCore1
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building Name Test"
    cd ti/runtime/name
    ${MAKECMD} dspCore0 dspCore1 dspArmCore0 dspArmCore1
    popd

    pushd $SYSLIB_INSTALL_PATH
    printInfoMsg "Building DAT Test"
    cd ti/runtime/dat
    ${MAKECMD} dspArmCore0 dspArmCore1 dspArmMemlogCore0 dspArmMemlogCore1 dspCore0 dspCore1
    popd

    # Move to the top level directory
    pushd $SYSLIB_INSTALL_PATH

    # Validating the unit tests. This is done to make sure that there are no build
    # errors while making the release.
    printInfoMsg "Validating DSP unit tests"
    ARMDLSUFFIX=""
    if [[ ${ARM_DSP_DOWNLOAD} -eq 1 ]]; then
        ARMDLSUFFIX="_armdl"
    fi

    SYSLIB_DSP_UNIT_TEST="ti/runtime/pktlib/test/dsp/core0_configPkg_$SYSLIB_DEVICE/pktlib_dsp_core0${ARMDLSUFFIX}.out
        ti/runtime/pktlib/test/dsp/core1_configPkg_$SYSLIB_DEVICE/pktlib_dsp_core1${ARMDLSUFFIX}.out
        ti/runtime/msgcom/test/dsp/core0_configPkg_$SYSLIB_DEVICE/msgcom_dsp_core0${ARMDLSUFFIX}.out
        ti/runtime/msgcom/test/dsp/core1_configPkg_$SYSLIB_DEVICE/msgcom_dsp_core1${ARMDLSUFFIX}.out
        ti/runtime/msgcom/test/stress/core0_configPkg_$SYSLIB_DEVICE/msgcom_stress_core0${ARMDLSUFFIX}.out
        ti/runtime/msgcom/test/stress/core1_configPkg_$SYSLIB_DEVICE/msgcom_stress_core1${ARMDLSUFFIX}.out
        ti/runtime/msgcom/test/virtual_queue/core0_configPkg_$SYSLIB_DEVICE/msgcom_virtual_core0${ARMDLSUFFIX}.out
        ti/runtime/msgcom/test/virtual_queue/core1_configPkg_$SYSLIB_DEVICE/msgcom_virtual_core1${ARMDLSUFFIX}.out
        ti/runtime/josh/test/core0_configPkg_$SYSLIB_DEVICE/josh_dsp_core0.out
        ti/runtime/josh/test/core1_configPkg_$SYSLIB_DEVICE/josh_dsp_core1.out
        ti/runtime/netfp/test/dsp/core0_configPkg_$SYSLIB_DEVICE/netfp_dsp_core0.out
        ti/runtime/netfp/test/dsp/core1_configPkg_$SYSLIB_DEVICE/netfp_dsp_core1.out
        ti/runtime/netfp/test/dsp-arm/core0_configPkg_$SYSLIB_DEVICE/netfp_dsp_arm_core0${ARMDLSUFFIX}.out
        ti/runtime/netfp/test/dsp-arm/core1_configPkg_$SYSLIB_DEVICE/netfp_dsp_arm_core1${ARMDLSUFFIX}.out
        ti/runtime/name/test/dsp/core0_configPkg_$SYSLIB_DEVICE/name_dsp_core0.out
        ti/runtime/name/test/dsp/core1_configPkg_$SYSLIB_DEVICE/name_dsp_core1.out
        ti/runtime/name/test/dsp_arm/core0_configPkg_$SYSLIB_DEVICE/name_dsp_arm_core0.out
        ti/runtime/name/test/dsp_arm/core1_configPkg_$SYSLIB_DEVICE/name_dsp_arm_core1.out
        ti/runtime/dat/test/arm_dsp_consumer/core0_configPkg_$SYSLIB_DEVICE/dat_arm_dsp_consumer_core0.out
        ti/runtime/dat/test/arm_dsp_consumer/core1_configPkg_$SYSLIB_DEVICE/dat_arm_dsp_consumer_core1.out
        ti/runtime/dat/test/dsp_consumer/core0_configPkg_$SYSLIB_DEVICE/dat_dsp_consumer_core0.out
        ti/runtime/dat/test/dsp_consumer/core1_configPkg_$SYSLIB_DEVICE/dat_dsp_consumer_core1.out
        ti/runtime/dat/test/arm_dsp_memLogging/core0_configPkg_$SYSLIB_DEVICE/dat_arm_dsp_memLogging_core0.out
        ti/runtime/dat/test/arm_dsp_memLogging/core1_configPkg_$SYSLIB_DEVICE/dat_arm_dsp_memLogging_core1.out"

    for file in $SYSLIB_DSP_UNIT_TEST
    do
      if [ ! -e "$file" ]       # Check if file exists.
      then
        printErrorMsg "$file"  "... [Failed]"
        return 1
       fi
       printSuccessMsg "$file" "... [Successful]"
    done
    popd
}

########################################################################################
# Sanity Check: Check if the necessary arguments are defined.
########################################################################################
if [ -z $ARMTOOLS_INSTALL_PATH ]; then
        printErrorMsg "ARMTOOLS_INSTALL_PATH not defined. Define the variable and run again"
        envUsageError
        return 1
fi
if [ -z $ARAGO_INSTALL_PATH ]; then
        printErrorMsg "ARAGO_INSTALL_PATH not defined. Define the variable and run again"
        envUsageError
        return 1
fi
if [ -z $CGT_INSTALL_PATH ]; then
        printErrorMsg "CGT_INSTALL_PATH not defined. Define the variable and run again"
        envUsageError
        return 1
fi
if [ -z $XDC_INSTALL_PATH ]; then
        printErrorMsg "XDC_INSTALL_PATH not defined. Define the variable and run again"
        envUsageError
        return 1
fi
if [ -z $PDK_INSTALL_PATH ]; then
        printErrorMsg "PDK_INSTALL_PATH not defined. Define the variable and run again"
        envUsageError
        return 1
fi
if [ -z $SNOW3G_INSTALL_PATH ]; then
        printErrorMsg "SNOW3G_INSTALL_PATH not defined. Define the variable and run again"
        envUsageError
        return 1
fi
