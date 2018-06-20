/******************************************************************************
 * FILE PURPOSE: Package specification file
 ******************************************************************************
 * FILE NAME: package.xdc
 *
 * DESCRIPTION:
 *  This file contains the package specification for the SYSLIB which includes
 *  information about all the sub-modules
 *
 * Copyright (C) 2013-2014 Texas Instruments, Inc.
 *****************************************************************************/

requires ti.runtime.dpm;
requires ti.runtime.msgcom;
requires ti.runtime.resmgr;
requires ti.runtime.josh;
requires ti.runtime.platforms.tmdxevm6638lxe;
requires ti.runtime.platforms.k2l;
requires ti.runtime.dat;
requires ti.runtime.fapi_tracing;
requires ti.runtime.pktlib;
requires ti.runtime.name;
requires ti.runtime.domain;
requires ti.runtime.netfp;
requires ti.runtime.memlog;
requires ti.runtime.root;
requires ti.runtime.domain;
requires ti.runtime.uintc;
requires ti.runtime.common;
requires ti.demo.lte;
requires ti.apps.soc_init;
requires ti.apps.name_proxy;
requires ti.apps.netfp_master;
requires ti.apps.resmgr_server;
requires ti.apps.netfp_server;
requires ti.apps.netfp_proxy;
requires ti.apps.dat_server;

package syslib[4, 00, 05, 00] {
}
