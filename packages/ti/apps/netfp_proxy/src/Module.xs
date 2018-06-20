/******************************************************************************
 * FILE PURPOSE: Source Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION:
 *  This file contains the source module specification.
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build the package and add all the source
 *  files and header files to the release package which are required
 *  to build it.
 **************************************************************************/
function modBuild()
{
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/core.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/ipsec.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/netmgr.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/route.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/iface.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/osal.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/netfp_proxy_plugin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/listlib.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/neigh.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/neigh2.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/arp.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/ndisc.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/test.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/pmtu4.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/pmtu6.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/ipsecmgr_snoop.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/ipsecmgr_snoop_knl.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/ipsecmgr_snoop_sm.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/ipsecmgr_snoop_user_ipc.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/ipsecmgr_snoop_fp.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/ipsecmgr_snoop_mq.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/ipsecmgr_snoop_timer.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/ipsecmgr_snoop_xfrm.c";
}

