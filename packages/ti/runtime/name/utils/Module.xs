/******************************************************************************
 * FILE PURPOSE: Package Include Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the Packages utils directory.
 *
 * Copyright (C) 2011, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to add all the internal header files of the DAT 
 *  driver to the package.
 **************************************************************************/
function modBuild() 
{
    /* Add all the source files to the release package. */
    var testFiles = libUtility.listAllFiles (".c", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Package the daemon executable(s) for all the supported devices */
    for (var device=0; device < Device.length; device++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = "utils/nr_mgr_" + Device[device] + ".out";
}

