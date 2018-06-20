/******************************************************************************
 * FILE PURPOSE: Source Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION:
 *  This file contains the module specification for the resource management.
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build the packet library and add all the source
 *  files and header files to the release package which are required
 *  to build it.
 **************************************************************************/
function modBuild()
{
    /* Cycle through all the supported devices and build the library for all the
     * supported devices */
    for (var device=0; device < Device.length; device++)
    {
        /* List of all the Resource Management Files */
        var resMgrLibFiles = [
            "src/resmgr.c",
            "src/resmgr_shmClient.c",
        ];

        /* Build the libraries for all the targets specified. */
        for (var targets=0; targets < Build.targets.length; targets++)
        {
            if (Device[device] == "k2h")
            {
                var libOptions = {
                    copts: "-DDEVICE_K2H",
                };
                resMgrLibFiles[resMgrLibFiles.length++] = "src/resmgr_k2hk.c";
            }
            else if (Device[device] == "k2k")
            {
                var libOptions = {
                    copts: "-DDEVICE_K2K",
                };
                resMgrLibFiles[resMgrLibFiles.length++] = "src/resmgr_k2hk.c";
            }
            else if (Device[device] == "k2l")
            {
                var libOptions = {
                    copts: "-DDEVICE_K2L",
                };
                resMgrLibFiles[resMgrLibFiles.length++] = "src/resmgr_k2l.c";
            }
            else
            {
                printf ("Error: Unsupported Device " + Device[device]);
            }
            libUtility.buildLibrary (Device[device], libOptions, "ti.runtime.resmgr", Build.targets[targets], resMgrLibFiles);
        }
    }

    /* Add all the .c files to the release package. */
    var testFiles = libUtility.listAllFiles (".c", "src");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .h files to the release package. */
    var testFiles = libUtility.listAllFiles (".h", "src");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];
}

