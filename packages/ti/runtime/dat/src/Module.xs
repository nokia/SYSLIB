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

/* List of all the Source Files */
var datSrcFiles = [
                      "src/dat_client.c",
                      "src/dat_srv.c",
                      "src/dat_log.c",
                      "src/dat_listlib.c",
                    ];

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
    /* Cycle through all the supported devices and build the library for all the
     * supported devices */
    for (var device=0; device < Device.length; device++)
    {
        /* Build the libraries for all the targets specified. */
        for (var targets=0; targets < Build.targets.length; targets++)
        {
            if (Device[device] == "k2h")
            {
                var libOptions = {
                    copts: "-DDEVICE_K2H",
                };
            }
            if (Device[device] == "k2k")
            {
                var libOptions = {
                    copts: "-DDEVICE_K2K",
                };
            }
            if (Device[device] == "k2l")
            {
                var libOptions = {
                    copts: "-DDEVICE_K2L",
                };
            }
            libUtility.buildLibrary (Device[device], libOptions, "ti.runtime.dat", Build.targets[targets], datSrcFiles);
        }
    }

    /* Add all the .c files to the release package. */
    var srcFiles = libUtility.listAllFiles (".c", "src");
    for (var k = 0 ; k < srcFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = srcFiles[k];

    /* Add all the .h files to the release package. */
    var srcFiles = libUtility.listAllFiles (".h", "src");
    for (var k = 0 ; k < srcFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = srcFiles[k];
}

