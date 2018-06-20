/******************************************************************************
 * FILE PURPOSE: Source Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION:
 *  This file contains the source module specification.
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/* List of all the Source Files */
var dpmLibFiles = [
    "src/dpm_report.c",
    "src/listlib.c",
    "src/sch_mergesort.c",
    "src/sch_quickselectTopK.c"
];

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
            libUtility.buildLibrary (Device[device], libOptions, "ti.runtime.dpm", Build.targets[targets], dpmLibFiles);
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

    /* Add the DPM Source file which is packaged but is not built with the library. */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "src/dpm.c";
}

