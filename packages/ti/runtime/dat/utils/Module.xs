/******************************************************************************
 * FILE PURPOSE: Package Include Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION:
 *  This file contains the module specification for the Packages Include
 *  Directory.
 *
 * Copyright (C) 2015, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to package the DAT Utils
 **************************************************************************/
function modBuild()
{
    /* Add all the .h files to the release package. */
    var testFiles = libUtility.listAllFiles (".txt", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .conf files to the release package. */
    var testFiles = libUtility.listAllFiles (".xs", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .pdf files to the release package. */
    var testFiles = libUtility.listAllFiles (".bat", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .txt files to the release package. */
    var testFiles = libUtility.listAllFiles (".pl", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .txt files to the release package. */
    var testFiles = libUtility.listAllFiles (".dll", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];
}

