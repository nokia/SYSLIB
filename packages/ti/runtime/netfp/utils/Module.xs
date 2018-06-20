/******************************************************************************
 * FILE PURPOSE: Package Include Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION:
 *  This file contains the module specification for the Packages Include
 *  Directory.
 *
 * Copyright (C) 2011, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to package the NETFP Utils
 **************************************************************************/
function modBuild()
{
    /* Add all the .h files to the release package. */
    var testFiles = libUtility.listAllFiles (".c", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .conf files to the release package. */
    var testFiles = libUtility.listAllFiles (".conf", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .pdf files to the release package. */
    var testFiles = libUtility.listAllFiles (".pdf", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .txt files to the release package. */
    var testFiles = libUtility.listAllFiles (".txt", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .txt files to the release package. */
    var testFiles = libUtility.listAllFiles (".cfg", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .txt files to the release package. */
    var testFiles = libUtility.listAllFiles (".cmd", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .der files to the release package. */
    var testFiles = libUtility.listAllFiles (".der", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .secrets files to the release package. */
    var testFiles = libUtility.listAllFiles (".secrets", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .secrets files to the release package. */
    var testFiles = libUtility.listAllFiles ("do_setup", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .sh files to the release package. */
    var testFiles = libUtility.listAllFiles (".sh", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .py files to the release package. */
    var testFiles = libUtility.listAllFiles (".py", "utils");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];
}

