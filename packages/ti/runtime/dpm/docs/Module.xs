/******************************************************************************
 * FILE PURPOSE: Resource Manager DOCS Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the Resource Manager
 *  Documentation.
 *
 * Copyright (C) 2010, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build the documentation and add it to the package.
 **************************************************************************/
function modBuild() 
{
    /* Create the actual PROLOGUE Section for the Documentation.*/
    Pkg.makePrologue += "release: dpm_document_generation\n";
    Pkg.makePrologue += "dpm_document_generation:\n";
    Pkg.makePrologue += "\t @echo ----------------------------\n";
    Pkg.makePrologue += "\t @echo Generating DPM Documentation\n";
    Pkg.makePrologue += "\t doxygen docs/Doxyfile\n";
    Pkg.makePrologue += "\t @echo DPM Documentation Generated \n";
    Pkg.makePrologue += "\t @echo ----------------------------\n";

    /* Add the documentation file to the package. */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/tifooter.htm";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/tiheader.htm";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/tilogo.gif";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/titagline.gif";

	/* Add the HTML documentation to the package */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/doxygen";
}

