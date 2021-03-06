/******************************************************************************
 * FILE PURPOSE: Build Library Utilities
 ******************************************************************************
 * FILE NAME: buildlib.xs
 *
 * DESCRIPTION: 
 *  This file contains common routines that are used by the various SRIO driver 
 *  components.
 *
 * Copyright (C) 2010, Texas Instruments, Inc.
 *****************************************************************************/

/**************************************************************************
 * FUNCTION NAME : createMiniPkg
 **************************************************************************
 * DESCRIPTION   :
 *  The function is responsible for creating the mini tar package
 *  The MINI package has the following files:- 
 *      - Driver Source Files. 
 *      - Header files (exported and internal driver files) 
 *      - Simple Makefiles. 
 **************************************************************************/
function createMiniPkg(pkgName)
{
    /* Get the package Name. */
    var packageRepository = xdc.getPackageRepository(Pkg.name);
    var packageBase       = xdc.getPackageBase(Pkg.name);
    var packageName       = packageBase.substring(packageRepository.length + 1);

    /* Convert the Package name by replacing back slashes with forward slashes. This is required because
     * otherwise with long names the tar is unable to change directory. */
    var newPkgName = new java.lang.String(packageRepository);
    var newPkgRep  = newPkgName.replace('\\', '/');

    /* Step1: Create the MINI Package and add the simple Big and Little Endian Makefiles to the package */
    Pkg.makeEpilogue += "release: mini_pkg\n";
    Pkg.makeEpilogue += "mini_pkg:\n";
    Pkg.makeEpilogue += "\t tar -C " + '"' + newPkgRep + '"' + " -cf packages/" + pkgName + "_mini.tar " + 
                        packageName + "simpleLE.mak " + packageName + "simpleBE.mak \n";
                        

    /* Step2: Add the exported header files to the package */
    var includeFiles = libUtility.listAllFiles (".h", ".", false);
    for (var k = 0 ; k < includeFiles.length; k++)
        Pkg.makeEpilogue += "\t tar -C " + '"' + newPkgRep + '"' + " -rf packages/" + pkgName + "_mini.tar " + 
                        packageName + includeFiles[k] + "\n";

    /* Step3: Add the driver source files to the package; the filter should have generated a source listing */
    Pkg.makeEpilogue += "\t tar -C " + '"' + newPkgRep + '"' + " -T src.lst -rf packages/" + pkgName + "_mini.tar " + "\n";

    /* Ensure that we clean up the mini package */
    Pkg.makeEpilogue += "clean::\n";
    Pkg.makeEpilogue += "\t $(RM) packages/" + pkgName + "_mini.tar\n";
}

/**************************************************************************
 * FUNCTION NAME : listAllFiles
 **************************************************************************
 * DESCRIPTION   :
 *  Utility function which lists all files with a specific extension 
 *  present in a directory and any directory inside it.
 **************************************************************************/
function listAllFiles(ext, dir, recurse)
{	
    var srcFile = [];
    var d;

    /* If recurse parameter is not specified we default to recursive search. */
    if (recurse == null)
        recurse = true;

    if (dir == undefined) 
	    d = ".";
    else 
    	d = dir;

    /* Get access to the current directory. */
    var file = new java.io.File(d);

    /* Check if the file exists and it is a directory. */
    if (file.exists() && file.isDirectory()) 
    {
        /* Get a list of all files in the specific directory. */
        var fileList = file.listFiles();
        for (var i = 0; i < fileList.length; i++) 
        {
            /* Dont add the generated directory 'package' and any of its files 
             * to the list here. */
            if (fileList[i].getName().matches("package") == false)
            {
                /* Check if the detected file is a directory */
                if (fileList[i].isDirectory())
                {
                    /* We will recurse into the subdirectory only if required to do so. */
                    if (recurse == true)
                    {
                        /* Generate the directory Name in which we will recurse. */ 
                        var directoryName = d + "/" + fileList[i].getName();

                        /* Get a list of all files in this directory */
                        var fileListing = listAllFiles (ext, directoryName, recurse);
                        if (fileListing != null)
                        {
                            /* Return a list of all file names in the directory. */
                            for (var j = 0 ; j < fileListing.length; j++) 
                                srcFile[srcFile.length++] = fileListing[j];
                        }
                    }
                }
                else
                {
                    /* This was a file. Check if the file name matches the extension */
                    if (fileList[i].getName().endsWith(ext) == true)
                        srcFile[srcFile.length++] = d + "/" + fileList[i].getName();
                }
            }
        }
        return srcFile;
    }
    return null;
}

/**************************************************************************
 * FUNCTION NAME : buildLibrary
 **************************************************************************
 * DESCRIPTION   :
 *  Utility function which will build a specific library
 **************************************************************************/
function buildLibrary (libOptions, libName, target, libFiles) 
{
    var lldFullLibraryPath = "./lib/" + platformType + "/" + libName;

    /* Create the library file and add all the objects to the file. */
    var lib = Pkg.addLibrary(lldFullLibraryPath, target, libOptions);
    lib.addObjects (libFiles);

    var cgXmlDir = java.lang.System.getenv("CGXML_DIR")
    var cgtDir = java.lang.System.getenv("CGT_DIR")
    if ( cgXmlDir && cgtDir )
    {
        /* Create the Epilogue; which executes after all the builds are completed. 
         * This is used to generate the benchmark information for the built library. 
         * Also add the benchmarking information file to the package. */
        Pkg.makeEpilogue += ".executables: benchmarking_" + lldFullLibraryPath + target.suffix + "\n";
        Pkg.makeEpilogue += "benchmarking_" + lldFullLibraryPath + target.suffix + ":";
        if (build.hostOSName == "Windows")
        {
            cgtDir = cgtDir.replace("\\","/")
            cgXmlDir = cgXmlDir.replace("\\","/")
        }
        Pkg.makeEpilogue += "\n\t " + cgtDir + "/bin/ofd6x -x " + lldFullLibraryPath + ".a" + target.suffix + " > tmp.xml";
        Pkg.makeEpilogue += "\n\t " + cgXmlDir + "/bin/sectti tmp.xml > " + lldFullLibraryPath + ".a" + target.suffix +  "_size.txt";
        Pkg.makeEpilogue += "\n\t $(RM) tmp.xml\n\n";
        Pkg.otherFiles[Pkg.otherFiles.length++] = lldFullLibraryPath + ".a" + target.suffix + "_size.txt";

        /* We need to clean after ourselves; extend the 'clean' target to take care of this. */
        Pkg.makeEpilogue += "clean::\n\t";
        Pkg.makeEpilogue += "$(RM) " + lldFullLibraryPath + ".a" + target.suffix + "_size.txt\n\n";
    }

    return lib;
}

