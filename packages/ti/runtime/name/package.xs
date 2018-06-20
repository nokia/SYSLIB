/*
 *  ======== package.xs ========
 *
 */


/*
 *  ======== Package.getLibs ========
 *  This function is called when a program's configuration files are
 *  being generated and it returns the name of a library appropriate
 *  for the program's configuration.
 */

function getLibs(prog)
{
    var lib = "";
    var suffix;

    /* find a compatible suffix */
    if ("findSuffix" in prog.build.target) {
        suffix = prog.build.target.findSuffix(this);
    }
    else {
        suffix = prog.build.target.suffix;
    }

    var name = this.$name + ".a" + suffix;

    if (this.Settings.deviceType == "")
        throw Error("Device not selected for module " + this.$name);

    /* Pick up the appropriate library for the device */
    lib = lib + "lib/" + this.Settings.deviceType + "/" + name;

    if (java.io.File(this.packageBase + lib).exists()) {
        print ("\tLinking with library: " + lib);
        return lib;
    }

    /* could not find any library, throw exception */
    throw Error("Device " + this.Settings.deviceType + " Library not found: " + name);
}

/*
 *  ======== package.close ========
 */
function close()
{
    if (xdc.om.$name != 'cfg') {
        return;
    }
}


