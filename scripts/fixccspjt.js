
function usage()
{
    print("\n\tUSAGE:\n");
    print("\t\txs -f fixccspjt.js inputFile\n");
    java.lang.System.exit(1);
}

function fixcdt(cdt)
{
    var srcFile=String(cdt);
    var dot = srcFile.lastIndexOf(".");
    var extension = srcFile.substr(dot,srcFile.length);
    var products2add = ['PDK_INSTALL_PATH',    'IPC_INSTALL_PATH',
                        'SYSLIB_INSTALL_PATH', 'SNOW_INSTALL_PATH',   
                        'UIA_INSTALL_PATH'];

    /* Check for the right input */
    if (extension == ".cdtbuild" || extension == ".cproject"  )
    {
        var fileModule = xdc.module('xdc.services.io.File');
        var openedFile= fileModule.open(srcFile,"r");
        if(openedFile == null)
        {
            print("Error: Cannot open " +srcFile+" file ");
            usage();
        }
        /* Temporary file to write  */
        var tmpfile = fileModule.open("temp.xml","w");
        var readLine;
        while((readLine=openedFile.readLine()) != null)
        {
            if(readLine.match(/&quot;\${BIOS_CG_ROOT}\/packages/))
            {
                for (var i =0; i < products2add.length;i++) {
                    readLine=readLine.replace(/(.*\&quot;)(\$\{BIOS_CG_ROOT\}\/packages)(.*)/g, "$1$2$3\n$1\$\{"+products2add[i]+"\}$3");
                }
            }
            else if(readLine.match(/\${BIOS_CG_ROOT}\/packages/))
            {
                for (var i =0; i < products2add.length;i++) {
                    readLine=readLine.replace(/(.*)(\$\{BIOS_CG_ROOT\}\/packages)(.*)/g, "$1$2$3\n$1\$\{"+products2add[i]+"\}$3");
                }
            }
            
            tmpfile.writeLine(readLine);
        }  
        openedFile.close();
        tmpfile.close();

        /* Replace source file with Temp file */
        /* Copy Module */
        var copy = xdc.loadCapsule('Copy.xs');
        copy.Move("temp.xml",cdt);
    }
    else
    {
        print(cdt +"is not a valid ccs project file"); 
        java.lang.System.exit(1);
    }
}

/* Main function starts here */
if(arguments.length < 1)
{
  usage();
}

fixcdt(arguments[0]);
