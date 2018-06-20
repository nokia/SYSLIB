/*
 *  Copyright 2012 by Texas Instruments Incorporated.
 *
 * \file PdmlToPacketBin.xs
 *
 * \brief XDCscript for converting Wireshark .pdml files to System Analyzer .bin files
 *
 * Usage: xs -f PdmlToPacketBin.xs arg1 arg2
 *
 *   arg1: Input .pdml File
 *
 *   arg2: Output System Analyzer Bin File
 */

try
{
    /* Get arguments. */
    var argIndex = 0;
    var inputFile = arguments[argIndex++].replace(/\\/g, '/');
    var outputFile = arguments[argIndex++].replace(/\\/g, '/');

    print("PdmlToPacketBin.xs: Arguments:");
    print("    System Analyzer Bin File = " + outputFile);
    print("    Input .pdml File     = " + inputFile);
}
catch (ReferenceError)
{
    print ("ERROR: Missing/invalid arguments! Exiting...");
    pcap2binUsage();
    java.lang.System.exit(1);
}

if (arguments.length < 2)
{
    print ("ERROR: Missing arguments! Exiting...");
    java.lang.System.exit(1)
}

var rxBuf = xdc.jre.java.lang.reflect.Array.newInstance(xdc.jre.java.lang.Byte.TYPE, 4096*4);
var UIA_UDP_PORT = 65261;
var UIA_UDP_PORT_HEX = "feed";

//convertPcap2Pdml();
run();

/*
 *  ======== convertPcap2Pdml ========
 *  Execute within a function context to reduce the scope of any variables.
 */
function pcap2binUsage()
{
    print( "Usage: xs -f PdmlToPacketBin.xs arg1 arg2");
    print( "  arg1: Input .pdml File");
    print( "  arg2: Output System Analyzer Bin File");
}

/*
 *  ======== run ========
 *  Execute within a function context to reduce the scope of any variables.
 */
function run()
{
    print("Converting "+inputFile+" to "+outputFile+"...");

    /* Verify outputPDMLFile exists. */
    var inFile = new java.io.FileReader(inputFile);
    var inFileReader = new java.io.BufferedReader(inFile);

    /* Create an output stream to write the bytes to a file. */
    var outFile = new xdc.jre.java.io.File(outputFile);
    var fos = new xdc.jre.java.io.FileOutputStream(outFile);

    var packetCnt = 0;
    var sCurrentLine;
    var uiaPktHdr;
    var sData;
    var indexOfData;
    var loopCtr = 0;
    var endIndex;
    var databyte;
    var i;
    var j;
    var buffer = [];
    var strByte = "";
    var len = 0;
    var byteCtr;
    var wordCtr;
    var dataWord = "";
	var pktStart = "<packet>";
	var icmpPacket = "Internet Control Message Protocol";
    var prefix = "<field name=\"data\" value=\"";
	var destPortPrefix = "show=\""+UIA_UDP_PORT+"\" value=\""+UIA_UDP_PORT_HEX+"\"";
	var isDestPortOK = false;
	var discardPacket = false;
	var indexOfDestPort;
	var pktStartOffset = 0;

    /* Read data from the target in a loop. */
    print("Reading data...");
    i = 0;
    j = 0;

    /* Parse the PDML file line by line. */
    while ((sCurrentLine = inFileReader.readLine()) != null) {
        loopCtr++;

        /* Is this the start of the packet? */
		if (sCurrentLine.indexOf(pktStart) >= 0){
			isDestPortOK = false;
			discardPacket = false;
		}

        /* Is this an ICMP packet? If yes, ignore. */
		if (sCurrentLine.indexOf(icmpPacket) >= 0){
			discardPacket = true;
		}

        /* Verify this is a packet with UIA destination port. */
		if (!discardPacket){
			if (!isDestPortOK){
				indexOfDestPort = sCurrentLine.indexOf(destPortPrefix);
				if (indexOfDestPort >= 0){
					isDestPortOK = true;
				}
			} else {
                /* Identify the payload data. */
				indexOfData = sCurrentLine.indexOf(prefix);
				if (indexOfData >= 0){
					isDestPortOK = false;

					/* Old ethereal format: endIndex = sCurrentLine.indexOf("\"/>"); */
					endIndex = sCurrentLine.indexOf("\">");

					/* Data is in hex characters, so 1 character = 4 bits. */
					if (endIndex > (indexOfData+3)){
						sData = sCurrentLine.substring(indexOfData+prefix.length,endIndex);
						i = 0;
						j = 0;
						byteCtr = 0;
						wordCtr = 0;
						dataWord = "";
						lenWord = "";

                        /* Debug prints. */
						if (packetCnt == 0){
						   print("Debug info for packet 0 (first 4 words of packet):");
						}

                        /* Get the length of the UDP payload. Some of this is padding
                         * that needs to be removed while writing to the output file. */
                        len = sData.length();

                        /* Loop through the packet data and extract the data bytes. */
						while (i < (len*2)){
							strByte = sData.substring(i,i+2);
							databyte = ((0xFF & parseInt(strByte,16))>>>0);
							dataWord = dataWord+strByte;

                            /* i =4 and 6 are the bytes corresponding to the UIA packet length;
                             * extract the length and read only that many bytes of data. */
                            if (i == 4 || i == 6)
                                lenWord = lenWord+strByte;
                            if (i == 6){
                                len = ((0xFFFF & parseInt(lenWord,16))>>>0);
                                lenWord = "";
							}
							if (byteCtr == 3){
								if (i <= 32){
								  /* Output the UIA Packet header words for the first packet
								   * to help debug the script. */
								  if (packetCnt  == 0){
									print(wordCtr+": word=0x"+dataWord);
								  }
								  dataWord = "";
								}
								byteCtr = 0;
								wordCtr++;
							} else {
								byteCtr++;
							}
							i += 2;

							/* Since java only understands signed values, any byte value > 127 will cause an exception.
							 * to get around this, convert to signed number (2's complement).
							 * see http://bytes.com/topic/java/answers/517554-unsigned-byte-values for more info. */
							if (databyte >= 128){
								databyte = (databyte - 256);
							}
							rxBuf[j++] = Number(databyte);
						}
					}

                    /* Debug prints. */
				    if ((packetCnt &0x1ff) == 0){
						print("Packet "+packetCnt+": length="+j+"\n");
				    }

                    /* Write the data to the file and increment packet count. */
					fos.write(rxBuf,0, j);
					packetCnt++;
				}
			}
        }
	}

    /* Close the files and exit. */
    inFileReader.close();
    fos.close();
	print("UIA packet data written to "+outFile);
}
