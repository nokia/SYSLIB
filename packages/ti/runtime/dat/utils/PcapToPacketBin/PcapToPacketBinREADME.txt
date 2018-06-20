/*
 * \file PcapToPacketBinREADME.txt
 */

-------------------------------------------------------------------------------
Description:
-------------------------------------------------------------------------------
PcapToPacketBin.xs is an XDCscript that converts an input PCAP file to a binary
file containing UIA packet data.
This script first converts the PCAP to a PDML format using 'tshark' which is
present in the Wireshark installation directory. Packets are filtered on the
basis of UDP destination port = 65261 (which corresponds to the UIA packets), when
writing to the PDML file. The PDML file is then parsed to obtain the payloads of
these packets, which is then written to the binary file specified by the user.

-------------------------------------------------------------------------------
Pre-requisites:
-------------------------------------------------------------------------------
XDC tools package should be installed on the system on which this script will be
executed.

-------------------------------------------------------------------------------
Instructions for execution:
-------------------------------------------------------------------------------
Open a command window and change the directory to where PcapToPacketBin.xs is
located.
From the command line, execute the following command:
    xs -f PcapToPacketBin.xs <arg1> <arg2> <arg3> <arg4>

where the arguments are:
    arg1: Input .pcap File
    arg2: Output System Analyzer Bin File
    arg3: Output .pdml File generated during the conversion
    arg4: Tshark installation directory
          Directory in which Wireshark/tshark is installed

Once the bin file is generated, it can be opened in System Analyzer by clicking
on Tools->System Analyzer->Open Binary File in a CCS debug session. You still
require a .usmxml file with endpoints configured correctly and paths to the
*.out, *.uia.xml and *.rta.xml files.

-------------------------------------------------------------------------------
Example:
-------------------------------------------------------------------------------
xs -f PcapToPacketBin.xs C:/myWork/uia.pcap C:/myWork/uia.bin C:/myWork/uia.pdml "C:/Program Files/Wireshark"

-------------------------------------------------------------------------------
NOTE:
-------------------------------------------------------------------------------
If there are spaces in the directory names, use "" as shown in the above
example.


