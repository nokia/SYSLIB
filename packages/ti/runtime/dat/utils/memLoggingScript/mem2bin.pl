#=================================================================================================
# mem2bin.pl
# ================================================================================================
# Description:
# This script is used for post-process Producer buffers captured through DAT memory  logging feature.
# It converts binary memory dump to bin file that can be analyzed by System Analyzer.
#
# This script has following assumptions:
# 1. The files begins with 4 lines of MetaData including Buffer information added by DAT module.
# 2. Each buffer begins with UIA packet header
# 3. Events have time stampe information
# ================================================================================================
# Parameters:
# 1. File name for captured memory dump file
# 2. File name for the output bin file
# ================================================================================================

use Time::Out qw(timeout) ;
use Time::Local;
use Time::localtime;
use FileHandle;

#-------------------------------------------------------------------------------------------------
#           GLOBAL VARIABLES
#-------------------------------------------------------------------------------------------------

#
$NumOfMetaData=4;

# Default capture buffer size
$BUFFERSIZE = 1408;

# Main function
# Read data from input file, then generate the bin file
#
{
    my $read;

    # Get input parameter
    $INPUT_HEX_DUMP = shift @ARGV;
    $OUTPUT_BIN_File = shift @ARGV;

    print "outpput file name: $OUTPUT_BIN_File \n";

    # Check if input file is available
    die "input file $INPUT_HEX_DUM not found\n" unless -f $INPUT_HEX_DUMP;

    my $filesize = -s $INPUT_HEX_DUMP;
    print " Input file $INPUT_HEX_DUMP has  $filesize bytes. \n";

    # Open input/output files
    open INPUT, "<  $INPUT_HEX_DUMP "
         or die "Can't open $INPUT_HEX_DUMP\n";

    open OUTPUT, "> $OUTPUT_BIN_File" ;

    # Read meta data from log file
    # line1: memory base
    # line2: memory size
    # line3: buffer size
    # line4: empty line
    for($idx=0; $idx < $NumOfMetaData; $idx++){
	$_=<INPUT>;
	print "$_";
	if ($_ eq "\n"){
	    last;
	}
        my ($key, $value) = split(/=/);
	print "$key : $value\n";

	if (lc($key) eq "buffersize")
	{
	    $BUFFERSIZE = $value;
	}
    }

    # Read all buffers and find the oldest buffer
    binmode(INPUT);
    binmode(OUTPUT);

    my $offset = 0;
    my $packetIdx = 0;
    my $ts_min = 0;
    my $lastPktIndex = 0;

    while ($filesize - $offset > $BUFFERSIZE)
    {
        # Read a buffer
	print " Process packet $packetIdx at offset $offset.\n";

        $read = read(INPUT, $buffer, $BUFFERSIZE, );
        if($read ==0 || $read < $BUFFERSIZE) {last;}

	$offset += $BUFFERSIZE;
	@word = unpack("N*", $buffer);

	# Get UIA Header size
	$UIAHdrSize = UIAPacket_getHeaderSize($word[0]);
	if ($UIAHdrSize == 0 || $UIAHdrSize % 4 !=0 )
	{
	    print "Error: Invalid UIA header Size $UIAHdrSize !!!\n";
            # Next UIA packet
  	    $packetIdx++;
	    #exit;
	    next;
	}

	# Get Event header offset
	$EventHdrOffset = $UIAHdrSize / 4;

	@word = unpack("I*", $buffer);
        $ts = UIAEvent_getTimeStamp($word[$EventHdrOffset], $word[$EventHdrOffset + 1], $word[$EventHdrOffset + 2]);
	if($ts_min == 0)
	{
	    $ts_min = $ts;
        }
	if ($ts < $ts_min)
	{
	    $ts_min = $ts;

	    printf ("Find the newest UIA packet buffer at index %d\n", ($packetIdx-1));
            # Save the packetIndex to be used later
	    $lastPktIndex =  $packetIdx - 1;
	    last;
	}

	# Next UIA packet
	$packetIdx++;

    };

    if($lastPktIdx==0)
    {
        # All the packet are in sequence but reached the end of file.
    }
    else
    {
        print " Process packet $packetIdx.\n";
        # Save current buffer in file
        UIAPacket_saveInFile();

        $packetIdx++;

        # Loop through to the end of the input file to dump UIA buffers in output file
        while ($filesize - $offset > $BUFFERSIZE)
        {
	    print " Process packet $packetIdx at offset $offset.\n";

    	    # Read UIA packet buffer
            $read = read(INPUT, $buffer, $BUFFERSIZE, );
            if($read != $BUFFERSIZE ) {
  	        print "Error in reading from file, read $read bytes\n";
	        # Next UIA packet
	        $packetIdx++;
	        next;
            }

	    $offset += $read;
	    UIAPacket_saveInFile();
	    $packetIdx++;
        };
    }
    # Add the rest of packet buffer in output file from the beginning to the oldest packet
    # Close and re-Open input/output files
    close($INPUT);
    open INPUT, "<  $INPUT_HEX_DUMP"
         or die "Can't re-open $INPUT_HEX_DUMP\n";

    # ignore the meta data region
    for($idx=0; $idx < $NumOfMetaData; $idx++){
	$_=<INPUT>;
    }

    binmode(INPUT);
    if( $lastPktIndex == 0){
        $lastPktIndex = $packetIdx;
    }
    $index = 0;
    $offset = 0;
    while($index <= $lastPktIndex)
    {
	print " Process packet $index at offset $offset.\n";
	# Read UIA packet buffer
        $read = read(INPUT, $buffer, $BUFFERSIZE, );
        if($read ==0) {last;}

	$offset += $read;
	UIAPacket_saveInFile();
	$index++;
    };

    close($INPUT);
    close($OUTPUT);
}

#-------------------------------------------------------------------------------------------------
#           Internal Sub-routines
#-------------------------------------------------------------------------------------------------
# Sub fucntion

# Get UIA packet length
# parameters:
# 1. UIA packet word 1
sub UIAPacket_getPacketLen
{
    # Find out the UIA packet type
    $UIAPacketType = @_[0] >> 28;

    # Get packet length for Event type(HdrType_EventPkt) (DSP producer)
    if ( $UIAPacketType == 10 ) {
        $UIAPacketLen =  @_[0] & 0x7FFFFFF;
    }
    # Get packet length for Event with CRC(EventPktWithCRC) (ARM producer)
    elsif ( $UIAPacketType == 2 ) {
    	    $UIAPacketLen =  @_[0] & 0x7FFFFFF;
    }
    # Not supported packet type
    else {
        $UIAPacketLen = 0;
    }

    return $UIAPacketLen;
}

# Get UIA packet header size
# parameters:
# 1. UIA packet header word1
# Return:
# UIA packet header size in bytes
sub UIAPacket_getHeaderSize
{
    # Find out the UIA packet type
    $UIAPacketType = @_[0] >> 28;

    # Get UIA packet header size for DSP producer
    # 4 words header - UIAPacket_Hdr
    if ( $UIAPacketType == 10 ) {
        $UIAPacketHeaderSize =  16;
    }
    # Get UIA packet header size for ARM producer
    # 5 words header - UIAPacket_Hdr5
    elsif ( $UIAPacketType == 2 ) {
        $UIAPacketHeaderSize =  20;
    }
    # Not supported packet type
    else {
        $UIAPacketHeaderSize = 0;
    }

    return $UIAPacketHeaderSize;
}

# Get Event timeStamp, there are 4 different events supported in UIA
# Two with time stamp and two without.
#  *  HdrType_Event
#  *    word0: EventHdr
#  *    word1: event Id (top 16 bits) & module Id (bottom 16 bits)
#  *
#  *  HdrType_EventWithTimestamp
#  *    word0: EventHdr
#  *    word1: Timestamp lower 32 bits
#  *    word2: Timestamp upper 32 bits
#  *    word3: event Id (top 16 bits) & module Id (bottom 16 bits)
#  *
#  *  HdrType_EventWithSnapshotId
#  *    word0: EventHdr
#  *    word1: event Id (top 16 bits) & module Id (bottom 16 bits)
#  *    word2: filename pointer
#  *    word3: linenum
#  *    word4: snapshotId
#  *    word5: address where the data was located
#  *    word6: total length of data (top 16-bits)
#  *           length for this record (bottom 16 bits)
#  *    word7: format pointer
#  *    data:  the rest of the record contains the data
#  *
#  *  HdrType_EventWithSnapshotIdAndTimestamp:
#  *    word0: EventHdr
#  *    word1: Timestamp lower 32 bits
#  *    word2: Timestamp upper 32 bits
#  *    word3: event Id (top 16 bits) & module Id (bottom 16 bits)
#  *    word4: filename pointer
#  *    word5: linenum
#  *    word6: snapshotId
#  *    word7: address where the data was located
#  *    word8: total length of data (top 16-bits)
#  *           length for this record (bottom 16 bits)
#  *    word9: format pointer
#  *    data:  the rest of the record contains the data
#
# parameters:
# 1. Event header
# Return:
# 64bit timestamp
sub UIAEvent_getTimeStamp
{
    # The first word has the Event type
    $UIAEventType = @_[0] >> 27;

    # 0 - ti_uia_runtime_EventHdr_HdrType_Event
    # 1 - ti_uia_runtime_EventHdr_HdrType_EventWithTimestamp
    # 2 - ti_uia_runtime_EventHdr_HdrType_EventWithSnapshotId
    # 3 - ti_uia_runtime_EventHdr_HdrType_EventWithSnapshotIdAndTimestamp
    # 5 - ti_uia_runtime_EventHdr_HdrType_EventWithTimestampAndEndpointId
    # 7 - ti_uia_runtime_EventHdr_HdrType_EventWithSnapshotIdAndTimestampAndEndpointId

    # Get time stamp for the event type 1, 3, 5, 7
    if ( ($UIAEventType == 1) || ($UIAEventType ==3) ||
         ($UIAEventType == 5) || ($UIAEventType ==7) )
    {
	# Time stamp is next two words
	# Debugging message
        $timeStamp =  @_[2] << 32;
        $timeStamp = $timeStamp	+  @_[1];
	printf("event time stamp: 0x%x-0x%x\n", @_[2], @_[1]);
    }
    else {
	# Otherwise return 0
        $timeStamp = 0;
    }

    return $timeStamp;
}

# Save current data buffer in output file
sub UIAPacket_saveInFile
{
    my @data = unpack("N*", $buffer);

    # Get UIA packet length
    $packetLen = UIAPacket_getPacketLen($data[0]);
    # Check if $len is valid
    if ($packetLen == 0)  {
	 print "Error: Invalid packet length found, dropping the packet\n";
	 return;
    }

    # pack data in byte array
    @bytes = unpack("W*", $buffer);
    $buffer2 = pack("C$packetLen", @bytes);

    # re-pack it with right data
    print OUTPUT $buffer2;
}

