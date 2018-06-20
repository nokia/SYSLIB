SECTIONS
{
    // Place all the MCSDK related data structures into core specific memory.
    GROUP(MCSDK_DATA_STRUCTURES)
    {
    	.cppi:      align=128  // CPPI LLD Multicore Datastructures
    	.qmss:      align=128  // QMSS LLD Multicore Datastructures
	} load=LTE1_MSMC_L2DP

	// Place DPM Data structure
    GROUP(MCSDK_DATA_STRUCTURES)
    {
		.dpm:       align=128  // DPM Datastructures
	} load=LTE1_DDR3_L2DP
}

