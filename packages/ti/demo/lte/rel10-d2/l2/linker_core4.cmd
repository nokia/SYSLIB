SECTIONS
{
    // Place all the MCSDK related data structures into core specific memory.
    GROUP(MCSDK_DATA_STRUCTURES)
    {
    	.cppi:      align=128  // CPPI LLD Multicore Datastructures
    	.qmss:      align=128  // QMSS LLD Multicore Datastructures
	} load=LTE2_MSMC_L2DP
}

