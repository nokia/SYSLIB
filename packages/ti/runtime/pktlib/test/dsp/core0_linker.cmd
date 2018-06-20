SECTIONS
{
    // Place all the Multicore Shared Libraries at well known memory locations across
    // all the cores.
    GROUP(EXTMEM_SHARED_DATASTRUCTURES)
    {
        .cppi:                  align=128  // CPPI LLD Multicore Datastructures
        .qmss:                  align=128  // QMSS LLD Multicore Datastructures
    } load=LTE1_MSMC_L2DP
}

