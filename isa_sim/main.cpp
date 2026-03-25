#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "riscv_main.h"
#include "riscv.h"
#include "cosim_api.h"

//-----------------------------------------------------------------
// main
//-----------------------------------------------------------------
int main(int argc, char *argv[])
{
    int exitcode;

    Riscv * sim = new Riscv();

    cosim::instance()->attach_cpu("sim", sim);
    cosim::instance()->attach_mem("sim", sim, 0, 0xFFFFFFFF);

    exitcode = riscv_main(sim, argc, argv);

    // Show execution stats
    sim->stats_dump();

    delete sim;
    sim = NULL;

    return exitcode;
}
