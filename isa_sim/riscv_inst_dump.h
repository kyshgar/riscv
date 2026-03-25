#ifndef __RISCV_INST_DUMP_H__
#define __RISCV_INST_DUMP_H__

#include <stdint.h>

//--------------------------------------------------------------------
// Prototypes:
//--------------------------------------------------------------------
bool riscv_inst_decode(char *str, uint32_t pc, uint32_t opcode);
void riscv_inst_print(uint32_t pc, uint32_t opcode);

#endif
