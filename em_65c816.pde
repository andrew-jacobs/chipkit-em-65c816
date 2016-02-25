//==============================================================================
//  _____ __  __        __  ____   ____ ___  _  __   
// | ____|  \/  |      / /_| ___| / ___( _ )/ |/ /_  
// |  _| | |\/| |_____| '_ \___ \| |   / _ \| | '_ \ 
// | |___| |  | |_____| (_) |__) | |__| (_) | | (_) |
// |_____|_|  |_|      \___/____/ \____\___/|_|\___/ 
//
// A WDC 65C816 Emulator for the ChipKit
//------------------------------------------------------------------------------
// Copyright (C),2015 HandCoded Software Ltd.
// All rights reserved.
//
// This work is made available under the terms of the Creative Commons
// Attribution-NonCommercial 2.0 license. Open the following URL to see the
// details.
//
// http://creativecommons.org/licenses/by-nc/2.0/
//------------------------------------------------------------------------------

#define TRACE(X) X

//==============================================================================
// Data Areas
//------------------------------------------------------------------------------

// Bit masks for processor flags
enum flag_t {
  C = 0x01, Z = 0x02, I = 0x04, D = 0x08,
  X = 0x10, M = 0x20, V = 0x40, N = 0x80 };

// The CPU register set
struct reg_t {
  uint16_t  pc;     // Program counter
  uint16_t  sp;     // Stack pointer
  uint16_t  dp;     // Direct page pointer
  union {
    struct{
      uint8_t a;    // Accumulator A
      uint8_t b;    // Accumulator B
    };
    uint16_t  c;    // Combined accumulators
  };
  uint16_t  x;      // Index register X
  uint16_t  y;      // Index register Y
  uint8_t  p;       // Status register
  uint32_t  pbr;    // Program bank register in <23:16>
  uint32_t  dbr;    // Data bank register in <23:16>
  uint8_t  e;       // Emulation bit
} 
reg;

const uint32_t  RAM_SIZE  = 384 * 1024;
const uint32_t  ROM_SIZE  = 128 * 1024;
const uint32_t  ADR_MASK  = 0x0007ffff;

// 384K RAM
static uint8_t  ram [RAM_SIZE];

// 128K Addon ROM
const uint8_t   rom [ROM_SIZE] = {
  0x00, 0x00  // #include from a file
};

// 32K Boot ROM
const uint8_t  boot [] = {
#include "boot.txt"
};

//==============================================================================
// Output Routines
//------------------------------------------------------------------------------

// Print a value in hex to the specified precision in digits
void printHex (uint32_t value, uint32_t digits)
{
  if (digits > 1) printHex (value >> 4, digits - 1);
  Serial.write ("0123456789abcdef"[value & 0x0f]);
}

// Print an address in bank:offset format
void printAddr (uint32_t addr)
{
  printHex (addr >> 16, 2);
  Serial.write (':');
  printHex (addr, 4);
}

//==============================================================================
// Memory Access
//------------------------------------------------------------------------------

// Fetch a single byte from RAM or ROM
inline uint8_t getByte (uint32_t addr)
{
  return ((addr < RAM_SIZE) ? ram [addr] : rom [addr - RAM_SIZE]);
}

// Fetch a work from RAM or ROM
inline uint16_t getWord (uint32_t addr)
{
  return (getByte (addr) | (getByte (addr + 1) << 8));
}

// Set a byte of memory to the specified value
inline void setByte (uint32_t addr, uint8_t data)
{
  if (addr < RAM_SIZE) ram [addr] = data;
}

// Set a word of memory to the specified value
inline void setWord (uint32_t addr, uint16_t data)
{
  setByte (addr, data);
  setByte (wrap (addr), data >> 8);
}

// Add one to an address and wrap it within the same bank
inline uint32_t wrap (uint32_t addr)
{
  return ((addr & 0xff0000) | ((addr + 1) & 0x00ffff));
}

// Combines a 16-bit address with the program bank
inline uint32_t pb_addr (uint16_t addr)
{
  return (reg.pbr | addr);

}

// Combines a 16-bit address with the data bank
inline uint32_t db_addr (uint16_t addr)
{
  return (reg.dbr | addr);

}

// Works out an address on direct page
inline uint32_t dp_addr (uint16_t addr)
{
  return ((reg.dp + addr) & 0xffff);
}

//==============================================================================
// Address Modes
//------------------------------------------------------------------------------

// Implied
inline uint32_t impl (uint32_t *pExtra)
{
  TRACE (Serial.print ("         "));

  return (0);
}

// Immediate (8-bits)
inline uint32_t immd_b (uint32_t *pExtra)
{
  register uint32_t ea = pb_addr (reg.pc);

  TRACE (Serial.print (" "));
  TRACE (printHex (getByte (pb_addr (reg.pc)), 2)); 
  TRACE (Serial.print ("      "));

  reg.pc += 1;
  return (ea & ADR_MASK);
}

// Immediate (16-bits)
inline uint32_t immd_w (uint32_t *pExtra)
{
  register uint32_t ea = pb_addr (reg.pc);

  TRACE (Serial.print (" "));
  TRACE (printHex (getByte (pb_addr (reg.pc + 0)), 2)); 
  TRACE (Serial.print (" "));
  TRACE (printHex (getByte (pb_addr (reg.pc + 1)), 2));
  TRACE (Serial.print ("   "));

  reg.pc += 2;
  return (ea & ADR_MASK);
}

// Relative
inline uint32_t rela (uint32_t *pExtra)
{
  register int16_t off = (int8_t) getByte (pb_addr (reg.pc)); 

  TRACE (Serial.print (" "));
  TRACE (printHex (off, 2));
  TRACE (Serial.print ("      "));
  
  reg.pc += 1;
  return (pb_addr (reg.pc + off) & ADR_MASK);
}

// Long Relative
inline uint32_t lrel (uint32_t *pExtra)
{
  register int16_t off = (int16_t) getWord (pb_addr (reg.pc)); 

  TRACE (Serial.print (" "));
  TRACE (printHex (off, 2));
  TRACE (Serial.print (" "));
  TRACE (printHex (off >> 8, 2));
  TRACE (Serial.print ("   "));
  
  reg.pc += 2;
  return (pb_addr (reg.pc + off) & ADR_MASK);
}

// Direct Page
inline uint32_t dpag (uint32_t *pExtra)
{
  register uint16_t addr = getByte (pb_addr (reg.pc));

  TRACE (Serial.print (" "));
  TRACE (printHex (addr, 2));
  TRACE (Serial.print ("      "));

  reg.pc += 1;
  return (dp_addr (addr) & ADR_MASK);
}

// Direct Page indexed with X
inline uint32_t dpgx (uint32_t *pExtra)
{
  register uint16_t addr = getByte (pb_addr (reg.pc));

  TRACE (Serial.print (" "));
  TRACE (printHex (addr, 2));
  TRACE (Serial.print ("      "));

  reg.pc += 1;
  return (dp_addr (addr + reg.x) & ADR_MASK);
}

// Direct Page indexed with Y
inline uint32_t dpgy (uint32_t *pExtra)
{
  register uint16_t addr = getByte (pb_addr (reg.pc));

  TRACE (Serial.print (" "));
  TRACE (printHex (addr, 2));
  TRACE (Serial.print ("      "));

  reg.pc += 1;
  return (dp_addr (addr + reg.y) & ADR_MASK);
}

// Direct Page Indirect
inline uint32_t dpgi (uint32_t *pExtra)
{
  register uint16_t addr = getByte (pb_addr (reg.pc));

  TRACE (Serial.print (" "));
  TRACE (printHex (addr, 2));
  TRACE (Serial.print ("      "));
  reg.pc += 1;
  
  return (db_addr (getWord (dp_addr (addr))) & ADR_MASK);
}

// Direct Page Indexed Indirect
inline uint32_t dpix (uint32_t *pExtra)
{
  register uint16_t addr = getByte (pb_addr (reg.pc));

  TRACE (Serial.print (" "));
  TRACE (printHex (addr, 2));
  TRACE (Serial.print ("      "));
  reg.pc += 1;
  
  return (db_addr (getWord (dp_addr (addr + reg.x))) & ADR_MASK);
}

// Direct Page Indirect Indexed
inline uint32_t dpiy (uint32_t *pExtra)
{
  register uint16_t addr = getByte (pb_addr (reg.pc));

  TRACE (Serial.print (" "));
  TRACE (printHex (addr, 2));
  TRACE (Serial.print ("      "));
  reg.pc += 1;
  
  return (db_addr (getWord (dp_addr (addr)) + reg.y) & ADR_MASK);
}

// Absolute
inline uint32_t abso (uint32_t *pExtra)
{
  register uint16_t addr = getWord (pb_addr (reg.pc));
  register uint32_t ea = db_addr (addr);

  TRACE (Serial.print (" "));
  TRACE (printHex (addr, 2));
  TRACE (Serial.print (" "));
  TRACE (printHex (addr >> 8, 2));
  TRACE (Serial.print ("   "));

  reg.pc += 2;
  return (ea & ADR_MASK);
}

// Absolute indexed X
inline uint32_t absx (uint32_t *pExtra)
{
  register uint16_t addr = getWord (pb_addr (reg.pc));
  register uint32_t ea = db_addr (addr) + reg.x;

  TRACE (Serial.print (" "));
  TRACE (printHex (addr, 2));
  TRACE (Serial.print (" "));
  TRACE (printHex (addr >> 8, 2));
  TRACE (Serial.print ("   "));

  reg.pc += 2;
  return (ea & ADR_MASK);
}

// Absolute indexed Y
inline uint32_t absy (uint32_t *pExtra)
{
  register uint16_t addr = getWord (pb_addr (reg.pc));
  register uint32_t ea = db_addr (addr) + reg.y;

  TRACE (Serial.print (" "));
  TRACE (printHex (addr, 2));
  TRACE (Serial.print (" "));
  TRACE (printHex (addr >> 8, 2));
  TRACE (Serial.print ("   "));

  reg.pc += 2;
  return (ea & ADR_MASK);
}

// Absolute Long
inline uint32_t alng (uint32_t *pExtra)
{
  register uint16_t addr = getWord (pb_addr (reg.pc));
  register uint8_t  bank = getByte (pb_addr (reg.pc + 2));
  register uint32_t ea = (bank << 16) | addr;

  TRACE (Serial.print (" "));
  TRACE (printHex (addr, 2));
  TRACE (Serial.print (" "));
  TRACE (printHex (addr >> 8, 2));
  TRACE (Serial.print (" "));
  TRACE (printHex (bank, 2));
  reg.pc += 3;

  return (ea & ADR_MASK);
}

// Absolute Indirect
inline uint32_t absi (uint32_t *pExtra)
{
  register uint16_t addr = getWord (pb_addr (reg.pc));
  register uint32_t ea = getWord (addr);

  TRACE (Serial.print (" "));
  TRACE (printHex (addr, 2));
  TRACE (Serial.print (" "));
  TRACE (printHex (addr >> 8, 2));
  TRACE (Serial.print ("   "));

  reg.pc += 2;
  return (ea & ADR_MASK);
}


//==============================================================================
// Flag Setting
//------------------------------------------------------------------------------

inline void setn_b (uint16_t value)
{
  if (value & 0x0080)
    reg.p |=  N;
  else
    reg.p &= ~N;
}

inline void setn_w (uint16_t value)
{
  if (value & 0x8000)
    reg.p |=  N;
  else
    reg.p &= ~N;
}

inline void setz_b (uint16_t value)
{
  if (value & 0x00ff)
    reg.p &= ~Z;
  else
    reg.p |=  Z;
}

inline void setz_w (uint16_t value)
{
  if (value)
    reg.p &= ~Z;
  else
    reg.p |=  Z;
}

inline void setc (bool value)
{
  if (value)
    reg.p |=  C;
  else
    reg.p &= ~C;
}

inline void setv (bool value)
{
  if (value)
    reg.p |=  V;
  else
    reg.p &= ~V;
}

//==============================================================================
// Opcodes
//------------------------------------------------------------------------------

// ADC
inline uint32_t adc_b (uint32_t addr)
{
  TRACE (Serial.print (" ADC"));
  if (reg.p & D) {
  }
  return (2);
}

//------------------------------------------------------------------------------

inline uint32_t and_b (uint32_t addr)
{
  TRACE (Serial.print (" AND"));
  reg.a &= getByte (addr);
  setn_b (reg.a);
  setz_b (reg.a);
  return (2);
}

inline uint32_t and_w (uint32_t addr)
{
  TRACE (Serial.print (" AND"));
  reg.c &= getWord (addr);
  setn_w (reg.c);
  setz_w (reg.c);
  return (2);
}

uint32_t and_immd_b (uint32_t *pExtra)
{
  return (and_b (immd_b (pExtra)));
}

uint32_t and_immd_w (uint32_t *pExtra)
{
  return (and_w (immd_w (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t asl_b (uint32_t addr)
{
  register uint8_t data = getByte (addr);

  TRACE (Serial.print (" ASL"));
  setc (data & 0x80);
  data = (data << 1);
  setByte (addr, data);
  setn_b (data);
  setz_b (data);
  return (2);
}

inline uint32_t asl_w (uint32_t addr)
{
  register uint16_t data = getWord (addr);

  TRACE (Serial.print (" ASL"));
  setc (data & 0x8000);
  data = (data << 1);
  setByte (addr, data);
  setn_w (data);
  setz_w (data);
  return (2);
}


uint32_t asl_dpag_b (uint32_t *pExtra)
{
  return (0);
}

//------------------------------------------------------------------------------

inline uint32_t bcc (uint32_t addr)
{
  TRACE (Serial.print (" BCC"));

  if (!(reg.p & C)) {
    reg.pc = addr;
    return (1);
  }
  return (0);
}

uint32_t bcc_rela (uint32_t *pExtra)
{
  return (bcc (rela (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t bcs (uint32_t addr)
{
  TRACE (Serial.print (" BCS"));

  if (reg.p & C) {
    reg.pc = addr;
    return (1);
  }
  return (0);
}

uint32_t bcs_rela (uint32_t *pExtra)
{
  return (bcs (rela (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t beq (uint32_t addr)
{
  TRACE (Serial.print (" BEQ"));

  if (reg.p & Z) {
    reg.pc = addr;
    return (1);
  }
  return (0);
}

uint32_t beq_rela (uint32_t *pExtra)
{
  return (beq (rela (pExtra)));
}

//------------------------------------------------------------------------------

// BIT

//------------------------------------------------------------------------------

inline uint32_t bpl (uint32_t addr)
{
  TRACE (Serial.print (" BPL"));

  if (!(reg.p & N)) {
    reg.pc = addr;
    return (1);
  }
  return (0);
}

uint32_t bpl_rela (uint32_t *pExtra)
{
  return (bpl (rela (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t bmi (uint32_t addr)
{
  TRACE (Serial.print (" BMI"));

  if (reg.p & N) {
    reg.pc = addr;
    return (1);
  }
  return (0);
}

uint32_t bmi_rela (uint32_t *pExtra)
{
  return (bmi (rela (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t bne (uint32_t addr)
{
  TRACE (Serial.print (" BNE"));

  if (!(reg.p & Z)) {
    reg.pc = addr;
    return (1);
  }
  return (0);
}

uint32_t bne_rela (uint32_t *pExtra)
{
  return (bne (rela (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t bra (uint32_t addr)
{
  TRACE (Serial.print (" BRA"));
  reg.pc = addr;
  return (0);
}

uint32_t bra_rela (uint32_t *pExtra)
{
  return (bra (rela (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t brk_e (uint32_t addr)
{
  TRACE (Serial.print (" BRK"));
  return (2);
}

inline uint32_t brk_n (uint32_t addr)
{
  TRACE (Serial.print (" BRK"));
  return (2);
}

uint32_t brk_immd_e (uint32_t *pExtra)
{
  return (brk_e (immd_b (pExtra)));
}

uint32_t brk_immd_n (uint32_t *pExtra)
{
  return (brk_n (immd_b (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t brl (uint32_t addr)
{
  TRACE (Serial.print (" BRL"));
  reg.pc = addr;
  return (2);
}

uint32_t brl_lrel (uint32_t *pExtra)
{
  return (bra (lrel (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t bvc (uint32_t addr)
{
  TRACE (Serial.print (" BVC"));

  if (!(reg.p & V)) {
    reg.pc = addr;
    return (1);
  }
  return (0);
}

uint32_t bvc_rela (uint32_t *pExtra)
{
  return (bvc (rela (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t bvs (uint32_t addr)
{
  TRACE (Serial.print (" BVS"));

  if (reg.p & V) {
    reg.pc = addr;
    return (1);
  }
  return (0);
}

uint32_t bvs_rela (uint32_t *pExtra)
{
  return (bvs (rela (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t clc (uint32_t addr)
{
  TRACE (Serial.print (" CLC"));
  reg.p &= ~C;
  return (2);
}

uint32_t clc_impl (uint32_t *pExtra)
{
  return (clc (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t cld (uint32_t addr)
{
  TRACE (Serial.print (" CLD"));
  reg.p &= ~D;
  return (2);
}

uint32_t cld_impl (uint32_t *pExtra)
{
  return (cld (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t cli (uint32_t addr)
{
  TRACE (Serial.print (" CLI"));
  reg.p &= ~I;
  return (2);
}

uint32_t cli_impl (uint32_t *pExtra)
{
  return (cli (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t clv (uint32_t addr)
{
  TRACE (Serial.print (" CLV"));
  reg.p &= ~V;
  return (2);
}

uint32_t clv_impl (uint32_t *pExtra)
{
  return (clv (impl (pExtra)));
}

//------------------------------------------------------------------------------

// CMP

//------------------------------------------------------------------------------

inline uint32_t cop_e (uint32_t addr)
{
  TRACE (Serial.print (" COP"));
  return (2);
}

inline uint32_t cop_n (uint32_t addr)
{
  TRACE (Serial.print (" COP"));
  return (2);
}

uint32_t cop_immd_e (uint32_t *pExtra)
{
  return (cop_e (immd_b (pExtra)));
}

uint32_t cop_immd_n (uint32_t *pExtra)
{
  return (cop_n (immd_b (pExtra)));
}

//------------------------------------------------------------------------------

// CPX

//------------------------------------------------------------------------------

// CPY

//------------------------------------------------------------------------------

// DEC

//------------------------------------------------------------------------------

inline uint32_t dex_b (uint32_t addr)
{
  TRACE (Serial.print (" DEX"));
  reg.x = (reg.x - 1) & 0x00ff;
  setn_b (reg.x);
  setz_b (reg.x);
  return (2);
}

inline uint32_t dex_w (uint32_t addr)
{
  TRACE (Serial.print (" DEX"));
  reg.x = (reg.x - 1);
  setn_w (reg.x);
  setz_w (reg.x);
  return (2);
}

uint32_t dex_impl_b (uint32_t *pExtra)
{
  return (dex_b (impl (pExtra)));
}

uint32_t dex_impl_w (uint32_t *pExtra)
{
  return (dex_b (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t dey_b (uint32_t addr)
{
  TRACE (Serial.print (" DEY"));
  reg.y = (reg.y - 1) & 0x00ff;
  setn_b (reg.y);
  setz_b (reg.y);
  return (2);
}

inline uint32_t dey_w (uint32_t addr)
{
  TRACE (Serial.print (" DEY"));
  reg.y = (reg.y - 1);
  setn_w (reg.y);
  setz_w (reg.y);
  return (2);
}

uint32_t dey_impl_b (uint32_t *pExtra)
{
  return (dey_b (impl (pExtra)));
}

uint32_t dey_impl_w (uint32_t *pExtra)
{
  return (dey_b (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t eor_b (uint32_t addr)
{
  TRACE (Serial.print (" EOR"));
  reg.a ^= getByte (addr);
  setn_b (reg.a);
  setz_b (reg.a);
  return (2);
}

inline uint32_t eor_w (uint32_t addr)
{
  TRACE (Serial.print (" EOR"));
  reg.c ^= getWord (addr);
  setn_w (reg.c);
  setz_w (reg.c);
  return (2);
}


//------------------------------------------------------------------------------

// INC

//------------------------------------------------------------------------------

inline uint32_t inx_b (uint32_t addr)
{
  TRACE (Serial.print (" INX"));
  reg.x = (reg.x + 1) & 0x00ff;
  setn_b (reg.x);
  setz_b (reg.x);
  return (2);
}

inline uint32_t inx_w (uint32_t addr)
{
  TRACE (Serial.print (" INX"));
  reg.x = (reg.x + 1) & 0xffff;
  setn_w (reg.x);
  setz_w (reg.x);
  return (2);
}

uint32_t inx_impl_b (uint32_t *pExtra)
{
  return (inx_b (impl (pExtra)));
}

uint32_t inx_impl_w (uint32_t *pExtra)
{
  return (inx_b (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t iny_b (uint32_t addr)
{
  TRACE (Serial.print (" INY"));
  reg.x = (reg.y + 1) & 0x00ff;
  setn_b (reg.y);
  setz_b (reg.y);
  return (2);
}

inline uint32_t iny_w (uint32_t addr)
{
  TRACE (Serial.print (" INY"));
  reg.x = (reg.y + 1) & 0xffff;
  setn_w (reg.y);
  setz_w (reg.y);
  return (2);
}

uint32_t iny_impl_b (uint32_t *pExtra)
{
  return (iny_b (impl (pExtra)));
}

uint32_t iny_impl_w (uint32_t *pExtra)
{
  return (iny_b (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t jmp (uint32_t addr)
{
  TRACE (Serial.print (" JMP"));
  reg.pc = addr;
  return (2);
}

uint32_t jmp_abso (uint32_t *pExtra)
{
  return (jmp (abso (pExtra)));
}

uint32_t jmp_absi (uint32_t *pExtra)
{
  return (jmp (absi (pExtra)));
}

//------------------------------------------------------------------------------

// JML

//------------------------------------------------------------------------------

// LDA
inline uint32_t lda_b (uint32_t addr)
{
  TRACE (Serial.print (" LDA"));
  reg.a = getByte (addr);
  return (2);
}

inline uint32_t lda_w (uint32_t addr)
{
  TRACE (Serial.print (" LDA"));
  reg.c = getWord (addr);
  return (2);
}

uint32_t lda_immd_b (uint32_t *pExtra)
{
  return (lda_b (immd_b (pExtra)));
}

uint32_t lda_immd_w (uint32_t *pExtra)
{
  return (lda_w (immd_w (pExtra)));
}

//------------------------------------------------------------------------------

// LDX
inline uint32_t ldx_b (uint32_t addr)
{
  TRACE (Serial.print (" LDX"));
  reg.x = getByte (addr);
  return (2);
}

inline uint32_t ldx_w (uint32_t addr)
{
  TRACE (Serial.print (" LDX"));
  reg.x = getWord (addr);
  return (2);
}

uint32_t ldx_immd_b (uint32_t *pExtra)
{
  return (ldx_b (immd_b (pExtra)));
}

uint32_t ldx_immd_w (uint32_t *pExtra)
{
  return (ldx_w (immd_w (pExtra)));
}

//------------------------------------------------------------------------------

// LDY
inline uint32_t ldy_b (uint32_t addr)
{
  TRACE (Serial.print (" LDY"));
  reg.y = getByte (addr);
  return (2);
}

inline uint32_t ldy_w (uint32_t addr)
{
  TRACE (Serial.print (" LDY"));
  reg.y = getWord (addr);
  return (2);
}

uint32_t ldy_immd_b (uint32_t *pExtra)
{
  return (ldy_b (immd_b (pExtra)));
}

uint32_t ldy_immd_w (uint32_t *pExtra)
{
  return (ldy_w (immd_w (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t lsr_b (uint32_t addr)
{
  register uint8_t data = getByte (addr);

  TRACE (Serial.print (" LSR"));
  setc (data & 0x01);
  data = (data >> 1) & 0x7f;
  setByte (addr, data);
  setn_b (data);
  setz_b (data);
  return (2);
}

inline uint32_t lsr_w (uint32_t addr)
{
  register uint16_t data = getWord (addr);

  TRACE (Serial.print (" LSR"));
  setc (data & 0x0001);
  data = (data >> 1) & 0x7fff;
  setByte (addr, data);
  setn_w (data);
  setz_w (data);
  return (2);
}


//------------------------------------------------------------------------------

// MVN

//------------------------------------------------------------------------------

// MVP

//------------------------------------------------------------------------------

inline uint32_t nop (uint32_t addr)
{
  TRACE (Serial.print (" NOP"));
  return (2);
}

uint32_t nop_impl (uint32_t *pExtra)
{
  return (nop (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t ora_b (uint32_t addr)
{
  TRACE (Serial.print (" ORA"));
  reg.a |= getByte (addr);
  setn_b (reg.a);
  setz_b (reg.a);
  return (2);
}

inline uint32_t ora_w (uint32_t addr)
{
  TRACE (Serial.print (" ORA"));
  reg.c |= getWord (addr);
  setn_w (reg.c);
  setz_w (reg.c);
  return (2);
}

uint32_t ora_dpag_b (uint32_t *pExtra)
{
  return (ora_b (dpag (pExtra)));
}

uint32_t ora_dpxi_b (uint32_t *pExtra)
{
  return (0);
}

uint32_t ora_lngi_b (uint32_t *pExtra)
{
  return (0);
}

uint32_t ora_srel_b (uint32_t *pExtra)
{
  return (0);
}

//------------------------------------------------------------------------------

// PEA

//------------------------------------------------------------------------------

// PEI

//------------------------------------------------------------------------------

// PER

//------------------------------------------------------------------------------

// PHA

//------------------------------------------------------------------------------

// PHB

//------------------------------------------------------------------------------

// PHD

//------------------------------------------------------------------------------

// PHK

//------------------------------------------------------------------------------

// PHP

//------------------------------------------------------------------------------

// PHX

//------------------------------------------------------------------------------

// PHY

//------------------------------------------------------------------------------

// PLA

//------------------------------------------------------------------------------

// PLB

//------------------------------------------------------------------------------

// PLD

//------------------------------------------------------------------------------

// PLP

//------------------------------------------------------------------------------

// PLX

//------------------------------------------------------------------------------

// PLY

//------------------------------------------------------------------------------

inline uint32_t rep_e (uint32_t addr)
{
  TRACE (Serial.print (" REP"));
  reg.p &= ~getByte (addr);
  reg.p |= M|X;
  return (3);
}

inline uint32_t rep_n (uint32_t addr)
{
  TRACE (Serial.print (" REP"));
  reg.p &= ~getByte (addr);
  return (3);
}

uint32_t rep_immd_e (uint32_t *pExtra)
{
  return (rep_e (immd_b (pExtra)));
}

uint32_t rep_immd_n (uint32_t *pExtra)
{
  return (rep_n (immd_b (pExtra)));
}

//------------------------------------------------------------------------------

// ROL
inline uint32_t rol_b (uint32_t addr)
{
  register uint8_t data = getByte (addr);
  register uint8_t oldc = (reg.p & C) ? 0x01 : 0;

  TRACE (Serial.print (" ROL"));
  setc (data & 0x80);
  data = (data << 1) | oldc;
  setByte (addr, data);
  setn_b (data);
  setz_b (data);
  return (2);
}

inline uint32_t rol_w (uint32_t addr)
{
  register uint16_t data = getWord (addr);
  register uint8_t oldc = (reg.p & C) ? 0x0001 : 0;

  TRACE (Serial.print (" ROL"));
  setc (data & 0x8000);
  data = (data << 1) | oldc;
  setByte (addr, data);
  setn_w (data);
  setz_w (data);
  return (2);
}

//------------------------------------------------------------------------------

// ROR
inline uint32_t ror_b (uint32_t addr)
{
  register uint8_t data = getByte (addr);
  register uint8_t oldc = (reg.p & C) ? 0x80 : 0;

  TRACE (Serial.print (" ROL"));
  setc (data & 0x01);
  data = (data >> 1) | oldc;
  setByte (addr, data);
  setn_b (data);
  setz_b (data);
  return (2);
}

inline uint32_t ror_w (uint32_t addr)
{
  register uint16_t data = getWord (addr);
  register uint8_t oldc = (reg.p & C) ? 0x8000 : 0;

  TRACE (Serial.print (" ROL"));
  setc (data & 0x8000);
  data = (data << 1) | oldc;
  setByte (addr, data);
  setn_w (data);
  setz_w (data);
  return (2);
}

//------------------------------------------------------------------------------

// RTI

//------------------------------------------------------------------------------

// RTL

//------------------------------------------------------------------------------

// RTS

//------------------------------------------------------------------------------

// SBC

//------------------------------------------------------------------------------

inline uint32_t sec (uint32_t addr)
{
  TRACE (Serial.print (" SEC"));
  reg.p |=  C;
  return (2);
}

uint32_t sec_impl (uint32_t *pExtra)
{
  return (sec (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t sed (uint32_t addr)
{
  TRACE (Serial.print (" SED"));
  reg.p |=  D;
  return (2);
}

uint32_t sed_impl (uint32_t *pExtra)
{
  return (sed (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t sei (uint32_t addr)
{
  TRACE (Serial.print (" SEI"));
  reg.p |=  I;
  return (2);
}

uint32_t sei_impl (uint32_t *pExtra)
{
  return (sei (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t sep_e (uint32_t addr)
{
  TRACE (Serial.print (" SEP"));
  reg.p |= getByte (addr);
  return (3);
}

inline uint32_t sep_n (uint32_t addr)
{
  TRACE (Serial.print (" SEP"));
  reg.p |= getByte (addr);
  return (3);
}

uint32_t sep_immd_e (uint32_t *pExtra)
{
  return (sep_e (immd_b (pExtra)));
}

uint32_t sep_immd_n (uint32_t *pExtra)
{
  return (sep_n (immd_b (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t sta_b (uint32_t addr)
{
  TRACE (Serial.print (" STA"));
  setByte (addr, reg.a);
  return (2);
}  

inline uint32_t sta_w (uint32_t addr)
{
  TRACE (Serial.print (" STA"));
  setWord (addr, reg.c);
  return (2);
}

uint32_t sta_abso_b (uint32_t *pExtra)
{
  return (sta_b (abso (pExtra)));
}

uint32_t sta_absx_b (uint32_t *pExtra)
{
  return (sta_b (absx (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t stp (uint32_t addr)
{
  TRACE (Serial.print (" STP"));
  return (3);
}

uint32_t stp_impl (uint32_t *pExtra)
{
  return (stp (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t stx_b (uint32_t addr)
{
  TRACE (Serial.print (" STX"));
  setByte (addr, reg.y);
  return (2);
}  

inline uint32_t stx_w (uint32_t addr)
{
  TRACE (Serial.print (" STX"));
  setWord (addr, reg.y);
  return (2);
}

uint32_t stx_abso_b (uint32_t *pExtra)
{
  return (stx_b (abso (pExtra)));
}

uint32_t stx_dpag_b (uint32_t *pExtra)
{
  return (stx_b (dpag (pExtra)));
}

uint32_t stx_dpgy_b (uint32_t *pExtra)
{
  return (stx_b (dpgy (pExtra)));
}

uint32_t stx_abso_w (uint32_t *pExtra)
{
  return (stx_w (abso (pExtra)));
}

uint32_t stx_dpag_w (uint32_t *pExtra)
{
  return (stx_w (dpag (pExtra)));
}

uint32_t stx_dpgy_w (uint32_t *pExtra)
{
  return (stx_w (dpgy (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t sty_b (uint32_t addr)
{
  TRACE (Serial.print (" STY"));
  setByte (addr, reg.y);
  return (2);
}  

inline uint32_t sty_w (uint32_t addr)
{
  TRACE (Serial.print (" STY"));
  setWord (addr, reg.y);
  return (2);
}

uint32_t sty_abso_b (uint32_t *pExtra)
{
  return (sty_b (abso (pExtra)));
}

uint32_t sty_dpag_b (uint32_t *pExtra)
{
  return (sty_b (dpag (pExtra)));
}

uint32_t sty_dpgx_b (uint32_t *pExtra)
{
  return (sty_b (dpgx (pExtra)));
}

uint32_t sty_abso_w (uint32_t *pExtra)
{
  return (sty_w (abso (pExtra)));
}

uint32_t sty_dpag_w (uint32_t *pExtra)
{
  return (sty_w (dpag (pExtra)));
}

uint32_t sty_dpgx_w (uint32_t *pExtra)
{
  return (sty_w (dpgx (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t stz_b (uint32_t addr)
{
  TRACE (Serial.print (" STZ"));
  setByte (addr, 0);
  return (2);
}  

inline uint32_t stz_w (uint32_t addr)
{
  TRACE (Serial.print (" STZ"));
  setWord (addr, 0);
  return (2);
}

uint32_t stz_abso_b (uint32_t *pExtra)
{
  return (stz_b (abso (pExtra)));
}

uint32_t stz_absx_b (uint32_t *pExtra)
{
  return (stz_b (absx (pExtra)));
}

uint32_t stz_dpag_b (uint32_t *pExtra)
{
  return (stz_b (dpag (pExtra)));
}

uint32_t stz_dpgx_b (uint32_t *pExtra)
{
  return (stz_b (dpgx (pExtra)));
}

uint32_t stz_abso_w (uint32_t *pExtra)
{
  return (stz_w (abso (pExtra)));
}

uint32_t stz_absx_w (uint32_t *pExtra)
{
  return (stz_w (absx (pExtra)));
}

uint32_t stz_dpag_w (uint32_t *pExtra)
{
  return (stz_w (dpag (pExtra)));
}

uint32_t stz_dpgx_w (uint32_t *pExtra)
{
  return (stz_w (dpgx (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t tax_b (uint32_t addr)
{
  TRACE (Serial.print (" TAX"));
  setn_b (reg.x = reg.a);
  setz_b (reg.x);
  return (2);
}

inline uint32_t tax_w (uint32_t addr)
{
  TRACE (Serial.print (" TAX"));
  setn_w (reg.x = reg.c);
  setz_b (reg.x);
  return (2);
}

uint32_t tax_impl_b (uint32_t *pExtra)
{
  return (tax_b (impl (pExtra)));
}

uint32_t tax_impl_w (uint32_t *pExtra)
{
  return (tax_b (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t tay_b (uint32_t addr)
{
  TRACE (Serial.print (" TAY"));
  setn_b (reg.y = reg.a);
  setz_b (reg.y);
  return (2);
}

inline uint32_t tay_w (uint32_t addr)
{
  TRACE (Serial.print (" TAY"));
  setn_w (reg.y = reg.c);
  setz_b (reg.y);
  return (2);
}

uint32_t tay_impl_b (uint32_t *pExtra)
{
  return (tay_b (impl (pExtra)));
}

uint32_t tay_impl_w (uint32_t *pExtra)
{
  return (tay_b (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t tcd (uint32_t addr)
{
  TRACE (Serial.print (" TCD"));
  reg.dp = reg.c;
  setn_w (reg.dp);
  setz_w (reg.dp);
  return (2);
}

uint32_t tcd_impl (uint32_t *pExtra)
{
  return (tcd (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t tcs_e (uint32_t addr)
{
  TRACE (Serial.print (" TCS"));
  reg.sp = 0x0100 | reg.a;
  return (2);
}

inline uint32_t tcs_n (uint32_t addr)
{
  TRACE (Serial.print (" TCS"));
  reg.sp = reg.c;
  return (2);
}

uint32_t tcs_impl_e (uint32_t *pExtra)
{
  return (tcs_e (impl (pExtra)));
}

uint32_t tcs_impl_n (uint32_t *pExtra)
{
  return (tcs_n (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t tdc (uint32_t addr)
{
  TRACE (Serial.print (" TDC"));
  reg.c = reg.dp;
  setn_w (reg.c);
  setz_w (reg.c);
  return (2);
}

uint32_t tdc_impl (uint32_t *pExtra)
{
  return (tdc (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t trb_b (uint32_t addr)
{
  register uint8_t  data = getByte (addr);
  register uint8_t  msk = data &  reg.a;
  register uint8_t  res = data & ~reg.a;

  TRACE (Serial.print (" TRB"));
  setByte (addr, res);
  setz_b (msk);
  return (2);
}

inline uint32_t trb_w (uint32_t addr)
{
  register uint16_t  data = getWord (addr);
  register uint16_t  msk = data &  reg.c;
  register uint16_t  res = data & ~reg.c;

  TRACE (Serial.print (" TRB"));
  setWord (addr, res);
  setz_w (msk);
  return (2);
}

uint32_t trb_abso_b (uint32_t *pExtra)
{
  return (trb_b (abso (pExtra)));
}

uint32_t trb_dpag_b (uint32_t *pExtra)
{
  return (trb_b (dpag (pExtra)));
}

uint32_t trb_abso_w (uint32_t *pExtra)
{
  return (trb_w (abso (pExtra)));
}

uint32_t trb_dpag_w (uint32_t *pExtra)
{
  return (trb_w (dpag (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t tsb_b (uint32_t addr)
{
  register uint8_t  data = getByte (addr);
  register uint8_t  msk = data & reg.a;
  register uint8_t  res = data | reg.a;

  TRACE (Serial.print (" TSB"));
  setByte (addr, res);
  setz_b (msk);
  return (2);
}

inline uint32_t tsb_w (uint32_t addr)
{
  register uint16_t   data = getWord (addr);
  register uint16_t   msk = data & reg.c;
  register uint16_t   res = data | reg.c;

  TRACE (Serial.print (" TSB"));
  setWord (addr, res);
  setz_w (msk);
  return (2);
}

uint32_t tsb_abso_b (uint32_t *pExtra)
{
  return (tsb_b (abso (pExtra)));
}

uint32_t tsb_dpag_b (uint32_t *pExtra)
{
  return (tsb_b (dpag (pExtra)));
}

uint32_t tsb_abso_w (uint32_t *pExtra)
{
  return (tsb_w (abso (pExtra)));
}

uint32_t tsb_dpag_w (uint32_t *pExtra)
{
  return (tsb_w (dpag (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t tsc (uint32_t addr)
{
  TRACE (Serial.print (" TSC"));
  reg.c = reg.sp;
  setn_w (reg.c);
  setz_w (reg.c);
  return (2);
}

uint32_t tsc_impl (uint32_t *pExtra)
{
  return (tsc (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t tsx_e (uint32_t addr)
{
  TRACE (Serial.print (" TXS"));
  reg.x = reg.sp & 0x00ff;
  setn_b (reg.x);
  setz_b (reg.x);
  return (2);
}

inline uint32_t tsx_b (uint32_t addr)
{
  TRACE (Serial.print (" TXS"));
  reg.x = reg.sp & 0x00ff;
  setn_b (reg.x);
  setz_b (reg.x);
  return (2);
}

inline uint32_t tsx_w (uint32_t addr)
{
  TRACE (Serial.print (" TXS"));
  reg.x = reg.sp;
  setn_w (reg.x);
  setz_w (reg.x);
  return (2);
}

uint32_t tsx_impl_e (uint32_t *pExtra)
{
  return (tsx_e (impl (pExtra)));
}

uint32_t tsx_impl_b (uint32_t *pExtra)
{
  return (tsx_b (impl (pExtra)));
}

uint32_t tsx_impl_w (uint32_t *pExtra)
{
  return (tsx_w (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t txa_b (uint32_t addr)
{
  TRACE (Serial.print (" TXA"));
  setn_b (reg.a = reg.x);
  setz_b (reg.a);
  return (2);
}

inline uint32_t txa_w (uint32_t addr)
{
  TRACE (Serial.print (" TXA"));
  setn_w (reg.c = reg.x);
  setz_b (reg.c);
  return (2);
}

uint32_t txa_impl_b (uint32_t *pExtra)
{
  return (txa_b (impl (pExtra)));
}

uint32_t txa_impl_w (uint32_t *pExtra)
{
  return (txa_b (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t txs_e (uint32_t addr)
{
  TRACE (Serial.print (" TXS"));
  reg.sp = 0x0100 | reg.x;
  return (2);
}

inline uint32_t txs_n (uint32_t addr)
{
  TRACE (Serial.print (" TXS"));
  reg.sp = reg.x;
  return (2);
}

uint32_t txs_impl_e (uint32_t *pExtra)
{
  return (txs_e (impl (pExtra)));
}

uint32_t txs_impl_n (uint32_t *pExtra)
{
  return (txs_n (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t txy_b (uint32_t addr)
{
  TRACE (Serial.print (" TXY"));
  setn_b (reg.y = reg.x);
  setz_b (reg.y);
  return (2);
}

inline uint32_t txy_w (uint32_t addr)
{
  TRACE (Serial.print (" TXY"));
  setn_w (reg.y = reg.x);
  setz_b (reg.y);
  return (2);
}

uint32_t txy_impl_b (uint32_t *pExtra)
{
  return (txy_b (impl (pExtra)));
}

uint32_t txy_impl_w (uint32_t *pExtra)
{
  return (txy_w (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t tya_b (uint32_t addr)
{
  TRACE (Serial.print (" TYA"));
  setn_b (reg.a = reg.y);
  setz_b (reg.a);
  return (2);
}

inline uint32_t tya_w (uint32_t addr)
{
  TRACE (Serial.print (" TYA"));
  setn_w (reg.c = reg.y);
  setz_b (reg.c);
  return (2);
}

uint32_t tya_impl_b (uint32_t *pExtra)
{
  return (tya_b (impl (pExtra)));
}

uint32_t tya_impl_w (uint32_t *pExtra)
{
  return (tya_b (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t tyx_b (uint32_t addr)
{
  TRACE (Serial.print (" TYX"));
  setn_b (reg.x = reg.y);
  setz_b (reg.x);
  return (2);
}

inline uint32_t tyx_w (uint32_t addr)
{
  TRACE (Serial.print (" TYX"));
  setn_w (reg.x = reg.y);
  setz_b (reg.x);
  return (2);
}

uint32_t tyx_impl_b (uint32_t *pExtra)
{
  return (tyx_b (impl (pExtra)));
}

uint32_t tyx_impl_w (uint32_t *pExtra)
{
  return (tyx_w (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t wai (uint32_t addr)
{
  TRACE (Serial.print (" WAI"));
  return (2);
}

uint32_t wai_impl (uint32_t *pExtra)
{
  return (wai (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t wdm (uint32_t addr)
{
  TRACE (Serial.print (" WDM"));
  return (2);
}

uint32_t wdm_immd_b (uint32_t *pExtra)
{
  return (wdm (immd_b (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t xba (uint32_t addr)
{
  TRACE (Serial.print (" XBA"));
  reg.c = (reg.a << 8)| reg.b;

  return (2);
}

uint32_t xba_impl (uint32_t *pExtra)
{
  return (xba (impl (pExtra)));
}

//------------------------------------------------------------------------------

inline uint32_t xce (uint32_t addr)
{
  register uint8_t    diff = (reg.e ^ reg.p) & C;

  TRACE (Serial.print (" XCE"));
#if 0
  reg.p ^= diff;
  if (reg.e ^= diff) {
    reg.p |= M|X;
    reg.sp = 0x0100 | (reg.sp & 0x00ff);
    reg.x &= 0x00ff;
    reg.y &= 0x00ff;
  }
#endif
  return (2);
}

uint32_t xce_impl (uint32_t *pExtra)
{
  return (xce (impl (pExtra)));
}

//------------------------------------------------------------------------------

uint32_t error (uint32_t *pExtra)
{
  TRACE (Serial.print ("          ???"));
  return (2);
}

//==============================================================================
// Dispatch Tables
//------------------------------------------------------------------------------

typedef uint32_t (*opcode_fn_t)(uint32_t *);

const opcode_fn_t em_opcode [256] = {
  brk_immd_e,   ora_dpxi_b,   cop_immd_e,   ora_srel_b, // 00
  tsb_dpag_b,   ora_dpag_b,   asl_dpag_b,   ora_lngi_b,
  error,        error,        error,        error,
  tsb_abso_b,   error,        error,        error,
  bpl_rela,     error,        error,        error,      // 10
  trb_dpag_b,   error,        error,        error,  
  clc_impl,     error,        error,        tcs_impl_e,
  trb_abso_b,   error,        error,        error,
  error,        error,        error,        error,      // 20
  error,        error,        error,        error, 
  error,        error,        error,        error,
  error,        error,        error,        error,
  bmi_rela,     error,        error,        error,      // 30
  error,        error,        error,        error, 
  sec_impl,     error,        error,        tsc_impl,
  error,        error,        error,        error,
  error,        error,        wdm_immd_b,   error,      // 40
  error,        error,        error,        error,
  error,        error,        error,        error,
  jmp_abso,     error,        error,        error,
  bvc_rela,     error,        error,        error,      // 50
  error,        error,        error,        error,
  cli_impl,     error,        error,        tcd_impl,
  error,        error,        error,        error,
  error,        error,        error,        error,      // 60
  stz_dpag_b,   error,        error,        error, 
  error,        error,        error,        error,
  jmp_absi,     error,        error,        error,
  bvs_rela,     error,        error,        error,      // 70
  stz_dpgx_b,   error,        error,        error, 
  sei_impl,     error,        error,        tdc_impl,
  error,        error,        error,        error,
  bra_rela,     error,        brl_lrel,     error,      // 80
  sty_dpag_b,   error,        stx_dpag_b,   error, 
  dey_impl_b,   error,        txa_impl_b,   error,
  sty_abso_b,   error,        stx_abso_b,   error,
  bcc_rela,     error,        error,        error,      // 90
  sty_dpgx_b,   error,        stx_dpgy_b,   error, 
  tya_impl_b,   error,        txs_impl_e,   txy_impl_b,
  stz_abso_b,   error,        stz_absx_b,   error,
  ldy_immd_b,   error,        ldx_immd_b,   error,      // a0
  error,        error,        error,        error, 
  error,        lda_immd_b,   tax_impl_b,   tay_impl_b,
  error,        error,        error,        error,
  bcs_rela,     error,        error,        error,      // b0
  error,        error,        error,        error, 
  clv_impl,     error,        tsx_impl_e,   tyx_impl_b,
  error,        error,        error,        error,
  error,        error,        rep_immd_e,   error,      // c0
  error,        error,        error,        error, 
  iny_impl_b,   error,        dex_impl_b,   wai_impl,
  error,        error,        error,        error,
  bne_rela,     error,        error,        error,      // d0
  error,        error,        error,        error,
  cld_impl,     error,        error,        stp_impl,
  error,        error,        error,        error,
  error,        error,        sep_immd_e,   error,      // e0
  error,        error,        error,        error,
  inx_impl_b,   error,        nop_impl,     error,
  error,        error,        error,        xba_impl,
  beq_rela,     error,        error,        error,      // f0
  error,        error,        error,        error,
  sed_impl,     error,        error,        xce_impl,
  error,        error,        error,        error
};

const opcode_fn_t na_opcode [4][256] = {
  {
  }
  ,
  {
  }
  ,
  {
  }
  ,
  {
  }
};

//==============================================================================
//------------------------------------------------------------------------------

// Resets the CPU state 
void reset ()
{
  memcpy (&(ram [0x8000]), boot, sizeof(boot));

  reg.e = 1;
  reg.pc = getWord (0xfffc);
  reg.sp = 0x0100;
  reg.dp = 0x0000;
  reg.p = I;
  reg.pbr = 0;
  reg.dbr = 0;
}

uint32_t step ()
{
  register uint32_t ea = (reg.pbr << 16) | reg.pc++;
  register uint8_t op = getByte (ea);
  uint32_t cycles = 0;

  TRACE (printAddr (ea));
  TRACE (Serial.print (' '));
  TRACE (printHex (op, 2));

  if (reg.e)
    cycles += (*em_opcode [op]) (&cycles);
  else
    cycles += (*na_opcode [(reg.p & (M|X)) >> 4][op]) (&cycles);

  TRACE (Serial.print (" C="));
  TRACE (printHex (reg.c, 4));
  TRACE (Serial.print (" X="));
  TRACE (printHex (reg.x, 4));
  TRACE (Serial.print (" Y="));
  TRACE (printHex (reg.y, 4));
  TRACE (Serial.print (" P="));
  TRACE (printHex (reg.p, 2));
  TRACE (Serial.print (" SP="));
  TRACE (printHex (reg.sp, 4));
  TRACE (Serial.print (" DP="));
  TRACE (printHex (reg.dp, 4));
  TRACE (Serial.print (" PBR="));
  TRACE (printHex (reg.pbr, 2));
  TRACE (Serial.print (" DBR="));
  TRACE (printHex (reg.dbr, 2));

  TRACE (Serial.println ());
  return (cycles);
}

//==============================================================================
//------------------------------------------------------------------------------

void setup ()
{
  Serial.begin (57600);
  Serial.println ("\nEM-65C816 [15.12]");
  Serial.println ("(C),2015 HandCoded Software Ltd.\n");

  reset ();
}

void loop ()
{
  step ();
  delay (10);
}


