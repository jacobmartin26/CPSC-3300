/* Cache statistics for a 4 KiB, 4-way set associative, write-back
 *   data cache with 32 bytes/line and pseudo-LRU replacement
 *
 * note that this simulation does not include the contents of the
 *   cache lines - instead, the cache directory bits (valid, dirty,
 *   and tag) and the per-set replacement state are used to determine
 *   the hit, miss, and write-back counts based on the addresses used
 *   to access the cache and the types of accesses (reads or writes)
 *
 * routines
 *
 *   void cache_init( void );
 *   void cache_access( unsigned int address, unsigned int type );
 *   void cache_stats( void );
 *
 * for each call to cache_access() address is the byte address, and
 *   type is either read (=0) or write (=1)
 *
 *
 * 4 KiB four-way set-associative cache, 32 bytes/line
 *   => 128 total lines, 4 banks, 32 lines/bank
 *   => 32-bit address partitioned into
 *         22-bit tag
 *          5-bit index         [ 5 = log2( 32 lines/bank ) ]
 *          5-bit byte offset   [ 5 = log2( 32 bytes/line ) ]
 *
 * index            bank 0          bank 1          bank 2          bank 3
 * (set) PLRU   v d tag cont    v d tag cont    v d tag cont    v d tag cont
 *       +--+  +-+-+---+----+  +-+-+---+----+  +-+-+---+----+  +-+-+---+----+
 *   0   |  |  | | |   |////|  | | |   |////|  | | |   |////|  | | |   |////|
 *       +--+  +-+-+---+----+  +-+-+---+----+  +-+-+---+----+  +-+-+---+----+
 *   1   |  |  | | |   |////|  | | |   |////|  | | |   |////|  | | |   |////|
 *       +--+  +-+-+---+----+  +-+-+---+----+  +-+-+---+----+  +-+-+---+----+
 *       ...        ...             ...             ...             ...
 *       +--+  +-+-+---+----+  +-+-+---+----+  +-+-+---+----+  +-+-+---+----+
 *  31   |  |  | | |   |////|  | | |   |////|  | | |   |////|  | | |   |////|
 *       +--+  +-+-+---+----+  +-+-+---+----+  +-+-+---+----+  +-+-+---+----+
 *
 *
 * pseudo-LRU replacement using three-bit state scheme for 4-way s.a.
 *
 *  each bit represents one branch point in a binary decision tree
 *
 *  let 1 represent that the left side has been referenced more
 *  recently than the right side, and 0 vice-versa
 *
 *             are all 4 lines valid?
 *                  /       \
 *                yes        no, use an invalid line
 *                 |
 *                 |
 *                 |
 *            bit_0 == 0?           state | replace    ref to | next state
 *             /       \            ------+--------    -------+-----------
 *            y         n            00x  |  line_0    line_0 |    11_
 *           /           \           01x  |  line_1    line_1 |    10_
 *    bit_1 == 0?    bit_2 == 0?     1x0  |  line_2    line_2 |    0_1
 *      /    \          /    \       1x1  |  line_3    line_3 |    0_0
 *     y      n        y      n
 *    /        \      /        \       ('x' means     ('_' means unchanged)
 *  line_0  line_1  line_2  line_3     don't care)
 *
 * see Figure 3-7, p. 3-18, in Intel Embedded Pentium Processor Family Dev.
 *   Manual, 1998, http://www.intel.com/design/intarch/manuals/273204.htm)
 *
 * note that there is separate state kept for each set (i.e., index value)
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>

#define LINES_PER_BANK 64

unsigned int
  valid[2][LINES_PER_BANK],    /* valid bit for each line    */
  dirty[2][LINES_PER_BANK],    /* dirty bit for each line    */
  tag[2][LINES_PER_BANK],      /* tag bits for each line     */
  lru[LINES_PER_BANK];

unsigned int
    cache_reads,  /* counter */
    cache_writes, /* counter */
    hits,         /* counter */
    misses,       /* counter */
    write_backs;  /* counter */

void cache_init(void){
  int i;
  for(i=0; i<LINES_PER_BANK; i++)
  {
    lru[i] = 0;
    valid[0][i] = dirty[0][i] = tag[0][i] = 0;
    valid[1][i] = dirty[1][i] = tag[1][i] = 0;
  }
  cache_reads = cache_writes = hits = misses = write_backs = 0;
}

void cache_stats(void){
  printf( "cache statistics (in decimal):\n" );
  printf( "  cache reads       = %d\n", cache_reads );
  printf( "  cache writes      = %d\n", cache_writes );
  printf( "  cache hits        = %d\n", hits );
  printf( "  cache misses      = %d\n", misses );
  printf( "  cache write backs = %d\n", write_backs );
}

/* address is byte address, type is read (=0) or write (=1) */
void cache_access( unsigned int address, unsigned int type )
{
  unsigned int
    addr_tag,    /* tag bits of address     */
    addr_index,  /* index bits of address   */
    bank;        /* bank that hit, or bank chosen for replacement */

  if(type == 0){
    cache_reads++;
  }else{
    cache_writes++;
  }

  addr_index = (address >> 3) & 0x3f;
  addr_tag = address >> 9;

  /* check bank 0 hit */
  if(valid[0][addr_index] && (addr_tag==tag[0][addr_index])){
    hits++;
    bank = 0;

  /* check bank 1 hit */
  }else if(valid[1][addr_index] && (addr_tag==tag[1][addr_index])){
    hits++;
    bank = 1;

  /* miss - choose replacement bank */
  }else{
    misses++;

    if(!valid[0][addr_index]) bank = 0;
    else if(!valid[1][addr_index]) bank = 1;
    else bank = lru[addr_index] ? 0 : 1;

    if(valid[bank][addr_index] && dirty[bank][addr_index])
    {
      write_backs++;
    }

    valid[bank][addr_index] = 1;
    dirty[bank][addr_index] = 0;
    tag[bank][addr_index] = addr_tag;
  }

  /* update replacement state for this set (i.e., index value) */
  lru[addr_index] = bank;

  /* update dirty bit on a write */
  if(type == 1) dirty[bank][addr_index] = 1;
}


/* behavioral simulation of MC88100 subset for CPSC 3300 at Clemson
 *
 * reference manual is http://www.bitsavers.org/components/motorola/
 *   88000/MC88100_RISC_Microprocessor_Users_Manual_2ed_1990.pdf
 *
 * processor state subset
 *   32 x 32-bit general registers
 *     note that r0 is always 0
 *   two 32-bit instruction address registers
 *     fip - fetch address pointer
 *     xip - execute address pointer
 * 
 * memory
 *   byte-addressable
 *   big-endian addressing
 *   aligned accesses
 *   - an instruction starts on an address that is a multiple of 4
 *   - a data word starts on an address that is a multiple of 4
 *
 *   note - we limit this simulation to a 1 MiB memory
 *
 *   note - instructions are one word (four bytes) in length and
 *     we also limit the instruction subset in this simulation to
 *     operations on on word-length data
 *
 * we implement all three addressing modes, see pages 3-7 to 3-10
 *
 *   register indirect with zero-extended immediate index
 *     eff_addr = reg[ s1 ] + imm16
 *
 *   register indirect with index
 *     eff_addr = reg[ s1 ] + reg[ s2 ]
 *
 *   register indirect with index (word scaling = 2)
 *     eff_addr = reg[ s1 ] + ( reg[ s2 ] << 2 )
 *
 * we implement 20 instructions derived from 12 base instructions
 *
 *   halt is added for the simulation
 *   add  is described on pages 3-29 to 3-30
 *   bcnd is described on pages 3-13 to 3-14 and 3-35 to 3-36
 *   br   is described on pages 3-16 and 3-37
 *   ext  is described on pages 3-44 to 3-45
 *   extu is described on pages 3-46 to 3-47
 *   mak  is described on pages 3-70 to 3-71
 *   rot  is described on page 3-76
 *   ld   is described on pages 3-65 to 3-66
 *   lda  is described on pages 3-67 to 3-68
 *   st   is described on pages 3-79 to 3-80
 *   sub  is described on pages 3-82 to 3-83
 *
 * decoding and instruction formats (op1 is first six bits)
 *
 *   op1 = 0 => halt
 *
 *   op1 = 0x05, 0x09, 0x0d, 0x1c, 0x1d =>
 *     opcodes are ld, st, lda, add, sub, respectively
 *     format has two registers and a 16-bit immediate
 *     immediate value is zero-extended
 *     signed words in normal mode used for load/stores,
 *       so p = 01, ty = 01, and u = 0
 *     carry and borrow are not used, so i = 0 and o = 0
 *
 *   op1 = 0x30 => br
 *     format has a single 26-bit displacement
 *     displacement is sign-extended
 *     displacement is in words and calculated from the
 *       address of the current instruction rather than
 *
 *   op1 = 0x3a => bcnd
 *     format has mask, register, and 16-bit displacement
 *     displacement is sign-extended
 *     displacement is in words and calculated from the
 *       address of the current instruction rather than
 *       the updated fetch address
 *     assert checks that displacement is non-zero
 *     delayed branching is not used, so n = 0
 *
 *   op1 = 0x3c => ext, extu, mak, rot
 *     format has two registers and a 5-bit immediate
 *     ext, extu, and mak are used as shifts so w5 = 0
 *       the updated fetch address
 *     assert checks that displacement is non-zero
 *     delayed branching is not used, so n = 0
 *
 *   op1 = 0x3d => ld, st, lda, add, sub
 *     format has three registers
 *     for add and sub:
 *       carry and borrow are not used, so i = 0 and o = 0
 *     for ld, sta, and lda:
 *       signed words in normal mode used for load/stores,
 *         so p = 01, ty = 01, and u = 0
 *       if bit 9 = 1, the third register is scaled
 */

/* since the simulation deals only with one-word instructions and */
/*   one-word operands, we represent memory as an array of words  */

#define MEM_SIZE_IN_WORDS 256*1024

int mem[MEM_SIZE_IN_WORDS];

/* processor state, simulation state, and instruction fields    */

int reg[32]   = {0}, /* general register set, r0 is always 0    */
    xip       = 0,   /* execute instruction pointer             */
    fip       = 0,   /* fetch instruction pointer               */
    halt_flag = 0,   /* set by halt instruction                 */
    verbose   = 0,   /* governs amount of detail in output      */
    ir,              /* 32-bit instruction register             */
    op1,             /* 6-bit primary opcode in bits 31 to 27   */
    op2,             /* 6-bit secondary opcode in bits 15 to 10 */
    d,               /* 5-bit destination register identifier   */
    s1,              /* 5-bit source 1 register identifier      */
    s2,              /* 5-bit source 2 register identifier      */
    imm16,           /* 16-bit immediate field                  */
    scaled,          /* scaled addressing mode bit 9            */
    eff_addr;        /* 32-bit effective address                */

/* dynamic execution statistics */

int inst_fetches = 0,
    memory_reads = 0,
    memory_writes = 0,
    branches = 0,
    taken_branches = 0;


/* load memory from stdin */

#define INPUT_WORD_LIMIT 255
void get_mem(){
  int w, count = 0;

  if( verbose > 1 ) printf( "reading words in hex from stdin:\n" );
  while( scanf( "%x", &w ) != EOF ){
    if( verbose > 1 ) printf( "  0%08x\n", w );
    if( count > INPUT_WORD_LIMIT ){
      printf( "too many words loaded\n" );
      exit( 0 );
    }
    mem[ count ] = w;
    count++;
  }
  if( verbose > 1 ) printf( "\n" );
}

void read_mem( int eff_addr, int reg_index ){
  int word_addr = eff_addr >> 2;
  if( verbose ) printf( "  read access at address %x\n", eff_addr );
  assert( ( word_addr >= 0 ) && ( word_addr < MEM_SIZE_IN_WORDS ) );
  reg[ reg_index ] = mem[ word_addr ];
  memory_reads++;
}

void write_mem( int eff_addr, int reg_index ){
  int word_addr = eff_addr >> 2;
  if( verbose ) printf( "  write access at address %x\n", eff_addr );
  assert( ( word_addr >= 0 ) && ( word_addr < MEM_SIZE_IN_WORDS ) );
  mem[ word_addr ] = reg[ reg_index ];
  memory_writes++;
}

/* extract fields - switch statements are in main loop */

void decode(){
  op1    = ( ir >> 26 ) & 0x3f;
  op2    = ( ir >> 10 ) & 0x3f;
  d      = ( ir >> 21 ) & 0x1f;
  s1     = ( ir >> 16 ) & 0x1f;
  s2     =   ir         & 0x1f;
  imm16  =   ir         & 0xffff;
  scaled = ( ir >>  9 ) & 1;
}

void halt(){
  if( verbose ) printf( "halt\n" );
  halt_flag = 1;
}

void imm_ld(){  /* pages 3-65 to 3-66 */
  if( verbose ) printf( "ld   r%x,r%x,%x\n", d, s1, imm16 );
  int address = reg[s1] + imm16;
  read_mem(address, d);

  cache_access(address, 0);
}

void imm_st(){  /* pages 3-79 to 3-80 */
  if( verbose ) printf( "st   r%x,r%x,%x\n", d, s1, imm16 );
  int address = reg[s1] + imm16;
  write_mem(address, d);

  cache_access(address, 1);
}

void imm_lda(){  /* pages 3-67 to 3-68 */
  if( verbose ) printf( "lda  r%x,r%x,%x\n", d, s1, imm16 );
  reg[d] = reg[s1] + imm16;
}

void imm_add(){  /* carry not used; pages 3-29 to 3-30 */
  if( verbose ) printf( "add  r%x,r%x,%x\n", d, s1, imm16 );
  reg[d] = reg[s1] + imm16;
}

void imm_sub(){  /* borrow not used; pages 3-82 to 3-83 */
  if( verbose ) printf( "sub  r%x,r%x,%x\n", d, s1, imm16 );
  reg[d] = reg[s1] - imm16;
}

void br(){  /* n = 0; * pages 3-16 and 3-37 */
    int d26 = ir & 0x03ffffff;
    assert(d26 != 0);
    if (verbose) printf("br %x", d26);
    d26 = d26 << 6;
    d26 = d26 >> 6;
    fip = xip + (d26 << 2);
    if (verbose) {
        if ((d26 < 0) || (d26 > 9)) {
            printf(" (= decimal %d)\n", d26);
        }
        else {
            printf("\n");
        }
    }
    branches++;
    taken_branches++;
}

void bcnd(){  /* n = 0; pages 3-13 to 3-14 and 3-35 to 3-36 */
    int d16 = ir & 0x0000ffff;
    assert(d16 != 0);

    int sign = ((unsigned int) reg[s1]) >> 31;
    int zero = (((unsigned int) reg[s1] << 1) == 0);
    int flag = (sign << 1) | zero;
    
    switch (d) {
        case 0x2: if(verbose) printf("bcnd eq0,r%d,%x", s1, d16);     break;
        case 0xd: if(verbose) printf("bcnd ne0,r%d,%x", s1, d16);     break;
        case 0x1: if(verbose) printf("bcnd gt0,r%d,%x", s1, d16);     break;
        case 0xc: if(verbose) printf("bcnd lt0,r%d,%x", s1, d16);     break;
        case 0x3: if(verbose) printf("bcnd ge0,r%d,%x", s1, d16);     break;
        case 0xe: if(verbose) printf("bcnd le0,r%d,%x", s1, d16);     break;
	case 0x8: if(verbose) printf("bcnd mask=8,r%d,%x", s1, d16);  break;
	case 0xf: if(verbose) printf("bcnd always,r%d,%x", s1, d16);  break;
	case 0:   if(verbose) printf("bcnd never,r%d,%x", s1, d16);   break;
    }

    branches++;

    d16 = d16 << 16;
    d16 = d16 >> 16;
    if(verbose){
	    if(0 <= d16)printf("\n");
	    else if(0 >= d16) printf(" (= decimal %d)\n", d16);
    }

    if ((1 & ((unsigned int)d >> flag)) == 1) {
        fip = xip + (d16 << 2);
	taken_branches++;
    }
}

void ext(){  /* immediate form, w5 = 0: pages 3-25 and 3-46 */
  if( verbose ) printf( "ext  r%x,r%x,%x\n", d, s1, s2 );
  reg[d] = reg[s1] >> s2;
}

void extu(){  /* immediate form, w5 = 0: pages 3-25 and 3-47 */
  if( verbose ) printf( "extu r%x,r%x,%x\n", d, s1, s2 );
  unsigned int u = (unsigned int)reg[s1];
  u = u >> s2;
  reg[d] = u;
}

void mak(){  /* immediate form, w5 = 0: pages 3-26 and 3-70 to 3-71 */
  if( verbose ) printf( "mak  r%x,r%x,%x\n", d, s1, s2 );
  reg[d] = reg[s1] << s2;
}

/* for rotate
 *
 *   (32-n) bits   n bits
 * +-------------+-------+
 * |      A      |   B   |
 * +-------------+-------+
 *
 * value 1 = AB shift left (32-n) bits
 * +-------+-------------+
 * |   B   |      0      |
 * +-------+-------------+
 *
 * value 2 = AB logical shift right n bits
 * +-------+-------------+
 * |   0   |      A      |
 * +-------+-------------+
 *
 * now or the two values together
 * +-------+-------------+
 * |   B   |      A      |
 * +-------+-------------+
 */
void rot(){  /* to the right, immediate form; pages 3-26 and 3-76 */
  if( verbose ) printf( "rot  r%x,r%x,%x\n", d, s1, s2 );
  reg[d] = (reg[s1] << (32 - s2)) | (reg[s1] >> s2);
}

void ld(){  /* pages 3-65 to 3-66 */
  if( scaled ){
    if( verbose ) printf( "ld   r%x,r%x[r%x]\n", d, s1, s2 );
    int address = (reg[s1] + (reg[s2] << 2));
    read_mem(address, d);

    cache_access(address, 0);
  }else{
    if( verbose ) printf( "ld   r%x,r%x,r%x\n", d, s1, s2 );
    int address = reg[s1] + reg[s2];
    read_mem(address, d);

    cache_access(address, 0);
  }
}

void st(){  /* pages 3-79 to 3-80 */
  if( scaled ){
    if( verbose ) printf( "st   r%x,r%x[r%x]\n", d, s1, s2 );
    int address = (reg[s1] + (reg[s2] << 2));
    write_mem(address, d);

    cache_access(address, 1);
  }else{
    if( verbose ) printf( "st   r%x,r%x,r%x\n", d, s1, s2 );
    int address = reg[s1] + reg[s2];
    write_mem(address, d);

    cache_access(address, 1);
  }
}

void lda(){  /* pages 3-67 to 3-68 */
  if( scaled ){
    if( verbose ) printf( "lda  r%x,r%x[r%x]\n", d, s1, s2 );
    //int address = (reg[s1] + (reg[s2] << 2));
    reg[d] = (reg[s1] + (reg[s2] << 2));
  }else{
    if( verbose ) printf( "lda  r%x,r%x,r%x\n", d, s1, s2 );
    //int address = reg[s1] + reg[s2];
    reg[d] = reg[s1] + reg[s2];
  }
}

void add(){  /* carry not used; pages 3-29 to 3-30 */
  if( verbose ) printf( "add  r%x,r%x,r%x\n", d, s1, s2 );
  reg[d] = reg[s1] + reg[s2];
}

void sub(){  /* borrow not used; pages 3-82 to 3-83 */
  if( verbose ) printf( "sub  r%x,r%x,r%x\n", d, s1, s2 );
  reg[d] = reg[s1] - reg[s2];
}

void unknown_op(){
  printf( "unknown instruction %08x\n", ir );
  printf( " op1=%x",  op1 );
  printf( " op2=%x",  op2 );
  printf( " d=%x",    d );
  printf( " s1=%x",   s1 );
  printf( " s2=%x\n", s2 );
  printf( "program terminates\n" );
  exit( -1 );
}


int main( int argc, char **argv ){

  if( argc > 1 ){
    if( ( argv[1][0] == '-' ) && ( argv[1][1] == 't' ) ){
      verbose = 1;
    }else if( ( argv[1][0] == '-' ) && ( argv[1][1] == 'v' ) ){
      verbose = 2;
    }else{
      printf( "usage:\n");
      printf( "  %s for just execution statistics\n", argv[0] );
      printf( "  %s -t for instruction trace\n", argv[0] );
      printf( "  %s -v for instructions, registers, and memory\n", argv[0] );
      printf( "input is read as hex 32-bit values from stdin\n" );
      exit( -1 );
    }
  }

  get_mem();
  cache_init();

  if( verbose ) printf( "instruction trace:\n" );
  while( !halt_flag ){

    if( verbose ) printf( "at %02x, ", fip );
    ir = mem[ fip >> 2 ];  /* adjust for word addressing of mem[] */
    xip = fip;
    fip = xip + 4;
    inst_fetches++;

    decode();

    switch( op1 ){
      case 0x00:        halt();      break;
      case 0x05:        imm_ld();    break;
      case 0x09:        imm_st();    break;
      case 0x0d:        imm_lda();   break;
      case 0x1c:        imm_add();   break;
      case 0x1d:        imm_sub();   break;
      case 0x30:        br();        break;
      case 0x3a:        bcnd();      break;
      case 0x3c:
        switch( op2 ){
          case 0x24:    ext();       break;
          case 0x26:    extu();      break;
          case 0x28:    mak();       break;
          case 0x2a:    rot();       break;
          default:      unknown_op();
        }
        break;
      case 0x3d:
        switch( op2 ){
          case 0x05:    ld();        break;
          case 0x09:    st();        break;
	  case 0x0d:    lda();       break;
          case 0x1c:    add();       break;
          case 0x1d:    sub();       break;
          default:      unknown_op();
        }
        break;
      default:          unknown_op();
    }

    reg[ 0 ] = 0;  /* make sure that r0 stays 0 */

    if( ( verbose > 1 ) || ( halt_flag && ( verbose == 1 )) ){
      for( int i = 0; i < 8 ; i++ ){
        printf( "  r%x: %08x", i , reg[ i ] );
        printf( "  r%x: %08x", i + 8 , reg[ i + 8 ] );
        printf( "  r%x: %08x", i + 16, reg[ i + 16 ] );
        printf( "  r%x: %08x\n", i + 24, reg[ i + 24 ] );
      }
    }
  }

  if( verbose ) printf( "\n" );
  printf( "execution statistics (in decimal):\n" );
  printf( "  instruction fetches = %d\n", inst_fetches );
  printf( "  data words read     = %d\n", memory_reads );
  printf( "  data words written  = %d\n", memory_writes );
  printf( "  branches executed   = %d\n", branches );
  if( taken_branches == 0 ){
    printf( "  branches taken      = 0\n" );
  }else{
    printf( "  branches taken      = %d (%.1f%%)\n",
      taken_branches, 100.0*((float)taken_branches)/((float)branches) );
  }
  cache_stats();
  return 0;
}
