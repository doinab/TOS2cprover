#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 38 "/usr/local/mspgcc/msp430/include/sys/inttypes.h"
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef int int16_t;
typedef unsigned int uint16_t;

typedef long int32_t;
typedef unsigned long uint32_t;

typedef long long int64_t;
typedef unsigned long long uint64_t;




typedef int16_t intptr_t;
typedef uint16_t uintptr_t;
# 151 "/usr/local/mspgcc/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 41 "/usr/local/mspgcc/msp430/include/sys/types.h"
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 64 "/usr/local/mspgcc/msp430/include/string.h"
extern void bzero(void *, size_t );
# 56 "/usr/local/mspgcc/msp430/include/stdlib.h"
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;




typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;
# 122 "/usr/local/mspgcc/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/local/mspgcc/msp430/include/sys/_types.h"
typedef long _off_t;
typedef long _ssize_t;
# 28 "/usr/local/mspgcc/msp430/include/sys/reent.h" 3
typedef __uint32_t __ULong;


struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int );




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/local/mspgcc/msp430/include/math.h"
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 208
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 261
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 91 "C:/cygwin/opt/tinyos-1.x/tos/system/tos.h"
typedef unsigned char bool;






enum __nesc_unnamed4247 {
  FALSE = 0, 
  TRUE = 1
};

uint16_t TOS_LOCAL_ADDRESS = 1;

enum __nesc_unnamed4248 {
  FAIL = 0, 
  SUCCESS = 1
};
static inline 

uint8_t rcombine(uint8_t r1, uint8_t r2);
typedef uint8_t  result_t;
static inline 






result_t rcombine(result_t r1, result_t r2);
static inline 
#line 133
result_t rcombine4(result_t r1, result_t r2, result_t r3, 
result_t r4);





enum __nesc_unnamed4249 {
  NULL = 0x0
};
# 27 "/usr/local/mspgcc/msp430/include/msp430/iostructures.h"
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4250 {
    unsigned char __p0 : 1, 
    __p1 : 1, 
    __p2 : 1, 
    __p3 : 1, 
    __p4 : 1, 
    __p5 : 1, 
    __p6 : 1, 
    __p7 : 1;
  } __pin;
} __attribute((packed))  ioregister_t;
# 106 "/usr/local/mspgcc/msp430/include/msp430/iostructures.h" 3
struct port_full_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t ifg;
  ioregister_t ies;
  ioregister_t ie;
  ioregister_t sel;
};




struct port_simple_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct port_full_t;



struct port_full_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;
# 109 "/usr/local/mspgcc/msp430/include/msp430/gpio.h" 3
volatile unsigned char P1IES __asm ("0x0024");
# 85 "/usr/local/mspgcc/msp430/include/msp430/usart.h"
volatile unsigned char U0CTL __asm ("0x0070");

volatile unsigned char U0TCTL __asm ("0x0071");



volatile unsigned char U0MCTL __asm ("0x0073");

volatile unsigned char U0BR0 __asm ("0x0074");

volatile unsigned char U0BR1 __asm ("0x0075");

volatile unsigned char U0RXBUF __asm ("0x0076");
# 253 "/usr/local/mspgcc/msp430/include/msp430/usart.h" 3
volatile unsigned char U1CTL __asm ("0x0078");

volatile unsigned char U1TCTL __asm ("0x0079");

volatile unsigned char U1RCTL __asm ("0x007A");

volatile unsigned char U1MCTL __asm ("0x007B");

volatile unsigned char U1BR0 __asm ("0x007C");

volatile unsigned char U1BR1 __asm ("0x007D");

volatile unsigned char U1RXBUF __asm ("0x007E");
# 22 "/usr/local/mspgcc/msp430/include/msp430/timera.h"
volatile unsigned int TAIV __asm ("0x012E");

volatile unsigned int TACTL __asm ("0x0160");







volatile unsigned int TAR __asm ("0x0170");
# 85 "/usr/local/mspgcc/msp430/include/msp430/timera.h" 3
typedef struct __nesc_unnamed4251 {
  volatile unsigned 
  taifg : 1, 
  taie : 1, 
  taclr : 1, 
  dummy : 1, 
  tamc : 2, 
  taid : 2, 
  tassel : 2;
} __attribute((packed))  tactl_t;

typedef struct __nesc_unnamed4252 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  dummy : 1, 
  scci : 1, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tacctl_t;


struct timera_t {
  tactl_t ctl;
  tacctl_t cctl0;
  tacctl_t cctl1;
  tacctl_t cctl2;
  volatile unsigned dummy[4];
  volatile unsigned tar;
  volatile unsigned taccr0;
  volatile unsigned taccr1;
  volatile unsigned taccr2;
};



struct timera_t;
# 22 "/usr/local/mspgcc/msp430/include/msp430/timerb.h"
volatile unsigned int TBIV __asm ("0x011E");

volatile unsigned int TBCTL __asm ("0x0180");

volatile unsigned int TBCCTL0 __asm ("0x0182");





volatile unsigned int TBR __asm ("0x0190");

volatile unsigned int TBCCR0 __asm ("0x0192");
#line 63
typedef struct __nesc_unnamed4253 {
  volatile unsigned 
  tbifg : 1, 
  tbie : 1, 
  tbclr : 1, 
  dummy1 : 1, 
  tbmc : 2, 
  tbid : 2, 
  tbssel : 2, 
  dummy2 : 1, 
  tbcntl : 2, 
  tbclgrp : 2;
} __attribute((packed))  tbctl_t;

typedef struct __nesc_unnamed4254 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  clld : 2, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tbcctl_t;


struct timerb_t {
  tbctl_t ctl;
  tbcctl_t cctl0;
  tbcctl_t cctl1;
  tbcctl_t cctl2;

  tbcctl_t cctl3;
  tbcctl_t cctl4;
  tbcctl_t cctl5;
  tbcctl_t cctl6;



  volatile unsigned tbr;
  volatile unsigned tbccr0;
  volatile unsigned tbccr1;
  volatile unsigned tbccr2;

  volatile unsigned tbccr3;
  volatile unsigned tbccr4;
  volatile unsigned tbccr5;
  volatile unsigned tbccr6;
};





struct timerb_t;
# 18 "/usr/local/mspgcc/msp430/include/msp430/basic_clock.h"
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 22 "/usr/local/mspgcc/msp430/include/msp430/flash.h"
volatile unsigned int FCTL3 __asm ("0x012C");
# 18 "/usr/local/mspgcc/msp430/include/msp430/adc12.h"
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");









typedef struct __nesc_unnamed4255 {
  volatile unsigned 
  adc12sc : 1, 
  enc : 1, 
  adc12tovie : 1, 
  adc12ovie : 1, 
  adc12on : 1, 
  refon : 1, 
  r2_5v : 1, 
  msc : 1, 
  sht0 : 4, 
  sht1 : 4;
} __attribute((packed))  adc12ctl0_t;

typedef struct __nesc_unnamed4256 {
  volatile unsigned 
  adc12busy : 1, 
  conseq : 2, 
  adc12ssel : 2, 
  adc12div : 3, 
  issh : 1, 
  shp : 1, 
  shs : 2, 
  cstartadd : 4;
} __attribute((packed))  adc12ctl1_t;

typedef struct __nesc_unnamed4257 {
  volatile unsigned 
  bit0 : 1, 
  bit1 : 1, 
  bit2 : 1, 
  bit3 : 1, 
  bit4 : 1, 
  bit5 : 1, 
  bit6 : 1, 
  bit7 : 1, 
  bit8 : 1, 
  bit9 : 1, 
  bit10 : 1, 
  bit11 : 1, 
  bit12 : 1, 
  bit13 : 1, 
  bit14 : 1, 
  bit15 : 1;
} __attribute((packed))  adc12xflg_t;


struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};




struct adc12_t;
# 48 "/usr/local/mspgcc/msp430/include/msp430x14x.h"
volatile unsigned char IFG1 __asm ("0x0002");







volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char IE2 __asm ("0x0001");









volatile unsigned char ME2 __asm ("0x0005");
static 
# 161 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
__inline void TOSH_wait(void );
static 
#line 174
__inline void TOSH_uwait(uint16_t u);
static inline 
#line 196
void __nesc_disable_interrupt(void);
static inline 




void __nesc_enable_interrupt(void);
static inline 



bool are_interrupts_enabled(void);




typedef bool __nesc_atomic_t;
static inline 
__nesc_atomic_t __nesc_atomic_start(void );
static inline void __nesc_atomic_end(__nesc_atomic_t oldSreg);
static inline 


__nesc_atomic_t __nesc_atomic_start(void );
static inline 





void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);
 







bool LPMode_disabled = FALSE;
static 








__inline void __nesc_atomic_sleep(void);
# 105 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12.h"
typedef struct __nesc_unnamed4258 {

  unsigned int refVolt2_5 : 1;
  unsigned int clockSourceSHT : 2;
  unsigned int clockSourceSAMPCON : 2;
  unsigned int clockDivSAMPCON : 2;
  unsigned int referenceVoltage : 3;
  unsigned int clockDivSHT : 3;
  unsigned int inputChannel : 4;
  unsigned int sampleHoldTime : 4;
  unsigned int  : 0;
} MSP430ADC12Settings_t;

typedef enum __nesc_unnamed4259 {

  MSP430ADC12_FAIL = 0, 
  MSP430ADC12_SUCCESS = 1, 
  MSP430ADC12_DELAYED = 2
} msp430ADCresult_t;

enum refVolt2_5_enum {

  REFVOLT_LEVEL_1_5 = 0, 
  REFVOLT_LEVEL_2_5 = 1
};

enum clockDivSHT_enum {

  SHT_CLOCK_DIV_1 = 0, 
  SHT_CLOCK_DIV_2 = 1, 
  SHT_CLOCK_DIV_3 = 2, 
  SHT_CLOCK_DIV_4 = 3, 
  SHT_CLOCK_DIV_5 = 4, 
  SHT_CLOCK_DIV_6 = 5, 
  SHT_CLOCK_DIV_7 = 6, 
  SHT_CLOCK_DIV_8 = 7
};

enum clockDivSAMPCON_enum {

  SAMPCON_CLOCK_DIV_1 = 0, 
  SAMPCON_CLOCK_DIV_2 = 1, 
  SAMPCON_CLOCK_DIV_3 = 2, 
  SAMPCON_CLOCK_DIV_4 = 3
};

enum clockSourceSAMPCON_enum {

  SAMPCON_SOURCE_TACLK = 0, 
  SAMPCON_SOURCE_ACLK = 1, 
  SAMPCON_SOURCE_SMCLK = 2, 
  SAMPCON_SOURCE_INCLK = 3
};

enum inputChannel_enum {


  INPUT_CHANNEL_A0 = 0, 
  INPUT_CHANNEL_A1 = 1, 
  INPUT_CHANNEL_A2 = 2, 
  INPUT_CHANNEL_A3 = 3, 
  INPUT_CHANNEL_A4 = 4, 
  INPUT_CHANNEL_A5 = 5, 
  INPUT_CHANNEL_A6 = 6, 
  INPUT_CHANNEL_A7 = 7, 
  EXTERNAL_REFERENCE_VOLTAGE = 8, 
  REFERENCE_VOLTAGE_NEGATIVE_TERMINAL = 9, 
  INTERNAL_TEMPERATURE = 10, 
  INTERNAL_VOLTAGE = 11
};

enum referenceVoltage_enum {

  REFERENCE_AVcc_AVss = 0, 
  REFERENCE_VREFplus_AVss = 1, 
  REFERENCE_VeREFplus_AVss = 2, 
  REFERENCE_AVcc_VREFnegterm = 4, 
  REFERENCE_VREFplus_VREFnegterm = 5, 
  REFERENCE_VeREFplus_VREFnegterm = 6
};

enum clockSourceSHT_enum {

  SHT_SOURCE_ADC12OSC = 0, 
  SHT_SOURCE_ACLK = 1, 
  SHT_SOURCE_MCLK = 2, 
  SHT_SOURCE_SMCLK = 3
};

enum sampleHold_enum {

  SAMPLE_HOLD_4_CYCLES = 0, 
  SAMPLE_HOLD_8_CYCLES = 1, 
  SAMPLE_HOLD_16_CYCLES = 2, 
  SAMPLE_HOLD_32_CYCLES = 3, 
  SAMPLE_HOLD_64_CYCLES = 4, 
  SAMPLE_HOLD_96_CYCLES = 5, 
  SAMPLE_HOLD_123_CYCLES = 6, 
  SAMPLE_HOLD_192_CYCLES = 7, 
  SAMPLE_HOLD_256_CYCLES = 8, 
  SAMPLE_HOLD_384_CYCLES = 9, 
  SAMPLE_HOLD_512_CYCLES = 10, 
  SAMPLE_HOLD_768_CYCLES = 11, 
  SAMPLE_HOLD_1024_CYCLES = 12
};






typedef union __nesc_unnamed4260 {
  uint32_t i;
  MSP430ADC12Settings_t s;
} MSP430ADC12Settings_ut;








enum __nesc_unnamed4261 {

  ADC_IDLE = 0, 
  SINGLE_CHANNEL = 1, 
  REPEAT_SINGLE_CHANNEL = 2, 
  SEQUENCE_OF_CHANNELS = 4, 
  REPEAT_SEQUENCE_OF_CHANNELS = 8, 
  TIMER_USED = 16, 
  RESERVED = 32, 
  VREF_WAIT = 64
};
#line 255
typedef struct __nesc_unnamed4262 {

  volatile unsigned 
  inch : 4, 
  sref : 3, 
  eos : 1;
} __attribute((packed))  adc12memctl_t;

typedef struct __nesc_unnamed4263 {

  unsigned int refVolt2_5 : 1;
  unsigned int gotRefVolt : 1;
  unsigned int result_16bit : 1;
  unsigned int clockSourceSHT : 2;
  unsigned int clockSourceSAMPCON : 2;
  unsigned int clockDivSAMPCON : 2;
  unsigned int clockDivSHT : 3;
  unsigned int sampleHoldTime : 4;
  adc12memctl_t memctl;
} __attribute((packed))  adc12settings_t;
# 58 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420Const.h"
enum __nesc_unnamed4264 {
  CC2420_TIME_BIT = 4, 
  CC2420_TIME_BYTE = CC2420_TIME_BIT << 3, 
  CC2420_TIME_SYMBOL = 16
};









enum __nesc_unnamed4265 {
  CC2420_MIN_CHANNEL = 11, 
  CC2420_MAX_CHANNEL = 26
};
#line 257
enum __nesc_unnamed4266 {
  CP_MAIN = 0, 
  CP_MDMCTRL0, 
  CP_MDMCTRL1, 
  CP_RSSI, 
  CP_SYNCWORD, 
  CP_TXCTRL, 
  CP_RXCTRL0, 
  CP_RXCTRL1, 
  CP_FSCTRL, 
  CP_SECCTRL0, 
  CP_SECCTRL1, 
  CP_BATTMON, 
  CP_IOCFG0, 
  CP_IOCFG1
};
# 46 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/AM.h"
enum __nesc_unnamed4267 {
  TOS_BCAST_ADDR = 0xffff, 
  TOS_UART_ADDR = 0x007e
};





enum __nesc_unnamed4268 {
  TOS_DEFAULT_AM_GROUP = 0x66
};

uint8_t TOS_AM_GROUP = TOS_DEFAULT_AM_GROUP;
#line 74
typedef struct TOS_Msg {


  uint8_t length;
  uint8_t fcfhi;
  uint8_t fcflo;
  uint8_t dsn;
  uint16_t destpan;
  uint16_t addr;
  uint8_t type;
  uint8_t group;
  int8_t data[66];







  uint8_t strength;
  uint8_t lqi;
  bool crc;
  bool ack;
  uint16_t time;
} __attribute((packed))  TOS_Msg;

enum __nesc_unnamed4269 {

  MSG_HEADER_SIZE = (size_t )& ((struct TOS_Msg *)0)->data - 1, 

  MSG_FOOTER_SIZE = 2, 

  MSG_DATA_SIZE = (size_t )& ((struct TOS_Msg *)0)->strength + sizeof(uint16_t ), 

  DATA_LENGTH = 66, 

  LENGTH_BYTE_NUMBER = (size_t )& ((struct TOS_Msg *)0)->length + 1
};

typedef TOS_Msg *TOS_MsgPtr;
static inline 
uint8_t TOS_MsgLength(uint8_t type);
static inline 
# 11 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
void TOSH_SET_RED_LED_PIN(void);
static inline 
#line 11
void TOSH_CLR_RED_LED_PIN(void);
static inline 
#line 11
void TOSH_MAKE_RED_LED_OUTPUT(void);
static inline void TOSH_SET_GREEN_LED_PIN(void);
static inline 
#line 12
void TOSH_CLR_GREEN_LED_PIN(void);
static inline 
#line 12
void TOSH_MAKE_GREEN_LED_OUTPUT(void);
static inline void TOSH_SET_YELLOW_LED_PIN(void);
static inline 
#line 13
void TOSH_CLR_YELLOW_LED_PIN(void);
static inline 
#line 13
void TOSH_MAKE_YELLOW_LED_OUTPUT(void);
static inline 

void TOSH_SET_RADIO_CSN_PIN(void);
static inline 
#line 16
void TOSH_CLR_RADIO_CSN_PIN(void);
static inline 
#line 16
void TOSH_MAKE_RADIO_CSN_OUTPUT(void);
static inline void TOSH_CLR_RADIO_VREF_PIN(void);
static inline 
#line 17
void TOSH_MAKE_RADIO_VREF_OUTPUT(void);
static inline void TOSH_SET_RADIO_RESET_PIN(void);
static inline 
#line 18
void TOSH_MAKE_RADIO_RESET_OUTPUT(void);
static inline void TOSH_MAKE_RADIO_FIFOP_INPUT(void);
static inline void TOSH_MAKE_RADIO_SFD_INPUT(void);
static inline void TOSH_MAKE_RADIO_GIO0_INPUT(void);
static inline 
void TOSH_MAKE_RADIO_GIO1_INPUT(void);
static inline uint8_t TOSH_READ_RADIO_CCA_PIN(void);
static inline 
uint8_t TOSH_READ_CC_FIFOP_PIN(void);
static inline uint8_t TOSH_READ_CC_FIFO_PIN(void);
static inline uint8_t TOSH_READ_CC_SFD_PIN(void);
static inline 
#line 28
void TOSH_SEL_CC_SFD_MODFUNC(void);
static inline 
#line 28
void TOSH_SEL_CC_SFD_IOFUNC(void);
static inline void TOSH_SET_CC_VREN_PIN(void);
static inline void TOSH_SET_CC_RSTN_PIN(void);
static inline 
#line 30
void TOSH_CLR_CC_RSTN_PIN(void);
static inline 

void TOSH_MAKE_SOMI0_INPUT(void);
static inline 
#line 33
void TOSH_SEL_SOMI0_MODFUNC(void);
static inline void TOSH_MAKE_SIMO0_INPUT(void);
static inline 
#line 34
void TOSH_SEL_SIMO0_MODFUNC(void);
static inline void TOSH_MAKE_UCLK0_INPUT(void);
static inline 
#line 35
void TOSH_SEL_UCLK0_MODFUNC(void);
static inline void TOSH_MAKE_UTXD0_INPUT(void);
static inline 
#line 36
void TOSH_SEL_UTXD0_IOFUNC(void);
static inline 
#line 36
bool TOSH_IS_UTXD0_MODFUNC(void);
static inline 
#line 36
bool TOSH_IS_UTXD0_IOFUNC(void);
static inline void TOSH_MAKE_URXD0_INPUT(void);
static inline 
#line 37
void TOSH_SEL_URXD0_IOFUNC(void);
static inline 
#line 37
bool TOSH_IS_URXD0_MODFUNC(void);
static inline 
#line 37
bool TOSH_IS_URXD0_IOFUNC(void);
static inline void TOSH_MAKE_UTXD1_INPUT(void);
static inline 
#line 38
void TOSH_SEL_UTXD1_MODFUNC(void);
static inline 
#line 38
void TOSH_SEL_UTXD1_IOFUNC(void);
static inline 
#line 38
bool TOSH_IS_UTXD1_MODFUNC(void);
static inline 
#line 38
bool TOSH_IS_UTXD1_IOFUNC(void);
static inline void TOSH_MAKE_URXD1_INPUT(void);
static inline 
#line 39
void TOSH_SEL_URXD1_MODFUNC(void);
static inline 
#line 39
void TOSH_SEL_URXD1_IOFUNC(void);
static inline 
#line 39
bool TOSH_IS_URXD1_MODFUNC(void);
static inline 
#line 39
bool TOSH_IS_URXD1_IOFUNC(void);
static inline void TOSH_MAKE_UCLK1_INPUT(void);
static inline 
#line 40
void TOSH_SEL_UCLK1_IOFUNC(void);
static inline void TOSH_MAKE_SOMI1_INPUT(void);
static inline 
#line 41
void TOSH_SEL_SOMI1_IOFUNC(void);
static inline void TOSH_MAKE_SIMO1_INPUT(void);
static inline 
#line 42
void TOSH_SEL_SIMO1_IOFUNC(void);
static inline 
#line 56
void TOSH_CLR_HUM_SDA_PIN(void);
static inline 
#line 56
void TOSH_MAKE_HUM_SDA_OUTPUT(void);
static inline void TOSH_CLR_HUM_SCL_PIN(void);
static inline 
#line 57
void TOSH_MAKE_HUM_SCL_OUTPUT(void);
static inline void TOSH_CLR_HUM_PWR_PIN(void);
static inline 
#line 58
void TOSH_MAKE_HUM_PWR_OUTPUT(void);
#line 77
enum __nesc_unnamed4270 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};
static inline 

void TOSH_SET_FLASH_PWR_PIN(void);
static inline 
#line 85
void TOSH_MAKE_FLASH_PWR_OUTPUT(void);
static inline void TOSH_SET_FLASH_CS_PIN(void);
static inline 
#line 86
void TOSH_MAKE_FLASH_CS_OUTPUT(void);
static inline 


void TOSH_MAKE_PROG_RX_INPUT(void);
static inline void TOSH_MAKE_PROG_TX_INPUT(void);
static inline 
void TOSH_SET_PIN_DIRECTIONS(void );
# 54 "C:/cygwin/opt/tinyos-1.x/tos/types/dbg_modes.h"
typedef long long TOS_dbg_mode;



enum __nesc_unnamed4271 {
  DBG_ALL = ~0ULL, 


  DBG_BOOT = 1ULL << 0, 
  DBG_CLOCK = 1ULL << 1, 
  DBG_TASK = 1ULL << 2, 
  DBG_SCHED = 1ULL << 3, 
  DBG_SENSOR = 1ULL << 4, 
  DBG_LED = 1ULL << 5, 
  DBG_CRYPTO = 1ULL << 6, 


  DBG_ROUTE = 1ULL << 7, 
  DBG_AM = 1ULL << 8, 
  DBG_CRC = 1ULL << 9, 
  DBG_PACKET = 1ULL << 10, 
  DBG_ENCODE = 1ULL << 11, 
  DBG_RADIO = 1ULL << 12, 


  DBG_LOG = 1ULL << 13, 
  DBG_ADC = 1ULL << 14, 
  DBG_I2C = 1ULL << 15, 
  DBG_UART = 1ULL << 16, 
  DBG_PROG = 1ULL << 17, 
  DBG_SOUNDER = 1ULL << 18, 
  DBG_TIME = 1ULL << 19, 
  DBG_POWER = 1ULL << 20, 



  DBG_SIM = 1ULL << 21, 
  DBG_QUEUE = 1ULL << 22, 
  DBG_SIMRADIO = 1ULL << 23, 
  DBG_HARD = 1ULL << 24, 
  DBG_MEM = 1ULL << 25, 



  DBG_USR1 = 1ULL << 27, 
  DBG_USR2 = 1ULL << 28, 
  DBG_USR3 = 1ULL << 29, 
  DBG_TEMP = 1ULL << 30, 

  DBG_ERROR = 1ULL << 31, 
  DBG_NONE = 0, 

  DBG_DEFAULT = DBG_ALL
};
# 59 "C:/cygwin/opt/tinyos-1.x/tos/system/sched.c"
typedef struct __nesc_unnamed4272 {
  void (*tp)(void);
} TOSH_sched_entry_T;

enum __nesc_unnamed4273 {






  TOSH_MAX_TASKS = 8, 

  TOSH_TASK_BITMASK = TOSH_MAX_TASKS - 1
};

volatile TOSH_sched_entry_T TOSH_queue[TOSH_MAX_TASKS];
uint8_t TOSH_sched_full;
volatile uint8_t TOSH_sched_free;
static inline 
void TOSH_sched_init(void );








bool TOS_post(void (*tp)(void));
#line 102
bool  TOS_post(void (*tp)(void));
static inline 
#line 136
bool TOSH_run_next_task(void);
static inline 
#line 159
void TOSH_run_task(void);
static 
# 149 "C:/cygwin/opt/tinyos-1.x/tos/system/tos.h"
void *nmemcpy(void *to, const void *from, size_t n);
# 28 "C:/cygwin/opt/tinyos-1.x/tos/system/Ident.h"
enum __nesc_unnamed4274 {

  IDENT_MAX_PROGRAM_NAME_LENGTH = 16
};

typedef struct __nesc_unnamed4275 {

  uint32_t unix_time;
  uint32_t user_hash;
  char program_name[IDENT_MAX_PROGRAM_NAME_LENGTH];
} Ident_t;
# 9 "UbiMonMsg.h"
enum __nesc_unnamed4276 {
  BUFFER_SIZE = 30
};

struct UbiMonMsg {





  uint16_t sourceID;
  uint16_t destination;
  uint16_t BSNcommand;
  uint8_t signalstrength;
  uint8_t redStatus;
  uint8_t greenStatus;
  uint8_t yellowStatus;
  uint16_t data[28];
};

struct UbiMonResetMsg {
};



enum __nesc_unnamed4277 {
  AM_OSCOPEMSG = 10, 
  AM_OSCOPERESETMSG = 10
};
# 28 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.h"
enum __nesc_unnamed4278 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};

typedef struct __nesc_unnamed4279 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} MSP430CompareControl_t;

typedef struct __nesc_unnamed4280 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} MSP430TimerAControl_t;

typedef struct __nesc_unnamed4281 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} MSP430TimerBControl_t;
# 39 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Timer.h"
enum __nesc_unnamed4282 {
  TIMER_REPEAT = 0, 
  TIMER_ONE_SHOT = 1, 
  NUM_TIMERS = 2
};
static 
# 12 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/byteorder.h"
__inline int is_host_lsb(void);
static 




__inline uint16_t toLSB16(uint16_t a);
static 



__inline uint16_t fromLSB16(uint16_t a);
# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/msp430usart.h"
typedef enum __nesc_unnamed4283 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;
# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/crc.h"
uint16_t const ccitt_crc16_table[256] = { 
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 
0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 
0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 
0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 
0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 
0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 
0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 
0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 
0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 
0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 
0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 
0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 
0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 
0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 
0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 
0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 
0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 
0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 
0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 
0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 
0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 
0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 
0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 
0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 
0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 
0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 
0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 
0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 
0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 
0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 
0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 
0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };
static 

uint16_t crcByte(uint16_t fcs, uint8_t c);
# 19 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/msp430baudrates.h"
enum __nesc_unnamed4284 {

  UBR_ACLK_1200 = 0x001B, UMCTL_ACLK_1200 = 0x94, 
  UBR_ACLK_1800 = 0x0012, UMCTL_ACLK_1800 = 0x84, 
  UBR_ACLK_2400 = 0x000D, UMCTL_ACLK_2400 = 0x6D, 
  UBR_ACLK_4800 = 0x0006, UMCTL_ACLK_4800 = 0x77, 
  UBR_ACLK_9600 = 0x0003, UMCTL_ACLK_9600 = 0x29, 


  UBR_SMCLK_1200 = 0x0369, UMCTL_SMCLK_1200 = 0x7B, 
  UBR_SMCLK_1800 = 0x0246, UMCTL_SMCLK_1800 = 0x55, 
  UBR_SMCLK_2400 = 0x01B4, UMCTL_SMCLK_2400 = 0xDF, 
  UBR_SMCLK_4800 = 0x00DA, UMCTL_SMCLK_4800 = 0xAA, 
  UBR_SMCLK_9600 = 0x006D, UMCTL_SMCLK_9600 = 0x44, 
  UBR_SMCLK_19200 = 0x0036, UMCTL_SMCLK_19200 = 0xB5, 
  UBR_SMCLK_38400 = 0x001B, UMCTL_SMCLK_38400 = 0x94, 
  UBR_SMCLK_57600 = 0x0012, UMCTL_SMCLK_57600 = 0x84, 
  UBR_SMCLK_76800 = 0x000D, UMCTL_SMCLK_76800 = 0x6D, 
  UBR_SMCLK_115200 = 0x0009, UMCTL_SMCLK_115200 = 0x10, 
  UBR_SMCLK_230400 = 0x0004, UMCTL_SMCLK_230400 = 0x55, 
  UBR_SMCLK_262144 = 4, UMCTL_SMCLK_262144 = 0
};
static  result_t HPLInitM$init(void);
static  void MSP430ClockM$MSP430ClockInit$default$initTimerB(void);
static  void MSP430ClockM$MSP430ClockInit$defaultInitTimerA(void);
static  void MSP430ClockM$MSP430ClockInit$default$initTimerA(void);
static  void MSP430ClockM$MSP430ClockInit$defaultInitTimerB(void);
static  void MSP430ClockM$MSP430ClockInit$default$initClocks(void);
static  void MSP430ClockM$MSP430ClockInit$defaultInitClocks(void);
static  result_t MSP430ClockM$StdControl$init(void);
static  result_t MSP430ClockM$StdControl$start(void);
static   void MSP430DCOCalibM$Timer32khz$overflow(void);
static   void MSP430DCOCalibM$TimerMicro$overflow(void);
static   MSP430CompareControl_t MSP430TimerM$ControlA2$getControl(void);
static   MSP430CompareControl_t MSP430TimerM$ControlB0$getControl(void);
static   uint16_t MSP430TimerM$CaptureA1$getEvent(void);
static   void MSP430TimerM$CaptureA1$default$captured(uint16_t arg_0xa6b0618);
static   uint16_t MSP430TimerM$CaptureB3$getEvent(void);
static   void MSP430TimerM$CaptureB3$default$captured(uint16_t arg_0xa6b0618);
static   void MSP430TimerM$CompareA1$default$fired(void);
static   void MSP430TimerM$CompareB3$setEventFromNow(uint16_t arg_0xa6b4d98);
static   uint16_t MSP430TimerM$CaptureB6$getEvent(void);
static   void MSP430TimerM$CaptureB6$default$captured(uint16_t arg_0xa6b0618);
static   MSP430CompareControl_t MSP430TimerM$ControlB4$getControl(void);
static   void MSP430TimerM$ControlB4$enableEvents(void);
static   void MSP430TimerM$ControlB4$setControlAsCompare(void);
static   void MSP430TimerM$ControlB4$disableEvents(void);
static   void MSP430TimerM$ControlB4$clearPendingInterrupt(void);
static   MSP430CompareControl_t MSP430TimerM$ControlA0$getControl(void);
static   uint16_t MSP430TimerM$CaptureB1$getEvent(void);
static   void MSP430TimerM$CaptureB1$clearOverflow(void);
static   bool MSP430TimerM$CaptureB1$isOverflowPending(void);
static   void MSP430TimerM$CompareB1$default$fired(void);
static   void MSP430TimerM$ControlB1$setControlAsCapture(bool arg_0xa6bb190);
static   MSP430CompareControl_t MSP430TimerM$ControlB1$getControl(void);
static   void MSP430TimerM$ControlB1$enableEvents(void);
static   void MSP430TimerM$ControlB1$disableEvents(void);
static   void MSP430TimerM$ControlB1$clearPendingInterrupt(void);
static   uint16_t MSP430TimerM$CaptureA2$getEvent(void);
static   void MSP430TimerM$CaptureA2$default$captured(uint16_t arg_0xa6b0618);
static   uint16_t MSP430TimerM$CaptureB4$getEvent(void);
static   void MSP430TimerM$CaptureB4$default$captured(uint16_t arg_0xa6b0618);
static   MSP430CompareControl_t MSP430TimerM$ControlB2$getControl(void);
static   void MSP430TimerM$CompareA2$default$fired(void);
static   void MSP430TimerM$CompareB4$setEventFromNow(uint16_t arg_0xa6b4d98);
static   MSP430CompareControl_t MSP430TimerM$ControlA1$getControl(void);
static   MSP430CompareControl_t MSP430TimerM$ControlB5$getControl(void);
static   uint16_t MSP430TimerM$CaptureA0$getEvent(void);
static   void MSP430TimerM$CaptureA0$default$captured(uint16_t arg_0xa6b0618);
static   uint16_t MSP430TimerM$CaptureB2$getEvent(void);
static   void MSP430TimerM$CaptureB2$default$captured(uint16_t arg_0xa6b0618);
static   void MSP430TimerM$CompareA0$default$fired(void);
static   void MSP430TimerM$CompareB2$default$fired(void);
static   uint16_t MSP430TimerM$CaptureB5$getEvent(void);
static   void MSP430TimerM$CaptureB5$default$captured(uint16_t arg_0xa6b0618);
static   MSP430CompareControl_t MSP430TimerM$ControlB3$getControl(void);
static   void MSP430TimerM$ControlB3$enableEvents(void);
static   void MSP430TimerM$ControlB3$setControlAsCompare(void);
static   void MSP430TimerM$ControlB3$disableEvents(void);
static   void MSP430TimerM$ControlB3$clearPendingInterrupt(void);
static   uint16_t MSP430TimerM$TimerB$read(void);
static   bool MSP430TimerM$TimerB$isOverflowPending(void);
static   void MSP430TimerM$CompareB5$default$fired(void);
static   uint16_t MSP430TimerM$CaptureB0$getEvent(void);
static   void MSP430TimerM$CaptureB0$default$captured(uint16_t arg_0xa6b0618);
static   void MSP430TimerM$CompareB6$default$fired(void);
static   void MSP430TimerM$CompareB0$default$fired(void);
static   MSP430CompareControl_t MSP430TimerM$ControlB6$getControl(void);
static  result_t BSN_RadioRangeM$DataMsg$sendDone(TOS_MsgPtr arg_0xa7daac8, result_t arg_0xa7dac18);
static  TOS_MsgPtr BSN_RadioRangeM$ReceiveMsg$receive(TOS_MsgPtr arg_0xa7dbeb8);
static  TOS_MsgPtr BSN_RadioRangeM$UARTReceive$receive(TOS_MsgPtr arg_0xa7dbeb8);
static  result_t BSN_RadioRangeM$UARTSend$sendDone(TOS_MsgPtr arg_0xa7b3e40, result_t arg_0xa7d6010);
static  result_t BSN_RadioRangeM$StdControl$init(void);
static  result_t BSN_RadioRangeM$StdControl$start(void);
static  result_t BSN_RadioRangeM$Timer$fired(void);
static  result_t TimerM$TimerMilli$default$fired(uint8_t arg_0xa7c3da8);
static   uint32_t TimerM$LocalTime$read(void);
static   void TimerM$AlarmCompare$fired(void);
static  result_t TimerM$TimerJiffy$default$fired(uint8_t arg_0xa80a7e0);
static   void TimerM$AlarmTimer$overflow(void);
static  result_t TimerM$StdControl$init(void);
static  result_t TimerM$StdControl$start(void);
static  result_t TimerM$Timer$default$fired(uint8_t arg_0xa7c3720);
static  result_t TimerM$Timer$start(uint8_t arg_0xa7c3720, char arg_0xa7b9cd0, uint32_t arg_0xa7b9e28);
static  result_t TimerM$Timer$stop(uint8_t arg_0xa7c3720);
static   result_t LedsC$Leds$yellowOff(void);
static   result_t LedsC$Leds$yellowOn(void);
static   result_t LedsC$Leds$init(void);
static   result_t LedsC$Leds$greenOff(void);
static   result_t LedsC$Leds$redOff(void);
static   result_t LedsC$Leds$greenToggle(void);
static   result_t LedsC$Leds$redToggle(void);
static   result_t LedsC$Leds$redOn(void);
static   result_t LedsC$Leds$greenOn(void);
static  TOS_MsgPtr AMStandard$ReceiveMsg$default$receive(uint8_t arg_0xa855228, TOS_MsgPtr arg_0xa7dbeb8);
static  result_t AMStandard$ActivityTimer$fired(void);
static  result_t AMStandard$UARTSend$sendDone(TOS_MsgPtr arg_0xa7b3e40, result_t arg_0xa7d6010);
static  TOS_MsgPtr AMStandard$RadioReceive$receive(TOS_MsgPtr arg_0xa7dbeb8);
static  result_t AMStandard$Control$init(void);
static  result_t AMStandard$Control$start(void);
static  result_t AMStandard$default$sendDone(void);
static  result_t AMStandard$RadioSend$sendDone(TOS_MsgPtr arg_0xa7b3e40, result_t arg_0xa7d6010);
static  result_t AMStandard$SendMsg$send(uint8_t arg_0xa854c70, uint16_t arg_0xa7da410, uint8_t arg_0xa7da558, TOS_MsgPtr arg_0xa7da6a8);
static  result_t AMStandard$SendMsg$default$sendDone(uint8_t arg_0xa854c70, TOS_MsgPtr arg_0xa7daac8, result_t arg_0xa7dac18);
static  TOS_MsgPtr AMStandard$UARTReceive$receive(TOS_MsgPtr arg_0xa7dbeb8);
static  result_t CC2420RadioM$SplitControl$default$initDone(void);
static  result_t CC2420RadioM$SplitControl$init(void);
static  result_t CC2420RadioM$SplitControl$default$startDone(void);
static  result_t CC2420RadioM$SplitControl$start(void);
static   result_t CC2420RadioM$FIFOP$fired(void);
static   result_t CC2420RadioM$BackoffTimerJiffy$fired(void);
static  result_t CC2420RadioM$Send$send(TOS_MsgPtr arg_0xa7b3928);
static   void CC2420RadioM$RadioReceiveCoordinator$default$startSymbol(uint8_t arg_0xa88b540, uint8_t arg_0xa88b688, TOS_MsgPtr arg_0xa88b7d8);
static   result_t CC2420RadioM$SFD$captured(uint16_t arg_0xa8d6688);
static   void CC2420RadioM$RadioSendCoordinator$default$startSymbol(uint8_t arg_0xa88b540, uint8_t arg_0xa88b688, TOS_MsgPtr arg_0xa88b7d8);
static   result_t CC2420RadioM$HPLChipconFIFO$TXFIFODone(uint8_t arg_0xa8a6f20, uint8_t *arg_0xa8a7080);
static   result_t CC2420RadioM$HPLChipconFIFO$RXFIFODone(uint8_t arg_0xa8a6890, uint8_t *arg_0xa8a69f0);
static  result_t CC2420RadioM$StdControl$init(void);
static  result_t CC2420RadioM$StdControl$start(void);
static   int16_t CC2420RadioM$MacBackoff$default$initialBackoff(TOS_MsgPtr arg_0xa867eb0);
static   int16_t CC2420RadioM$MacBackoff$default$congestionBackoff(TOS_MsgPtr arg_0xa88a310);
static  result_t CC2420RadioM$CC2420SplitControl$initDone(void);
static  result_t CC2420RadioM$CC2420SplitControl$startDone(void);
static  result_t CC2420ControlM$SplitControl$init(void);
static  result_t CC2420ControlM$SplitControl$start(void);
static   result_t CC2420ControlM$CCA$fired(void);
static   result_t CC2420ControlM$HPLChipconRAM$writeDone(uint16_t arg_0xa906738, uint8_t arg_0xa906880, uint8_t *arg_0xa9069e0);
static   result_t CC2420ControlM$CC2420Control$VREFOn(void);
static   result_t CC2420ControlM$CC2420Control$RxMode(void);
static  result_t CC2420ControlM$CC2420Control$TuneManual(uint16_t arg_0xa89b1a8);
static  result_t CC2420ControlM$CC2420Control$setShortAddress(uint16_t arg_0xa892cc8);
static   result_t CC2420ControlM$CC2420Control$OscillatorOn(void);
static   uint16_t HPLCC2420M$HPLCC2420$read(uint8_t arg_0xa8aa798);
static   uint8_t HPLCC2420M$HPLCC2420$write(uint8_t arg_0xa8aa1a0, uint16_t arg_0xa8aa2f0);
static   uint8_t HPLCC2420M$HPLCC2420$cmd(uint8_t arg_0xa8b5d30);
static   result_t HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(uint8_t arg_0xa8a6210, uint8_t *arg_0xa8a6370);
static   result_t HPLCC2420M$HPLCC2420FIFO$readRXFIFO(uint8_t arg_0xa8abab8, uint8_t *arg_0xa8abc18);
static   result_t HPLCC2420M$HPLCC2420RAM$write(uint16_t arg_0xa906010, uint8_t arg_0xa906158, uint8_t *arg_0xa9062b8);
static  result_t HPLCC2420M$StdControl$init(void);
static  result_t HPLCC2420M$StdControl$start(void);
static  result_t HPLCC2420M$BusArbitration$busFree(void);
static   void HPLUSART0M$HPLI2CInterrupt$default$fired(void);
static   result_t HPLUSART0M$USARTData$default$rxDone(uint8_t arg_0xa982da8);
static   result_t HPLUSART0M$USARTData$default$txDone(void);
static   result_t HPLUSART0M$USARTControl$isTxEmpty(void);
static   bool HPLUSART0M$USARTControl$isSPI(void);
static   void HPLUSART0M$USARTControl$disableUART(void);
static   bool HPLUSART0M$USARTControl$isUART(void);
static   bool HPLUSART0M$USARTControl$isI2C(void);
static   result_t HPLUSART0M$USARTControl$disableRxIntr(void);
static   result_t HPLUSART0M$USARTControl$disableTxIntr(void);
static   bool HPLUSART0M$USARTControl$isUARTtx(void);
static   void HPLUSART0M$USARTControl$disableI2C(void);
static   void HPLUSART0M$USARTControl$setModeSPI(void);
static   msp430_usartmode_t HPLUSART0M$USARTControl$getMode(void);
static   result_t HPLUSART0M$USARTControl$isTxIntrPending(void);
static   result_t HPLUSART0M$USARTControl$tx(uint8_t arg_0xa96ff30);
static   uint8_t HPLUSART0M$USARTControl$rx(void);
static   result_t HPLUSART0M$USARTControl$isRxIntrPending(void);
static   bool HPLUSART0M$USARTControl$isUARTrx(void);
static   result_t HPLCC2420InterruptM$FIFO$default$fired(void);
static   result_t HPLCC2420InterruptM$FIFOP$disable(void);
static   result_t HPLCC2420InterruptM$FIFOP$startWait(bool arg_0xa8da5c0);
static   void HPLCC2420InterruptM$CCAInterrupt$fired(void);
static   void HPLCC2420InterruptM$FIFOInterrupt$fired(void);
static   result_t HPLCC2420InterruptM$CCA$startWait(bool arg_0xa8da5c0);
static   void HPLCC2420InterruptM$SFDCapture$captured(uint16_t arg_0xa6b0618);
static   result_t HPLCC2420InterruptM$SFD$disable(void);
static   result_t HPLCC2420InterruptM$SFD$enableCapture(bool arg_0xa8d6160);
static   void HPLCC2420InterruptM$FIFOPInterrupt$fired(void);
static   void MSP430InterruptM$Port14$clear(void);
static   void MSP430InterruptM$Port14$disable(void);
static   void MSP430InterruptM$Port14$edge(bool arg_0xa9d1b80);
static   void MSP430InterruptM$Port14$enable(void);
static   void MSP430InterruptM$Port26$clear(void);
static   void MSP430InterruptM$Port26$default$fired(void);
static   void MSP430InterruptM$Port17$clear(void);
static   void MSP430InterruptM$Port17$default$fired(void);
static   void MSP430InterruptM$Port21$clear(void);
static   void MSP430InterruptM$Port21$default$fired(void);
static   void MSP430InterruptM$Port12$clear(void);
static   void MSP430InterruptM$Port12$default$fired(void);
static   void MSP430InterruptM$Port24$clear(void);
static   void MSP430InterruptM$Port24$default$fired(void);
static   void MSP430InterruptM$ACCV$clear(void);
static   void MSP430InterruptM$ACCV$default$fired(void);
static   void MSP430InterruptM$Port15$clear(void);
static   void MSP430InterruptM$Port15$default$fired(void);
static   void MSP430InterruptM$Port27$clear(void);
static   void MSP430InterruptM$Port27$default$fired(void);
static   void MSP430InterruptM$Port10$clear(void);
static   void MSP430InterruptM$Port10$disable(void);
static   void MSP430InterruptM$Port10$edge(bool arg_0xa9d1b80);
static   void MSP430InterruptM$Port10$enable(void);
static   void MSP430InterruptM$Port22$clear(void);
static   void MSP430InterruptM$Port22$default$fired(void);
static   void MSP430InterruptM$OF$clear(void);
static   void MSP430InterruptM$OF$default$fired(void);
static   void MSP430InterruptM$Port13$clear(void);
static   void MSP430InterruptM$Port13$disable(void);
static   void MSP430InterruptM$Port25$clear(void);
static   void MSP430InterruptM$Port25$default$fired(void);
static   void MSP430InterruptM$Port16$clear(void);
static   void MSP430InterruptM$Port16$default$fired(void);
static   void MSP430InterruptM$NMI$clear(void);
static   void MSP430InterruptM$NMI$default$fired(void);
static   void MSP430InterruptM$Port20$clear(void);
static   void MSP430InterruptM$Port20$default$fired(void);
static   void MSP430InterruptM$Port11$clear(void);
static   void MSP430InterruptM$Port11$default$fired(void);
static   void MSP430InterruptM$Port23$clear(void);
static   void MSP430InterruptM$Port23$default$fired(void);
static  result_t BusArbitrationM$BusArbitration$default$busFree(uint8_t arg_0xaa45c10);
static   result_t BusArbitrationM$BusArbitration$releaseBus(uint8_t arg_0xaa45c10);
static   result_t BusArbitrationM$BusArbitration$getBus(uint8_t arg_0xaa45c10);
static   uint16_t RandomLFSR$Random$rand(void);
static   result_t RandomLFSR$Random$init(void);
static   result_t TimerJiffyAsyncM$TimerJiffyAsync$setOneShot(uint32_t arg_0xa8a41f0);
static   bool TimerJiffyAsyncM$TimerJiffyAsync$isSet(void);
static   result_t TimerJiffyAsyncM$TimerJiffyAsync$stop(void);
static   void TimerJiffyAsyncM$AlarmCompare$fired(void);
static  result_t TimerJiffyAsyncM$StdControl$init(void);
static  result_t TimerJiffyAsyncM$StdControl$start(void);
static   result_t FramerM$ByteComm$txDone(void);
static   result_t FramerM$ByteComm$txByteReady(bool arg_0xaac87e0);
static   result_t FramerM$ByteComm$rxByteReady(uint8_t arg_0xaac8010, bool arg_0xaac8158, uint16_t arg_0xaac82b0);
static  result_t FramerM$BareSendMsg$send(TOS_MsgPtr arg_0xa7b3928);
static  result_t FramerM$StdControl$init(void);
static  result_t FramerM$StdControl$start(void);
static  result_t FramerM$TokenReceiveMsg$ReflectToken(uint8_t arg_0xaacc010);
static  TOS_MsgPtr FramerAckM$ReceiveMsg$receive(TOS_MsgPtr arg_0xa7dbeb8);
static  TOS_MsgPtr FramerAckM$TokenReceiveMsg$receive(TOS_MsgPtr arg_0xaa89800, uint8_t arg_0xaa89948);
static   result_t UARTM$HPLUART$get(uint8_t arg_0xaad4300);
static   result_t UARTM$HPLUART$putDone(void);
static   result_t UARTM$ByteComm$txByte(uint8_t arg_0xaacdaf0);
static  result_t UARTM$Control$init(void);
static  result_t UARTM$Control$start(void);
static   result_t HPLUARTM$USARTData$rxDone(uint8_t arg_0xa982da8);
static   result_t HPLUARTM$USARTData$txDone(void);
static   result_t HPLUARTM$UART$init(void);
static   result_t HPLUARTM$UART$put(uint8_t arg_0xaaddd60);
static   bool HPLUSART1M$USARTControl$isSPI(void);
static   void HPLUSART1M$USARTControl$disableSPI(void);
static   void HPLUSART1M$USARTControl$setClockRate(uint16_t arg_0xa96e430, uint8_t arg_0xa96e578);
static   void HPLUSART1M$USARTControl$disableUART(void);
static   void HPLUSART1M$USARTControl$setClockSource(uint8_t arg_0xa96e010);
static   bool HPLUSART1M$USARTControl$isUART(void);
static   result_t HPLUSART1M$USARTControl$enableRxIntr(void);
static   bool HPLUSART1M$USARTControl$isI2C(void);
static   result_t HPLUSART1M$USARTControl$enableTxIntr(void);
static   bool HPLUSART1M$USARTControl$isUARTtx(void);
static   msp430_usartmode_t HPLUSART1M$USARTControl$getMode(void);
static   result_t HPLUSART1M$USARTControl$tx(uint8_t arg_0xa96ff30);
static   void HPLUSART1M$USARTControl$setModeUART(void);
static   bool HPLUSART1M$USARTControl$isUARTrx(void);
static   uint8_t HPLPowerManagementM$PowerManagement$adjustPower(void);
static  result_t NoCRCPacket$Send$send(TOS_MsgPtr arg_0xa7b3928);
static  result_t NoCRCPacket$SendVarLenPacket$default$sendDone(uint8_t *arg_0xab4f758, result_t arg_0xab4f8a8);
static   result_t NoCRCPacket$ByteComm$txDone(void);
static   result_t NoCRCPacket$ByteComm$txByteReady(bool arg_0xaac87e0);
static   result_t NoCRCPacket$ByteComm$rxByteReady(uint8_t arg_0xaac8010, bool arg_0xaac8158, uint16_t arg_0xaac82b0);
static  result_t NoCRCPacket$Control$init(void);
static  result_t NoCRCPacket$Control$start(void);
static  result_t NoCRCPacket$txBytes(uint8_t *arg_0xab4a100, uint8_t arg_0xab4a250);
static  
# 47 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MainM.nc"
result_t MainM$hardwareInit(void);
static  
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
result_t MainM$StdControl$init(void);
static  





result_t MainM$StdControl$start(void);
# 52 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MainM.nc"
int   main(void);
static  
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
result_t HPLInitM$MSP430ClockControl$init(void);
static  





result_t HPLInitM$MSP430ClockControl$start(void);
static inline  
# 35 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLInitM.nc"
result_t HPLInitM$init(void);
static  
# 29 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockInit.nc"
void MSP430ClockM$MSP430ClockInit$initTimerB(void);
static  
#line 28
void MSP430ClockM$MSP430ClockInit$initTimerA(void);
static  
#line 27
void MSP430ClockM$MSP430ClockInit$initClocks(void);
 
# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
static volatile uint8_t MSP430ClockM$IE1 __asm ("0x0000");
 static volatile uint16_t MSP430ClockM$TACTL __asm ("0x0160");
 static volatile uint16_t MSP430ClockM$TAIV __asm ("0x012E");
 static volatile uint16_t MSP430ClockM$TBCTL __asm ("0x0180");
 static volatile uint16_t MSP430ClockM$TBIV __asm ("0x011E");

enum MSP430ClockM$__nesc_unnamed4285 {

  MSP430ClockM$ACLK_CALIB_PERIOD = 8, 
  MSP430ClockM$ACLK_KHZ = 32, 
  MSP430ClockM$TARGET_DCO_KHZ = 4096, 
  MSP430ClockM$TARGET_DCO_DELTA = MSP430ClockM$TARGET_DCO_KHZ / MSP430ClockM$ACLK_KHZ * MSP430ClockM$ACLK_CALIB_PERIOD
};
static inline  
void MSP430ClockM$MSP430ClockInit$defaultInitClocks(void);
static inline  
#line 69
void MSP430ClockM$MSP430ClockInit$defaultInitTimerA(void);
static inline  
#line 84
void MSP430ClockM$MSP430ClockInit$defaultInitTimerB(void);
static inline   
#line 99
void MSP430ClockM$MSP430ClockInit$default$initClocks(void);
static inline   



void MSP430ClockM$MSP430ClockInit$default$initTimerA(void);
static inline   



void MSP430ClockM$MSP430ClockInit$default$initTimerB(void);
static inline 




void MSP430ClockM$startTimerA(void);
static inline 
#line 127
void MSP430ClockM$startTimerB(void);
static 
#line 139
void MSP430ClockM$set_dco_calib(int calib);
static inline 




uint16_t MSP430ClockM$test_calib_busywait_delta(int calib);
static inline 
#line 168
void MSP430ClockM$busyCalibrateDCO(void);
static inline  
#line 201
result_t MSP430ClockM$StdControl$init(void);
static inline  
#line 220
result_t MSP430ClockM$StdControl$start(void);
static   
# 30 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
uint16_t MSP430DCOCalibM$Timer32khz$read(void);
# 32 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430DCOCalibM.nc"
uint16_t MSP430DCOCalibM$m_prev;

enum MSP430DCOCalibM$__nesc_unnamed4286 {

  MSP430DCOCalibM$TARGET_DELTA = 2048, 
  MSP430DCOCalibM$MAX_DEVIATION = 7
};
static inline   

void MSP430DCOCalibM$TimerMicro$overflow(void);
static inline   
#line 75
void MSP430DCOCalibM$Timer32khz$overflow(void);
static   
# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
void MSP430TimerM$CaptureA1$captured(uint16_t arg_0xa6b0618);
static   
#line 74
void MSP430TimerM$CaptureB3$captured(uint16_t arg_0xa6b0618);
static   
# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void MSP430TimerM$CompareA1$fired(void);
static   
#line 34
void MSP430TimerM$CompareB3$fired(void);
static   
# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
void MSP430TimerM$CaptureB6$captured(uint16_t arg_0xa6b0618);
static   
#line 74
void MSP430TimerM$CaptureB1$captured(uint16_t arg_0xa6b0618);
static   
# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void MSP430TimerM$CompareB1$fired(void);
static   
# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
void MSP430TimerM$CaptureA2$captured(uint16_t arg_0xa6b0618);
static   
#line 74
void MSP430TimerM$CaptureB4$captured(uint16_t arg_0xa6b0618);
static   
# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void MSP430TimerM$CompareA2$fired(void);
static   
# 33 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
void MSP430TimerM$TimerA$overflow(void);
static   
# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void MSP430TimerM$CompareB4$fired(void);
static   
# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
void MSP430TimerM$CaptureA0$captured(uint16_t arg_0xa6b0618);
static   
#line 74
void MSP430TimerM$CaptureB2$captured(uint16_t arg_0xa6b0618);
static   
# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void MSP430TimerM$CompareA0$fired(void);
static   
#line 34
void MSP430TimerM$CompareB2$fired(void);
static   
# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
void MSP430TimerM$CaptureB5$captured(uint16_t arg_0xa6b0618);
static   
# 33 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
void MSP430TimerM$TimerB$overflow(void);
static   
# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void MSP430TimerM$CompareB5$fired(void);
static   
# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
void MSP430TimerM$CaptureB0$captured(uint16_t arg_0xa6b0618);
static   
# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void MSP430TimerM$CompareB6$fired(void);
static   
#line 34
void MSP430TimerM$CompareB0$fired(void);
 
# 69 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static volatile uint16_t MSP430TimerM$TACCTL0 __asm ("0x0162");
 static volatile uint16_t MSP430TimerM$TACCTL1 __asm ("0x0164");
 static volatile uint16_t MSP430TimerM$TACCTL2 __asm ("0x0166");
 
static volatile uint16_t MSP430TimerM$TACCR0 __asm ("0x0172");
 static volatile uint16_t MSP430TimerM$TACCR1 __asm ("0x0174");
 static volatile uint16_t MSP430TimerM$TACCR2 __asm ("0x0176");
 
static volatile uint16_t MSP430TimerM$TBCCTL0 __asm ("0x0182");
 static volatile uint16_t MSP430TimerM$TBCCTL1 __asm ("0x0184");
 static volatile uint16_t MSP430TimerM$TBCCTL2 __asm ("0x0186");
 static volatile uint16_t MSP430TimerM$TBCCTL3 __asm ("0x0188");
 static volatile uint16_t MSP430TimerM$TBCCTL4 __asm ("0x018A");
 static volatile uint16_t MSP430TimerM$TBCCTL5 __asm ("0x018C");
 static volatile uint16_t MSP430TimerM$TBCCTL6 __asm ("0x018E");
 
static volatile uint16_t MSP430TimerM$TBCCR0 __asm ("0x0192");
 static volatile uint16_t MSP430TimerM$TBCCR1 __asm ("0x0194");
 static volatile uint16_t MSP430TimerM$TBCCR2 __asm ("0x0196");
 static volatile uint16_t MSP430TimerM$TBCCR3 __asm ("0x0198");
 static volatile uint16_t MSP430TimerM$TBCCR4 __asm ("0x019A");
 static volatile uint16_t MSP430TimerM$TBCCR5 __asm ("0x019C");
 static volatile uint16_t MSP430TimerM$TBCCR6 __asm ("0x019E");

typedef MSP430CompareControl_t MSP430TimerM$CC_t;
static inline 
uint16_t MSP430TimerM$CC2int(MSP430TimerM$CC_t x);
static inline MSP430TimerM$CC_t MSP430TimerM$int2CC(uint16_t x);
static inline 
uint16_t MSP430TimerM$compareControl(void);
static inline 
#line 110
uint16_t MSP430TimerM$captureControl(uint8_t l_cm);
#line 123
void __attribute((interrupt(12))) __attribute((wakeup))  sig_TIMERA0_VECTOR(void);







void __attribute((interrupt(10))) __attribute((wakeup))  sig_TIMERA1_VECTOR(void);
static inline    
#line 157
void MSP430TimerM$CompareA0$default$fired(void);
static inline    void MSP430TimerM$CompareA1$default$fired(void);
static inline    void MSP430TimerM$CompareA2$default$fired(void);
static inline    void MSP430TimerM$CaptureA0$default$captured(uint16_t time);
static inline    void MSP430TimerM$CaptureA1$default$captured(uint16_t time);
static inline    void MSP430TimerM$CaptureA2$default$captured(uint16_t time);
static inline   


uint16_t MSP430TimerM$TimerB$read(void);
static inline   

bool MSP430TimerM$TimerB$isOverflowPending(void);
static inline   
#line 205
MSP430TimerM$CC_t MSP430TimerM$ControlA0$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlA1$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlA2$getControl(void);
static inline   
#line 253
uint16_t MSP430TimerM$CaptureA0$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureA1$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureA2$getEvent(void);
#line 277
void __attribute((interrupt(26))) __attribute((wakeup))  sig_TIMERB0_VECTOR(void);







void __attribute((interrupt(24))) __attribute((wakeup))  sig_TIMERB1_VECTOR(void);
static inline    
#line 331
void MSP430TimerM$CompareB0$default$fired(void);
static inline    void MSP430TimerM$CompareB1$default$fired(void);
static inline    void MSP430TimerM$CompareB2$default$fired(void);
static inline    

void MSP430TimerM$CompareB5$default$fired(void);
static inline    void MSP430TimerM$CompareB6$default$fired(void);
static inline    void MSP430TimerM$CaptureB0$default$captured(uint16_t time);
static inline    
void MSP430TimerM$CaptureB2$default$captured(uint16_t time);
static inline    void MSP430TimerM$CaptureB3$default$captured(uint16_t time);
static inline    void MSP430TimerM$CaptureB4$default$captured(uint16_t time);
static inline    void MSP430TimerM$CaptureB5$default$captured(uint16_t time);
static inline    void MSP430TimerM$CaptureB6$default$captured(uint16_t time);
static inline   

MSP430TimerM$CC_t MSP430TimerM$ControlB0$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB1$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB2$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB3$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB4$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB5$getControl(void);
static inline   MSP430TimerM$CC_t MSP430TimerM$ControlB6$getControl(void);
static inline   









void MSP430TimerM$ControlB1$clearPendingInterrupt(void);
static inline   
void MSP430TimerM$ControlB3$clearPendingInterrupt(void);
static inline   void MSP430TimerM$ControlB4$clearPendingInterrupt(void);
static inline   
#line 382
void MSP430TimerM$ControlB3$setControlAsCompare(void);
static inline   void MSP430TimerM$ControlB4$setControlAsCompare(void);
static inline   



void MSP430TimerM$ControlB1$setControlAsCapture(uint8_t cm);
static inline   
#line 412
void MSP430TimerM$ControlB1$enableEvents(void);
static inline   
void MSP430TimerM$ControlB3$enableEvents(void);
static inline   void MSP430TimerM$ControlB4$enableEvents(void);
static inline   



void MSP430TimerM$ControlB1$disableEvents(void);
static inline   
void MSP430TimerM$ControlB3$disableEvents(void);
static inline   void MSP430TimerM$ControlB4$disableEvents(void);
static inline   
#line 443
uint16_t MSP430TimerM$CaptureB0$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureB1$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureB2$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureB3$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureB4$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureB5$getEvent(void);
static inline   uint16_t MSP430TimerM$CaptureB6$getEvent(void);
static inline   
#line 470
void MSP430TimerM$CompareB3$setEventFromNow(uint16_t x);
static inline   void MSP430TimerM$CompareB4$setEventFromNow(uint16_t x);
static inline   



bool MSP430TimerM$CaptureB1$isOverflowPending(void);
static inline   






void MSP430TimerM$CaptureB1$clearOverflow(void);
static  
# 48 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SendMsg.nc"
result_t BSN_RadioRangeM$DataMsg$send(uint16_t arg_0xa7da410, uint8_t arg_0xa7da558, TOS_MsgPtr arg_0xa7da6a8);
static  
# 58 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
result_t BSN_RadioRangeM$UARTSend$send(TOS_MsgPtr arg_0xa7b3928);
static  
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
result_t BSN_RadioRangeM$CommControl$init(void);
static  





result_t BSN_RadioRangeM$CommControl$start(void);
static   
# 122 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Leds.nc"
result_t BSN_RadioRangeM$Leds$yellowOff(void);
static   
#line 114
result_t BSN_RadioRangeM$Leds$yellowOn(void);
static   
#line 56
result_t BSN_RadioRangeM$Leds$init(void);
static   
#line 97
result_t BSN_RadioRangeM$Leds$greenOff(void);
static   
#line 72
result_t BSN_RadioRangeM$Leds$redOff(void);
static   
#line 106
result_t BSN_RadioRangeM$Leds$greenToggle(void);
static   
#line 81
result_t BSN_RadioRangeM$Leds$redToggle(void);
static   
#line 64
result_t BSN_RadioRangeM$Leds$redOn(void);
static   
#line 89
result_t BSN_RadioRangeM$Leds$greenOn(void);
static  
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
result_t BSN_RadioRangeM$UARTControl$init(void);
static  





result_t BSN_RadioRangeM$UARTControl$start(void);
static  
# 59 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Timer.nc"
result_t BSN_RadioRangeM$Timer$start(char arg_0xa7b9cd0, uint32_t arg_0xa7b9e28);
static  







result_t BSN_RadioRangeM$Timer$stop(void);
# 29 "BSN_RadioRangeM.nc"
TOS_Msg BSN_RadioRangeM$msg[2];
uint8_t BSN_RadioRangeM$currentMsg;
uint16_t BSN_RadioRangeM$count;
uint8_t BSN_RadioRangeM$UARTConnected;
uint16_t BSN_RadioRangeM$state;
uint8_t BSN_RadioRangeM$sentRadio;
uint8_t BSN_RadioRangeM$strength;
uint8_t BSN_RadioRangeM$heartbeatfromUART;
uint16_t BSN_RadioRangeM$parent;
TOS_Msg BSN_RadioRangeM$replymsg;
#line 38
TOS_Msg BSN_RadioRangeM$forwardmsg;

enum BSN_RadioRangeM$__nesc_unnamed4287 {
#line 40
  BSN_RadioRangeM$REGISTER = 0, BSN_RadioRangeM$STARTED = 1
};
static inline  


result_t BSN_RadioRangeM$StdControl$init(void);
static inline  
#line 70
result_t BSN_RadioRangeM$StdControl$start(void);
static inline 
#line 86
uint8_t BSN_RadioRangeM$ToRadio(uint16_t destination, TOS_MsgPtr data);
static inline  









result_t BSN_RadioRangeM$Timer$fired(void);
static  
#line 139
void BSN_RadioRangeM$ForwardRadio(void);
static inline  






void BSN_RadioRangeM$ForwardUart(void);
static  





void BSN_RadioRangeM$ReplyHeartBeat(void);
static inline 
#line 172
TOS_MsgPtr BSN_RadioRangeM$receive(TOS_MsgPtr data, bool fromUART);
static inline  
#line 229
TOS_MsgPtr BSN_RadioRangeM$ReceiveMsg$receive(TOS_MsgPtr data);
static inline  





result_t BSN_RadioRangeM$DataMsg$sendDone(TOS_MsgPtr sent, result_t success);
static inline 



TOS_MsgPtr BSN_RadioRangeM$UartReceive(TOS_MsgPtr data);
static inline  
#line 297
TOS_MsgPtr BSN_RadioRangeM$UARTReceive$receive(TOS_MsgPtr data);
static inline  



result_t BSN_RadioRangeM$UARTSend$sendDone(TOS_MsgPtr data, result_t success);
static  
# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
result_t TimerM$TimerMilli$fired(
# 32 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
uint8_t arg_0xa7c3da8);
static   
# 38 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
void TimerM$AlarmControl$enableEvents(void);
static   
#line 35
void TimerM$AlarmControl$setControlAsCompare(void);
static   


void TimerM$AlarmControl$disableEvents(void);
static   
#line 32
void TimerM$AlarmControl$clearPendingInterrupt(void);
static   
# 32 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void TimerM$AlarmCompare$setEventFromNow(uint16_t arg_0xa6b4d98);
static  
# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerJiffy.nc"
result_t TimerM$TimerJiffy$fired(
# 33 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
uint8_t arg_0xa80a7e0);
static   
# 30 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
uint16_t TimerM$AlarmTimer$read(void);
static   bool TimerM$AlarmTimer$isOverflowPending(void);
static  
# 73 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Timer.nc"
result_t TimerM$Timer$fired(
# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
uint8_t arg_0xa7c3720);
#line 43
enum TimerM$__nesc_unnamed4288 {

  TimerM$COUNT_TIMER_OLD = 2, 
  TimerM$COUNT_TIMER_MILLI = 0, 
  TimerM$COUNT_TIMER_JIFFY = 0, 

  TimerM$OFFSET_TIMER_OLD = 0, 
  TimerM$OFFSET_TIMER_MILLI = TimerM$OFFSET_TIMER_OLD + TimerM$COUNT_TIMER_OLD, 
  TimerM$OFFSET_TIMER_JIFFY = TimerM$OFFSET_TIMER_MILLI + TimerM$COUNT_TIMER_MILLI, 
  TimerM$NUM_TIMERS = TimerM$OFFSET_TIMER_JIFFY + TimerM$COUNT_TIMER_JIFFY, 

  TimerM$EMPTY_LIST = 255
};

typedef struct TimerM$Timer_s {

  uint32_t alarm;
  uint8_t next;
  bool isperiodic : 1;
  bool isset : 1;
  bool isqueued : 1;
  int _reserved_flags : 5;
  uint8_t _reserved_byte;
} TimerM$Timer_t;

TimerM$Timer_t TimerM$m_timers[TimerM$NUM_TIMERS];
int32_t TimerM$m_period[TimerM$NUM_TIMERS];
uint16_t TimerM$m_hinow;
uint8_t TimerM$m_head_short;
uint8_t TimerM$m_head_long;
bool TimerM$m_posted_checkShortTimers;
static  
result_t TimerM$StdControl$init(void);
static inline  
#line 87
result_t TimerM$StdControl$start(void);
static 








void TimerM$insertTimer(uint8_t num, bool isshort);
static inline 
#line 118
void TimerM$removeTimer(uint8_t num);
static inline 



void TimerM$signal_timer_fired(uint8_t num);
static 
#line 146
void TimerM$executeTimers(uint8_t head);
static inline  
#line 190
void TimerM$checkShortTimers(void);
static inline 
void TimerM$post_checkShortTimers(void);
static 
#line 204
void TimerM$setNextShortEvent(void);
static inline  
#line 266
void TimerM$checkShortTimers(void);
static inline  







void TimerM$checkLongTimers(void);
static   






uint32_t TimerM$LocalTime$read(void);
static inline   
#line 303
void TimerM$AlarmCompare$fired(void);
static inline   



void TimerM$AlarmTimer$overflow(void);
static 




result_t TimerM$setTimer(uint8_t num, int32_t jiffy, bool isperiodic);
static inline   
#line 383
result_t TimerM$TimerJiffy$default$fired(uint8_t num);
static inline   
#line 432
result_t TimerM$TimerMilli$default$fired(uint8_t num);
static  






result_t TimerM$Timer$start(uint8_t num, char type, uint32_t milli);
static inline  
#line 454
result_t TimerM$Timer$stop(uint8_t num);
static inline   




result_t TimerM$Timer$default$fired(uint8_t num);
# 50 "C:/cygwin/opt/tinyos-1.x/tos/system/LedsC.nc"
uint8_t LedsC$ledsOn;

enum LedsC$__nesc_unnamed4289 {
  LedsC$RED_BIT = 1, 
  LedsC$GREEN_BIT = 2, 
  LedsC$YELLOW_BIT = 4
};
static inline   
result_t LedsC$Leds$init(void);
static inline   
#line 72
result_t LedsC$Leds$redOn(void);
static inline   







result_t LedsC$Leds$redOff(void);
static inline   







result_t LedsC$Leds$redToggle(void);
static inline   









result_t LedsC$Leds$greenOn(void);
static inline   







result_t LedsC$Leds$greenOff(void);
static inline   







result_t LedsC$Leds$greenToggle(void);
static inline   









result_t LedsC$Leds$yellowOn(void);
static inline   







result_t LedsC$Leds$yellowOff(void);
static  
# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
TOS_MsgPtr AMStandard$ReceiveMsg$receive(
# 56 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
uint8_t arg_0xa855228, 
# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
TOS_MsgPtr arg_0xa7dbeb8);
static  
# 59 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Timer.nc"
result_t AMStandard$ActivityTimer$start(char arg_0xa7b9cd0, uint32_t arg_0xa7b9e28);
static  
# 58 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
result_t AMStandard$UARTSend$send(TOS_MsgPtr arg_0xa7b3928);
static   
# 41 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/PowerManagement.nc"
uint8_t AMStandard$PowerManagement$adjustPower(void);
static  
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
result_t AMStandard$RadioControl$init(void);
static  





result_t AMStandard$RadioControl$start(void);
static  
#line 63
result_t AMStandard$TimerControl$init(void);
static  





result_t AMStandard$TimerControl$start(void);
static  
#line 63
result_t AMStandard$UARTControl$init(void);
static  





result_t AMStandard$UARTControl$start(void);
static  
# 65 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
result_t AMStandard$sendDone(void);
static  
# 58 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
result_t AMStandard$RadioSend$send(TOS_MsgPtr arg_0xa7b3928);
static  
# 49 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SendMsg.nc"
result_t AMStandard$SendMsg$sendDone(
# 55 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
uint8_t arg_0xa854c70, 
# 49 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SendMsg.nc"
TOS_MsgPtr arg_0xa7daac8, result_t arg_0xa7dac18);
# 81 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
bool AMStandard$state;
TOS_MsgPtr AMStandard$buffer;
uint16_t AMStandard$lastCount;
uint16_t AMStandard$counter;
static inline  

bool AMStandard$Control$init(void);
static inline  
#line 103
bool AMStandard$Control$start(void);
static inline 
#line 132
void AMStandard$dbgPacket(TOS_MsgPtr data);
static 









result_t AMStandard$reportSendDone(TOS_MsgPtr msg, result_t success);
static inline  






result_t AMStandard$ActivityTimer$fired(void);
static inline   




result_t AMStandard$SendMsg$default$sendDone(uint8_t id, TOS_MsgPtr msg, result_t success);
static inline   

result_t AMStandard$default$sendDone(void);
static inline  



void AMStandard$sendTask(void);
static  
#line 179
result_t AMStandard$SendMsg$send(uint8_t id, uint16_t addr, uint8_t length, TOS_MsgPtr data);
static inline  
#line 207
result_t AMStandard$UARTSend$sendDone(TOS_MsgPtr msg, result_t success);
static inline  

result_t AMStandard$RadioSend$sendDone(TOS_MsgPtr msg, result_t success);




TOS_MsgPtr   received(TOS_MsgPtr packet);
static inline   
#line 242
TOS_MsgPtr AMStandard$ReceiveMsg$default$receive(uint8_t id, TOS_MsgPtr msg);
static inline  


TOS_MsgPtr AMStandard$UARTReceive$receive(TOS_MsgPtr packet);
static inline  




TOS_MsgPtr AMStandard$RadioReceive$receive(TOS_MsgPtr packet);
static  
# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SplitControl.nc"
result_t CC2420RadioM$SplitControl$initDone(void);
static  
#line 85
result_t CC2420RadioM$SplitControl$startDone(void);
static   
# 59 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
result_t CC2420RadioM$FIFOP$disable(void);
static   
#line 43
result_t CC2420RadioM$FIFOP$startWait(bool arg_0xa8da5c0);
static   
# 6 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/TimerJiffyAsync.nc"
result_t CC2420RadioM$BackoffTimerJiffy$setOneShot(uint32_t arg_0xa8a41f0);
static   


bool CC2420RadioM$BackoffTimerJiffy$isSet(void);
static   
#line 8
result_t CC2420RadioM$BackoffTimerJiffy$stop(void);
static  
# 67 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
result_t CC2420RadioM$Send$sendDone(TOS_MsgPtr arg_0xa7b3e40, result_t arg_0xa7d6010);
static   
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Random.nc"
uint16_t CC2420RadioM$Random$rand(void);
static   
#line 57
result_t CC2420RadioM$Random$init(void);
static  
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
result_t CC2420RadioM$TimerControl$init(void);
static  





result_t CC2420RadioM$TimerControl$start(void);
static  
# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
TOS_MsgPtr CC2420RadioM$Receive$receive(TOS_MsgPtr arg_0xa7dbeb8);
static   
# 61 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
uint16_t CC2420RadioM$HPLChipcon$read(uint8_t arg_0xa8aa798);
static   
#line 47
uint8_t CC2420RadioM$HPLChipcon$cmd(uint8_t arg_0xa8b5d30);
static   
# 33 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/RadioCoordinator.nc"
void CC2420RadioM$RadioReceiveCoordinator$startSymbol(uint8_t arg_0xa88b540, uint8_t arg_0xa88b688, TOS_MsgPtr arg_0xa88b7d8);
static   
# 60 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Capture.nc"
result_t CC2420RadioM$SFD$disable(void);
static   
#line 43
result_t CC2420RadioM$SFD$enableCapture(bool arg_0xa8d6160);
static   
# 33 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/RadioCoordinator.nc"
void CC2420RadioM$RadioSendCoordinator$startSymbol(uint8_t arg_0xa88b540, uint8_t arg_0xa88b688, TOS_MsgPtr arg_0xa88b7d8);
static   
# 29 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
result_t CC2420RadioM$HPLChipconFIFO$writeTXFIFO(uint8_t arg_0xa8a6210, uint8_t *arg_0xa8a6370);
static   
#line 19
result_t CC2420RadioM$HPLChipconFIFO$readRXFIFO(uint8_t arg_0xa8abab8, uint8_t *arg_0xa8abc18);
static   
# 163 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420Control.nc"
result_t CC2420RadioM$CC2420Control$RxMode(void);
static   
# 74 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/MacBackoff.nc"
int16_t CC2420RadioM$MacBackoff$initialBackoff(TOS_MsgPtr arg_0xa867eb0);
static   int16_t CC2420RadioM$MacBackoff$congestionBackoff(TOS_MsgPtr arg_0xa88a310);
static  
# 64 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SplitControl.nc"
result_t CC2420RadioM$CC2420SplitControl$init(void);
static  
#line 77
result_t CC2420RadioM$CC2420SplitControl$start(void);
# 76 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
enum CC2420RadioM$__nesc_unnamed4290 {
  CC2420RadioM$DISABLED_STATE = 0, 
  CC2420RadioM$DISABLED_STATE_STARTTASK, 
  CC2420RadioM$IDLE_STATE, 
  CC2420RadioM$TX_STATE, 
  CC2420RadioM$TX_WAIT, 
  CC2420RadioM$PRE_TX_STATE, 
  CC2420RadioM$POST_TX_STATE, 
  CC2420RadioM$POST_TX_ACK_STATE, 
  CC2420RadioM$RX_STATE, 
  CC2420RadioM$POWER_DOWN_STATE, 
  CC2420RadioM$WARMUP_STATE, 

  CC2420RadioM$TIMER_INITIAL = 0, 
  CC2420RadioM$TIMER_BACKOFF, 
  CC2420RadioM$TIMER_ACK
};
 


uint8_t CC2420RadioM$countRetry;
uint8_t CC2420RadioM$stateRadio;
 uint8_t CC2420RadioM$stateTimer;
 uint8_t CC2420RadioM$currentDSN;
 bool CC2420RadioM$bAckEnable;
bool CC2420RadioM$bPacketReceiving;
uint8_t CC2420RadioM$txlength;
 TOS_MsgPtr CC2420RadioM$txbufptr;
 TOS_MsgPtr CC2420RadioM$rxbufptr;
TOS_Msg CC2420RadioM$RxBuf;

volatile uint16_t CC2420RadioM$LocalAddr;
static 




void CC2420RadioM$sendFailed(void);
static 




void CC2420RadioM$flushRXFIFO(void);
static 







__inline result_t CC2420RadioM$setInitialTimer(uint16_t jiffy);
static 






__inline result_t CC2420RadioM$setBackoffTimer(uint16_t jiffy);
static 






__inline result_t CC2420RadioM$setAckTimer(uint16_t jiffy);
static inline  







void CC2420RadioM$PacketRcvd(void);
static  
#line 168
void CC2420RadioM$PacketSent(void);
static inline  
#line 186
result_t CC2420RadioM$StdControl$init(void);
static inline  



result_t CC2420RadioM$SplitControl$init(void);
static inline  
#line 208
result_t CC2420RadioM$CC2420SplitControl$initDone(void);
static inline   


result_t CC2420RadioM$SplitControl$default$initDone(void);
static inline  
#line 239
void CC2420RadioM$startRadio(void);
static inline  
#line 253
result_t CC2420RadioM$StdControl$start(void);
static inline  
#line 277
result_t CC2420RadioM$SplitControl$start(void);
static inline  
#line 294
result_t CC2420RadioM$CC2420SplitControl$startDone(void);
static inline   
#line 312
result_t CC2420RadioM$SplitControl$default$startDone(void);
static inline 







void CC2420RadioM$sendPacket(void);
static inline   
#line 344
result_t CC2420RadioM$SFD$captured(uint16_t time);
static  
#line 393
void CC2420RadioM$startSend(void);
static 
#line 410
void CC2420RadioM$tryToSend(void);
static inline   
#line 449
result_t CC2420RadioM$BackoffTimerJiffy$fired(void);
static inline  
#line 491
result_t CC2420RadioM$Send$send(TOS_MsgPtr pMsg);
static 
#line 534
void CC2420RadioM$delayedRXFIFO(void);
static inline  
void CC2420RadioM$delayedRXFIFOtask(void);
static 


void CC2420RadioM$delayedRXFIFO(void);
static inline   
#line 591
result_t CC2420RadioM$FIFOP$fired(void);
static inline   
#line 628
result_t CC2420RadioM$HPLChipconFIFO$RXFIFODone(uint8_t length, uint8_t *data);
static inline   
#line 721
result_t CC2420RadioM$HPLChipconFIFO$TXFIFODone(uint8_t length, uint8_t *data);
static inline    
#line 744
int16_t CC2420RadioM$MacBackoff$default$initialBackoff(TOS_MsgPtr m);
static inline    





int16_t CC2420RadioM$MacBackoff$default$congestionBackoff(TOS_MsgPtr m);
static inline    






void CC2420RadioM$RadioSendCoordinator$default$startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff);
static inline    
void CC2420RadioM$RadioReceiveCoordinator$default$startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff);
static  
# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SplitControl.nc"
result_t CC2420ControlM$SplitControl$initDone(void);
static  
#line 85
result_t CC2420ControlM$SplitControl$startDone(void);
static   
# 61 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
uint16_t CC2420ControlM$HPLChipcon$read(uint8_t arg_0xa8aa798);
static   
#line 54
uint8_t CC2420ControlM$HPLChipcon$write(uint8_t arg_0xa8aa1a0, uint16_t arg_0xa8aa2f0);
static   
#line 47
uint8_t CC2420ControlM$HPLChipcon$cmd(uint8_t arg_0xa8b5d30);
static   
# 43 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
result_t CC2420ControlM$CCA$startWait(bool arg_0xa8da5c0);
static  
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
result_t CC2420ControlM$HPLChipconControl$init(void);
static  





result_t CC2420ControlM$HPLChipconControl$start(void);
static   
# 47 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
result_t CC2420ControlM$HPLChipconRAM$write(uint16_t arg_0xa906010, uint8_t arg_0xa906158, uint8_t *arg_0xa9062b8);
# 63 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
enum CC2420ControlM$__nesc_unnamed4291 {
  CC2420ControlM$IDLE_STATE = 0, 
  CC2420ControlM$INIT_STATE, 
  CC2420ControlM$INIT_STATE_DONE, 
  CC2420ControlM$START_STATE, 
  CC2420ControlM$START_STATE_DONE, 
  CC2420ControlM$STOP_STATE
};

uint8_t CC2420ControlM$state = 0;
 uint16_t CC2420ControlM$gCurrentParameters[14];
static inline 





bool CC2420ControlM$SetRegs(void);
static inline  
#line 108
void CC2420ControlM$taskInitDone(void);
static inline  






void CC2420ControlM$PostOscillatorOn(void);
static inline  
#line 129
result_t CC2420ControlM$SplitControl$init(void);
static inline  
#line 227
result_t CC2420ControlM$SplitControl$start(void);
static inline  
#line 286
result_t CC2420ControlM$CC2420Control$TuneManual(uint16_t DesiredFreq);
static inline   
#line 343
result_t CC2420ControlM$CC2420Control$RxMode(void);
static inline   
#line 368
result_t CC2420ControlM$CC2420Control$OscillatorOn(void);
static inline   
#line 400
result_t CC2420ControlM$CC2420Control$VREFOn(void);
static inline  
#line 432
result_t CC2420ControlM$CC2420Control$setShortAddress(uint16_t addr);
static inline   







result_t CC2420ControlM$HPLChipconRAM$writeDone(uint16_t addr, uint8_t length, uint8_t *buffer);
static inline   


result_t CC2420ControlM$CCA$fired(void);
static   
# 50 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
result_t HPLCC2420M$HPLCC2420FIFO$TXFIFODone(uint8_t arg_0xa8a6f20, uint8_t *arg_0xa8a7080);
static   
#line 39
result_t HPLCC2420M$HPLCC2420FIFO$RXFIFODone(uint8_t arg_0xa8a6890, uint8_t *arg_0xa8a69f0);
static   
# 191 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
result_t HPLCC2420M$USARTControl$isTxEmpty(void);
static   
#line 172
result_t HPLCC2420M$USARTControl$disableRxIntr(void);
static   result_t HPLCC2420M$USARTControl$disableTxIntr(void);
static   
#line 135
void HPLCC2420M$USARTControl$setModeSPI(void);
static   
#line 180
result_t HPLCC2420M$USARTControl$isTxIntrPending(void);
static   
#line 202
result_t HPLCC2420M$USARTControl$tx(uint8_t arg_0xa96ff30);
static   





uint8_t HPLCC2420M$USARTControl$rx(void);
static   
#line 185
result_t HPLCC2420M$USARTControl$isRxIntrPending(void);
static   
# 49 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
result_t HPLCC2420M$HPLCC2420RAM$writeDone(uint16_t arg_0xa906738, uint8_t arg_0xa906880, uint8_t *arg_0xa9069e0);
static   
# 38 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/BusArbitration.nc"
result_t HPLCC2420M$BusArbitration$releaseBus(void);
static   
#line 37
result_t HPLCC2420M$BusArbitration$getBus(void);
 
# 57 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420M.nc"
uint8_t *HPLCC2420M$rxbuf;
 uint8_t *HPLCC2420M$txbuf;
 uint8_t *HPLCC2420M$rambuf;
 
uint8_t HPLCC2420M$txlen;
 uint8_t HPLCC2420M$rxlen;
 uint8_t HPLCC2420M$ramlen;
 uint16_t HPLCC2420M$ramaddr;



bool HPLCC2420M$rxbufBusy;
bool HPLCC2420M$txbufBusy;
static inline 




uint8_t HPLCC2420M$adjustStatusByte(uint8_t status);
static inline  


result_t HPLCC2420M$StdControl$init(void);
static inline  
#line 91
result_t HPLCC2420M$StdControl$start(void);
static   
#line 110
uint8_t HPLCC2420M$HPLCC2420$cmd(uint8_t addr);
static   
#line 143
uint8_t HPLCC2420M$HPLCC2420$write(uint8_t addr, uint16_t data);
static   
#line 184
uint16_t HPLCC2420M$HPLCC2420$read(uint8_t addr);
static inline  
#line 263
void HPLCC2420M$signalRAMWr(void);
static inline   


result_t HPLCC2420M$HPLCC2420RAM$write(uint16_t addr, uint8_t length, uint8_t *buffer);
static inline  
#line 295
void HPLCC2420M$signalRXFIFO(void);
static inline   
#line 319
result_t HPLCC2420M$HPLCC2420FIFO$readRXFIFO(uint8_t length, uint8_t *data);
static inline  
#line 412
void HPLCC2420M$signalTXFIFO(void);
static inline   
#line 433
result_t HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(uint8_t length, uint8_t *data);
static inline  
#line 493
result_t HPLCC2420M$BusArbitration$busFree(void);
static   
# 43 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLI2CInterrupt.nc"
void HPLUSART0M$HPLI2CInterrupt$fired(void);
static   
# 53 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
result_t HPLUSART0M$USARTData$rxDone(uint8_t arg_0xa982da8);
static   
#line 46
result_t HPLUSART0M$USARTData$txDone(void);
 
# 47 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static volatile uint8_t HPLUSART0M$IE1 __asm ("0x0000");
 static volatile uint8_t HPLUSART0M$ME1 __asm ("0x0004");
 static volatile uint8_t HPLUSART0M$IFG1 __asm ("0x0002");
 static volatile uint8_t HPLUSART0M$U0TCTL __asm ("0x0071");
 static volatile uint8_t HPLUSART0M$U0TXBUF __asm ("0x0077");


uint16_t HPLUSART0M$l_br;

uint8_t HPLUSART0M$l_ssel;

void __attribute((interrupt(18))) __attribute((wakeup))  sig_UART0RX_VECTOR(void);




void __attribute((interrupt(16))) __attribute((wakeup))  sig_UART0TX_VECTOR(void);
static inline    





void HPLUSART0M$HPLI2CInterrupt$default$fired(void);
static inline   
bool HPLUSART0M$USARTControl$isSPI(void);
static inline   







bool HPLUSART0M$USARTControl$isUART(void);
static inline   









bool HPLUSART0M$USARTControl$isUARTtx(void);
static inline   









bool HPLUSART0M$USARTControl$isUARTrx(void);
static inline   









bool HPLUSART0M$USARTControl$isI2C(void);
static inline   









msp430_usartmode_t HPLUSART0M$USARTControl$getMode(void);
static inline   
#line 172
void HPLUSART0M$USARTControl$disableUART(void);
static inline   
#line 218
void HPLUSART0M$USARTControl$disableI2C(void);
static   





void HPLUSART0M$USARTControl$setModeSPI(void);
static inline   
#line 424
result_t HPLUSART0M$USARTControl$isTxIntrPending(void);
static inline   






result_t HPLUSART0M$USARTControl$isTxEmpty(void);
static inline   





result_t HPLUSART0M$USARTControl$isRxIntrPending(void);
static inline   






result_t HPLUSART0M$USARTControl$disableRxIntr(void);
static inline   



result_t HPLUSART0M$USARTControl$disableTxIntr(void);
static inline   
#line 473
result_t HPLUSART0M$USARTControl$tx(uint8_t data);
static inline   



uint8_t HPLUSART0M$USARTControl$rx(void);
static inline    






result_t HPLUSART0M$USARTData$default$txDone(void);
static inline    
result_t HPLUSART0M$USARTData$default$rxDone(uint8_t data);
static   
# 51 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
result_t HPLCC2420InterruptM$FIFO$fired(void);
static   
#line 51
result_t HPLCC2420InterruptM$FIFOP$fired(void);
static   
# 40 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
void HPLCC2420InterruptM$CCAInterrupt$clear(void);
static   
#line 35
void HPLCC2420InterruptM$CCAInterrupt$disable(void);
static   
#line 54
void HPLCC2420InterruptM$CCAInterrupt$edge(bool arg_0xa9d1b80);
static   
#line 30
void HPLCC2420InterruptM$CCAInterrupt$enable(void);
static   
# 36 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
void HPLCC2420InterruptM$SFDControl$setControlAsCapture(bool arg_0xa6bb190);
static   
void HPLCC2420InterruptM$SFDControl$enableEvents(void);
static   void HPLCC2420InterruptM$SFDControl$disableEvents(void);
static   
#line 32
void HPLCC2420InterruptM$SFDControl$clearPendingInterrupt(void);
static   
# 40 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
void HPLCC2420InterruptM$FIFOInterrupt$clear(void);
static   
#line 35
void HPLCC2420InterruptM$FIFOInterrupt$disable(void);
static   
# 51 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
result_t HPLCC2420InterruptM$CCA$fired(void);
static   
# 56 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
void HPLCC2420InterruptM$SFDCapture$clearOverflow(void);
static   
#line 51
bool HPLCC2420InterruptM$SFDCapture$isOverflowPending(void);
static   
# 53 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Capture.nc"
result_t HPLCC2420InterruptM$SFD$captured(uint16_t arg_0xa8d6688);
static   
# 40 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
void HPLCC2420InterruptM$FIFOPInterrupt$clear(void);
static   
#line 35
void HPLCC2420InterruptM$FIFOPInterrupt$disable(void);
static   
#line 54
void HPLCC2420InterruptM$FIFOPInterrupt$edge(bool arg_0xa9d1b80);
static   
#line 30
void HPLCC2420InterruptM$FIFOPInterrupt$enable(void);
static   
# 65 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420InterruptM.nc"
result_t HPLCC2420InterruptM$FIFOP$startWait(bool low_to_high);
static inline   
#line 78
result_t HPLCC2420InterruptM$FIFOP$disable(void);
static inline   









void HPLCC2420InterruptM$FIFOPInterrupt$fired(void);
static inline   
#line 130
void HPLCC2420InterruptM$FIFOInterrupt$fired(void);
static inline    








result_t HPLCC2420InterruptM$FIFO$default$fired(void);
static inline   





result_t HPLCC2420InterruptM$CCA$startWait(bool low_to_high);
static inline   
#line 171
void HPLCC2420InterruptM$CCAInterrupt$fired(void);
static   
#line 185
result_t HPLCC2420InterruptM$SFD$enableCapture(bool low_to_high);
static inline   
#line 200
result_t HPLCC2420InterruptM$SFD$disable(void);
static inline   







void HPLCC2420InterruptM$SFDCapture$captured(uint16_t time);
static   
# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
void MSP430InterruptM$Port14$fired(void);
static   
#line 59
void MSP430InterruptM$Port26$fired(void);
static   
#line 59
void MSP430InterruptM$Port17$fired(void);
static   
#line 59
void MSP430InterruptM$Port21$fired(void);
static   
#line 59
void MSP430InterruptM$Port12$fired(void);
static   
#line 59
void MSP430InterruptM$Port24$fired(void);
static   
#line 59
void MSP430InterruptM$ACCV$fired(void);
static   
#line 59
void MSP430InterruptM$Port15$fired(void);
static   
#line 59
void MSP430InterruptM$Port27$fired(void);
static   
#line 59
void MSP430InterruptM$Port10$fired(void);
static   
#line 59
void MSP430InterruptM$Port22$fired(void);
static   
#line 59
void MSP430InterruptM$OF$fired(void);
static   
#line 59
void MSP430InterruptM$Port13$fired(void);
static   
#line 59
void MSP430InterruptM$Port25$fired(void);
static   
#line 59
void MSP430InterruptM$Port16$fired(void);
static   
#line 59
void MSP430InterruptM$NMI$fired(void);
static   
#line 59
void MSP430InterruptM$Port20$fired(void);
static   
#line 59
void MSP430InterruptM$Port11$fired(void);
static   
#line 59
void MSP430InterruptM$Port23$fired(void);
 
# 51 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
static volatile uint8_t MSP430InterruptM$P1IE __asm ("0x0025");
 static volatile uint8_t MSP430InterruptM$P2IE __asm ("0x002D");
 static volatile uint8_t MSP430InterruptM$P1IFG __asm ("0x0023");
 static volatile uint8_t MSP430InterruptM$P2IFG __asm ("0x002B");

void __attribute((interrupt(8))) __attribute((wakeup))  sig_PORT1_VECTOR(void);
#line 71
void __attribute((interrupt(2))) __attribute((wakeup))  sig_PORT2_VECTOR(void);
#line 85
void __attribute((interrupt(28))) __attribute((wakeup))  sig_NMI_VECTOR(void);
static inline    







void MSP430InterruptM$Port11$default$fired(void);
static inline    void MSP430InterruptM$Port12$default$fired(void);
static inline    

void MSP430InterruptM$Port15$default$fired(void);
static inline    void MSP430InterruptM$Port16$default$fired(void);
static inline    void MSP430InterruptM$Port17$default$fired(void);
static inline    
void MSP430InterruptM$Port20$default$fired(void);
static inline    void MSP430InterruptM$Port21$default$fired(void);
static inline    void MSP430InterruptM$Port22$default$fired(void);
static inline    void MSP430InterruptM$Port23$default$fired(void);
static inline    void MSP430InterruptM$Port24$default$fired(void);
static inline    void MSP430InterruptM$Port25$default$fired(void);
static inline    void MSP430InterruptM$Port26$default$fired(void);
static inline    void MSP430InterruptM$Port27$default$fired(void);
static inline    
void MSP430InterruptM$NMI$default$fired(void);
static inline    void MSP430InterruptM$OF$default$fired(void);
static inline    void MSP430InterruptM$ACCV$default$fired(void);
static inline   
void MSP430InterruptM$Port10$enable(void);
static inline   


void MSP430InterruptM$Port14$enable(void);
static inline   
#line 146
void MSP430InterruptM$Port10$disable(void);
static inline   

void MSP430InterruptM$Port13$disable(void);
static inline   void MSP430InterruptM$Port14$disable(void);
static inline   
#line 177
void MSP430InterruptM$Port10$clear(void);
static inline   void MSP430InterruptM$Port11$clear(void);
static inline   void MSP430InterruptM$Port12$clear(void);
static inline   void MSP430InterruptM$Port13$clear(void);
static inline   void MSP430InterruptM$Port14$clear(void);
static inline   void MSP430InterruptM$Port15$clear(void);
static inline   void MSP430InterruptM$Port16$clear(void);
static inline   void MSP430InterruptM$Port17$clear(void);
static inline   
void MSP430InterruptM$Port20$clear(void);
static inline   void MSP430InterruptM$Port21$clear(void);
static inline   void MSP430InterruptM$Port22$clear(void);
static inline   void MSP430InterruptM$Port23$clear(void);
static inline   void MSP430InterruptM$Port24$clear(void);
static inline   void MSP430InterruptM$Port25$clear(void);
static inline   void MSP430InterruptM$Port26$clear(void);
static inline   void MSP430InterruptM$Port27$clear(void);
static inline   
void MSP430InterruptM$NMI$clear(void);
static inline   void MSP430InterruptM$OF$clear(void);
static inline   void MSP430InterruptM$ACCV$clear(void);
static inline   
#line 221
void MSP430InterruptM$Port10$edge(bool l2h);
static inline   
#line 245
void MSP430InterruptM$Port14$edge(bool l2h);
static  
# 39 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/BusArbitration.nc"
result_t BusArbitrationM$BusArbitration$busFree(
# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/BusArbitrationM.nc"
uint8_t arg_0xaa45c10);





uint8_t BusArbitrationM$state;
uint8_t BusArbitrationM$busid;
bool BusArbitrationM$isBusReleasedPending;
enum BusArbitrationM$__nesc_unnamed4292 {
#line 40
  BusArbitrationM$BUS_IDLE, BusArbitrationM$BUS_BUSY, BusArbitrationM$BUS_OFF
};
static inline  void BusArbitrationM$busReleased(void);
static   
#line 94
result_t BusArbitrationM$BusArbitration$getBus(uint8_t id);
static   
#line 108
result_t BusArbitrationM$BusArbitration$releaseBus(uint8_t id);
static inline   
#line 125
result_t BusArbitrationM$BusArbitration$default$busFree(uint8_t id);
# 54 "C:/cygwin/opt/tinyos-1.x/tos/system/RandomLFSR.nc"
uint16_t RandomLFSR$shiftReg;
uint16_t RandomLFSR$initSeed;
uint16_t RandomLFSR$mask;
static inline   

result_t RandomLFSR$Random$init(void);
static   









uint16_t RandomLFSR$Random$rand(void);
static   
# 38 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
void TimerJiffyAsyncM$AlarmControl$enableEvents(void);
static   
#line 35
void TimerJiffyAsyncM$AlarmControl$setControlAsCompare(void);
static   


void TimerJiffyAsyncM$AlarmControl$disableEvents(void);
static   
#line 32
void TimerJiffyAsyncM$AlarmControl$clearPendingInterrupt(void);
static   
# 12 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/TimerJiffyAsync.nc"
result_t TimerJiffyAsyncM$TimerJiffyAsync$fired(void);
static   
# 32 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void TimerJiffyAsyncM$AlarmCompare$setEventFromNow(uint16_t arg_0xa6b4d98);
# 13 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/TimerJiffyAsyncM.nc"
uint32_t TimerJiffyAsyncM$jiffy;
bool TimerJiffyAsyncM$bSet;
static inline  
result_t TimerJiffyAsyncM$StdControl$init(void);
static inline  





result_t TimerJiffyAsyncM$StdControl$start(void);
static inline   
#line 41
void TimerJiffyAsyncM$AlarmCompare$fired(void);
static   
#line 70
result_t TimerJiffyAsyncM$TimerJiffyAsync$setOneShot(uint32_t _jiffy);
static inline   
#line 97
bool TimerJiffyAsyncM$TimerJiffyAsync$isSet(void);
static inline   





result_t TimerJiffyAsyncM$TimerJiffyAsync$stop(void);
static  
# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
TOS_MsgPtr FramerM$ReceiveMsg$receive(TOS_MsgPtr arg_0xa7dbeb8);
static   
# 55 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ByteComm.nc"
result_t FramerM$ByteComm$txByte(uint8_t arg_0xaacdaf0);
static  
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
result_t FramerM$ByteControl$init(void);
static  





result_t FramerM$ByteControl$start(void);
static  
# 67 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
result_t FramerM$BareSendMsg$sendDone(TOS_MsgPtr arg_0xa7b3e40, result_t arg_0xa7d6010);
static  
# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/TokenReceiveMsg.nc"
TOS_MsgPtr FramerM$TokenReceiveMsg$receive(TOS_MsgPtr arg_0xaa89800, uint8_t arg_0xaa89948);
# 82 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
enum FramerM$__nesc_unnamed4293 {
  FramerM$HDLC_QUEUESIZE = 2, 
  FramerM$HDLC_MTU = sizeof(TOS_Msg ), 
  FramerM$HDLC_FLAG_BYTE = 0x7e, 
  FramerM$HDLC_CTLESC_BYTE = 0x7d, 
  FramerM$PROTO_ACK = 64, 
  FramerM$PROTO_PACKET_ACK = 65, 
  FramerM$PROTO_PACKET_NOACK = 66, 
  FramerM$PROTO_UNKNOWN = 255
};

enum FramerM$__nesc_unnamed4294 {
  FramerM$RXSTATE_NOSYNC, 
  FramerM$RXSTATE_PROTO, 
  FramerM$RXSTATE_TOKEN, 
  FramerM$RXSTATE_INFO, 
  FramerM$RXSTATE_ESC
};

enum FramerM$__nesc_unnamed4295 {
  FramerM$TXSTATE_IDLE, 
  FramerM$TXSTATE_PROTO, 
  FramerM$TXSTATE_INFO, 
  FramerM$TXSTATE_ESC, 
  FramerM$TXSTATE_FCS1, 
  FramerM$TXSTATE_FCS2, 
  FramerM$TXSTATE_ENDFLAG, 
  FramerM$TXSTATE_FINISH, 
  FramerM$TXSTATE_ERROR
};

enum FramerM$__nesc_unnamed4296 {
  FramerM$FLAGS_TOKENPEND = 0x2, 
  FramerM$FLAGS_DATAPEND = 0x4, 
  FramerM$FLAGS_UNKNOWN = 0x8
};

TOS_Msg FramerM$gMsgRcvBuf[FramerM$HDLC_QUEUESIZE];

typedef struct FramerM$_MsgRcvEntry {
  uint8_t Proto;
  uint8_t Token;
  uint16_t Length;
  TOS_MsgPtr pMsg;
} FramerM$MsgRcvEntry_t;

FramerM$MsgRcvEntry_t FramerM$gMsgRcvTbl[FramerM$HDLC_QUEUESIZE];

uint8_t *FramerM$gpRxBuf;
uint8_t *FramerM$gpTxBuf;

uint8_t FramerM$gFlags;
 

uint8_t FramerM$gTxState;
 uint8_t FramerM$gPrevTxState;
 uint16_t FramerM$gTxProto;
 uint16_t FramerM$gTxByteCnt;
 uint16_t FramerM$gTxLength;
 uint16_t FramerM$gTxRunningCRC;


uint8_t FramerM$gRxState;
uint8_t FramerM$gRxHeadIndex;
uint8_t FramerM$gRxTailIndex;
uint16_t FramerM$gRxByteCnt;

uint16_t FramerM$gRxRunningCRC;

TOS_MsgPtr FramerM$gpTxMsg;
uint8_t FramerM$gTxTokenBuf;
uint8_t FramerM$gTxUnknownBuf;
 uint8_t FramerM$gTxEscByte;
static  
void FramerM$PacketSent(void);
static 
result_t FramerM$StartTx(void);
static inline  
#line 202
void FramerM$PacketUnknown(void);
static inline  






void FramerM$PacketRcvd(void);
static  
#line 246
void FramerM$PacketSent(void);
static 
#line 268
void FramerM$HDLCInitialize(void);
static inline  
#line 291
result_t FramerM$StdControl$init(void);
static inline  



result_t FramerM$StdControl$start(void);
static inline  








result_t FramerM$BareSendMsg$send(TOS_MsgPtr pMsg);
static inline  
#line 328
result_t FramerM$TokenReceiveMsg$ReflectToken(uint8_t Token);
static   
#line 348
result_t FramerM$ByteComm$rxByteReady(uint8_t data, bool error, uint16_t strength);
static 
#line 469
result_t FramerM$TxArbitraryByte(uint8_t inByte);
static inline   
#line 482
result_t FramerM$ByteComm$txByteReady(bool LastByteSuccess);
static inline   
#line 552
result_t FramerM$ByteComm$txDone(void);
static  
# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
TOS_MsgPtr FramerAckM$ReceiveCombined$receive(TOS_MsgPtr arg_0xa7dbeb8);
static  
# 88 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/TokenReceiveMsg.nc"
result_t FramerAckM$TokenReceiveMsg$ReflectToken(uint8_t arg_0xaacc010);
# 72 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerAckM.nc"
uint8_t FramerAckM$gTokenBuf;
static inline  
void FramerAckM$SendAckTask(void);
static inline  



TOS_MsgPtr FramerAckM$TokenReceiveMsg$receive(TOS_MsgPtr Msg, uint8_t token);
static inline  
#line 91
TOS_MsgPtr FramerAckM$ReceiveMsg$receive(TOS_MsgPtr Msg);
static   
# 62 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/HPLUART.nc"
result_t UARTM$HPLUART$init(void);
static   
#line 80
result_t UARTM$HPLUART$put(uint8_t arg_0xaaddd60);
static   
# 83 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ByteComm.nc"
result_t UARTM$ByteComm$txDone(void);
static   
#line 75
result_t UARTM$ByteComm$txByteReady(bool arg_0xaac87e0);
static   
#line 66
result_t UARTM$ByteComm$rxByteReady(uint8_t arg_0xaac8010, bool arg_0xaac8158, uint16_t arg_0xaac82b0);
# 58 "C:/cygwin/opt/tinyos-1.x/tos/system/UARTM.nc"
bool UARTM$state;
static inline  
result_t UARTM$Control$init(void);
static inline  






result_t UARTM$Control$start(void);
static inline   







result_t UARTM$HPLUART$get(uint8_t data);
static   








result_t UARTM$HPLUART$putDone(void);
static   
#line 110
result_t UARTM$ByteComm$txByte(uint8_t data);
static   
# 88 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/HPLUART.nc"
result_t HPLUARTM$UART$get(uint8_t arg_0xaad4300);
static   






result_t HPLUARTM$UART$putDone(void);
static   
# 169 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
void HPLUARTM$USARTControl$setClockRate(uint16_t arg_0xa96e430, uint8_t arg_0xa96e578);
static   
#line 167
void HPLUARTM$USARTControl$setClockSource(uint8_t arg_0xa96e010);
static   





result_t HPLUARTM$USARTControl$enableRxIntr(void);
static   result_t HPLUARTM$USARTControl$enableTxIntr(void);
static   
#line 202
result_t HPLUARTM$USARTControl$tx(uint8_t arg_0xa96ff30);
static   
#line 153
void HPLUARTM$USARTControl$setModeUART(void);
static   
# 50 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
result_t HPLUARTM$UART$init(void);
static inline   
#line 90
result_t HPLUARTM$USARTData$rxDone(uint8_t b);
static inline   


result_t HPLUARTM$USARTData$txDone(void);
static inline   


result_t HPLUARTM$UART$put(uint8_t data);
static   
# 53 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
result_t HPLUSART1M$USARTData$rxDone(uint8_t arg_0xa982da8);
static   
#line 46
result_t HPLUSART1M$USARTData$txDone(void);
 
# 46 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
static volatile uint8_t HPLUSART1M$ME2 __asm ("0x0005");
 static volatile uint8_t HPLUSART1M$IFG2 __asm ("0x0003");
 static volatile uint8_t HPLUSART1M$U1TCTL __asm ("0x0079");
 static volatile uint8_t HPLUSART1M$U1TXBUF __asm ("0x007F");

uint16_t HPLUSART1M$l_br;
uint8_t HPLUSART1M$l_mctl;
uint8_t HPLUSART1M$l_ssel;

void __attribute((interrupt(6))) __attribute((wakeup))  sig_UART1RX_VECTOR(void);




void __attribute((interrupt(4))) __attribute((wakeup))  sig_UART1TX_VECTOR(void);
static inline   


bool HPLUSART1M$USARTControl$isSPI(void);
static inline   







bool HPLUSART1M$USARTControl$isUART(void);
static inline   









bool HPLUSART1M$USARTControl$isUARTtx(void);
static inline   









bool HPLUSART1M$USARTControl$isUARTrx(void);
static inline   
#line 107
bool HPLUSART1M$USARTControl$isI2C(void);
static inline   


msp430_usartmode_t HPLUSART1M$USARTControl$getMode(void);
static inline   
#line 158
void HPLUSART1M$USARTControl$disableUART(void);
static inline   
#line 191
void HPLUSART1M$USARTControl$disableSPI(void);
static inline 
#line 252
void HPLUSART1M$setUARTModeCommon(void);
static inline   
#line 325
void HPLUSART1M$USARTControl$setModeUART(void);
static inline   
#line 341
void HPLUSART1M$USARTControl$setClockSource(uint8_t source);
static inline   






void HPLUSART1M$USARTControl$setClockRate(uint16_t baudrate, uint8_t mctl);
static inline   
#line 392
result_t HPLUSART1M$USARTControl$enableRxIntr(void);
static inline   






result_t HPLUSART1M$USARTControl$enableTxIntr(void);
static inline   






result_t HPLUSART1M$USARTControl$tx(uint8_t data);
static inline   
# 53 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLPowerManagementM.nc"
uint8_t HPLPowerManagementM$PowerManagement$adjustPower(void);
static  
# 67 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
result_t NoCRCPacket$Send$sendDone(TOS_MsgPtr arg_0xa7b3e40, result_t arg_0xa7d6010);
static  
# 62 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SendVarLenPacket.nc"
result_t NoCRCPacket$SendVarLenPacket$sendDone(uint8_t *arg_0xab4f758, result_t arg_0xab4f8a8);
static  
# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
TOS_MsgPtr NoCRCPacket$Receive$receive(TOS_MsgPtr arg_0xa7dbeb8);
static   
# 55 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ByteComm.nc"
result_t NoCRCPacket$ByteComm$txByte(uint8_t arg_0xaacdaf0);
static  
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
result_t NoCRCPacket$ByteControl$init(void);
static  





result_t NoCRCPacket$ByteControl$start(void);
# 67 "C:/cygwin/opt/tinyos-1.x/tos/system/NoCRCPacket.nc"
uint8_t NoCRCPacket$rxCount;
#line 67
uint8_t NoCRCPacket$rxLength;
#line 67
uint8_t NoCRCPacket$txCount;
#line 67
uint8_t NoCRCPacket$txLength;
TOS_Msg NoCRCPacket$buffers[2];
TOS_Msg *NoCRCPacket$bufferPtrs[2];
uint8_t NoCRCPacket$bufferIndex;
uint8_t *NoCRCPacket$recPtr;
uint8_t *NoCRCPacket$sendPtr;

enum NoCRCPacket$__nesc_unnamed4297 {
  NoCRCPacket$IDLE, 
  NoCRCPacket$PACKET, 
  NoCRCPacket$BYTES
};
uint8_t NoCRCPacket$state;
static inline  
#line 94
result_t NoCRCPacket$Control$init(void);
static inline  
#line 111
result_t NoCRCPacket$Control$start(void);
static inline  









result_t NoCRCPacket$txBytes(uint8_t *bytes, uint8_t numBytes);
static  
#line 149
result_t NoCRCPacket$Send$send(TOS_MsgPtr msg);
static inline  
#line 190
void NoCRCPacket$sendDoneFailTask(void);
static inline  








void NoCRCPacket$sendDoneSuccessTask(void);
static inline  








void NoCRCPacket$sendVarLenFailTask(void);
static inline  








void NoCRCPacket$sendVarLenSuccessTask(void);
static 








void NoCRCPacket$sendComplete(result_t success);
static inline   
#line 271
result_t NoCRCPacket$SendVarLenPacket$default$sendDone(uint8_t *packet, result_t success);
static inline   









result_t NoCRCPacket$ByteComm$txByteReady(bool success);
static inline   
#line 309
result_t NoCRCPacket$ByteComm$txDone(void);
static inline  









void NoCRCPacket$receiveTask(void);
static   
#line 337
result_t NoCRCPacket$ByteComm$rxByteReady(uint8_t data, bool error, 
uint16_t strength);
static inline 
# 127 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
void MSP430ClockM$startTimerB(void)
{

  MSP430ClockM$TBCTL = 0x0020 | (MSP430ClockM$TBCTL & ~(0x0020 | 0x0010));
}

static inline 
#line 115
void MSP430ClockM$startTimerA(void)
{

  MSP430ClockM$TACTL = 0x0020 | (MSP430ClockM$TACTL & ~(0x0020 | 0x0010));
}

static inline  
#line 220
result_t MSP430ClockM$StdControl$start(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      MSP430ClockM$startTimerA();
      MSP430ClockM$startTimerB();
    }
#line 226
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t HPLInitM$MSP430ClockControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = MSP430ClockM$StdControl$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
static inline  
# 84 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
void MSP430ClockM$MSP430ClockInit$defaultInitTimerB(void)
{
  TBR = 0;









  MSP430ClockM$TBCTL = 0x0100 | 0x0002;
}

static inline   









void MSP430ClockM$MSP430ClockInit$default$initTimerB(void)
{
  MSP430ClockM$MSP430ClockInit$defaultInitTimerB();
}

# 29 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockInit.nc"
inline static  void MSP430ClockM$MSP430ClockInit$initTimerB(void){
#line 29
  MSP430ClockM$MSP430ClockInit$default$initTimerB();
#line 29
}
#line 29
static inline  
# 69 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
void MSP430ClockM$MSP430ClockInit$defaultInitTimerA(void)
{
  TAR = 0;









  MSP430ClockM$TACTL = 0x0200 | 0x0002;
}

static inline   
#line 104
void MSP430ClockM$MSP430ClockInit$default$initTimerA(void)
{
  MSP430ClockM$MSP430ClockInit$defaultInitTimerA();
}

# 28 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockInit.nc"
inline static  void MSP430ClockM$MSP430ClockInit$initTimerA(void){
#line 28
  MSP430ClockM$MSP430ClockInit$default$initTimerA();
#line 28
}
#line 28
static inline  
# 48 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
void MSP430ClockM$MSP430ClockInit$defaultInitClocks(void)
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  MSP430ClockM$IE1 &= ~(1 << 1);
}

static inline   
#line 99
void MSP430ClockM$MSP430ClockInit$default$initClocks(void)
{
  MSP430ClockM$MSP430ClockInit$defaultInitClocks();
}

# 27 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockInit.nc"
inline static  void MSP430ClockM$MSP430ClockInit$initClocks(void){
#line 27
  MSP430ClockM$MSP430ClockInit$default$initClocks();
#line 27
}
#line 27
static inline 
# 145 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
uint16_t MSP430ClockM$test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  MSP430ClockM$set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + MSP430ClockM$ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TAR;
    }

  return dco_curr - dco_prev;
}

static inline 


void MSP430ClockM$busyCalibrateDCO(void)
{

  int calib;
  int step;



  MSP430ClockM$TACTL = 0x0200 | 0x0020;
  MSP430ClockM$TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (MSP430ClockM$test_calib_busywait_delta(calib | step) <= MSP430ClockM$TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  MSP430ClockM$set_dco_calib(calib);
}

static inline  result_t MSP430ClockM$StdControl$init(void)
{

  MSP430ClockM$TACTL = 0x0004;
  MSP430ClockM$TAIV = 0;
  MSP430ClockM$TBCTL = 0x0004;
  MSP430ClockM$TBIV = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      MSP430ClockM$busyCalibrateDCO();
      MSP430ClockM$MSP430ClockInit$initClocks();
      MSP430ClockM$MSP430ClockInit$initTimerA();
      MSP430ClockM$MSP430ClockInit$initTimerB();
    }
#line 215
    __nesc_atomic_end(__nesc_atomic); }

  return SUCCESS;
}

# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t HPLInitM$MSP430ClockControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = MSP430ClockM$StdControl$init();
#line 63

#line 63
  return result;
#line 63
}
#line 63
static inline 
# 58 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
void TOSH_CLR_HUM_PWR_PIN(void)
#line 58
{
   
#line 58
  static volatile uint8_t r __asm ("0x0021");

#line 58
  r &= ~(1 << 7);
}

static inline 
#line 56
void TOSH_CLR_HUM_SDA_PIN(void)
#line 56
{
   
#line 56
  static volatile uint8_t r __asm ("0x0021");

#line 56
  r &= ~(1 << 5);
}

static inline 
#line 57
void TOSH_CLR_HUM_SCL_PIN(void)
#line 57
{
   
#line 57
  static volatile uint8_t r __asm ("0x0021");

#line 57
  r &= ~(1 << 6);
}

static inline 
#line 58
void TOSH_MAKE_HUM_PWR_OUTPUT(void)
#line 58
{
   
#line 58
  static volatile uint8_t r __asm ("0x0022");

#line 58
  r |= 1 << 7;
}

static inline 
#line 56
void TOSH_MAKE_HUM_SDA_OUTPUT(void)
#line 56
{
   
#line 56
  static volatile uint8_t r __asm ("0x0022");

#line 56
  r |= 1 << 5;
}

static inline 
#line 57
void TOSH_MAKE_HUM_SCL_OUTPUT(void)
#line 57
{
   
#line 57
  static volatile uint8_t r __asm ("0x0022");

#line 57
  r |= 1 << 6;
}

static inline 
#line 86
void TOSH_SET_FLASH_CS_PIN(void)
#line 86
{
   
#line 86
  static volatile uint8_t r __asm ("0x001D");

#line 86
  r |= 1 << 4;
}

static inline 
#line 86
void TOSH_MAKE_FLASH_CS_OUTPUT(void)
#line 86
{
   
#line 86
  static volatile uint8_t r __asm ("0x001E");

#line 86
  r |= 1 << 4;
}

static inline 
#line 85
void TOSH_SET_FLASH_PWR_PIN(void)
#line 85
{
   
#line 85
  static volatile uint8_t r __asm ("0x001D");

#line 85
  r |= 1 << 3;
}

static inline 
#line 85
void TOSH_MAKE_FLASH_PWR_OUTPUT(void)
#line 85
{
   
#line 85
  static volatile uint8_t r __asm ("0x001E");

#line 85
  r |= 1 << 3;
}

static inline 


void TOSH_MAKE_PROG_TX_INPUT(void)
#line 91
{
   
#line 91
  static volatile uint8_t r __asm ("0x002A");

#line 91
  r &= ~(1 << 2);
}

static inline 
#line 90
void TOSH_MAKE_PROG_RX_INPUT(void)
#line 90
{
   
#line 90
  static volatile uint8_t r __asm ("0x0022");

#line 90
  r &= ~(1 << 1);
}

static inline 
#line 39
void TOSH_MAKE_URXD1_INPUT(void)
#line 39
{
   
#line 39
  static volatile uint8_t r __asm ("0x001A");

#line 39
  r &= ~(1 << 7);
}

static inline 
#line 38
void TOSH_MAKE_UTXD1_INPUT(void)
#line 38
{
   
#line 38
  static volatile uint8_t r __asm ("0x001A");

#line 38
  r &= ~(1 << 6);
}

static inline 
#line 37
void TOSH_MAKE_URXD0_INPUT(void)
#line 37
{
   
#line 37
  static volatile uint8_t r __asm ("0x001A");

#line 37
  r &= ~(1 << 5);
}

static inline 
#line 36
void TOSH_MAKE_UTXD0_INPUT(void)
#line 36
{
   
#line 36
  static volatile uint8_t r __asm ("0x001A");

#line 36
  r &= ~(1 << 4);
}

static inline 
#line 23
void TOSH_MAKE_RADIO_GIO1_INPUT(void)
#line 23
{
   
#line 23
  static volatile uint8_t r __asm ("0x0022");

#line 23
  r &= ~(1 << 4);
}

static inline 
#line 20
void TOSH_MAKE_RADIO_SFD_INPUT(void)
#line 20
{
   
#line 20
  static volatile uint8_t r __asm ("0x001E");

#line 20
  r &= ~(1 << 1);
}

static inline 
#line 21
void TOSH_MAKE_RADIO_GIO0_INPUT(void)
#line 21
{
   
#line 21
  static volatile uint8_t r __asm ("0x0022");

#line 21
  r &= ~(1 << 3);
}

static inline 
#line 19
void TOSH_MAKE_RADIO_FIFOP_INPUT(void)
#line 19
{
   
#line 19
  static volatile uint8_t r __asm ("0x0022");

#line 19
  r &= ~(1 << 0);
}

static inline 
#line 16
void TOSH_MAKE_RADIO_CSN_OUTPUT(void)
#line 16
{
   
#line 16
  static volatile uint8_t r __asm ("0x001E");

#line 16
  r |= 1 << 2;
}

static inline 
#line 16
void TOSH_SET_RADIO_CSN_PIN(void)
#line 16
{
   
#line 16
  static volatile uint8_t r __asm ("0x001D");

#line 16
  r |= 1 << 2;
}

static inline 
#line 17
void TOSH_MAKE_RADIO_VREF_OUTPUT(void)
#line 17
{
   
#line 17
  static volatile uint8_t r __asm ("0x001E");

#line 17
  r |= 1 << 5;
}

static inline 
#line 17
void TOSH_CLR_RADIO_VREF_PIN(void)
#line 17
{
   
#line 17
  static volatile uint8_t r __asm ("0x001D");

#line 17
  r &= ~(1 << 5);
}

static inline 
#line 18
void TOSH_MAKE_RADIO_RESET_OUTPUT(void)
#line 18
{
   
#line 18
  static volatile uint8_t r __asm ("0x001E");

#line 18
  r |= 1 << 6;
}

static inline 
#line 18
void TOSH_SET_RADIO_RESET_PIN(void)
#line 18
{
   
#line 18
  static volatile uint8_t r __asm ("0x001D");

#line 18
  r |= 1 << 6;
}

static inline 
#line 40
void TOSH_MAKE_UCLK1_INPUT(void)
#line 40
{
   
#line 40
  static volatile uint8_t r __asm ("0x0032");

#line 40
  r &= ~(1 << 3);
}

static inline 
#line 42
void TOSH_MAKE_SIMO1_INPUT(void)
#line 42
{
   
#line 42
  static volatile uint8_t r __asm ("0x0032");

#line 42
  r &= ~(1 << 1);
}

static inline 
#line 41
void TOSH_MAKE_SOMI1_INPUT(void)
#line 41
{
   
#line 41
  static volatile uint8_t r __asm ("0x0032");

#line 41
  r &= ~(1 << 2);
}

static inline 
#line 35
void TOSH_MAKE_UCLK0_INPUT(void)
#line 35
{
   
#line 35
  static volatile uint8_t r __asm ("0x001A");

#line 35
  r &= ~(1 << 3);
}

static inline 
#line 34
void TOSH_MAKE_SIMO0_INPUT(void)
#line 34
{
   
#line 34
  static volatile uint8_t r __asm ("0x001A");

#line 34
  r &= ~(1 << 1);
}

static inline 
#line 33
void TOSH_MAKE_SOMI0_INPUT(void)
#line 33
{
   
#line 33
  static volatile uint8_t r __asm ("0x001A");

#line 33
  r &= ~(1 << 2);
}

static inline 
#line 13
void TOSH_MAKE_YELLOW_LED_OUTPUT(void)
#line 13
{
   
#line 13
  static volatile uint8_t r __asm ("0x0032");

#line 13
  r |= 1 << 6;
}

static inline 
#line 12
void TOSH_MAKE_GREEN_LED_OUTPUT(void)
#line 12
{
   
#line 12
  static volatile uint8_t r __asm ("0x0032");

#line 12
  r |= 1 << 5;
}

static inline 
#line 11
void TOSH_MAKE_RED_LED_OUTPUT(void)
#line 11
{
   
#line 11
  static volatile uint8_t r __asm ("0x0032");

#line 11
  r |= 1 << 4;
}

static inline 
#line 13
void TOSH_SET_YELLOW_LED_PIN(void)
#line 13
{
   
#line 13
  static volatile uint8_t r __asm ("0x0031");

#line 13
  r |= 1 << 6;
}

static inline 
#line 12
void TOSH_SET_GREEN_LED_PIN(void)
#line 12
{
   
#line 12
  static volatile uint8_t r __asm ("0x0031");

#line 12
  r |= 1 << 5;
}

static inline 
#line 11
void TOSH_SET_RED_LED_PIN(void)
#line 11
{
   
#line 11
  static volatile uint8_t r __asm ("0x0031");

#line 11
  r |= 1 << 4;
}

static inline 
#line 93
void TOSH_SET_PIN_DIRECTIONS(void )
{

  TOSH_SET_RED_LED_PIN();
  TOSH_SET_GREEN_LED_PIN();
  TOSH_SET_YELLOW_LED_PIN();
  TOSH_MAKE_RED_LED_OUTPUT();
  TOSH_MAKE_GREEN_LED_OUTPUT();
  TOSH_MAKE_YELLOW_LED_OUTPUT();



  TOSH_MAKE_SOMI0_INPUT();
  TOSH_MAKE_SIMO0_INPUT();
  TOSH_MAKE_UCLK0_INPUT();
  TOSH_MAKE_SOMI1_INPUT();
  TOSH_MAKE_SIMO1_INPUT();
  TOSH_MAKE_UCLK1_INPUT();
  TOSH_SET_RADIO_RESET_PIN();
  TOSH_MAKE_RADIO_RESET_OUTPUT();
  TOSH_CLR_RADIO_VREF_PIN();
  TOSH_MAKE_RADIO_VREF_OUTPUT();
  TOSH_SET_RADIO_CSN_PIN();
  TOSH_MAKE_RADIO_CSN_OUTPUT();
  TOSH_MAKE_RADIO_FIFOP_INPUT();
  TOSH_MAKE_RADIO_GIO0_INPUT();
  TOSH_MAKE_RADIO_SFD_INPUT();
  TOSH_MAKE_RADIO_GIO1_INPUT();


  TOSH_MAKE_UTXD0_INPUT();
  TOSH_MAKE_URXD0_INPUT();
  TOSH_MAKE_UTXD1_INPUT();
  TOSH_MAKE_URXD1_INPUT();


  TOSH_MAKE_PROG_RX_INPUT();
  TOSH_MAKE_PROG_TX_INPUT();


  TOSH_MAKE_FLASH_PWR_OUTPUT();
  TOSH_SET_FLASH_PWR_PIN();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_CS_PIN();


  TOSH_MAKE_HUM_SCL_OUTPUT();
  TOSH_MAKE_HUM_SDA_OUTPUT();
  TOSH_MAKE_HUM_PWR_OUTPUT();
  TOSH_CLR_HUM_SCL_PIN();
  TOSH_CLR_HUM_SDA_PIN();
  TOSH_CLR_HUM_PWR_PIN();
}

static inline  
# 35 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLInitM.nc"
result_t HPLInitM$init(void)
{
  TOSH_SET_PIN_DIRECTIONS();
  HPLInitM$MSP430ClockControl$init();
  HPLInitM$MSP430ClockControl$start();
  return SUCCESS;
}

# 47 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MainM.nc"
inline static  result_t MainM$hardwareInit(void){
#line 47
  unsigned char result;
#line 47

#line 47
  result = HPLInitM$init();
#line 47

#line 47
  return result;
#line 47
}
#line 47
static inline 
# 79 "C:/cygwin/opt/tinyos-1.x/tos/system/sched.c"
void TOSH_sched_init(void )
{
  int i;

#line 82
  TOSH_sched_free = 0;
  TOSH_sched_full = 0;
  for (i = 0; i < TOSH_MAX_TASKS; i++) 
    TOSH_queue[i].tp = NULL;
}

static inline 
# 120 "C:/cygwin/opt/tinyos-1.x/tos/system/tos.h"
result_t rcombine(result_t r1, result_t r2)



{
  return r1 == FAIL ? FAIL : r2;
}

static inline  
# 60 "C:/cygwin/opt/tinyos-1.x/tos/system/UARTM.nc"
result_t UARTM$Control$init(void)
#line 60
{
  {
  }
#line 61
  ;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 62
    {
      UARTM$state = FALSE;
    }
#line 64
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t NoCRCPacket$ByteControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = UARTM$Control$init();
#line 63

#line 63
  return result;
#line 63
}
#line 63
static inline  
# 94 "C:/cygwin/opt/tinyos-1.x/tos/system/NoCRCPacket.nc"
result_t NoCRCPacket$Control$init(void)
#line 94
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 95
    {
      NoCRCPacket$recPtr = (uint8_t *)&NoCRCPacket$buffers[0];
      NoCRCPacket$bufferIndex = 0;
      NoCRCPacket$bufferPtrs[0] = &NoCRCPacket$buffers[0];
      NoCRCPacket$bufferPtrs[1] = &NoCRCPacket$buffers[1];

      NoCRCPacket$state = NoCRCPacket$IDLE;
      NoCRCPacket$txCount = NoCRCPacket$rxCount = 0;

      NoCRCPacket$rxLength = (size_t )& ((TOS_Msg *)0)->type + 1;
      {
      }
#line 105
      ;
    }
#line 106
    __nesc_atomic_end(__nesc_atomic); }
  return NoCRCPacket$ByteControl$init();
}

# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t BSN_RadioRangeM$UARTControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = NoCRCPacket$Control$init();
#line 63

#line 63
  return result;
#line 63
}
#line 63
static inline   
# 212 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$SplitControl$default$initDone(void)
#line 212
{
  return SUCCESS;
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static  result_t CC2420RadioM$SplitControl$initDone(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = CC2420RadioM$SplitControl$default$initDone();
#line 70

#line 70
  return result;
#line 70
}
#line 70
static inline  
# 208 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$CC2420SplitControl$initDone(void)
#line 208
{
  return CC2420RadioM$SplitControl$initDone();
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static  result_t CC2420ControlM$SplitControl$initDone(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = CC2420RadioM$CC2420SplitControl$initDone();
#line 70

#line 70
  return result;
#line 70
}
#line 70
static inline  
# 108 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
void CC2420ControlM$taskInitDone(void)
#line 108
{
  CC2420ControlM$SplitControl$initDone();
}

static inline   
# 452 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
result_t HPLUSART0M$USARTControl$disableTxIntr(void)
#line 452
{
  HPLUSART0M$IE1 &= ~(1 << 7);
  return SUCCESS;
}

# 173 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$disableTxIntr(void){
#line 173
  unsigned char result;
#line 173

#line 173
  result = HPLUSART0M$USARTControl$disableTxIntr();
#line 173

#line 173
  return result;
#line 173
}
#line 173
static inline   
# 447 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
result_t HPLUSART0M$USARTControl$disableRxIntr(void)
#line 447
{
  HPLUSART0M$IE1 &= ~(1 << 6);
  return SUCCESS;
}

# 172 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$disableRxIntr(void){
#line 172
  unsigned char result;
#line 172

#line 172
  result = HPLUSART0M$USARTControl$disableRxIntr();
#line 172

#line 172
  return result;
#line 172
}
#line 172
#line 135
inline static   void HPLCC2420M$USARTControl$setModeSPI(void){
#line 135
  HPLUSART0M$USARTControl$setModeSPI();
#line 135
}
#line 135
static inline  
# 79 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420M.nc"
result_t HPLCC2420M$StdControl$init(void)
#line 79
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 80
    HPLCC2420M$rxbufBusy = FALSE;
#line 80
    __nesc_atomic_end(__nesc_atomic); }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 81
    HPLCC2420M$txbufBusy = FALSE;
#line 81
    __nesc_atomic_end(__nesc_atomic); }

  TOSH_SET_RADIO_CSN_PIN();
  TOSH_MAKE_RADIO_CSN_OUTPUT();
  HPLCC2420M$USARTControl$setModeSPI();
  HPLCC2420M$USARTControl$disableRxIntr();
  HPLCC2420M$USARTControl$disableTxIntr();
  return SUCCESS;
}

# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t CC2420ControlM$HPLChipconControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = HPLCC2420M$StdControl$init();
#line 63

#line 63
  return result;
#line 63
}
#line 63
static inline  
# 129 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
result_t CC2420ControlM$SplitControl$init(void)
#line 129
{

  uint8_t _state = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 133
    {
      if (CC2420ControlM$state == CC2420ControlM$IDLE_STATE) {
          CC2420ControlM$state = CC2420ControlM$INIT_STATE;
          _state = TRUE;
        }
    }
#line 138
    __nesc_atomic_end(__nesc_atomic); }
  if (!_state) {
    return FAIL;
    }
  CC2420ControlM$HPLChipconControl$init();


  CC2420ControlM$gCurrentParameters[CP_MAIN] = 0xf800;
  CC2420ControlM$gCurrentParameters[CP_MDMCTRL0] = ((((0 << 11) | (
  2 << 8)) | (3 << 6)) | (
  1 << 5)) | (2 << 0);

  CC2420ControlM$gCurrentParameters[CP_MDMCTRL1] = 20 << 6;

  CC2420ControlM$gCurrentParameters[CP_RSSI] = 0xE080;
  CC2420ControlM$gCurrentParameters[CP_SYNCWORD] = 0xA70F;
  CC2420ControlM$gCurrentParameters[CP_TXCTRL] = ((((1 << 14) | (
  1 << 13)) | (3 << 6)) | (
  1 << 5)) | (0x1F << 0);

  CC2420ControlM$gCurrentParameters[CP_RXCTRL0] = (((((1 << 12) | (
  2 << 8)) | (3 << 6)) | (
  2 << 4)) | (1 << 2)) | (
  1 << 0);

  CC2420ControlM$gCurrentParameters[CP_RXCTRL1] = (((((1 << 11) | (
  1 << 9)) | (1 << 6)) | (
  1 << 4)) | (1 << 2)) | (
  2 << 0);

  CC2420ControlM$gCurrentParameters[CP_FSCTRL] = (1 << 14) | ((
  357 + 5 * (14 - 11)) << 0);

  CC2420ControlM$gCurrentParameters[CP_SECCTRL0] = (((1 << 8) | (
  1 << 7)) | (1 << 6)) | (
  1 << 2);

  CC2420ControlM$gCurrentParameters[CP_SECCTRL1] = 0;
  CC2420ControlM$gCurrentParameters[CP_BATTMON] = 0;



  CC2420ControlM$gCurrentParameters[CP_IOCFG0] = (127 << 0) | (
  1 << 9);

  CC2420ControlM$gCurrentParameters[CP_IOCFG1] = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 185
    CC2420ControlM$state = CC2420ControlM$INIT_STATE_DONE;
#line 185
    __nesc_atomic_end(__nesc_atomic); }
  TOS_post(CC2420ControlM$taskInitDone);
  return SUCCESS;
}

# 64 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static  result_t CC2420RadioM$CC2420SplitControl$init(void){
#line 64
  unsigned char result;
#line 64

#line 64
  result = CC2420ControlM$SplitControl$init();
#line 64

#line 64
  return result;
#line 64
}
#line 64
static inline   
# 59 "C:/cygwin/opt/tinyos-1.x/tos/system/RandomLFSR.nc"
result_t RandomLFSR$Random$init(void)
#line 59
{
  {
  }
#line 60
  ;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 61
    {
      RandomLFSR$shiftReg = 119 * 119 * (TOS_LOCAL_ADDRESS + 1);
      RandomLFSR$initSeed = RandomLFSR$shiftReg;
      RandomLFSR$mask = 137 * 29 * (TOS_LOCAL_ADDRESS + 1);
    }
#line 65
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 57 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Random.nc"
inline static   result_t CC2420RadioM$Random$init(void){
#line 57
  unsigned char result;
#line 57

#line 57
  result = RandomLFSR$Random$init();
#line 57

#line 57
  return result;
#line 57
}
#line 57
static inline   
# 423 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$ControlB4$disableEvents(void)
#line 423
{
#line 423
  MSP430TimerM$TBCCTL4 &= ~0x0010;
}

# 39 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerJiffyAsyncM$AlarmControl$disableEvents(void){
#line 39
  MSP430TimerM$ControlB4$disableEvents();
#line 39
}
#line 39
static inline 
# 95 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$CC2int(MSP430TimerM$CC_t x)
#line 95
{
#line 95
  union __nesc_unnamed4298 {
#line 95
    MSP430TimerM$CC_t f;
#line 95
    uint16_t t;
  } 
#line 95
  c = { .f = x };

#line 95
  return c.t;
}

static inline uint16_t MSP430TimerM$compareControl(void)
{
  MSP430TimerM$CC_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return MSP430TimerM$CC2int(x);
}

static inline   
#line 383
void MSP430TimerM$ControlB4$setControlAsCompare(void)
#line 383
{
#line 383
  MSP430TimerM$TBCCTL4 = MSP430TimerM$compareControl();
}

# 35 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerJiffyAsyncM$AlarmControl$setControlAsCompare(void){
#line 35
  MSP430TimerM$ControlB4$setControlAsCompare();
#line 35
}
#line 35
static inline  
# 16 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/TimerJiffyAsyncM.nc"
result_t TimerJiffyAsyncM$StdControl$init(void)
{
  TimerJiffyAsyncM$AlarmControl$setControlAsCompare();
  TimerJiffyAsyncM$AlarmControl$disableEvents();
  return SUCCESS;
}

# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t CC2420RadioM$TimerControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = TimerJiffyAsyncM$StdControl$init();
#line 63

#line 63
  return result;
#line 63
}
#line 63
static inline  
# 191 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$SplitControl$init(void)
#line 191
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 193
    {
      CC2420RadioM$stateRadio = CC2420RadioM$DISABLED_STATE;
      CC2420RadioM$currentDSN = 0;
      CC2420RadioM$bAckEnable = FALSE;
      CC2420RadioM$bPacketReceiving = FALSE;
      CC2420RadioM$rxbufptr = &CC2420RadioM$RxBuf;
      CC2420RadioM$rxbufptr->length = 0;
    }
#line 200
    __nesc_atomic_end(__nesc_atomic); }

  CC2420RadioM$TimerControl$init();
  CC2420RadioM$Random$init();
  CC2420RadioM$LocalAddr = TOS_LOCAL_ADDRESS;
  return CC2420RadioM$CC2420SplitControl$init();
}

static inline  
#line 186
result_t CC2420RadioM$StdControl$init(void)
#line 186
{
  return CC2420RadioM$SplitControl$init();
}

# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t AMStandard$RadioControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = CC2420RadioM$StdControl$init();
#line 63

#line 63
  return result;
#line 63
}
#line 63
inline static  result_t FramerM$ByteControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = UARTM$Control$init();
#line 63

#line 63
  return result;
#line 63
}
#line 63
static inline  
# 291 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
result_t FramerM$StdControl$init(void)
#line 291
{
  FramerM$HDLCInitialize();
  return FramerM$ByteControl$init();
}

# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t AMStandard$UARTControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = FramerM$StdControl$init();
#line 63

#line 63
  return result;
#line 63
}
#line 63
inline static  result_t AMStandard$TimerControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = TimerM$StdControl$init();
#line 63

#line 63
  return result;
#line 63
}
#line 63
static inline  
# 87 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
bool AMStandard$Control$init(void)
#line 87
{
  result_t ok1;
#line 88
  result_t ok2;

  AMStandard$TimerControl$init();
  ok1 = AMStandard$UARTControl$init();
  ok2 = AMStandard$RadioControl$init();

  AMStandard$state = FALSE;
  AMStandard$lastCount = 0;
  AMStandard$counter = 0;
  {
  }
#line 97
  ;

  return rcombine(ok1, ok2);
}

# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t BSN_RadioRangeM$CommControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = AMStandard$Control$init();
#line 63

#line 63
  return result;
#line 63
}
#line 63
static inline   
# 110 "C:/cygwin/opt/tinyos-1.x/tos/system/LedsC.nc"
result_t LedsC$Leds$greenOff(void)
#line 110
{
  {
  }
#line 111
  ;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    {
      TOSH_SET_GREEN_LED_PIN();
      LedsC$ledsOn &= ~LedsC$GREEN_BIT;
    }
#line 115
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 97 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Leds.nc"
inline static   result_t BSN_RadioRangeM$Leds$greenOff(void){
#line 97
  unsigned char result;
#line 97

#line 97
  result = LedsC$Leds$greenOff();
#line 97

#line 97
  return result;
#line 97
}
#line 97
static inline   
# 81 "C:/cygwin/opt/tinyos-1.x/tos/system/LedsC.nc"
result_t LedsC$Leds$redOff(void)
#line 81
{
  {
  }
#line 82
  ;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 83
    {
      TOSH_SET_RED_LED_PIN();
      LedsC$ledsOn &= ~LedsC$RED_BIT;
    }
#line 86
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 72 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Leds.nc"
inline static   result_t BSN_RadioRangeM$Leds$redOff(void){
#line 72
  unsigned char result;
#line 72

#line 72
  result = LedsC$Leds$redOff();
#line 72

#line 72
  return result;
#line 72
}
#line 72
static inline   
# 139 "C:/cygwin/opt/tinyos-1.x/tos/system/LedsC.nc"
result_t LedsC$Leds$yellowOff(void)
#line 139
{
  {
  }
#line 140
  ;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 141
    {
      TOSH_SET_YELLOW_LED_PIN();
      LedsC$ledsOn &= ~LedsC$YELLOW_BIT;
    }
#line 144
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 122 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Leds.nc"
inline static   result_t BSN_RadioRangeM$Leds$yellowOff(void){
#line 122
  unsigned char result;
#line 122

#line 122
  result = LedsC$Leds$yellowOff();
#line 122

#line 122
  return result;
#line 122
}
#line 122
static inline   
# 58 "C:/cygwin/opt/tinyos-1.x/tos/system/LedsC.nc"
result_t LedsC$Leds$init(void)
#line 58
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 59
    {
      LedsC$ledsOn = 0;
      {
      }
#line 61
      ;
      TOSH_MAKE_RED_LED_OUTPUT();
      TOSH_MAKE_YELLOW_LED_OUTPUT();
      TOSH_MAKE_GREEN_LED_OUTPUT();
      TOSH_SET_RED_LED_PIN();
      TOSH_SET_YELLOW_LED_PIN();
      TOSH_SET_GREEN_LED_PIN();
    }
#line 68
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 56 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Leds.nc"
inline static   result_t BSN_RadioRangeM$Leds$init(void){
#line 56
  unsigned char result;
#line 56

#line 56
  result = LedsC$Leds$init();
#line 56

#line 56
  return result;
#line 56
}
#line 56
static inline  
# 45 "BSN_RadioRangeM.nc"
result_t BSN_RadioRangeM$StdControl$init(void)
#line 45
{
  BSN_RadioRangeM$Leds$init();
  BSN_RadioRangeM$Leds$yellowOff();
#line 47
  BSN_RadioRangeM$Leds$redOff();
#line 47
  BSN_RadioRangeM$Leds$greenOff();


  BSN_RadioRangeM$CommControl$init();
  BSN_RadioRangeM$UARTControl$init();

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 53
    {
      BSN_RadioRangeM$currentMsg = 0;
      BSN_RadioRangeM$count = 0;
      BSN_RadioRangeM$UARTConnected = 0;
      BSN_RadioRangeM$state = BSN_RadioRangeM$REGISTER;
      BSN_RadioRangeM$sentRadio = 0;
      BSN_RadioRangeM$strength = 0;
      BSN_RadioRangeM$heartbeatfromUART = 0;
      BSN_RadioRangeM$parent = TOS_BCAST_ADDR;
    }
#line 62
    __nesc_atomic_end(__nesc_atomic); }

  {
  }
#line 64
  ;
  return SUCCESS;
}

# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t MainM$StdControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = BSN_RadioRangeM$StdControl$init();
#line 63
  result = rcombine(result, TimerM$StdControl$init());
#line 63

#line 63
  return result;
#line 63
}
#line 63
static inline   
# 382 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$ControlB3$setControlAsCompare(void)
#line 382
{
#line 382
  MSP430TimerM$TBCCTL3 = MSP430TimerM$compareControl();
}

# 35 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerM$AlarmControl$setControlAsCompare(void){
#line 35
  MSP430TimerM$ControlB3$setControlAsCompare();
#line 35
}
#line 35
static inline   
# 422 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$ControlB3$disableEvents(void)
#line 422
{
#line 422
  MSP430TimerM$TBCCTL3 &= ~0x0010;
}

# 39 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerM$AlarmControl$disableEvents(void){
#line 39
  MSP430TimerM$ControlB3$disableEvents();
#line 39
}
#line 39
static inline   
# 114 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
bool HPLUSART0M$USARTControl$isI2C(void)
#line 114
{
  bool _ret = FALSE;






  return _ret;
}

static inline   
#line 72
bool HPLUSART0M$USARTControl$isSPI(void)
#line 72
{
  bool _ret = FALSE;

#line 74
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 74
    {
      if (HPLUSART0M$ME1 & (1 << 6)) {
        _ret = TRUE;
        }
    }
#line 78
    __nesc_atomic_end(__nesc_atomic); }
#line 78
  return _ret;
}

static inline 
# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
bool TOSH_IS_URXD0_IOFUNC(void)
#line 37
{
   
#line 37
  static volatile uint8_t r __asm ("0x001B");

#line 37
  return r | ~(1 << 5);
}

static inline 
#line 36
bool TOSH_IS_UTXD0_MODFUNC(void)
#line 36
{
   
#line 36
  static volatile uint8_t r __asm ("0x001B");

#line 36
  return r & (1 << 4);
}

static inline   
# 92 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
bool HPLUSART0M$USARTControl$isUARTtx(void)
#line 92
{
  bool _ret = FALSE;

#line 94
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 94
    {

      if (
#line 95
      HPLUSART0M$ME1 & (1 << 7) && 
      TOSH_IS_UTXD0_MODFUNC() && 
      TOSH_IS_URXD0_IOFUNC()) {
        _ret = TRUE;
        }
    }
#line 100
    __nesc_atomic_end(__nesc_atomic); }
#line 100
  return _ret;
}

static inline 
# 36 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
bool TOSH_IS_UTXD0_IOFUNC(void)
#line 36
{
   
#line 36
  static volatile uint8_t r __asm ("0x001B");

#line 36
  return r | ~(1 << 4);
}

static inline 
#line 37
bool TOSH_IS_URXD0_MODFUNC(void)
#line 37
{
   
#line 37
  static volatile uint8_t r __asm ("0x001B");

#line 37
  return r & (1 << 5);
}

static inline   
# 103 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
bool HPLUSART0M$USARTControl$isUARTrx(void)
#line 103
{
  bool _ret = FALSE;

#line 105
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 105
    {

      if (
#line 106
      HPLUSART0M$ME1 & (1 << 6) && 
      TOSH_IS_URXD0_MODFUNC() && 
      TOSH_IS_UTXD0_IOFUNC()) {
        _ret = TRUE;
        }
    }
#line 111
    __nesc_atomic_end(__nesc_atomic); }
#line 111
  return _ret;
}

static inline   
#line 81
bool HPLUSART0M$USARTControl$isUART(void)
#line 81
{
  bool _ret = FALSE;

#line 83
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 83
    {

      if (
#line 84
      HPLUSART0M$ME1 & (1 << 7) && HPLUSART0M$ME1 & (1 << 6) && 
      TOSH_IS_URXD0_MODFUNC() && 
      TOSH_IS_UTXD0_MODFUNC()) {
        _ret = TRUE;
        }
    }
#line 89
    __nesc_atomic_end(__nesc_atomic); }
#line 89
  return _ret;
}

static inline   
#line 125
msp430_usartmode_t HPLUSART0M$USARTControl$getMode(void)
#line 125
{
  if (HPLUSART0M$USARTControl$isUART()) {
    return USART_UART;
    }
  else {
#line 128
    if (HPLUSART0M$USARTControl$isUARTrx()) {
      return USART_UART_RX;
      }
    else {
#line 130
      if (HPLUSART0M$USARTControl$isUARTtx()) {
        return USART_UART_TX;
        }
      else {
#line 132
        if (HPLUSART0M$USARTControl$isSPI()) {
          return USART_SPI;
          }
        else {
#line 134
          if (HPLUSART0M$USARTControl$isI2C()) {
            return USART_I2C;
            }
          else {
#line 137
            return USART_NONE;
            }
          }
        }
      }
    }
}

static inline 
# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
void TOSH_SEL_URXD0_IOFUNC(void)
#line 37
{
   
#line 37
  static volatile uint8_t r __asm ("0x001B");

#line 37
  r &= ~(1 << 5);
}

static inline 
#line 36
void TOSH_SEL_UTXD0_IOFUNC(void)
#line 36
{
   
#line 36
  static volatile uint8_t r __asm ("0x001B");

#line 36
  r &= ~(1 << 4);
}

static inline   
# 172 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
void HPLUSART0M$USARTControl$disableUART(void)
#line 172
{
  HPLUSART0M$ME1 &= ~((1 << 7) | (1 << 6));
  TOSH_SEL_UTXD0_IOFUNC();
  TOSH_SEL_URXD0_IOFUNC();
}

static inline   
#line 218
void HPLUSART0M$USARTControl$disableI2C(void)
#line 218
{
}

static inline 
# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
void TOSH_SEL_SIMO0_MODFUNC(void)
#line 34
{
   
#line 34
  static volatile uint8_t r __asm ("0x001B");

#line 34
  r |= 1 << 1;
}

static inline 
#line 33
void TOSH_SEL_SOMI0_MODFUNC(void)
#line 33
{
   
#line 33
  static volatile uint8_t r __asm ("0x001B");

#line 33
  r |= 1 << 2;
}

static inline 
#line 35
void TOSH_SEL_UCLK0_MODFUNC(void)
#line 35
{
   
#line 35
  static volatile uint8_t r __asm ("0x001B");

#line 35
  r |= 1 << 3;
}

static inline  
# 87 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
result_t TimerM$StdControl$start(void)
{
  return SUCCESS;
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Timer.nc"
inline static  result_t BSN_RadioRangeM$Timer$start(char arg_0xa7b9cd0, uint32_t arg_0xa7b9e28){
#line 59
  unsigned char result;
#line 59

#line 59
  result = TimerM$Timer$start(0, arg_0xa7b9cd0, arg_0xa7b9e28);
#line 59

#line 59
  return result;
#line 59
}
#line 59
static inline 
# 133 "C:/cygwin/opt/tinyos-1.x/tos/system/tos.h"
result_t rcombine4(result_t r1, result_t r2, result_t r3, 
result_t r4)
{
  return rcombine(r1, rcombine(r2, rcombine(r3, r4)));
}

static inline   
# 53 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLPowerManagementM.nc"
uint8_t HPLPowerManagementM$PowerManagement$adjustPower(void)
#line 53
{
  return SUCCESS;
}

# 41 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/PowerManagement.nc"
inline static   uint8_t AMStandard$PowerManagement$adjustPower(void){
#line 41
  unsigned char result;
#line 41

#line 41
  result = HPLPowerManagementM$PowerManagement$adjustPower();
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 59 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Timer.nc"
inline static  result_t AMStandard$ActivityTimer$start(char arg_0xa7b9cd0, uint32_t arg_0xa7b9e28){
#line 59
  unsigned char result;
#line 59

#line 59
  result = TimerM$Timer$start(1, arg_0xa7b9cd0, arg_0xa7b9e28);
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 47 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static   uint8_t CC2420ControlM$HPLChipcon$cmd(uint8_t arg_0xa8b5d30){
#line 47
  unsigned char result;
#line 47

#line 47
  result = HPLCC2420M$HPLCC2420$cmd(arg_0xa8b5d30);
#line 47

#line 47
  return result;
#line 47
}
#line 47
static inline   
# 119 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port14$enable(void)
#line 119
{
#line 119
  MSP430InterruptM$P1IE |= 1 << 4;
}

# 30 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$CCAInterrupt$enable(void){
#line 30
  MSP430InterruptM$Port14$enable();
#line 30
}
#line 30
static inline   
# 245 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port14$edge(bool l2h)
#line 245
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 246
    {
      if (l2h) {
#line 247
        P1IES &= ~(1 << 4);
        }
      else {
#line 248
        P1IES |= 1 << 4;
        }
    }
#line 250
    __nesc_atomic_end(__nesc_atomic); }
}

# 54 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$CCAInterrupt$edge(bool arg_0xa9d1b80){
#line 54
  MSP430InterruptM$Port14$edge(arg_0xa9d1b80);
#line 54
}
#line 54
static inline   
# 181 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port14$clear(void)
#line 181
{
#line 181
  MSP430InterruptM$P1IFG &= ~(1 << 4);
}

# 40 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$CCAInterrupt$clear(void){
#line 40
  MSP430InterruptM$Port14$clear();
#line 40
}
#line 40
static inline   
# 150 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port14$disable(void)
#line 150
{
#line 150
  MSP430InterruptM$P1IE &= ~(1 << 4);
}

# 35 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$CCAInterrupt$disable(void){
#line 35
  MSP430InterruptM$Port14$disable();
#line 35
}
#line 35
static inline   
# 147 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420InterruptM.nc"
result_t HPLCC2420InterruptM$CCA$startWait(bool low_to_high)
#line 147
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 148
    {
      HPLCC2420InterruptM$CCAInterrupt$disable();
      HPLCC2420InterruptM$CCAInterrupt$clear();
      HPLCC2420InterruptM$CCAInterrupt$edge(low_to_high);
      HPLCC2420InterruptM$CCAInterrupt$enable();
    }
#line 153
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 43 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
inline static   result_t CC2420ControlM$CCA$startWait(bool arg_0xa8da5c0){
#line 43
  unsigned char result;
#line 43

#line 43
  result = HPLCC2420InterruptM$CCA$startWait(arg_0xa8da5c0);
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 54 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static   uint8_t CC2420ControlM$HPLChipcon$write(uint8_t arg_0xa8aa1a0, uint16_t arg_0xa8aa2f0){
#line 54
  unsigned char result;
#line 54

#line 54
  result = HPLCC2420M$HPLCC2420$write(arg_0xa8aa1a0, arg_0xa8aa2f0);
#line 54

#line 54
  return result;
#line 54
}
#line 54
static inline   
# 368 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
result_t CC2420ControlM$CC2420Control$OscillatorOn(void)
#line 368
{
  uint16_t i;
  uint8_t status;

  i = 0;
#line 384
  CC2420ControlM$HPLChipcon$write(0x1D, 24);


  CC2420ControlM$CCA$startWait(TRUE);


  status = CC2420ControlM$HPLChipcon$cmd(0x01);

  return SUCCESS;
}

static 
# 161 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
__inline void TOSH_wait(void )
{
   __asm volatile ("nop"); __asm volatile ("nop");}

static inline 
# 30 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
void TOSH_SET_CC_RSTN_PIN(void)
#line 30
{
   
#line 30
  static volatile uint8_t r __asm ("0x001D");

#line 30
  r |= 1 << 6;
}

static inline 
#line 30
void TOSH_CLR_CC_RSTN_PIN(void)
#line 30
{
   
#line 30
  static volatile uint8_t r __asm ("0x001D");

#line 30
  r &= ~(1 << 6);
}

static 
# 174 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
__inline void TOSH_uwait(uint16_t u)
{
  uint16_t i;

#line 177
  if (u < 500) {
    for (i = 2; i < u; i++) {
         __asm volatile ("nop\n\t"
        "nop\n\t"
        "nop\n\t"
        "nop\n\t");}
    }
  else {

    for (i = 0; i < u; i++) {
         __asm volatile ("nop\n\t"
        "nop\n\t"
        "nop\n\t"
        "nop\n\t");}
    }
}

static inline 
# 29 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
void TOSH_SET_CC_VREN_PIN(void)
#line 29
{
   
#line 29
  static volatile uint8_t r __asm ("0x001D");

#line 29
  r |= 1 << 5;
}

static inline   
# 400 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
result_t CC2420ControlM$CC2420Control$VREFOn(void)
#line 400
{
  TOSH_SET_CC_VREN_PIN();

  TOSH_uwait(600);
  return SUCCESS;
}

static inline  
# 91 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420M.nc"
result_t HPLCC2420M$StdControl$start(void)
#line 91
{
  TOSH_SET_RADIO_CSN_PIN();
  TOSH_MAKE_RADIO_CSN_OUTPUT();
  HPLCC2420M$USARTControl$setModeSPI();
  HPLCC2420M$USARTControl$disableRxIntr();
  HPLCC2420M$USARTControl$disableTxIntr();
  return SUCCESS;
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t CC2420ControlM$HPLChipconControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = HPLCC2420M$StdControl$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
static inline  
# 227 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
result_t CC2420ControlM$SplitControl$start(void)
#line 227
{
  result_t status;
  uint8_t _state = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 231
    {
      if (CC2420ControlM$state == CC2420ControlM$INIT_STATE_DONE) {
          CC2420ControlM$state = CC2420ControlM$START_STATE;
          _state = TRUE;
        }
    }
#line 236
    __nesc_atomic_end(__nesc_atomic); }
  if (!_state) {
    return FAIL;
    }
  CC2420ControlM$HPLChipconControl$start();

  CC2420ControlM$CC2420Control$VREFOn();

  TOSH_CLR_CC_RSTN_PIN();
  TOSH_wait();
  TOSH_SET_CC_RSTN_PIN();
  TOSH_wait();


  status = CC2420ControlM$CC2420Control$OscillatorOn();

  return status;
}

# 77 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static  result_t CC2420RadioM$CC2420SplitControl$start(void){
#line 77
  unsigned char result;
#line 77

#line 77
  result = CC2420ControlM$SplitControl$start();
#line 77

#line 77
  return result;
#line 77
}
#line 77
static inline  
# 23 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/TimerJiffyAsyncM.nc"
result_t TimerJiffyAsyncM$StdControl$start(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 25
    {
      TimerJiffyAsyncM$bSet = FALSE;
      TimerJiffyAsyncM$AlarmControl$disableEvents();
    }
#line 28
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t CC2420RadioM$TimerControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = TimerJiffyAsyncM$StdControl$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
static inline  
# 277 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$SplitControl$start(void)
#line 277
{
  uint8_t chkstateRadio;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 280
    chkstateRadio = CC2420RadioM$stateRadio;
#line 280
    __nesc_atomic_end(__nesc_atomic); }

  if (chkstateRadio == CC2420RadioM$DISABLED_STATE) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 283
        {
          CC2420RadioM$stateRadio = CC2420RadioM$WARMUP_STATE;
          CC2420RadioM$countRetry = 0;
          CC2420RadioM$rxbufptr->length = 0;
        }
#line 287
        __nesc_atomic_end(__nesc_atomic); }
      CC2420RadioM$TimerControl$start();
      return CC2420RadioM$CC2420SplitControl$start();
    }
  return FAIL;
}

static inline  
#line 239
void CC2420RadioM$startRadio(void)
#line 239
{
  result_t success = FAIL;

#line 241
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 241
    {
      if (CC2420RadioM$stateRadio == CC2420RadioM$DISABLED_STATE_STARTTASK) {
          CC2420RadioM$stateRadio = CC2420RadioM$DISABLED_STATE;
          success = SUCCESS;
        }
    }
#line 246
    __nesc_atomic_end(__nesc_atomic); }

  if (success == SUCCESS) {
    CC2420RadioM$SplitControl$start();
    }
}

static inline  result_t CC2420RadioM$StdControl$start(void)
#line 253
{







  result_t success = FAIL;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 263
    {
      if (CC2420RadioM$stateRadio == CC2420RadioM$DISABLED_STATE) {

          if (TOS_post(CC2420RadioM$startRadio)) {
              success = SUCCESS;
              CC2420RadioM$stateRadio = CC2420RadioM$DISABLED_STATE_STARTTASK;
            }
        }
    }
#line 271
    __nesc_atomic_end(__nesc_atomic); }

  return success;
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t AMStandard$RadioControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = CC2420RadioM$StdControl$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
# 62 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/HPLUART.nc"
inline static   result_t UARTM$HPLUART$init(void){
#line 62
  unsigned char result;
#line 62

#line 62
  result = HPLUARTM$UART$init();
#line 62

#line 62
  return result;
#line 62
}
#line 62
static inline  
# 68 "C:/cygwin/opt/tinyos-1.x/tos/system/UARTM.nc"
result_t UARTM$Control$start(void)
#line 68
{
  return UARTM$HPLUART$init();
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t FramerM$ByteControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = UARTM$Control$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
static inline  
# 296 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
result_t FramerM$StdControl$start(void)
#line 296
{
  FramerM$HDLCInitialize();
  return FramerM$ByteControl$start();
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t AMStandard$UARTControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = FramerM$StdControl$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
inline static  result_t AMStandard$TimerControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = TimerM$StdControl$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
static inline  
# 103 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
bool AMStandard$Control$start(void)
#line 103
{
  result_t ok0 = AMStandard$TimerControl$start();
  result_t ok1 = AMStandard$UARTControl$start();
  result_t ok2 = AMStandard$RadioControl$start();
  result_t ok3 = AMStandard$ActivityTimer$start(TIMER_REPEAT, 1000);



  AMStandard$state = FALSE;

  AMStandard$PowerManagement$adjustPower();

  return rcombine4(ok0, ok1, ok2, ok3);
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t BSN_RadioRangeM$CommControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = AMStandard$Control$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
inline static  result_t NoCRCPacket$ByteControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = UARTM$Control$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
static inline  
# 111 "C:/cygwin/opt/tinyos-1.x/tos/system/NoCRCPacket.nc"
result_t NoCRCPacket$Control$start(void)
#line 111
{

  return NoCRCPacket$ByteControl$start();
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t BSN_RadioRangeM$UARTControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = NoCRCPacket$Control$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
static inline  
# 70 "BSN_RadioRangeM.nc"
result_t BSN_RadioRangeM$StdControl$start(void)
#line 70
{
  BSN_RadioRangeM$UARTControl$start();
  BSN_RadioRangeM$CommControl$start();
  BSN_RadioRangeM$Timer$start(TIMER_REPEAT, 2234);
  return SUCCESS;
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t MainM$StdControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = BSN_RadioRangeM$StdControl$start();
#line 70
  result = rcombine(result, TimerM$StdControl$start());
#line 70

#line 70
  return result;
#line 70
}
#line 70
static inline 
# 252 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
void HPLUSART1M$setUARTModeCommon(void)
#line 252
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 253
    {
      U1CTL = 0x01;
      U1CTL |= 0x10;

      U1RCTL &= ~0x08;

      U1CTL = 0x01;
      U1CTL |= 0x10;

      if (HPLUSART1M$l_ssel & 0x80) {
          HPLUSART1M$U1TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
          HPLUSART1M$U1TCTL |= HPLUSART1M$l_ssel & 0x7F;
        }
      else {
          HPLUSART1M$U1TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
          HPLUSART1M$U1TCTL |= 0x10;
        }

      if (HPLUSART1M$l_mctl != 0 || HPLUSART1M$l_br != 0) {
          U1BR0 = HPLUSART1M$l_br & 0x0FF;
          U1BR1 = (HPLUSART1M$l_br >> 8) & 0x0FF;
          U1MCTL = HPLUSART1M$l_mctl;
        }
      else {
          U1BR0 = 0x03;
          U1BR1 = 0x00;
          U1MCTL = 0x4A;
        }

      HPLUSART1M$ME2 &= ~(1 << 4);
      HPLUSART1M$ME2 |= (1 << 5) | (1 << 4);

      U1CTL &= ~0x01;

      HPLUSART1M$IFG2 &= ~((1 << 5) | (1 << 4));
      IE2 &= ~((1 << 5) | (1 << 4));
    }
#line 289
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

static inline 
# 39 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
void TOSH_SEL_URXD1_MODFUNC(void)
#line 39
{
   
#line 39
  static volatile uint8_t r __asm ("0x001B");

#line 39
  r |= 1 << 7;
}

static inline 
#line 38
void TOSH_SEL_UTXD1_MODFUNC(void)
#line 38
{
   
#line 38
  static volatile uint8_t r __asm ("0x001B");

#line 38
  r |= 1 << 6;
}

static inline 
#line 39
void TOSH_SEL_URXD1_IOFUNC(void)
#line 39
{
   
#line 39
  static volatile uint8_t r __asm ("0x001B");

#line 39
  r &= ~(1 << 7);
}

static inline 
#line 38
void TOSH_SEL_UTXD1_IOFUNC(void)
#line 38
{
   
#line 38
  static volatile uint8_t r __asm ("0x001B");

#line 38
  r &= ~(1 << 6);
}

static inline   
# 158 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
void HPLUSART1M$USARTControl$disableUART(void)
#line 158
{
  HPLUSART1M$ME2 &= ~((1 << 5) | (1 << 4));
  TOSH_SEL_UTXD1_IOFUNC();
  TOSH_SEL_URXD1_IOFUNC();
}

static inline 
# 40 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
void TOSH_SEL_UCLK1_IOFUNC(void)
#line 40
{
   
#line 40
  static volatile uint8_t r __asm ("0x0033");

#line 40
  r &= ~(1 << 3);
}

static inline 
#line 41
void TOSH_SEL_SOMI1_IOFUNC(void)
#line 41
{
   
#line 41
  static volatile uint8_t r __asm ("0x0033");

#line 41
  r &= ~(1 << 2);
}

static inline 
#line 42
void TOSH_SEL_SIMO1_IOFUNC(void)
#line 42
{
   
#line 42
  static volatile uint8_t r __asm ("0x0033");

#line 42
  r &= ~(1 << 1);
}

static inline   
# 191 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
void HPLUSART1M$USARTControl$disableSPI(void)
#line 191
{
  HPLUSART1M$ME2 &= ~(1 << 4);
  TOSH_SEL_SIMO1_IOFUNC();
  TOSH_SEL_SOMI1_IOFUNC();
  TOSH_SEL_UCLK1_IOFUNC();
}

static inline   
#line 107
bool HPLUSART1M$USARTControl$isI2C(void)
#line 107
{
  return FALSE;
}

static inline   
#line 64
bool HPLUSART1M$USARTControl$isSPI(void)
#line 64
{
  bool _ret = FALSE;

#line 66
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 66
    {
      if (HPLUSART1M$ME2 & (1 << 4)) {
        _ret = TRUE;
        }
    }
#line 70
    __nesc_atomic_end(__nesc_atomic); }
#line 70
  return _ret;
}

static inline 
# 39 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
bool TOSH_IS_URXD1_IOFUNC(void)
#line 39
{
   
#line 39
  static volatile uint8_t r __asm ("0x001B");

#line 39
  return r | ~(1 << 7);
}

static inline 
#line 38
bool TOSH_IS_UTXD1_MODFUNC(void)
#line 38
{
   
#line 38
  static volatile uint8_t r __asm ("0x001B");

#line 38
  return r & (1 << 6);
}

static inline   
# 84 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
bool HPLUSART1M$USARTControl$isUARTtx(void)
#line 84
{
  bool _ret = FALSE;

#line 86
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 86
    {

      if (
#line 87
      HPLUSART1M$ME2 & (1 << 5) && 
      TOSH_IS_UTXD1_MODFUNC() && 
      TOSH_IS_URXD1_IOFUNC()) {
        _ret = TRUE;
        }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
#line 92
  return _ret;
}

static inline 
# 38 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
bool TOSH_IS_UTXD1_IOFUNC(void)
#line 38
{
   
#line 38
  static volatile uint8_t r __asm ("0x001B");

#line 38
  return r | ~(1 << 6);
}

static inline 
#line 39
bool TOSH_IS_URXD1_MODFUNC(void)
#line 39
{
   
#line 39
  static volatile uint8_t r __asm ("0x001B");

#line 39
  return r & (1 << 7);
}

static inline   
# 95 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
bool HPLUSART1M$USARTControl$isUARTrx(void)
#line 95
{
  bool _ret = FALSE;

#line 97
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 97
    {

      if (
#line 98
      HPLUSART1M$ME2 & (1 << 4) && 
      TOSH_IS_URXD1_MODFUNC() && 
      TOSH_IS_UTXD1_IOFUNC()) {
        _ret = TRUE;
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
#line 103
  return _ret;
}

static inline   
#line 73
bool HPLUSART1M$USARTControl$isUART(void)
#line 73
{
  bool _ret = FALSE;

#line 75
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 75
    {

      if (
#line 76
      HPLUSART1M$ME2 & (1 << 5) && HPLUSART1M$ME2 & (1 << 4) && 
      TOSH_IS_URXD1_MODFUNC() && 
      TOSH_IS_UTXD1_MODFUNC()) {
        _ret = TRUE;
        }
    }
#line 81
    __nesc_atomic_end(__nesc_atomic); }
#line 81
  return _ret;
}

static inline   
#line 111
msp430_usartmode_t HPLUSART1M$USARTControl$getMode(void)
#line 111
{
  if (HPLUSART1M$USARTControl$isUART()) {
    return USART_UART;
    }
  else {
#line 114
    if (HPLUSART1M$USARTControl$isUARTrx()) {
      return USART_UART_RX;
      }
    else {
#line 116
      if (HPLUSART1M$USARTControl$isUARTtx()) {
        return USART_UART_TX;
        }
      else {
#line 118
        if (HPLUSART1M$USARTControl$isSPI()) {
          return USART_SPI;
          }
        else {
#line 120
          if (HPLUSART1M$USARTControl$isI2C()) {
            return USART_I2C;
            }
          else {
#line 123
            return USART_NONE;
            }
          }
        }
      }
    }
}

static inline   
#line 325
void HPLUSART1M$USARTControl$setModeUART(void)
#line 325
{

  if (HPLUSART1M$USARTControl$getMode() == USART_UART) {
    return;
    }
  HPLUSART1M$USARTControl$disableSPI();
  HPLUSART1M$USARTControl$disableUART();

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 333
    {
      TOSH_SEL_UTXD1_MODFUNC();
      TOSH_SEL_URXD1_MODFUNC();
    }
#line 336
    __nesc_atomic_end(__nesc_atomic); }
  HPLUSART1M$setUARTModeCommon();
  return;
}

# 153 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   void HPLUARTM$USARTControl$setModeUART(void){
#line 153
  HPLUSART1M$USARTControl$setModeUART();
#line 153
}
#line 153
static inline   
# 341 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
void HPLUSART1M$USARTControl$setClockSource(uint8_t source)
#line 341
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 342
    {
      HPLUSART1M$l_ssel = source | 0x80;
      HPLUSART1M$U1TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
      HPLUSART1M$U1TCTL |= HPLUSART1M$l_ssel & 0x7F;
    }
#line 346
    __nesc_atomic_end(__nesc_atomic); }
}

# 167 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   void HPLUARTM$USARTControl$setClockSource(uint8_t arg_0xa96e010){
#line 167
  HPLUSART1M$USARTControl$setClockSource(arg_0xa96e010);
#line 167
}
#line 167
static inline   
# 349 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
void HPLUSART1M$USARTControl$setClockRate(uint16_t baudrate, uint8_t mctl)
#line 349
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 350
    {
      HPLUSART1M$l_br = baudrate;
      HPLUSART1M$l_mctl = mctl;
      U1BR0 = baudrate & 0x0FF;
      U1BR1 = (baudrate >> 8) & 0x0FF;
      U1MCTL = mctl;
    }
#line 356
    __nesc_atomic_end(__nesc_atomic); }
}

# 169 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   void HPLUARTM$USARTControl$setClockRate(uint16_t arg_0xa96e430, uint8_t arg_0xa96e578){
#line 169
  HPLUSART1M$USARTControl$setClockRate(arg_0xa96e430, arg_0xa96e578);
#line 169
}
#line 169
static inline   
# 392 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
result_t HPLUSART1M$USARTControl$enableRxIntr(void)
#line 392
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 393
    {
      HPLUSART1M$IFG2 &= ~(1 << 4);
      IE2 |= 1 << 4;
    }
#line 396
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 174 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLUARTM$USARTControl$enableRxIntr(void){
#line 174
  unsigned char result;
#line 174

#line 174
  result = HPLUSART1M$USARTControl$enableRxIntr();
#line 174

#line 174
  return result;
#line 174
}
#line 174
static inline   
# 400 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
result_t HPLUSART1M$USARTControl$enableTxIntr(void)
#line 400
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 401
    {
      HPLUSART1M$IFG2 &= ~(1 << 5);
      IE2 |= 1 << 5;
    }
#line 404
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 175 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLUARTM$USARTControl$enableTxIntr(void){
#line 175
  unsigned char result;
#line 175

#line 175
  result = HPLUSART1M$USARTControl$enableTxIntr();
#line 175

#line 175
  return result;
#line 175
}
#line 175
static inline 
# 75 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420M.nc"
uint8_t HPLCC2420M$adjustStatusByte(uint8_t status)
#line 75
{
  return status & 0x7E;
}

static inline  
#line 493
result_t HPLCC2420M$BusArbitration$busFree(void)
#line 493
{
  return SUCCESS;
}

static inline   
# 125 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/BusArbitrationM.nc"
result_t BusArbitrationM$BusArbitration$default$busFree(uint8_t id)
#line 125
{
  return SUCCESS;
}

# 39 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/BusArbitration.nc"
inline static  result_t BusArbitrationM$BusArbitration$busFree(uint8_t arg_0xaa45c10){
#line 39
  unsigned char result;
#line 39

#line 39
  switch (arg_0xaa45c10) {
#line 39
    case 0:
#line 39
      result = HPLCC2420M$BusArbitration$busFree();
#line 39
      break;
#line 39
    default:
#line 39
      result = BusArbitrationM$BusArbitration$default$busFree(arg_0xaa45c10);
#line 39
    }
#line 39

#line 39
  return result;
#line 39
}
#line 39
static inline  
# 42 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/BusArbitrationM.nc"
void BusArbitrationM$busReleased(void)
#line 42
{
  uint8_t i;
  uint8_t currentstate;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    BusArbitrationM$isBusReleasedPending = FALSE;
#line 46
    __nesc_atomic_end(__nesc_atomic); }
  for (i = 0; i < 1; i++) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 48
        currentstate = BusArbitrationM$state;
#line 48
        __nesc_atomic_end(__nesc_atomic); }
      if (currentstate == BusArbitrationM$BUS_IDLE) {
        BusArbitrationM$BusArbitration$busFree(i);
        }
    }
}

static inline   
# 166 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$TimerB$read(void)
#line 166
{
#line 166
  return TBR;
}

# 30 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   uint16_t TimerM$AlarmTimer$read(void){
#line 30
  unsigned int result;
#line 30

#line 30
  result = MSP430TimerM$TimerB$read();
#line 30

#line 30
  return result;
#line 30
}
#line 30
static inline   
# 169 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
bool MSP430TimerM$TimerB$isOverflowPending(void)
#line 169
{
#line 169
  return TBCTL & 0x0001;
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   bool TimerM$AlarmTimer$isOverflowPending(void){
#line 31
  unsigned char result;
#line 31

#line 31
  result = MSP430TimerM$TimerB$isOverflowPending();
#line 31

#line 31
  return result;
#line 31
}
#line 31
# 58 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
inline static  result_t BSN_RadioRangeM$UARTSend$send(TOS_MsgPtr arg_0xa7b3928){
#line 58
  unsigned char result;
#line 58

#line 58
  result = NoCRCPacket$Send$send(arg_0xa7b3928);
#line 58

#line 58
  return result;
#line 58
}
#line 58
static inline 
# 12 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
void TOSH_CLR_GREEN_LED_PIN(void)
#line 12
{
   
#line 12
  static volatile uint8_t r __asm ("0x0031");

#line 12
  r &= ~(1 << 5);
}

static inline   
# 101 "C:/cygwin/opt/tinyos-1.x/tos/system/LedsC.nc"
result_t LedsC$Leds$greenOn(void)
#line 101
{
  {
  }
#line 102
  ;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 103
    {
      TOSH_CLR_GREEN_LED_PIN();
      LedsC$ledsOn |= LedsC$GREEN_BIT;
    }
#line 106
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

static inline   








result_t LedsC$Leds$greenToggle(void)
#line 119
{
  result_t rval;

#line 121
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 121
    {
      if (LedsC$ledsOn & LedsC$GREEN_BIT) {
        rval = LedsC$Leds$greenOff();
        }
      else {
#line 125
        rval = LedsC$Leds$greenOn();
        }
    }
#line 127
    __nesc_atomic_end(__nesc_atomic); }
#line 127
  return rval;
}

# 106 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Leds.nc"
inline static   result_t BSN_RadioRangeM$Leds$greenToggle(void){
#line 106
  unsigned char result;
#line 106

#line 106
  result = LedsC$Leds$greenToggle();
#line 106

#line 106
  return result;
#line 106
}
#line 106
static inline 
# 11 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
void TOSH_CLR_RED_LED_PIN(void)
#line 11
{
   
#line 11
  static volatile uint8_t r __asm ("0x0031");

#line 11
  r &= ~(1 << 4);
}

static inline   
# 72 "C:/cygwin/opt/tinyos-1.x/tos/system/LedsC.nc"
result_t LedsC$Leds$redOn(void)
#line 72
{
  {
  }
#line 73
  ;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 74
    {
      TOSH_CLR_RED_LED_PIN();
      LedsC$ledsOn |= LedsC$RED_BIT;
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

static inline   








result_t LedsC$Leds$redToggle(void)
#line 90
{
  result_t rval;

#line 92
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 92
    {
      if (LedsC$ledsOn & LedsC$RED_BIT) {
        rval = LedsC$Leds$redOff();
        }
      else {
#line 96
        rval = LedsC$Leds$redOn();
        }
    }
#line 98
    __nesc_atomic_end(__nesc_atomic); }
#line 98
  return rval;
}

# 81 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Leds.nc"
inline static   result_t BSN_RadioRangeM$Leds$redToggle(void){
#line 81
  unsigned char result;
#line 81

#line 81
  result = LedsC$Leds$redToggle();
#line 81

#line 81
  return result;
#line 81
}
#line 81
# 48 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SendMsg.nc"
inline static  result_t BSN_RadioRangeM$DataMsg$send(uint16_t arg_0xa7da410, uint8_t arg_0xa7da558, TOS_MsgPtr arg_0xa7da6a8){
#line 48
  unsigned char result;
#line 48

#line 48
  result = AMStandard$SendMsg$send(AM_OSCOPEMSG, arg_0xa7da410, arg_0xa7da558, arg_0xa7da6a8);
#line 48

#line 48
  return result;
#line 48
}
#line 48
static inline 
# 86 "BSN_RadioRangeM.nc"
uint8_t BSN_RadioRangeM$ToRadio(uint16_t destination, TOS_MsgPtr data)
{
  if (BSN_RadioRangeM$DataMsg$send(destination, sizeof(struct UbiMonMsg ), data)) 
    {
      return 1;
    }
  return 0;
}

static inline  

result_t BSN_RadioRangeM$Timer$fired(void)
#line 97
{
  if (BSN_RadioRangeM$state == BSN_RadioRangeM$REGISTER) 
    {
      struct UbiMonMsg *pack;

#line 101
      pack = (struct UbiMonMsg *)BSN_RadioRangeM$msg[BSN_RadioRangeM$currentMsg].data;
      pack->sourceID = TOS_LOCAL_ADDRESS;
      pack->BSNcommand = 0xffff;
      pack->destination = TOS_BCAST_ADDR;
      if (BSN_RadioRangeM$sentRadio) 
        {
          BSN_RadioRangeM$ToRadio(TOS_BCAST_ADDR, &BSN_RadioRangeM$msg[BSN_RadioRangeM$currentMsg]);
          BSN_RadioRangeM$Leds$redToggle();
        }
      else {
          pack->destination = 0xffff;
          BSN_RadioRangeM$msg[BSN_RadioRangeM$currentMsg].addr = TOS_UART_ADDR;
          BSN_RadioRangeM$Leds$greenToggle();
          BSN_RadioRangeM$UARTSend$send(&BSN_RadioRangeM$msg[BSN_RadioRangeM$currentMsg]);
        }
      BSN_RadioRangeM$sentRadio = !BSN_RadioRangeM$sentRadio;
    }
  return SUCCESS;
}

static inline  
# 151 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
result_t AMStandard$ActivityTimer$fired(void)
#line 151
{
  AMStandard$lastCount = AMStandard$counter;
  AMStandard$counter = 0;
  return SUCCESS;
}

static inline   
# 460 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
result_t TimerM$Timer$default$fired(uint8_t num)
{
  return SUCCESS;
}

# 73 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Timer.nc"
inline static  result_t TimerM$Timer$fired(uint8_t arg_0xa7c3720){
#line 73
  unsigned char result;
#line 73

#line 73
  switch (arg_0xa7c3720) {
#line 73
    case 0:
#line 73
      result = BSN_RadioRangeM$Timer$fired();
#line 73
      break;
#line 73
    case 1:
#line 73
      result = AMStandard$ActivityTimer$fired();
#line 73
      break;
#line 73
    default:
#line 73
      result = TimerM$Timer$default$fired(arg_0xa7c3720);
#line 73
    }
#line 73

#line 73
  return result;
#line 73
}
#line 73
static inline   
# 432 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
result_t TimerM$TimerMilli$default$fired(uint8_t num)
{
  return SUCCESS;
}

# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
inline static  result_t TimerM$TimerMilli$fired(uint8_t arg_0xa7c3da8){
#line 37
  unsigned char result;
#line 37

#line 37
    result = TimerM$TimerMilli$default$fired(arg_0xa7c3da8);
#line 37

#line 37
  return result;
#line 37
}
#line 37
static inline   
#line 383
result_t TimerM$TimerJiffy$default$fired(uint8_t num)
{
  return SUCCESS;
}

# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerJiffy.nc"
inline static  result_t TimerM$TimerJiffy$fired(uint8_t arg_0xa80a7e0){
#line 37
  unsigned char result;
#line 37

#line 37
    result = TimerM$TimerJiffy$default$fired(arg_0xa80a7e0);
#line 37

#line 37
  return result;
#line 37
}
#line 37
static inline 
#line 123
void TimerM$signal_timer_fired(uint8_t num)
{



  const int16_t num16 = num;

  if (TimerM$COUNT_TIMER_JIFFY > 0 && num16 >= TimerM$OFFSET_TIMER_JIFFY) 
    {
      TimerM$TimerJiffy$fired(num - TimerM$OFFSET_TIMER_JIFFY);
    }
  else {
#line 134
    if (TimerM$COUNT_TIMER_MILLI > 0 && num16 >= TimerM$OFFSET_TIMER_MILLI) 
      {
        TimerM$TimerMilli$fired(num - TimerM$OFFSET_TIMER_MILLI);
      }
    else 
      {
        TimerM$Timer$fired(num);
      }
    }
}

# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Random.nc"
inline static   uint16_t CC2420RadioM$Random$rand(void){
#line 63
  unsigned int result;
#line 63

#line 63
  result = RandomLFSR$Random$rand();
#line 63

#line 63
  return result;
#line 63
}
#line 63
static inline    
# 744 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
int16_t CC2420RadioM$MacBackoff$default$initialBackoff(TOS_MsgPtr m)
#line 744
{
  return (CC2420RadioM$Random$rand() & 0xF) + 1;
}

# 74 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/MacBackoff.nc"
inline static   int16_t CC2420RadioM$MacBackoff$initialBackoff(TOS_MsgPtr arg_0xa867eb0){
#line 74
  int result;
#line 74

#line 74
  result = CC2420RadioM$MacBackoff$default$initialBackoff(arg_0xa867eb0);
#line 74

#line 74
  return result;
#line 74
}
#line 74
# 6 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/TimerJiffyAsync.nc"
inline static   result_t CC2420RadioM$BackoffTimerJiffy$setOneShot(uint32_t arg_0xa8a41f0){
#line 6
  unsigned char result;
#line 6

#line 6
  result = TimerJiffyAsyncM$TimerJiffyAsync$setOneShot(arg_0xa8a41f0);
#line 6

#line 6
  return result;
#line 6
}
#line 6
static 
# 128 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
__inline result_t CC2420RadioM$setInitialTimer(uint16_t jiffy)
#line 128
{
  CC2420RadioM$stateTimer = CC2420RadioM$TIMER_INITIAL;
  if (jiffy == 0) {

    return CC2420RadioM$BackoffTimerJiffy$setOneShot(2);
    }
#line 133
  return CC2420RadioM$BackoffTimerJiffy$setOneShot(jiffy);
}

static 
# 12 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/byteorder.h"
__inline int is_host_lsb(void)
{
  const uint8_t n[2] = { 1, 0 };

#line 15
  return * (uint16_t *)n == 1;
}

static __inline uint16_t toLSB16(uint16_t a)
{
  return is_host_lsb() ? a : (a << 8) | (a >> 8);
}

static inline  
# 491 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$Send$send(TOS_MsgPtr pMsg)
#line 491
{
  uint8_t currentstate;

#line 493
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 493
    currentstate = CC2420RadioM$stateRadio;
#line 493
    __nesc_atomic_end(__nesc_atomic); }

  if (currentstate == CC2420RadioM$IDLE_STATE) {

      pMsg->fcflo = 0x08;
      if (CC2420RadioM$bAckEnable) {
        pMsg->fcfhi = 0x21;
        }
      else {
#line 501
        pMsg->fcfhi = 0x01;
        }
      pMsg->destpan = TOS_BCAST_ADDR;

      pMsg->addr = toLSB16(pMsg->addr);

      pMsg->length = pMsg->length + MSG_HEADER_SIZE + MSG_FOOTER_SIZE;

      pMsg->dsn = ++CC2420RadioM$currentDSN;

      pMsg->time = 0;

      CC2420RadioM$txlength = pMsg->length - MSG_FOOTER_SIZE;
      CC2420RadioM$txbufptr = pMsg;
      CC2420RadioM$countRetry = 8;

      if (CC2420RadioM$setInitialTimer(CC2420RadioM$MacBackoff$initialBackoff(CC2420RadioM$txbufptr) * 10)) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 518
            CC2420RadioM$stateRadio = CC2420RadioM$PRE_TX_STATE;
#line 518
            __nesc_atomic_end(__nesc_atomic); }
          return SUCCESS;
        }
    }
  return FAIL;
}

# 58 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
inline static  result_t AMStandard$RadioSend$send(TOS_MsgPtr arg_0xa7b3928){
#line 58
  unsigned char result;
#line 58

#line 58
  result = CC2420RadioM$Send$send(arg_0xa7b3928);
#line 58

#line 58
  return result;
#line 58
}
#line 58
static inline  
# 306 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
result_t FramerM$BareSendMsg$send(TOS_MsgPtr pMsg)
#line 306
{
  result_t Result = SUCCESS;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 309
    {
      if (!(FramerM$gFlags & FramerM$FLAGS_DATAPEND)) {
          FramerM$gFlags |= FramerM$FLAGS_DATAPEND;
          FramerM$gpTxMsg = pMsg;
        }
      else 

        {
          Result = FAIL;
        }
    }
#line 319
    __nesc_atomic_end(__nesc_atomic); }

  if (Result == SUCCESS) {
      Result = FramerM$StartTx();
    }

  return Result;
}

# 58 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
inline static  result_t AMStandard$UARTSend$send(TOS_MsgPtr arg_0xa7b3928){
#line 58
  unsigned char result;
#line 58

#line 58
  result = FramerM$BareSendMsg$send(arg_0xa7b3928);
#line 58

#line 58
  return result;
#line 58
}
#line 58
static inline  
# 165 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
void AMStandard$sendTask(void)
#line 165
{
  result_t ok;
  TOS_MsgPtr buf;

#line 168
  buf = AMStandard$buffer;
  if (buf->addr == TOS_UART_ADDR) {
    ok = AMStandard$UARTSend$send(buf);
    }
  else {
#line 172
    ok = AMStandard$RadioSend$send(buf);
    }
  if (ok == FAIL) {
    AMStandard$reportSendDone(AMStandard$buffer, FAIL);
    }
}

static inline   
# 408 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
result_t HPLUSART1M$USARTControl$tx(uint8_t data)
#line 408
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 409
    {
      HPLUSART1M$U1TXBUF = data;
    }
#line 411
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 202 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLUARTM$USARTControl$tx(uint8_t arg_0xa96ff30){
#line 202
  unsigned char result;
#line 202

#line 202
  result = HPLUSART1M$USARTControl$tx(arg_0xa96ff30);
#line 202

#line 202
  return result;
#line 202
}
#line 202
static inline   
# 98 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
result_t HPLUARTM$UART$put(uint8_t data)
#line 98
{
  return HPLUARTM$USARTControl$tx(data);
}

# 80 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/HPLUART.nc"
inline static   result_t UARTM$HPLUART$put(uint8_t arg_0xaaddd60){
#line 80
  unsigned char result;
#line 80

#line 80
  result = HPLUARTM$UART$put(arg_0xaaddd60);
#line 80

#line 80
  return result;
#line 80
}
#line 80
static inline  
# 207 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
result_t AMStandard$UARTSend$sendDone(TOS_MsgPtr msg, result_t success)
#line 207
{
  return AMStandard$reportSendDone(msg, success);
}

# 67 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
inline static  result_t FramerM$BareSendMsg$sendDone(TOS_MsgPtr arg_0xa7b3e40, result_t arg_0xa7d6010){
#line 67
  unsigned char result;
#line 67

#line 67
  result = AMStandard$UARTSend$sendDone(arg_0xa7b3e40, arg_0xa7d6010);
#line 67

#line 67
  return result;
#line 67
}
#line 67
static inline  
# 236 "BSN_RadioRangeM.nc"
result_t BSN_RadioRangeM$DataMsg$sendDone(TOS_MsgPtr sent, result_t success)
#line 236
{
  return SUCCESS;
}

static inline   
# 157 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
result_t AMStandard$SendMsg$default$sendDone(uint8_t id, TOS_MsgPtr msg, result_t success)
#line 157
{
  return SUCCESS;
}

# 49 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SendMsg.nc"
inline static  result_t AMStandard$SendMsg$sendDone(uint8_t arg_0xa854c70, TOS_MsgPtr arg_0xa7daac8, result_t arg_0xa7dac18){
#line 49
  unsigned char result;
#line 49

#line 49
  switch (arg_0xa854c70) {
#line 49
    case AM_OSCOPEMSG:
#line 49
      result = BSN_RadioRangeM$DataMsg$sendDone(arg_0xa7daac8, arg_0xa7dac18);
#line 49
      break;
#line 49
    default:
#line 49
      result = AMStandard$SendMsg$default$sendDone(arg_0xa854c70, arg_0xa7daac8, arg_0xa7dac18);
#line 49
    }
#line 49

#line 49
  return result;
#line 49
}
#line 49
static inline   
# 160 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
result_t AMStandard$default$sendDone(void)
#line 160
{
  return SUCCESS;
}

#line 65
inline static  result_t AMStandard$sendDone(void){
#line 65
  unsigned char result;
#line 65

#line 65
  result = AMStandard$default$sendDone();
#line 65

#line 65
  return result;
#line 65
}
#line 65
static inline 
#line 132
void AMStandard$dbgPacket(TOS_MsgPtr data)
#line 132
{
  uint8_t i;

  for (i = 0; i < sizeof(TOS_Msg ); i++) 
    {
      {
      }
#line 137
      ;
    }
  {
  }
#line 139
  ;
}

static inline 
# 115 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/AM.h"
uint8_t TOS_MsgLength(uint8_t type)
{








  return (size_t )& ((TOS_Msg *)0)->strength;
}

# 55 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ByteComm.nc"
inline static   result_t NoCRCPacket$ByteComm$txByte(uint8_t arg_0xaacdaf0){
#line 55
  unsigned char result;
#line 55

#line 55
  result = UARTM$ByteComm$txByte(arg_0xaacdaf0);
#line 55

#line 55
  return result;
#line 55
}
#line 55
static inline  
# 122 "C:/cygwin/opt/tinyos-1.x/tos/system/NoCRCPacket.nc"
result_t NoCRCPacket$txBytes(uint8_t *bytes, uint8_t numBytes)
#line 122
{
  uint8_t byteToSend = 0;
  bool sending = FALSE;

#line 125
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 125
    {
      if (NoCRCPacket$txCount == 0) 
        {
          NoCRCPacket$txCount = 1;
          NoCRCPacket$txLength = numBytes;
          NoCRCPacket$sendPtr = bytes;
          byteToSend = NoCRCPacket$sendPtr[0];
          sending = TRUE;
        }
    }
#line 134
    __nesc_atomic_end(__nesc_atomic); }
  if (sending) {

      if (NoCRCPacket$ByteComm$txByte(byteToSend)) {
        return SUCCESS;
        }
      else 
#line 139
        {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 140
            {
              NoCRCPacket$txCount = 0;
            }
#line 142
            __nesc_atomic_end(__nesc_atomic); }
        }
    }
  return FAIL;
}

static inline   
# 470 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB3$setEventFromNow(uint16_t x)
#line 470
{
#line 470
  MSP430TimerM$TBCCR3 = TBR + x;
}

# 32 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerM$AlarmCompare$setEventFromNow(uint16_t arg_0xa6b4d98){
#line 32
  MSP430TimerM$CompareB3$setEventFromNow(arg_0xa6b4d98);
#line 32
}
#line 32
static inline   
# 366 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$ControlB3$clearPendingInterrupt(void)
#line 366
{
#line 366
  MSP430TimerM$TBCCTL3 &= ~0x0001;
}

# 32 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerM$AlarmControl$clearPendingInterrupt(void){
#line 32
  MSP430TimerM$ControlB3$clearPendingInterrupt();
#line 32
}
#line 32
static inline   
# 414 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$ControlB3$enableEvents(void)
#line 414
{
#line 414
  MSP430TimerM$TBCCTL3 |= 0x0010;
}

# 38 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerM$AlarmControl$enableEvents(void){
#line 38
  MSP430TimerM$ControlB3$enableEvents();
#line 38
}
#line 38
static inline 
# 202 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
void __nesc_enable_interrupt(void)
{
   __asm volatile ("eint");}

static inline 
#line 226
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

static 
#line 245
__inline void __nesc_atomic_sleep(void)
#line 245
{








  uint16_t LPMode_bits = 0;

  if (LPMode_disabled) {
      __nesc_enable_interrupt();
      return;
    }
  else 
#line 259
    {
      LPMode_bits = 0x0080 + 0x0040 + 0x0010;



      if ((((
#line 262
      TACTL & (3 << 4)) != 0 << 4 && (TACTL & (3 << 8)) == 2 << 8)
       || (ME1 & ((1 << 7) | (1 << 6)) && U0TCTL & 0x20))
       || (ME2 & ((1 << 5) | (1 << 4)) && U1TCTL & 0x20)) {






        LPMode_bits = 0x0040 + 0x0010;
        }

      if (ADC12CTL1 & 1) {
          if (!(ADC12CTL0 & 0x0080) && (TACTL & (3 << 8)) == 2 << 8) {
            LPMode_bits = 0x0040 + 0x0010;
            }
          else {
#line 278
            switch (ADC12CTL1 & (3 << 3)) {
                case 2 << 3: LPMode_bits = 0;
#line 279
                break;
                case 3 << 3: LPMode_bits = 0x0040 + 0x0010;
#line 280
                break;
              }
            }
        }
      LPMode_bits |= 0x0008;
       __asm volatile ("bis  %0, r2" :  : "m"((uint16_t )LPMode_bits));}
}

static inline 
#line 196
void __nesc_disable_interrupt(void)
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

static inline 





bool are_interrupts_enabled(void)
{
  return (({
#line 209
    uint16_t __x;

#line 209
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 209
   & 0x0008) != 0;
}

static inline 






__nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = are_interrupts_enabled();

#line 222
  __nesc_disable_interrupt();
  return result;
}

static inline 
# 136 "C:/cygwin/opt/tinyos-1.x/tos/system/sched.c"
bool TOSH_run_next_task(void)
{
  __nesc_atomic_t fInterruptFlags;
  uint8_t old_full;
  void (*func)(void );

  fInterruptFlags = __nesc_atomic_start();
  old_full = TOSH_sched_full;
  func = TOSH_queue[old_full].tp;
  if (func == NULL) 
    {
      __nesc_atomic_sleep();
      return 0;
    }

  TOSH_queue[old_full].tp = NULL;
  TOSH_sched_full = (old_full + 1) & TOSH_TASK_BITMASK;
  __nesc_atomic_end(fInterruptFlags);
  func();

  return 1;
}

static inline void TOSH_run_task(void)
#line 159
{
  for (; ; ) 
    TOSH_run_next_task();
}

static inline    
# 157 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareA0$default$fired(void)
#line 157
{
}

# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareA0$fired(void){
#line 34
  MSP430TimerM$CompareA0$default$fired();
#line 34
}
#line 34
static inline   
# 253 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$CaptureA0$getEvent(void)
#line 253
{
#line 253
  return MSP430TimerM$TACCR0;
}

static inline    
#line 160
void MSP430TimerM$CaptureA0$default$captured(uint16_t time)
#line 160
{
}

# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureA0$captured(uint16_t arg_0xa6b0618){
#line 74
  MSP430TimerM$CaptureA0$default$captured(arg_0xa6b0618);
#line 74
}
#line 74
static inline 
# 96 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
MSP430TimerM$CC_t MSP430TimerM$int2CC(uint16_t x)
#line 96
{
#line 96
  union __nesc_unnamed4299 {
#line 96
    uint16_t f;
#line 96
    MSP430TimerM$CC_t t;
  } 
#line 96
  c = { .f = x };

#line 96
  return c.t;
}

static inline   
#line 205
MSP430TimerM$CC_t MSP430TimerM$ControlA0$getControl(void)
#line 205
{
#line 205
  return MSP430TimerM$int2CC(MSP430TimerM$TACCTL0);
}

#line 123
void __attribute((interrupt(12))) __attribute((wakeup))  sig_TIMERA0_VECTOR(void)
{
  if (MSP430TimerM$ControlA0$getControl().cap) {
    MSP430TimerM$CaptureA0$captured(MSP430TimerM$CaptureA0$getEvent());
    }
  else {
#line 128
    MSP430TimerM$CompareA0$fired();
    }
}

static inline   
#line 206
MSP430TimerM$CC_t MSP430TimerM$ControlA1$getControl(void)
#line 206
{
#line 206
  return MSP430TimerM$int2CC(MSP430TimerM$TACCTL1);
}

static inline    
#line 161
void MSP430TimerM$CaptureA1$default$captured(uint16_t time)
#line 161
{
}

# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureA1$captured(uint16_t arg_0xa6b0618){
#line 74
  MSP430TimerM$CaptureA1$default$captured(arg_0xa6b0618);
#line 74
}
#line 74
static inline   
# 254 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$CaptureA1$getEvent(void)
#line 254
{
#line 254
  return MSP430TimerM$TACCR1;
}

static inline    
#line 158
void MSP430TimerM$CompareA1$default$fired(void)
#line 158
{
}

# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareA1$fired(void){
#line 34
  MSP430TimerM$CompareA1$default$fired();
#line 34
}
#line 34
static inline   
# 207 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
MSP430TimerM$CC_t MSP430TimerM$ControlA2$getControl(void)
#line 207
{
#line 207
  return MSP430TimerM$int2CC(MSP430TimerM$TACCTL2);
}

static inline    
#line 162
void MSP430TimerM$CaptureA2$default$captured(uint16_t time)
#line 162
{
}

# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureA2$captured(uint16_t arg_0xa6b0618){
#line 74
  MSP430TimerM$CaptureA2$default$captured(arg_0xa6b0618);
#line 74
}
#line 74
static inline   
# 255 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$CaptureA2$getEvent(void)
#line 255
{
#line 255
  return MSP430TimerM$TACCR2;
}

static inline    
#line 159
void MSP430TimerM$CompareA2$default$fired(void)
#line 159
{
}

# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareA2$fired(void){
#line 34
  MSP430TimerM$CompareA2$default$fired();
#line 34
}
#line 34
# 30 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   uint16_t MSP430DCOCalibM$Timer32khz$read(void){
#line 30
  unsigned int result;
#line 30

#line 30
  result = MSP430TimerM$TimerB$read();
#line 30

#line 30
  return result;
#line 30
}
#line 30
static inline   
# 41 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430DCOCalibM.nc"
void MSP430DCOCalibM$TimerMicro$overflow(void)
{
  uint16_t now = MSP430DCOCalibM$Timer32khz$read();
  uint16_t delta = now - MSP430DCOCalibM$m_prev;

#line 45
  MSP430DCOCalibM$m_prev = now;

  if (delta > MSP430DCOCalibM$TARGET_DELTA + MSP430DCOCalibM$MAX_DEVIATION) 
    {

      if (DCOCTL < 0xe0) 
        {
          DCOCTL++;
        }
      else {
#line 54
        if ((BCSCTL1 & 7) < 7) 
          {
            BCSCTL1++;
            DCOCTL = 96;
          }
        }
    }
  else {
#line 60
    if (delta < MSP430DCOCalibM$TARGET_DELTA - MSP430DCOCalibM$MAX_DEVIATION) 
      {

        if (DCOCTL > 0) 
          {
            DCOCTL--;
          }
        else {
#line 67
          if ((BCSCTL1 & 7) > 0) 
            {
              BCSCTL1--;
              DCOCTL = 128;
            }
          }
      }
    }
}

# 33 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void MSP430TimerM$TimerA$overflow(void){
#line 33
  MSP430DCOCalibM$TimerMicro$overflow();
#line 33
}
#line 33
static inline    
# 331 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB0$default$fired(void)
#line 331
{
}

# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB0$fired(void){
#line 34
  MSP430TimerM$CompareB0$default$fired();
#line 34
}
#line 34
static inline   
# 443 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$CaptureB0$getEvent(void)
#line 443
{
#line 443
  return MSP430TimerM$TBCCR0;
}

static inline    
#line 338
void MSP430TimerM$CaptureB0$default$captured(uint16_t time)
#line 338
{
}

# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB0$captured(uint16_t arg_0xa6b0618){
#line 74
  MSP430TimerM$CaptureB0$default$captured(arg_0xa6b0618);
#line 74
}
#line 74
static inline   
# 347 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
MSP430TimerM$CC_t MSP430TimerM$ControlB0$getControl(void)
#line 347
{
#line 347
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL0);
}

#line 277
void __attribute((interrupt(26))) __attribute((wakeup))  sig_TIMERB0_VECTOR(void)
{
  if (MSP430TimerM$ControlB0$getControl().cap) {
    MSP430TimerM$CaptureB0$captured(MSP430TimerM$CaptureB0$getEvent());
    }
  else {
#line 282
    MSP430TimerM$CompareB0$fired();
    }
}

static inline   
#line 348
MSP430TimerM$CC_t MSP430TimerM$ControlB1$getControl(void)
#line 348
{
#line 348
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL1);
}

static inline   
#line 484
void MSP430TimerM$CaptureB1$clearOverflow(void)
#line 484
{
#line 484
  MSP430TimerM$TBCCTL1 &= ~0x0002;
}

# 56 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void HPLCC2420InterruptM$SFDCapture$clearOverflow(void){
#line 56
  MSP430TimerM$CaptureB1$clearOverflow();
#line 56
}
#line 56
static inline   
# 476 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
bool MSP430TimerM$CaptureB1$isOverflowPending(void)
#line 476
{
#line 476
  return MSP430TimerM$TBCCTL1 & 0x0002;
}

# 51 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   bool HPLCC2420InterruptM$SFDCapture$isOverflowPending(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = MSP430TimerM$CaptureB1$isOverflowPending();
#line 51

#line 51
  return result;
#line 51
}
#line 51
static inline   
# 364 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$ControlB1$clearPendingInterrupt(void)
#line 364
{
#line 364
  MSP430TimerM$TBCCTL1 &= ~0x0001;
}

# 32 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void HPLCC2420InterruptM$SFDControl$clearPendingInterrupt(void){
#line 32
  MSP430TimerM$ControlB1$clearPendingInterrupt();
#line 32
}
#line 32
static inline   
# 420 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$ControlB1$disableEvents(void)
#line 420
{
#line 420
  MSP430TimerM$TBCCTL1 &= ~0x0010;
}

# 39 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void HPLCC2420InterruptM$SFDControl$disableEvents(void){
#line 39
  MSP430TimerM$ControlB1$disableEvents();
#line 39
}
#line 39
static inline    
# 761 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
void CC2420RadioM$RadioReceiveCoordinator$default$startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff)
#line 761
{
}

# 33 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/RadioCoordinator.nc"
inline static   void CC2420RadioM$RadioReceiveCoordinator$startSymbol(uint8_t arg_0xa88b540, uint8_t arg_0xa88b688, TOS_MsgPtr arg_0xa88b7d8){
#line 33
  CC2420RadioM$RadioReceiveCoordinator$default$startSymbol(arg_0xa88b540, arg_0xa88b688, arg_0xa88b7d8);
#line 33
}
#line 33
static 
# 144 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
__inline result_t CC2420RadioM$setAckTimer(uint16_t jiffy)
#line 144
{
  CC2420RadioM$stateTimer = CC2420RadioM$TIMER_ACK;
  return CC2420RadioM$BackoffTimerJiffy$setOneShot(jiffy);
}

# 43 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Capture.nc"
inline static   result_t CC2420RadioM$SFD$enableCapture(bool arg_0xa8d6160){
#line 43
  unsigned char result;
#line 43

#line 43
  result = HPLCC2420InterruptM$SFD$enableCapture(arg_0xa8d6160);
#line 43

#line 43
  return result;
#line 43
}
#line 43
static inline 
# 28 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
void TOSH_SEL_CC_SFD_IOFUNC(void)
#line 28
{
   
#line 28
  static volatile uint8_t r __asm ("0x001F");

#line 28
  r &= ~(1 << 1);
}

static inline   
# 200 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420InterruptM.nc"
result_t HPLCC2420InterruptM$SFD$disable(void)
#line 200
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 201
    {
      HPLCC2420InterruptM$SFDControl$disableEvents();
      HPLCC2420InterruptM$SFDControl$clearPendingInterrupt();
      TOSH_SEL_CC_SFD_IOFUNC();
    }
#line 205
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 60 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Capture.nc"
inline static   result_t CC2420RadioM$SFD$disable(void){
#line 60
  unsigned char result;
#line 60

#line 60
  result = HPLCC2420InterruptM$SFD$disable();
#line 60

#line 60
  return result;
#line 60
}
#line 60
static inline    
# 759 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
void CC2420RadioM$RadioSendCoordinator$default$startSymbol(uint8_t bitsPerBlock, uint8_t offset, TOS_MsgPtr msgBuff)
#line 759
{
}

# 33 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/RadioCoordinator.nc"
inline static   void CC2420RadioM$RadioSendCoordinator$startSymbol(uint8_t arg_0xa88b540, uint8_t arg_0xa88b688, TOS_MsgPtr arg_0xa88b7d8){
#line 33
  CC2420RadioM$RadioSendCoordinator$default$startSymbol(arg_0xa88b540, arg_0xa88b688, arg_0xa88b7d8);
#line 33
}
#line 33
static inline 
# 28 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
uint8_t TOSH_READ_CC_SFD_PIN(void)
#line 28
{
   
#line 28
  static volatile uint8_t r __asm ("0x001C");

#line 28
  return r & (1 << 1);
}

static inline   
# 344 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$SFD$captured(uint16_t time)
#line 344
{
  switch (CC2420RadioM$stateRadio) {
      case CC2420RadioM$TX_STATE: 

        CC2420RadioM$SFD$enableCapture(FALSE);


      if (!TOSH_READ_CC_SFD_PIN()) {
          CC2420RadioM$SFD$disable();
        }
      else {
          CC2420RadioM$stateRadio = CC2420RadioM$TX_WAIT;
        }

      CC2420RadioM$txbufptr->time = time;
      CC2420RadioM$RadioSendCoordinator$startSymbol(8, 0, CC2420RadioM$txbufptr);


      if (CC2420RadioM$stateRadio == CC2420RadioM$TX_WAIT) {
          break;
        }
      case CC2420RadioM$TX_WAIT: 

        CC2420RadioM$stateRadio = CC2420RadioM$POST_TX_STATE;
      CC2420RadioM$SFD$disable();

      CC2420RadioM$SFD$enableCapture(TRUE);

      if (CC2420RadioM$bAckEnable && CC2420RadioM$txbufptr->addr != TOS_BCAST_ADDR) {
          if (!CC2420RadioM$setAckTimer(75)) {
            CC2420RadioM$sendFailed();
            }
        }
      else {
          if (!TOS_post(CC2420RadioM$PacketSent)) {
            CC2420RadioM$sendFailed();
            }
        }
#line 381
      break;
      default: 

        CC2420RadioM$rxbufptr->time = time;
      CC2420RadioM$RadioReceiveCoordinator$startSymbol(8, 0, CC2420RadioM$rxbufptr);
    }
  return SUCCESS;
}

# 53 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Capture.nc"
inline static   result_t HPLCC2420InterruptM$SFD$captured(uint16_t arg_0xa8d6688){
#line 53
  unsigned char result;
#line 53

#line 53
  result = CC2420RadioM$SFD$captured(arg_0xa8d6688);
#line 53

#line 53
  return result;
#line 53
}
#line 53
static inline   
# 209 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420InterruptM.nc"
void HPLCC2420InterruptM$SFDCapture$captured(uint16_t time)
#line 209
{
  result_t val = SUCCESS;

#line 211
  HPLCC2420InterruptM$SFDControl$clearPendingInterrupt();
  val = HPLCC2420InterruptM$SFD$captured(time);
  if (val == FAIL) {
      HPLCC2420InterruptM$SFDControl$disableEvents();
      HPLCC2420InterruptM$SFDControl$clearPendingInterrupt();
    }
  else {
      if (HPLCC2420InterruptM$SFDCapture$isOverflowPending()) {
        HPLCC2420InterruptM$SFDCapture$clearOverflow();
        }
    }
}

# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB1$captured(uint16_t arg_0xa6b0618){
#line 74
  HPLCC2420InterruptM$SFDCapture$captured(arg_0xa6b0618);
#line 74
}
#line 74
static inline 
# 28 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
void TOSH_SEL_CC_SFD_MODFUNC(void)
#line 28
{
   
#line 28
  static volatile uint8_t r __asm ("0x001F");

#line 28
  r |= 1 << 1;
}

static inline 
# 110 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$captureControl(uint8_t l_cm)
{
  MSP430TimerM$CC_t x = { 
  .cm = l_cm & 0x03, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 1, 
  .scs = 1, 
  .ccie = 0 };

  return MSP430TimerM$CC2int(x);
}

static inline   
#line 388
void MSP430TimerM$ControlB1$setControlAsCapture(uint8_t cm)
#line 388
{
#line 388
  MSP430TimerM$TBCCTL1 = MSP430TimerM$captureControl(cm);
}

# 36 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void HPLCC2420InterruptM$SFDControl$setControlAsCapture(bool arg_0xa6bb190){
#line 36
  MSP430TimerM$ControlB1$setControlAsCapture(arg_0xa6bb190);
#line 36
}
#line 36
static inline   
# 412 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$ControlB1$enableEvents(void)
#line 412
{
#line 412
  MSP430TimerM$TBCCTL1 |= 0x0010;
}

# 38 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void HPLCC2420InterruptM$SFDControl$enableEvents(void){
#line 38
  MSP430TimerM$ControlB1$enableEvents();
#line 38
}
#line 38
static inline  
# 210 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
result_t AMStandard$RadioSend$sendDone(TOS_MsgPtr msg, result_t success)
#line 210
{
  return AMStandard$reportSendDone(msg, success);
}

# 67 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
inline static  result_t CC2420RadioM$Send$sendDone(TOS_MsgPtr arg_0xa7b3e40, result_t arg_0xa7d6010){
#line 67
  unsigned char result;
#line 67

#line 67
  result = AMStandard$RadioSend$sendDone(arg_0xa7b3e40, arg_0xa7d6010);
#line 67

#line 67
  return result;
#line 67
}
#line 67
static inline   
# 444 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$CaptureB1$getEvent(void)
#line 444
{
#line 444
  return MSP430TimerM$TBCCR1;
}

static inline    
#line 332
void MSP430TimerM$CompareB1$default$fired(void)
#line 332
{
}

# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB1$fired(void){
#line 34
  MSP430TimerM$CompareB1$default$fired();
#line 34
}
#line 34
static inline   
# 349 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
MSP430TimerM$CC_t MSP430TimerM$ControlB2$getControl(void)
#line 349
{
#line 349
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL2);
}

static inline    
#line 340
void MSP430TimerM$CaptureB2$default$captured(uint16_t time)
#line 340
{
}

# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB2$captured(uint16_t arg_0xa6b0618){
#line 74
  MSP430TimerM$CaptureB2$default$captured(arg_0xa6b0618);
#line 74
}
#line 74
static inline   
# 445 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$CaptureB2$getEvent(void)
#line 445
{
#line 445
  return MSP430TimerM$TBCCR2;
}

static inline    
#line 333
void MSP430TimerM$CompareB2$default$fired(void)
#line 333
{
}

# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB2$fired(void){
#line 34
  MSP430TimerM$CompareB2$default$fired();
#line 34
}
#line 34
static inline   
# 350 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
MSP430TimerM$CC_t MSP430TimerM$ControlB3$getControl(void)
#line 350
{
#line 350
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL3);
}

static inline    
#line 341
void MSP430TimerM$CaptureB3$default$captured(uint16_t time)
#line 341
{
}

# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB3$captured(uint16_t arg_0xa6b0618){
#line 74
  MSP430TimerM$CaptureB3$default$captured(arg_0xa6b0618);
#line 74
}
#line 74
static inline   
# 446 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$CaptureB3$getEvent(void)
#line 446
{
#line 446
  return MSP430TimerM$TBCCR3;
}

static inline  
# 266 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
void TimerM$checkShortTimers(void)
{
  uint8_t head = TimerM$m_head_short;

#line 269
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 269
    TimerM$m_posted_checkShortTimers = FALSE;
#line 269
    __nesc_atomic_end(__nesc_atomic); }
  TimerM$m_head_short = TimerM$EMPTY_LIST;
  TimerM$executeTimers(head);
  TimerM$setNextShortEvent();
}

static inline 
#line 192
void TimerM$post_checkShortTimers(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (!TimerM$m_posted_checkShortTimers) 
        {
          if (TOS_post(TimerM$checkShortTimers)) {
            TimerM$m_posted_checkShortTimers = TRUE;
            }
        }
    }
#line 202
    __nesc_atomic_end(__nesc_atomic); }
}

static inline   
#line 303
void TimerM$AlarmCompare$fired(void)
{
  TimerM$post_checkShortTimers();
}

# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB3$fired(void){
#line 34
  TimerM$AlarmCompare$fired();
#line 34
}
#line 34
static inline   
# 351 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
MSP430TimerM$CC_t MSP430TimerM$ControlB4$getControl(void)
#line 351
{
#line 351
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL4);
}

static inline    
#line 342
void MSP430TimerM$CaptureB4$default$captured(uint16_t time)
#line 342
{
}

# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB4$captured(uint16_t arg_0xa6b0618){
#line 74
  MSP430TimerM$CaptureB4$default$captured(arg_0xa6b0618);
#line 74
}
#line 74
static inline   
# 447 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$CaptureB4$getEvent(void)
#line 447
{
#line 447
  return MSP430TimerM$TBCCR4;
}

static inline   
#line 415
void MSP430TimerM$ControlB4$enableEvents(void)
#line 415
{
#line 415
  MSP430TimerM$TBCCTL4 |= 0x0010;
}

# 38 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerJiffyAsyncM$AlarmControl$enableEvents(void){
#line 38
  MSP430TimerM$ControlB4$enableEvents();
#line 38
}
#line 38
static inline   
# 367 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$ControlB4$clearPendingInterrupt(void)
#line 367
{
#line 367
  MSP430TimerM$TBCCTL4 &= ~0x0001;
}

# 32 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerControl.nc"
inline static   void TimerJiffyAsyncM$AlarmControl$clearPendingInterrupt(void){
#line 32
  MSP430TimerM$ControlB4$clearPendingInterrupt();
#line 32
}
#line 32
static inline   
# 471 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB4$setEventFromNow(uint16_t x)
#line 471
{
#line 471
  MSP430TimerM$TBCCR4 = TBR + x;
}

# 32 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerJiffyAsyncM$AlarmCompare$setEventFromNow(uint16_t arg_0xa6b4d98){
#line 32
  MSP430TimerM$CompareB4$setEventFromNow(arg_0xa6b4d98);
#line 32
}
#line 32
static inline   
# 449 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$BackoffTimerJiffy$fired(void)
#line 449
{
  uint8_t currentstate;

#line 451
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 451
    currentstate = CC2420RadioM$stateRadio;
#line 451
    __nesc_atomic_end(__nesc_atomic); }

  switch (CC2420RadioM$stateTimer) {
      case CC2420RadioM$TIMER_INITIAL: 
        if (!TOS_post(CC2420RadioM$startSend)) {
            CC2420RadioM$sendFailed();
          }
      break;
      case CC2420RadioM$TIMER_BACKOFF: 
        CC2420RadioM$tryToSend();
      break;
      case CC2420RadioM$TIMER_ACK: 
        if (currentstate == CC2420RadioM$POST_TX_STATE) {





            { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 469
              {
                CC2420RadioM$txbufptr->ack = 0;
                CC2420RadioM$stateRadio = CC2420RadioM$POST_TX_ACK_STATE;
              }
#line 472
              __nesc_atomic_end(__nesc_atomic); }
            if (!TOS_post(CC2420RadioM$PacketSent)) {
              CC2420RadioM$sendFailed();
              }
          }
#line 476
      break;
    }
  return SUCCESS;
}

# 12 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/TimerJiffyAsync.nc"
inline static   result_t TimerJiffyAsyncM$TimerJiffyAsync$fired(void){
#line 12
  unsigned char result;
#line 12

#line 12
  result = CC2420RadioM$BackoffTimerJiffy$fired();
#line 12

#line 12
  return result;
#line 12
}
#line 12
static inline   
# 41 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/TimerJiffyAsyncM.nc"
void TimerJiffyAsyncM$AlarmCompare$fired(void)
{
  if (TimerJiffyAsyncM$jiffy < 0xFFFF) {
      TimerJiffyAsyncM$AlarmControl$disableEvents();
      TimerJiffyAsyncM$bSet = FALSE;
      TimerJiffyAsyncM$TimerJiffyAsync$fired();
    }
  else {
      TimerJiffyAsyncM$jiffy = TimerJiffyAsyncM$jiffy - 0xFFFF;
      if (TimerJiffyAsyncM$jiffy > 0xFFFF) {
        TimerJiffyAsyncM$AlarmCompare$setEventFromNow(0xFFFF);
        }
      else 
#line 52
        {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 53
            {




              if (TimerJiffyAsyncM$jiffy > 2) {
                TimerJiffyAsyncM$AlarmCompare$setEventFromNow(TimerJiffyAsyncM$jiffy);
                }
              else {
#line 61
                TimerJiffyAsyncM$AlarmCompare$setEventFromNow(2);
                }
            }
#line 63
            __nesc_atomic_end(__nesc_atomic); }
        }
      TimerJiffyAsyncM$AlarmControl$clearPendingInterrupt();
      TimerJiffyAsyncM$AlarmControl$enableEvents();
    }
}

# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB4$fired(void){
#line 34
  TimerJiffyAsyncM$AlarmCompare$fired();
#line 34
}
#line 34
static inline   
# 721 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$HPLChipconFIFO$TXFIFODone(uint8_t length, uint8_t *data)
#line 721
{
  CC2420RadioM$tryToSend();
  return SUCCESS;
}

# 50 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
inline static   result_t HPLCC2420M$HPLCC2420FIFO$TXFIFODone(uint8_t arg_0xa8a6f20, uint8_t *arg_0xa8a7080){
#line 50
  unsigned char result;
#line 50

#line 50
  result = CC2420RadioM$HPLChipconFIFO$TXFIFODone(arg_0xa8a6f20, arg_0xa8a7080);
#line 50

#line 50
  return result;
#line 50
}
#line 50
static inline  
# 412 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420M.nc"
void HPLCC2420M$signalTXFIFO(void)
#line 412
{
  uint8_t _txlen;
  uint8_t *_txbuf;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 416
    {
      _txlen = HPLCC2420M$txlen;
      _txbuf = HPLCC2420M$txbuf;
      HPLCC2420M$txbufBusy = FALSE;
    }
#line 420
    __nesc_atomic_end(__nesc_atomic); }

  HPLCC2420M$HPLCC2420FIFO$TXFIFODone(_txlen, _txbuf);
}

# 38 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/BusArbitration.nc"
inline static   result_t HPLCC2420M$BusArbitration$releaseBus(void){
#line 38
  unsigned char result;
#line 38

#line 38
  result = BusArbitrationM$BusArbitration$releaseBus(0);
#line 38

#line 38
  return result;
#line 38
}
#line 38
static inline   
# 432 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
result_t HPLUSART0M$USARTControl$isTxEmpty(void)
#line 432
{
  if (HPLUSART0M$U0TCTL & 0x01) {
      return SUCCESS;
    }
  return FAIL;
}

# 191 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$isTxEmpty(void){
#line 191
  unsigned char result;
#line 191

#line 191
  result = HPLUSART0M$USARTControl$isTxEmpty();
#line 191

#line 191
  return result;
#line 191
}
#line 191
static inline   
# 424 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
result_t HPLUSART0M$USARTControl$isTxIntrPending(void)
#line 424
{
  if (HPLUSART0M$IFG1 & (1 << 7)) {
      HPLUSART0M$IFG1 &= ~(1 << 7);
      return SUCCESS;
    }
  return FAIL;
}

# 180 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$isTxIntrPending(void){
#line 180
  unsigned char result;
#line 180

#line 180
  result = HPLUSART0M$USARTControl$isTxIntrPending();
#line 180

#line 180
  return result;
#line 180
}
#line 180
static inline   
# 473 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
result_t HPLUSART0M$USARTControl$tx(uint8_t data)
#line 473
{
  HPLUSART0M$U0TXBUF = data;
  return SUCCESS;
}

# 202 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$tx(uint8_t arg_0xa96ff30){
#line 202
  unsigned char result;
#line 202

#line 202
  result = HPLUSART0M$USARTControl$tx(arg_0xa96ff30);
#line 202

#line 202
  return result;
#line 202
}
#line 202
static inline   
# 478 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
uint8_t HPLUSART0M$USARTControl$rx(void)
#line 478
{
  uint8_t value;

#line 480
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 480
    {
      value = U0RXBUF;
    }
#line 482
    __nesc_atomic_end(__nesc_atomic); }
  return value;
}

# 209 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   uint8_t HPLCC2420M$USARTControl$rx(void){
#line 209
  unsigned char result;
#line 209

#line 209
  result = HPLUSART0M$USARTControl$rx();
#line 209

#line 209
  return result;
#line 209
}
#line 209
static inline 
# 16 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
void TOSH_CLR_RADIO_CSN_PIN(void)
#line 16
{
   
#line 16
  static volatile uint8_t r __asm ("0x001D");

#line 16
  r &= ~(1 << 2);
}

# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/BusArbitration.nc"
inline static   result_t HPLCC2420M$BusArbitration$getBus(void){
#line 37
  unsigned char result;
#line 37

#line 37
  result = BusArbitrationM$BusArbitration$getBus(0);
#line 37

#line 37
  return result;
#line 37
}
#line 37
static inline   
# 433 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420M.nc"
result_t HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(uint8_t length, uint8_t *data)
#line 433
{
  uint8_t i = 0;
  bool returnFail = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 437
    {
      if (HPLCC2420M$txbufBusy) {
        returnFail = TRUE;
        }
      else {
#line 441
        HPLCC2420M$txbufBusy = TRUE;
        }
    }
#line 443
    __nesc_atomic_end(__nesc_atomic); }
  if (returnFail) {
    return FAIL;
    }
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 448
        {
          HPLCC2420M$txlen = length;
          HPLCC2420M$txbuf = data;
        }
#line 451
        __nesc_atomic_end(__nesc_atomic); }

      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(0x3E);
      while (!HPLCC2420M$USARTControl$isTxIntrPending()) ;
      for (i = 0; i < HPLCC2420M$txlen; i++) {
          HPLCC2420M$USARTControl$tx(HPLCC2420M$txbuf[i]);
          while (!HPLCC2420M$USARTControl$isTxIntrPending()) ;
        }
      while (!HPLCC2420M$USARTControl$isTxEmpty()) ;
      TOSH_SET_RADIO_CSN_PIN();
#line 479
      HPLCC2420M$BusArbitration$releaseBus();
    }
  else {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 482
        HPLCC2420M$txbufBusy = FALSE;
#line 482
        __nesc_atomic_end(__nesc_atomic); }
      return FAIL;
    }
  if (TOS_post(HPLCC2420M$signalTXFIFO) == FAIL) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 486
        HPLCC2420M$txbufBusy = FALSE;
#line 486
        __nesc_atomic_end(__nesc_atomic); }
      return FAIL;
    }
  return SUCCESS;
}

# 29 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
inline static   result_t CC2420RadioM$HPLChipconFIFO$writeTXFIFO(uint8_t arg_0xa8a6210, uint8_t *arg_0xa8a6370){
#line 29
  unsigned char result;
#line 29

#line 29
  result = HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(arg_0xa8a6210, arg_0xa8a6370);
#line 29

#line 29
  return result;
#line 29
}
#line 29
# 61 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static   uint16_t CC2420RadioM$HPLChipcon$read(uint8_t arg_0xa8aa798){
#line 61
  unsigned int result;
#line 61

#line 61
  result = HPLCC2420M$HPLCC2420$read(arg_0xa8aa798);
#line 61

#line 61
  return result;
#line 61
}
#line 61
static inline   
# 221 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port10$edge(bool l2h)
#line 221
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 222
    {
      if (l2h) {
#line 223
        P1IES &= ~(1 << 0);
        }
      else {
#line 224
        P1IES |= 1 << 0;
        }
    }
#line 226
    __nesc_atomic_end(__nesc_atomic); }
}

# 54 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOPInterrupt$edge(bool arg_0xa9d1b80){
#line 54
  MSP430InterruptM$Port10$edge(arg_0xa9d1b80);
#line 54
}
#line 54
static inline   
# 115 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port10$enable(void)
#line 115
{
#line 115
  MSP430InterruptM$P1IE |= 1 << 0;
}

# 30 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOPInterrupt$enable(void){
#line 30
  MSP430InterruptM$Port10$enable();
#line 30
}
#line 30
static inline 
# 24 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
uint8_t TOSH_READ_RADIO_CCA_PIN(void)
#line 24
{
   
#line 24
  static volatile uint8_t r __asm ("0x0020");

#line 24
  return r & (1 << 4);
}

static inline    
# 751 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
int16_t CC2420RadioM$MacBackoff$default$congestionBackoff(TOS_MsgPtr m)
#line 751
{
  return (CC2420RadioM$Random$rand() & 0x3F) + 1;
}

# 75 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/MacBackoff.nc"
inline static   int16_t CC2420RadioM$MacBackoff$congestionBackoff(TOS_MsgPtr arg_0xa88a310){
#line 75
  int result;
#line 75

#line 75
  result = CC2420RadioM$MacBackoff$default$congestionBackoff(arg_0xa88a310);
#line 75

#line 75
  return result;
#line 75
}
#line 75
static 
# 136 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
__inline result_t CC2420RadioM$setBackoffTimer(uint16_t jiffy)
#line 136
{
  CC2420RadioM$stateTimer = CC2420RadioM$TIMER_BACKOFF;
  if (jiffy == 0) {

    return CC2420RadioM$BackoffTimerJiffy$setOneShot(2);
    }
#line 141
  return CC2420RadioM$BackoffTimerJiffy$setOneShot(jiffy);
}

# 47 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static   uint8_t CC2420RadioM$HPLChipcon$cmd(uint8_t arg_0xa8b5d30){
#line 47
  unsigned char result;
#line 47

#line 47
  result = HPLCC2420M$HPLCC2420$cmd(arg_0xa8b5d30);
#line 47

#line 47
  return result;
#line 47
}
#line 47
static inline 
# 321 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
void CC2420RadioM$sendPacket(void)
#line 321
{
  uint8_t status;

  CC2420RadioM$HPLChipcon$cmd(0x05);
  status = CC2420RadioM$HPLChipcon$cmd(0x00);
  if ((status >> 3) & 0x01) {

      CC2420RadioM$SFD$enableCapture(TRUE);
    }
  else {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 332
        CC2420RadioM$stateRadio = CC2420RadioM$PRE_TX_STATE;
#line 332
        __nesc_atomic_end(__nesc_atomic); }
      if (!CC2420RadioM$setBackoffTimer(CC2420RadioM$MacBackoff$congestionBackoff(CC2420RadioM$txbufptr) * 10)) {
          CC2420RadioM$sendFailed();
        }
    }
}

static inline   
# 352 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
MSP430TimerM$CC_t MSP430TimerM$ControlB5$getControl(void)
#line 352
{
#line 352
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL5);
}

static inline    
#line 343
void MSP430TimerM$CaptureB5$default$captured(uint16_t time)
#line 343
{
}

# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB5$captured(uint16_t arg_0xa6b0618){
#line 74
  MSP430TimerM$CaptureB5$default$captured(arg_0xa6b0618);
#line 74
}
#line 74
static inline   
# 448 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$CaptureB5$getEvent(void)
#line 448
{
#line 448
  return MSP430TimerM$TBCCR5;
}

static inline    
#line 336
void MSP430TimerM$CompareB5$default$fired(void)
#line 336
{
}

# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB5$fired(void){
#line 34
  MSP430TimerM$CompareB5$default$fired();
#line 34
}
#line 34
static inline   
# 353 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
MSP430TimerM$CC_t MSP430TimerM$ControlB6$getControl(void)
#line 353
{
#line 353
  return MSP430TimerM$int2CC(MSP430TimerM$TBCCTL6);
}

static inline    
#line 344
void MSP430TimerM$CaptureB6$default$captured(uint16_t time)
#line 344
{
}

# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Capture.nc"
inline static   void MSP430TimerM$CaptureB6$captured(uint16_t arg_0xa6b0618){
#line 74
  MSP430TimerM$CaptureB6$default$captured(arg_0xa6b0618);
#line 74
}
#line 74
static inline   
# 449 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$CaptureB6$getEvent(void)
#line 449
{
#line 449
  return MSP430TimerM$TBCCR6;
}

static inline    
#line 337
void MSP430TimerM$CompareB6$default$fired(void)
#line 337
{
}

# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB6$fired(void){
#line 34
  MSP430TimerM$CompareB6$default$fired();
#line 34
}
#line 34
static inline   
# 75 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430DCOCalibM.nc"
void MSP430DCOCalibM$Timer32khz$overflow(void)
{
}

static inline  
# 275 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
void TimerM$checkLongTimers(void)
{
  uint8_t head = TimerM$m_head_long;

#line 278
  TimerM$m_head_long = TimerM$EMPTY_LIST;
  TimerM$executeTimers(head);
  TimerM$setNextShortEvent();
}

static inline   
#line 308
void TimerM$AlarmTimer$overflow(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 310
    TimerM$m_hinow++;
#line 310
    __nesc_atomic_end(__nesc_atomic); }
  TOS_post(TimerM$checkLongTimers);
}

# 33 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void MSP430TimerM$TimerB$overflow(void){
#line 33
  TimerM$AlarmTimer$overflow();
#line 33
  MSP430DCOCalibM$Timer32khz$overflow();
#line 33
}
#line 33
static inline  
# 147 "BSN_RadioRangeM.nc"
void BSN_RadioRangeM$ForwardUart(void)
{
  struct UbiMonMsg *pack;

#line 150
  pack = (struct UbiMonMsg *)BSN_RadioRangeM$forwardmsg.data;
  BSN_RadioRangeM$forwardmsg.addr = TOS_UART_ADDR;
  BSN_RadioRangeM$UARTSend$send(&BSN_RadioRangeM$forwardmsg);
}

static inline 
# 118 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
void TimerM$removeTimer(uint8_t num)
{
  TimerM$m_timers[num].isset = FALSE;
}

static inline  
#line 454
result_t TimerM$Timer$stop(uint8_t num)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 456
    TimerM$removeTimer(num);
#line 456
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 68 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Timer.nc"
inline static  result_t BSN_RadioRangeM$Timer$stop(void){
#line 68
  unsigned char result;
#line 68

#line 68
  result = TimerM$Timer$stop(0);
#line 68

#line 68
  return result;
#line 68
}
#line 68
static inline 
# 13 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
void TOSH_CLR_YELLOW_LED_PIN(void)
#line 13
{
   
#line 13
  static volatile uint8_t r __asm ("0x0031");

#line 13
  r &= ~(1 << 6);
}

static inline   
# 130 "C:/cygwin/opt/tinyos-1.x/tos/system/LedsC.nc"
result_t LedsC$Leds$yellowOn(void)
#line 130
{
  {
  }
#line 131
  ;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 132
    {
      TOSH_CLR_YELLOW_LED_PIN();
      LedsC$ledsOn |= LedsC$YELLOW_BIT;
    }
#line 135
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 114 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Leds.nc"
inline static   result_t BSN_RadioRangeM$Leds$yellowOn(void){
#line 114
  unsigned char result;
#line 114

#line 114
  result = LedsC$Leds$yellowOn();
#line 114

#line 114
  return result;
#line 114
}
#line 114
#line 89
inline static   result_t BSN_RadioRangeM$Leds$greenOn(void){
#line 89
  unsigned char result;
#line 89

#line 89
  result = LedsC$Leds$greenOn();
#line 89

#line 89
  return result;
#line 89
}
#line 89
#line 64
inline static   result_t BSN_RadioRangeM$Leds$redOn(void){
#line 64
  unsigned char result;
#line 64

#line 64
  result = LedsC$Leds$redOn();
#line 64

#line 64
  return result;
#line 64
}
#line 64
static inline 
# 172 "BSN_RadioRangeM.nc"
TOS_MsgPtr BSN_RadioRangeM$receive(TOS_MsgPtr data, bool fromUART)
{
  struct UbiMonMsg *pack = (struct UbiMonMsg *)data->data;

#line 175
  if (pack->destination == TOS_LOCAL_ADDRESS || pack->destination == TOS_BCAST_ADDR) 
    {
      if (pack->BSNcommand == 0) 
        {
          if (pack->redStatus) {
#line 179
            BSN_RadioRangeM$Leds$redOn();
            }
          else {
#line 180
            BSN_RadioRangeM$Leds$redOff();
            }
#line 181
          if (pack->greenStatus) {
#line 181
            BSN_RadioRangeM$Leds$greenOn();
            }
          else {
#line 182
            BSN_RadioRangeM$Leds$greenOff();
            }
#line 183
          if (pack->yellowStatus) {
#line 183
            BSN_RadioRangeM$Leds$yellowOn();
            }
          else {
#line 184
            BSN_RadioRangeM$Leds$yellowOff();
            }
        }
      else {
#line 186
        if (pack->BSNcommand == 0xfffe) 
          {
            BSN_RadioRangeM$Timer$stop();
            BSN_RadioRangeM$Leds$redOff();
            BSN_RadioRangeM$Leds$yellowOff();
            BSN_RadioRangeM$Leds$greenOff();
            BSN_RadioRangeM$state = BSN_RadioRangeM$STARTED;
          }
        else {
#line 194
          if (pack->BSNcommand == 0x80) 
            {
              if (fromUART) {
#line 196
                BSN_RadioRangeM$heartbeatfromUART = 1;
                }
              else {
#line 197
                BSN_RadioRangeM$heartbeatfromUART = 0;
                }
#line 198
              TOS_post(BSN_RadioRangeM$ReplyHeartBeat);
            }
          }
        }
    }
#line 201
  if (pack->destination == TOS_BCAST_ADDR) 
    {
      if (!fromUART) 
        {
          if (pack->BSNcommand == 0x40) 
            {
              BSN_RadioRangeM$state = BSN_RadioRangeM$REGISTER;
              BSN_RadioRangeM$Leds$redOff();
              BSN_RadioRangeM$Leds$greenOff();
              BSN_RadioRangeM$Leds$yellowOff();
              BSN_RadioRangeM$Timer$start(TIMER_REPEAT, 2234);
            }
        }
      if (!fromUART && BSN_RadioRangeM$UARTConnected) 
        {
          BSN_RadioRangeM$forwardmsg = *data;
          pack->signalstrength = data->strength;
          if (pack->BSNcommand == 0xffff) {
            pack->destination = TOS_LOCAL_ADDRESS;
            }
#line 220
          nmemcpy(BSN_RadioRangeM$forwardmsg.data, data->data, 66);
          TOS_post(BSN_RadioRangeM$ForwardUart);
        }
    }
  return data;
}

static inline  

TOS_MsgPtr BSN_RadioRangeM$ReceiveMsg$receive(TOS_MsgPtr data)
{
  return BSN_RadioRangeM$receive(data, FALSE);
}

static inline   
# 242 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
TOS_MsgPtr AMStandard$ReceiveMsg$default$receive(uint8_t id, TOS_MsgPtr msg)
#line 242
{
  return msg;
}

# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
inline static  TOS_MsgPtr AMStandard$ReceiveMsg$receive(uint8_t arg_0xa855228, TOS_MsgPtr arg_0xa7dbeb8){
#line 75
  struct TOS_Msg *result;
#line 75

#line 75
  switch (arg_0xa855228) {
#line 75
    case AM_OSCOPEMSG:
#line 75
      result = BSN_RadioRangeM$ReceiveMsg$receive(arg_0xa7dbeb8);
#line 75
      break;
#line 75
    default:
#line 75
      result = AMStandard$ReceiveMsg$default$receive(arg_0xa855228, arg_0xa7dbeb8);
#line 75
    }
#line 75

#line 75
  return result;
#line 75
}
#line 75
static inline    
# 488 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
result_t HPLUSART0M$USARTData$default$rxDone(uint8_t data)
#line 488
{
#line 488
  return SUCCESS;
}

# 53 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
inline static   result_t HPLUSART0M$USARTData$rxDone(uint8_t arg_0xa982da8){
#line 53
  unsigned char result;
#line 53

#line 53
  result = HPLUSART0M$USARTData$default$rxDone(arg_0xa982da8);
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 58 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
void __attribute((interrupt(18))) __attribute((wakeup))  sig_UART0RX_VECTOR(void)
#line 58
{
  uint8_t temp = U0RXBUF;

#line 60
  HPLUSART0M$USARTData$rxDone(temp);
}

static inline    
#line 486
result_t HPLUSART0M$USARTData$default$txDone(void)
#line 486
{
#line 486
  return SUCCESS;
}

# 46 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
inline static   result_t HPLUSART0M$USARTData$txDone(void){
#line 46
  unsigned char result;
#line 46

#line 46
  result = HPLUSART0M$USARTData$default$txDone();
#line 46

#line 46
  return result;
#line 46
}
#line 46
static inline    
# 70 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
void HPLUSART0M$HPLI2CInterrupt$default$fired(void)
#line 70
{
}

# 43 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLI2CInterrupt.nc"
inline static   void HPLUSART0M$HPLI2CInterrupt$fired(void){
#line 43
  HPLUSART0M$HPLI2CInterrupt$default$fired();
#line 43
}
#line 43
# 63 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
void __attribute((interrupt(16))) __attribute((wakeup))  sig_UART0TX_VECTOR(void)
#line 63
{
  if (HPLUSART0M$USARTControl$isI2C()) {
    HPLUSART0M$HPLI2CInterrupt$fired();
    }
  else {
#line 67
    HPLUSART0M$USARTData$txDone();
    }
}

static inline   
# 177 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port10$clear(void)
#line 177
{
#line 177
  MSP430InterruptM$P1IFG &= ~(1 << 0);
}

# 40 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOPInterrupt$clear(void){
#line 40
  MSP430InterruptM$Port10$clear();
#line 40
}
#line 40
static inline   
# 146 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port10$disable(void)
#line 146
{
#line 146
  MSP430InterruptM$P1IE &= ~(1 << 0);
}

# 35 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOPInterrupt$disable(void){
#line 35
  MSP430InterruptM$Port10$disable();
#line 35
}
#line 35
static inline   
# 78 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420InterruptM.nc"
result_t HPLCC2420InterruptM$FIFOP$disable(void)
#line 78
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 79
    {
      HPLCC2420InterruptM$FIFOPInterrupt$disable();
      HPLCC2420InterruptM$FIFOPInterrupt$clear();
    }
#line 82
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
inline static   result_t CC2420RadioM$FIFOP$disable(void){
#line 59
  unsigned char result;
#line 59

#line 59
  result = HPLCC2420InterruptM$FIFOP$disable();
#line 59

#line 59
  return result;
#line 59
}
#line 59
static inline  
# 536 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
void CC2420RadioM$delayedRXFIFOtask(void)
#line 536
{
  CC2420RadioM$delayedRXFIFO();
}

static inline 
# 27 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
uint8_t TOSH_READ_CC_FIFO_PIN(void)
#line 27
{
   
#line 27
  static volatile uint8_t r __asm ("0x0020");

#line 27
  return r & (1 << 3);
}

static inline   
# 104 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/TimerJiffyAsyncM.nc"
result_t TimerJiffyAsyncM$TimerJiffyAsync$stop(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 106
    {
      TimerJiffyAsyncM$bSet = FALSE;
      TimerJiffyAsyncM$AlarmControl$disableEvents();
    }
#line 109
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 8 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/TimerJiffyAsync.nc"
inline static   result_t CC2420RadioM$BackoffTimerJiffy$stop(void){
#line 8
  unsigned char result;
#line 8

#line 8
  result = TimerJiffyAsyncM$TimerJiffyAsync$stop();
#line 8

#line 8
  return result;
#line 8
}
#line 8
static inline   
# 97 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/TimerJiffyAsyncM.nc"
bool TimerJiffyAsyncM$TimerJiffyAsync$isSet(void)
{
  bool _isSet;

#line 100
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 100
    _isSet = TimerJiffyAsyncM$bSet;
#line 100
    __nesc_atomic_end(__nesc_atomic); }
  return _isSet;
}

# 10 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/TimerJiffyAsync.nc"
inline static   bool CC2420RadioM$BackoffTimerJiffy$isSet(void){
#line 10
  unsigned char result;
#line 10

#line 10
  result = TimerJiffyAsyncM$TimerJiffyAsync$isSet();
#line 10

#line 10
  return result;
#line 10
}
#line 10
static inline   
# 591 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$FIFOP$fired(void)
#line 591
{






  if (CC2420RadioM$bAckEnable && CC2420RadioM$stateRadio == CC2420RadioM$PRE_TX_STATE) {
      if (CC2420RadioM$BackoffTimerJiffy$isSet()) {
          CC2420RadioM$BackoffTimerJiffy$stop();
          CC2420RadioM$BackoffTimerJiffy$setOneShot(CC2420RadioM$MacBackoff$congestionBackoff(CC2420RadioM$txbufptr) * 10 + 75);
        }
    }


  if (!TOSH_READ_CC_FIFO_PIN()) {
      CC2420RadioM$flushRXFIFO();
      return SUCCESS;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 611
    {
      if (TOS_post(CC2420RadioM$delayedRXFIFOtask)) {
          CC2420RadioM$FIFOP$disable();
        }
      else {
          CC2420RadioM$flushRXFIFO();
        }
    }
#line 618
    __nesc_atomic_end(__nesc_atomic); }


  return SUCCESS;
}

# 51 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
inline static   result_t HPLCC2420InterruptM$FIFOP$fired(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = CC2420RadioM$FIFOP$fired();
#line 51

#line 51
  return result;
#line 51
}
#line 51
static inline   
# 89 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420InterruptM.nc"
void HPLCC2420InterruptM$FIFOPInterrupt$fired(void)
#line 89
{
  result_t val = SUCCESS;

#line 91
  HPLCC2420InterruptM$FIFOPInterrupt$clear();
  val = HPLCC2420InterruptM$FIFOP$fired();
  if (val == FAIL) {
      HPLCC2420InterruptM$FIFOPInterrupt$disable();
      HPLCC2420InterruptM$FIFOPInterrupt$clear();
    }
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port10$fired(void){
#line 59
  HPLCC2420InterruptM$FIFOPInterrupt$fired();
#line 59
}
#line 59
static inline 
# 26 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/hardware.h"
uint8_t TOSH_READ_CC_FIFOP_PIN(void)
#line 26
{
   
#line 26
  static volatile uint8_t r __asm ("0x0020");

#line 26
  return r & (1 << 0);
}

static inline  
# 252 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
TOS_MsgPtr AMStandard$RadioReceive$receive(TOS_MsgPtr packet)
#line 252
{
  return received(packet);
}

# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
inline static  TOS_MsgPtr CC2420RadioM$Receive$receive(TOS_MsgPtr arg_0xa7dbeb8){
#line 75
  struct TOS_Msg *result;
#line 75

#line 75
  result = AMStandard$RadioReceive$receive(arg_0xa7dbeb8);
#line 75

#line 75
  return result;
#line 75
}
#line 75
static inline  
# 153 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
void CC2420RadioM$PacketRcvd(void)
#line 153
{
  TOS_MsgPtr pBuf;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 156
    {
      pBuf = CC2420RadioM$rxbufptr;
    }
#line 158
    __nesc_atomic_end(__nesc_atomic); }
  pBuf = CC2420RadioM$Receive$receive((TOS_MsgPtr )pBuf);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 160
    {
      if (pBuf) {
#line 161
        CC2420RadioM$rxbufptr = pBuf;
        }
#line 162
      CC2420RadioM$rxbufptr->length = 0;
      CC2420RadioM$bPacketReceiving = FALSE;
    }
#line 164
    __nesc_atomic_end(__nesc_atomic); }
}

static 
# 23 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/byteorder.h"
__inline uint16_t fromLSB16(uint16_t a)
{
  return is_host_lsb() ? a : (a << 8) | (a >> 8);
}

static inline   
# 628 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$HPLChipconFIFO$RXFIFODone(uint8_t length, uint8_t *data)
#line 628
{





  uint8_t currentstate;

#line 635
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 635
    {
      currentstate = CC2420RadioM$stateRadio;
    }
#line 637
    __nesc_atomic_end(__nesc_atomic); }




  if (((
#line 641
  !TOSH_READ_CC_FIFO_PIN() && !TOSH_READ_CC_FIFOP_PIN())
   || length == 0) || length > MSG_DATA_SIZE) {
      CC2420RadioM$flushRXFIFO();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 644
        CC2420RadioM$bPacketReceiving = FALSE;
#line 644
        __nesc_atomic_end(__nesc_atomic); }
      return SUCCESS;
    }

  CC2420RadioM$rxbufptr = (TOS_MsgPtr )data;




  if (
#line 651
  CC2420RadioM$bAckEnable && currentstate == CC2420RadioM$POST_TX_STATE && (
  CC2420RadioM$rxbufptr->fcfhi & 0x07) == 0x02 && 
  CC2420RadioM$rxbufptr->dsn == CC2420RadioM$currentDSN && 
  data[length - 1] >> 7 == 1) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 655
        {
          CC2420RadioM$txbufptr->ack = 1;
          CC2420RadioM$txbufptr->strength = data[length - 2];
          CC2420RadioM$txbufptr->lqi = data[length - 1] & 0x7F;

          CC2420RadioM$stateRadio = CC2420RadioM$POST_TX_ACK_STATE;
          CC2420RadioM$bPacketReceiving = FALSE;
        }
#line 662
        __nesc_atomic_end(__nesc_atomic); }
      if (!TOS_post(CC2420RadioM$PacketSent)) {
        CC2420RadioM$sendFailed();
        }
#line 665
      return SUCCESS;
    }




  if ((CC2420RadioM$rxbufptr->fcfhi & 0x07) != 0x01 || 
  CC2420RadioM$rxbufptr->fcflo != 0x08) {
      CC2420RadioM$flushRXFIFO();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 674
        CC2420RadioM$bPacketReceiving = FALSE;
#line 674
        __nesc_atomic_end(__nesc_atomic); }
      return SUCCESS;
    }

  CC2420RadioM$rxbufptr->length = CC2420RadioM$rxbufptr->length - MSG_HEADER_SIZE - MSG_FOOTER_SIZE;

  if (CC2420RadioM$rxbufptr->length > 66) {
      CC2420RadioM$flushRXFIFO();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 682
        CC2420RadioM$bPacketReceiving = FALSE;
#line 682
        __nesc_atomic_end(__nesc_atomic); }
      return SUCCESS;
    }


  CC2420RadioM$rxbufptr->addr = fromLSB16(CC2420RadioM$rxbufptr->addr);


  CC2420RadioM$rxbufptr->crc = data[length - 1] >> 7;

  CC2420RadioM$rxbufptr->strength = data[length - 2];

  CC2420RadioM$rxbufptr->lqi = data[length - 1] & 0x7F;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 696
    {
      if (!TOS_post(CC2420RadioM$PacketRcvd)) {
          CC2420RadioM$bPacketReceiving = FALSE;
        }
    }
#line 700
    __nesc_atomic_end(__nesc_atomic); }

  if (!TOSH_READ_CC_FIFO_PIN() && !TOSH_READ_CC_FIFOP_PIN()) {
      CC2420RadioM$flushRXFIFO();
      return SUCCESS;
    }

  if (!TOSH_READ_CC_FIFOP_PIN()) {
      if (TOS_post(CC2420RadioM$delayedRXFIFOtask)) {
        return SUCCESS;
        }
    }
#line 711
  CC2420RadioM$flushRXFIFO();


  return SUCCESS;
}

# 39 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
inline static   result_t HPLCC2420M$HPLCC2420FIFO$RXFIFODone(uint8_t arg_0xa8a6890, uint8_t *arg_0xa8a69f0){
#line 39
  unsigned char result;
#line 39

#line 39
  result = CC2420RadioM$HPLChipconFIFO$RXFIFODone(arg_0xa8a6890, arg_0xa8a69f0);
#line 39

#line 39
  return result;
#line 39
}
#line 39
static inline  
# 295 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420M.nc"
void HPLCC2420M$signalRXFIFO(void)
#line 295
{
  uint8_t _rxlen;
  uint8_t *_rxbuf;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 299
    {
      _rxlen = HPLCC2420M$rxlen;
      _rxbuf = HPLCC2420M$rxbuf;
      HPLCC2420M$rxbufBusy = FALSE;
    }
#line 303
    __nesc_atomic_end(__nesc_atomic); }

  HPLCC2420M$HPLCC2420FIFO$RXFIFODone(_rxlen, _rxbuf);
}

static inline   
# 439 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
result_t HPLUSART0M$USARTControl$isRxIntrPending(void)
#line 439
{
  if (HPLUSART0M$IFG1 & (1 << 6)) {
      HPLUSART0M$IFG1 &= ~(1 << 6);
      return SUCCESS;
    }
  return FAIL;
}

# 185 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$isRxIntrPending(void){
#line 185
  unsigned char result;
#line 185

#line 185
  result = HPLUSART0M$USARTControl$isRxIntrPending();
#line 185

#line 185
  return result;
#line 185
}
#line 185
static inline   
# 319 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420M.nc"
result_t HPLCC2420M$HPLCC2420FIFO$readRXFIFO(uint8_t length, uint8_t *data)
#line 319
{
  uint8_t i;
  bool returnFail = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 323
    {
      if (HPLCC2420M$rxbufBusy) {
        returnFail = TRUE;
        }
      else {
#line 327
        HPLCC2420M$rxbufBusy = TRUE;
        }
    }
#line 329
    __nesc_atomic_end(__nesc_atomic); }
  if (returnFail) {
    return FAIL;
    }

  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 335
        {
          HPLCC2420M$rxbuf = data;
          TOSH_CLR_RADIO_CSN_PIN();

          HPLCC2420M$USARTControl$isTxIntrPending();
          HPLCC2420M$USARTControl$rx();
          HPLCC2420M$USARTControl$tx(0x3F | 0x40);
          while (!HPLCC2420M$USARTControl$isRxIntrPending()) ;
          HPLCC2420M$rxlen = HPLCC2420M$USARTControl$rx();
          HPLCC2420M$USARTControl$tx(0);
          while (!HPLCC2420M$USARTControl$isRxIntrPending()) ;

          HPLCC2420M$rxlen = HPLCC2420M$USARTControl$rx();
        }
#line 348
        __nesc_atomic_end(__nesc_atomic); }
      if (HPLCC2420M$rxlen > 0) {
          HPLCC2420M$rxbuf[0] = HPLCC2420M$rxlen;

          HPLCC2420M$rxlen++;

          if (HPLCC2420M$rxlen > length) {
#line 354
            HPLCC2420M$rxlen = length;
            }
#line 355
          for (i = 1; i < HPLCC2420M$rxlen; i++) {
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 356
                {
                  HPLCC2420M$USARTControl$tx(0);
                  while (!HPLCC2420M$USARTControl$isRxIntrPending()) ;
                  HPLCC2420M$rxbuf[i] = HPLCC2420M$USARTControl$rx();
                }
#line 360
                __nesc_atomic_end(__nesc_atomic); }
            }
        }
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$BusArbitration$releaseBus();
    }
  else 
#line 401
    {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 402
        HPLCC2420M$rxbufBusy = FALSE;
#line 402
        __nesc_atomic_end(__nesc_atomic); }
      return FAIL;
    }
  if (TOS_post(HPLCC2420M$signalRXFIFO) == FAIL) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 406
        HPLCC2420M$rxbufBusy = FALSE;
#line 406
        __nesc_atomic_end(__nesc_atomic); }
      return FAIL;
    }
  return SUCCESS;
}

# 19 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
inline static   result_t CC2420RadioM$HPLChipconFIFO$readRXFIFO(uint8_t arg_0xa8abab8, uint8_t *arg_0xa8abc18){
#line 19
  unsigned char result;
#line 19

#line 19
  result = HPLCC2420M$HPLCC2420FIFO$readRXFIFO(arg_0xa8abab8, arg_0xa8abc18);
#line 19

#line 19
  return result;
#line 19
}
#line 19
static inline   
# 178 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port11$clear(void)
#line 178
{
#line 178
  MSP430InterruptM$P1IFG &= ~(1 << 1);
}

static inline    
#line 94
void MSP430InterruptM$Port11$default$fired(void)
#line 94
{
#line 94
  MSP430InterruptM$Port11$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port11$fired(void){
#line 59
  MSP430InterruptM$Port11$default$fired();
#line 59
}
#line 59
static inline   
# 179 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port12$clear(void)
#line 179
{
#line 179
  MSP430InterruptM$P1IFG &= ~(1 << 2);
}

static inline    
#line 95
void MSP430InterruptM$Port12$default$fired(void)
#line 95
{
#line 95
  MSP430InterruptM$Port12$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port12$fired(void){
#line 59
  MSP430InterruptM$Port12$default$fired();
#line 59
}
#line 59
static inline   
# 180 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port13$clear(void)
#line 180
{
#line 180
  MSP430InterruptM$P1IFG &= ~(1 << 3);
}

# 40 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOInterrupt$clear(void){
#line 40
  MSP430InterruptM$Port13$clear();
#line 40
}
#line 40
static inline   
# 149 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port13$disable(void)
#line 149
{
#line 149
  MSP430InterruptM$P1IE &= ~(1 << 3);
}

# 35 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420InterruptM$FIFOInterrupt$disable(void){
#line 35
  MSP430InterruptM$Port13$disable();
#line 35
}
#line 35
static inline    
# 140 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420InterruptM.nc"
result_t HPLCC2420InterruptM$FIFO$default$fired(void)
#line 140
{
#line 140
  return FAIL;
}

# 51 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
inline static   result_t HPLCC2420InterruptM$FIFO$fired(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = HPLCC2420InterruptM$FIFO$default$fired();
#line 51

#line 51
  return result;
#line 51
}
#line 51
static inline   
# 130 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420InterruptM.nc"
void HPLCC2420InterruptM$FIFOInterrupt$fired(void)
#line 130
{
  result_t val = SUCCESS;

#line 132
  HPLCC2420InterruptM$FIFOInterrupt$clear();
  val = HPLCC2420InterruptM$FIFO$fired();
  if (val == FAIL) {
      HPLCC2420InterruptM$FIFOInterrupt$disable();
      HPLCC2420InterruptM$FIFOInterrupt$clear();
    }
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port13$fired(void){
#line 59
  HPLCC2420InterruptM$FIFOInterrupt$fired();
#line 59
}
#line 59
static inline   
# 312 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$SplitControl$default$startDone(void)
#line 312
{
  return SUCCESS;
}

# 85 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static  result_t CC2420RadioM$SplitControl$startDone(void){
#line 85
  unsigned char result;
#line 85

#line 85
  result = CC2420RadioM$SplitControl$default$startDone();
#line 85

#line 85
  return result;
#line 85
}
#line 85
# 43 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
inline static   result_t CC2420RadioM$FIFOP$startWait(bool arg_0xa8da5c0){
#line 43
  unsigned char result;
#line 43

#line 43
  result = HPLCC2420InterruptM$FIFOP$startWait(arg_0xa8da5c0);
#line 43

#line 43
  return result;
#line 43
}
#line 43
static inline   
# 343 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
result_t CC2420ControlM$CC2420Control$RxMode(void)
#line 343
{
  CC2420ControlM$HPLChipcon$cmd(0x03);
  return SUCCESS;
}

# 163 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420Control.nc"
inline static   result_t CC2420RadioM$CC2420Control$RxMode(void){
#line 163
  unsigned char result;
#line 163

#line 163
  result = CC2420ControlM$CC2420Control$RxMode();
#line 163

#line 163
  return result;
#line 163
}
#line 163
static inline  
# 294 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$CC2420SplitControl$startDone(void)
#line 294
{
  uint8_t chkstateRadio;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 297
    chkstateRadio = CC2420RadioM$stateRadio;
#line 297
    __nesc_atomic_end(__nesc_atomic); }

  if (chkstateRadio == CC2420RadioM$WARMUP_STATE) {
      CC2420RadioM$CC2420Control$RxMode();

      CC2420RadioM$FIFOP$startWait(FALSE);

      CC2420RadioM$SFD$enableCapture(TRUE);

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 306
        CC2420RadioM$stateRadio = CC2420RadioM$IDLE_STATE;
#line 306
        __nesc_atomic_end(__nesc_atomic); }
    }
  CC2420RadioM$SplitControl$startDone();
  return SUCCESS;
}

# 85 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SplitControl.nc"
inline static  result_t CC2420ControlM$SplitControl$startDone(void){
#line 85
  unsigned char result;
#line 85

#line 85
  result = CC2420RadioM$CC2420SplitControl$startDone();
#line 85

#line 85
  return result;
#line 85
}
#line 85
static inline  
# 286 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
result_t CC2420ControlM$CC2420Control$TuneManual(uint16_t DesiredFreq)
#line 286
{
  int fsctrl;
  uint8_t status;

  fsctrl = DesiredFreq - 2048;
  CC2420ControlM$gCurrentParameters[CP_FSCTRL] = (CC2420ControlM$gCurrentParameters[CP_FSCTRL] & 0xfc00) | (fsctrl << 0);
  status = CC2420ControlM$HPLChipcon$write(0x18, CC2420ControlM$gCurrentParameters[CP_FSCTRL]);


  if (status & (1 << 6)) {
    CC2420ControlM$HPLChipcon$cmd(0x03);
    }
#line 297
  return SUCCESS;
}

static inline   
#line 441
result_t CC2420ControlM$HPLChipconRAM$writeDone(uint16_t addr, uint8_t length, uint8_t *buffer)
#line 441
{
  return SUCCESS;
}

# 49 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
inline static   result_t HPLCC2420M$HPLCC2420RAM$writeDone(uint16_t arg_0xa906738, uint8_t arg_0xa906880, uint8_t *arg_0xa9069e0){
#line 49
  unsigned char result;
#line 49

#line 49
  result = CC2420ControlM$HPLChipconRAM$writeDone(arg_0xa906738, arg_0xa906880, arg_0xa9069e0);
#line 49

#line 49
  return result;
#line 49
}
#line 49
static inline  
# 263 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420M.nc"
void HPLCC2420M$signalRAMWr(void)
#line 263
{
  HPLCC2420M$HPLCC2420RAM$writeDone(HPLCC2420M$ramaddr, HPLCC2420M$ramlen, HPLCC2420M$rambuf);
}

static inline   result_t HPLCC2420M$HPLCC2420RAM$write(uint16_t addr, uint8_t length, uint8_t *buffer)
#line 267
{
  uint8_t i = 0;

#line 269
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 270
        {
          HPLCC2420M$ramaddr = addr;
          HPLCC2420M$ramlen = length;
          HPLCC2420M$rambuf = buffer;
        }
#line 274
        __nesc_atomic_end(__nesc_atomic); }
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx((HPLCC2420M$ramaddr & 0x7F) | 0x80);
      while (!HPLCC2420M$USARTControl$isTxIntrPending()) ;
      HPLCC2420M$USARTControl$tx((HPLCC2420M$ramaddr >> 1) & 0xC0);
      while (!HPLCC2420M$USARTControl$isTxIntrPending()) ;
      for (i = 0; i < HPLCC2420M$ramlen; i++) {
          HPLCC2420M$USARTControl$tx(HPLCC2420M$rambuf[i]);
          while (!HPLCC2420M$USARTControl$isTxIntrPending()) ;
        }
      while (!HPLCC2420M$USARTControl$isTxEmpty()) ;
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$BusArbitration$releaseBus();
      return TOS_post(HPLCC2420M$signalRAMWr);
    }
  return FAIL;
}

# 47 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
inline static   result_t CC2420ControlM$HPLChipconRAM$write(uint16_t arg_0xa906010, uint8_t arg_0xa906158, uint8_t *arg_0xa9062b8){
#line 47
  unsigned char result;
#line 47

#line 47
  result = HPLCC2420M$HPLCC2420RAM$write(arg_0xa906010, arg_0xa906158, arg_0xa9062b8);
#line 47

#line 47
  return result;
#line 47
}
#line 47
static inline  
# 432 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
result_t CC2420ControlM$CC2420Control$setShortAddress(uint16_t addr)
#line 432
{
  addr = toLSB16(addr);
  return CC2420ControlM$HPLChipconRAM$write(0x16A, 2, (uint8_t *)&addr);
}

# 61 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static   uint16_t CC2420ControlM$HPLChipcon$read(uint8_t arg_0xa8aa798){
#line 61
  unsigned int result;
#line 61

#line 61
  result = HPLCC2420M$HPLCC2420$read(arg_0xa8aa798);
#line 61

#line 61
  return result;
#line 61
}
#line 61
static inline 
# 80 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
bool CC2420ControlM$SetRegs(void)
#line 80
{
  uint16_t data;

  CC2420ControlM$HPLChipcon$write(0x10, CC2420ControlM$gCurrentParameters[CP_MAIN]);
  CC2420ControlM$HPLChipcon$write(0x11, CC2420ControlM$gCurrentParameters[CP_MDMCTRL0]);
  data = CC2420ControlM$HPLChipcon$read(0x11);
  if (data != CC2420ControlM$gCurrentParameters[CP_MDMCTRL0]) {
#line 86
    return FALSE;
    }
  CC2420ControlM$HPLChipcon$write(0x12, CC2420ControlM$gCurrentParameters[CP_MDMCTRL1]);
  CC2420ControlM$HPLChipcon$write(0x13, CC2420ControlM$gCurrentParameters[CP_RSSI]);
  CC2420ControlM$HPLChipcon$write(0x14, CC2420ControlM$gCurrentParameters[CP_SYNCWORD]);
  CC2420ControlM$HPLChipcon$write(0x15, CC2420ControlM$gCurrentParameters[CP_TXCTRL]);
  CC2420ControlM$HPLChipcon$write(0x16, CC2420ControlM$gCurrentParameters[CP_RXCTRL0]);
  CC2420ControlM$HPLChipcon$write(0x17, CC2420ControlM$gCurrentParameters[CP_RXCTRL1]);
  CC2420ControlM$HPLChipcon$write(0x18, CC2420ControlM$gCurrentParameters[CP_FSCTRL]);

  CC2420ControlM$HPLChipcon$write(0x19, CC2420ControlM$gCurrentParameters[CP_SECCTRL0]);
  CC2420ControlM$HPLChipcon$write(0x1A, CC2420ControlM$gCurrentParameters[CP_SECCTRL1]);
  CC2420ControlM$HPLChipcon$write(0x1C, CC2420ControlM$gCurrentParameters[CP_IOCFG0]);
  CC2420ControlM$HPLChipcon$write(0x1D, CC2420ControlM$gCurrentParameters[CP_IOCFG1]);

  CC2420ControlM$HPLChipcon$cmd(0x09);
  CC2420ControlM$HPLChipcon$cmd(0x08);

  return TRUE;
}

static inline  








void CC2420ControlM$PostOscillatorOn(void)
#line 116
{

  CC2420ControlM$SetRegs();
  CC2420ControlM$CC2420Control$setShortAddress(TOS_LOCAL_ADDRESS);
  CC2420ControlM$CC2420Control$TuneManual(((CC2420ControlM$gCurrentParameters[CP_FSCTRL] << 0) & 0x1FF) + 2048);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 121
    CC2420ControlM$state = CC2420ControlM$START_STATE_DONE;
#line 121
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlM$SplitControl$startDone();
}

static inline   
#line 445
result_t CC2420ControlM$CCA$fired(void)
#line 445
{

  CC2420ControlM$HPLChipcon$write(0x1D, 0);
  TOS_post(CC2420ControlM$PostOscillatorOn);
  return FAIL;
}

# 51 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420Interrupt.nc"
inline static   result_t HPLCC2420InterruptM$CCA$fired(void){
#line 51
  unsigned char result;
#line 51

#line 51
  result = CC2420ControlM$CCA$fired();
#line 51

#line 51
  return result;
#line 51
}
#line 51
static inline   
# 171 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420InterruptM.nc"
void HPLCC2420InterruptM$CCAInterrupt$fired(void)
#line 171
{
  result_t val = SUCCESS;

#line 173
  HPLCC2420InterruptM$CCAInterrupt$clear();
  val = HPLCC2420InterruptM$CCA$fired();
  if (val == FAIL) {
      HPLCC2420InterruptM$CCAInterrupt$disable();
      HPLCC2420InterruptM$CCAInterrupt$clear();
    }
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port14$fired(void){
#line 59
  HPLCC2420InterruptM$CCAInterrupt$fired();
#line 59
}
#line 59
static inline   
# 182 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port15$clear(void)
#line 182
{
#line 182
  MSP430InterruptM$P1IFG &= ~(1 << 5);
}

static inline    
#line 98
void MSP430InterruptM$Port15$default$fired(void)
#line 98
{
#line 98
  MSP430InterruptM$Port15$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port15$fired(void){
#line 59
  MSP430InterruptM$Port15$default$fired();
#line 59
}
#line 59
static inline   
# 183 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port16$clear(void)
#line 183
{
#line 183
  MSP430InterruptM$P1IFG &= ~(1 << 6);
}

static inline    
#line 99
void MSP430InterruptM$Port16$default$fired(void)
#line 99
{
#line 99
  MSP430InterruptM$Port16$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port16$fired(void){
#line 59
  MSP430InterruptM$Port16$default$fired();
#line 59
}
#line 59
static inline   
# 184 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port17$clear(void)
#line 184
{
#line 184
  MSP430InterruptM$P1IFG &= ~(1 << 7);
}

static inline    
#line 100
void MSP430InterruptM$Port17$default$fired(void)
#line 100
{
#line 100
  MSP430InterruptM$Port17$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port17$fired(void){
#line 59
  MSP430InterruptM$Port17$default$fired();
#line 59
}
#line 59
static inline   
# 186 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port20$clear(void)
#line 186
{
#line 186
  MSP430InterruptM$P2IFG &= ~(1 << 0);
}

static inline    
#line 102
void MSP430InterruptM$Port20$default$fired(void)
#line 102
{
#line 102
  MSP430InterruptM$Port20$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port20$fired(void){
#line 59
  MSP430InterruptM$Port20$default$fired();
#line 59
}
#line 59
static inline   
# 187 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port21$clear(void)
#line 187
{
#line 187
  MSP430InterruptM$P2IFG &= ~(1 << 1);
}

static inline    
#line 103
void MSP430InterruptM$Port21$default$fired(void)
#line 103
{
#line 103
  MSP430InterruptM$Port21$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port21$fired(void){
#line 59
  MSP430InterruptM$Port21$default$fired();
#line 59
}
#line 59
static inline   
# 188 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port22$clear(void)
#line 188
{
#line 188
  MSP430InterruptM$P2IFG &= ~(1 << 2);
}

static inline    
#line 104
void MSP430InterruptM$Port22$default$fired(void)
#line 104
{
#line 104
  MSP430InterruptM$Port22$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port22$fired(void){
#line 59
  MSP430InterruptM$Port22$default$fired();
#line 59
}
#line 59
static inline   
# 189 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port23$clear(void)
#line 189
{
#line 189
  MSP430InterruptM$P2IFG &= ~(1 << 3);
}

static inline    
#line 105
void MSP430InterruptM$Port23$default$fired(void)
#line 105
{
#line 105
  MSP430InterruptM$Port23$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port23$fired(void){
#line 59
  MSP430InterruptM$Port23$default$fired();
#line 59
}
#line 59
static inline   
# 190 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port24$clear(void)
#line 190
{
#line 190
  MSP430InterruptM$P2IFG &= ~(1 << 4);
}

static inline    
#line 106
void MSP430InterruptM$Port24$default$fired(void)
#line 106
{
#line 106
  MSP430InterruptM$Port24$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port24$fired(void){
#line 59
  MSP430InterruptM$Port24$default$fired();
#line 59
}
#line 59
static inline   
# 191 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port25$clear(void)
#line 191
{
#line 191
  MSP430InterruptM$P2IFG &= ~(1 << 5);
}

static inline    
#line 107
void MSP430InterruptM$Port25$default$fired(void)
#line 107
{
#line 107
  MSP430InterruptM$Port25$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port25$fired(void){
#line 59
  MSP430InterruptM$Port25$default$fired();
#line 59
}
#line 59
static inline   
# 192 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port26$clear(void)
#line 192
{
#line 192
  MSP430InterruptM$P2IFG &= ~(1 << 6);
}

static inline    
#line 108
void MSP430InterruptM$Port26$default$fired(void)
#line 108
{
#line 108
  MSP430InterruptM$Port26$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port26$fired(void){
#line 59
  MSP430InterruptM$Port26$default$fired();
#line 59
}
#line 59
static inline   
# 193 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port27$clear(void)
#line 193
{
#line 193
  MSP430InterruptM$P2IFG &= ~(1 << 7);
}

static inline    
#line 109
void MSP430InterruptM$Port27$default$fired(void)
#line 109
{
#line 109
  MSP430InterruptM$Port27$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port27$fired(void){
#line 59
  MSP430InterruptM$Port27$default$fired();
#line 59
}
#line 59
static inline   
# 195 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$NMI$clear(void)
#line 195
{
#line 195
  IFG1 &= ~(1 << 4);
}

static inline    
#line 111
void MSP430InterruptM$NMI$default$fired(void)
#line 111
{
#line 111
  MSP430InterruptM$NMI$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$NMI$fired(void){
#line 59
  MSP430InterruptM$NMI$default$fired();
#line 59
}
#line 59
static inline   
# 196 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$OF$clear(void)
#line 196
{
#line 196
  IFG1 &= ~(1 << 1);
}

static inline    
#line 112
void MSP430InterruptM$OF$default$fired(void)
#line 112
{
#line 112
  MSP430InterruptM$OF$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$OF$fired(void){
#line 59
  MSP430InterruptM$OF$default$fired();
#line 59
}
#line 59
static inline   
# 197 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$ACCV$clear(void)
#line 197
{
#line 197
  FCTL3 &= ~0x0004;
}

static inline    
#line 113
void MSP430InterruptM$ACCV$default$fired(void)
#line 113
{
#line 113
  MSP430InterruptM$ACCV$clear();
}

# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$ACCV$fired(void){
#line 59
  MSP430InterruptM$ACCV$default$fired();
#line 59
}
#line 59
# 66 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ByteComm.nc"
inline static   result_t UARTM$ByteComm$rxByteReady(uint8_t arg_0xaac8010, bool arg_0xaac8158, uint16_t arg_0xaac82b0){
#line 66
  unsigned char result;
#line 66

#line 66
  result = NoCRCPacket$ByteComm$rxByteReady(arg_0xaac8010, arg_0xaac8158, arg_0xaac82b0);
#line 66
  result = rcombine(result, FramerM$ByteComm$rxByteReady(arg_0xaac8010, arg_0xaac8158, arg_0xaac82b0));
#line 66

#line 66
  return result;
#line 66
}
#line 66
static inline   
# 77 "C:/cygwin/opt/tinyos-1.x/tos/system/UARTM.nc"
result_t UARTM$HPLUART$get(uint8_t data)
#line 77
{




  UARTM$ByteComm$rxByteReady(data, FALSE, 0);
  {
  }
#line 83
  ;
  return SUCCESS;
}

# 88 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/HPLUART.nc"
inline static   result_t HPLUARTM$UART$get(uint8_t arg_0xaad4300){
#line 88
  unsigned char result;
#line 88

#line 88
  result = UARTM$HPLUART$get(arg_0xaad4300);
#line 88

#line 88
  return result;
#line 88
}
#line 88
static inline   
# 90 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
result_t HPLUARTM$USARTData$rxDone(uint8_t b)
#line 90
{
  return HPLUARTM$UART$get(b);
}

# 53 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
inline static   result_t HPLUSART1M$USARTData$rxDone(uint8_t arg_0xa982da8){
#line 53
  unsigned char result;
#line 53

#line 53
  result = HPLUARTM$USARTData$rxDone(arg_0xa982da8);
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 55 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
void __attribute((interrupt(6))) __attribute((wakeup))  sig_UART1RX_VECTOR(void)
#line 55
{
  uint8_t temp = U1RXBUF;

#line 57
  HPLUSART1M$USARTData$rxDone(temp);
}

static inline 
# 241 "BSN_RadioRangeM.nc"
TOS_MsgPtr BSN_RadioRangeM$UartReceive(TOS_MsgPtr data)
{
  struct UbiMonMsg *pack = (struct UbiMonMsg *)data->data;

#line 244
  if (pack->destination == TOS_LOCAL_ADDRESS || 
  pack->destination == TOS_BCAST_ADDR) 
    {
      if (pack->BSNcommand == 0) 
        {
          if (pack->redStatus) {
#line 249
            BSN_RadioRangeM$Leds$redOn();
            }
          else {
#line 250
            BSN_RadioRangeM$Leds$redOff();
            }
#line 251
          if (pack->greenStatus) {
#line 251
            BSN_RadioRangeM$Leds$greenOn();
            }
          else {
#line 252
            BSN_RadioRangeM$Leds$greenOff();
            }
#line 253
          if (pack->yellowStatus) {
#line 253
            BSN_RadioRangeM$Leds$yellowOn();
            }
          else {
#line 254
            BSN_RadioRangeM$Leds$yellowOff();
            }
        }
      else {
#line 256
        if (pack->BSNcommand == 0xfffe) 
          {
            BSN_RadioRangeM$Timer$stop();
            BSN_RadioRangeM$Leds$redOff();
            BSN_RadioRangeM$Leds$yellowOff();
            BSN_RadioRangeM$Leds$greenOff();
            BSN_RadioRangeM$state = BSN_RadioRangeM$STARTED;
          }
        else {
#line 264
          if (pack->BSNcommand == 0x40) 
            {
              BSN_RadioRangeM$state = BSN_RadioRangeM$REGISTER;
              BSN_RadioRangeM$Leds$redOff();
              BSN_RadioRangeM$Leds$greenOff();
              BSN_RadioRangeM$Leds$yellowOff();
              BSN_RadioRangeM$Timer$start(TIMER_REPEAT, 2234);
              if (pack->destination == TOS_BCAST_ADDR) 
                {
                  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 273
                    {
                      BSN_RadioRangeM$forwardmsg = *data;
                      nmemcpy(BSN_RadioRangeM$forwardmsg.data, data->data, 66);
                    }
#line 276
                    __nesc_atomic_end(__nesc_atomic); }
                  TOS_post(BSN_RadioRangeM$ForwardRadio);
                }
            }
          else {
#line 280
            if (pack->BSNcommand == 0x80) 
              {
                BSN_RadioRangeM$heartbeatfromUART = 1;
                TOS_post(BSN_RadioRangeM$ReplyHeartBeat);
              }
            }
          }
        }
    }
#line 286
  if (pack->destination != TOS_LOCAL_ADDRESS) 
    {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 288
        {
          BSN_RadioRangeM$forwardmsg = *data;
          nmemcpy(BSN_RadioRangeM$forwardmsg.data, data->data, 66);
        }
#line 291
        __nesc_atomic_end(__nesc_atomic); }
      TOS_post(BSN_RadioRangeM$ForwardRadio);
    }
  return data;
}

static inline  TOS_MsgPtr BSN_RadioRangeM$UARTReceive$receive(TOS_MsgPtr data)
#line 297
{
  BSN_RadioRangeM$UARTConnected = 1;
  return BSN_RadioRangeM$UartReceive(data);
}

# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
inline static  TOS_MsgPtr NoCRCPacket$Receive$receive(TOS_MsgPtr arg_0xa7dbeb8){
#line 75
  struct TOS_Msg *result;
#line 75

#line 75
  result = BSN_RadioRangeM$UARTReceive$receive(arg_0xa7dbeb8);
#line 75

#line 75
  return result;
#line 75
}
#line 75
static inline  
# 320 "C:/cygwin/opt/tinyos-1.x/tos/system/NoCRCPacket.nc"
void NoCRCPacket$receiveTask(void)
#line 320
{
  TOS_MsgPtr tmp;

#line 322
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 322
    {
      tmp = NoCRCPacket$bufferPtrs[NoCRCPacket$bufferIndex ^ 1];

      tmp->crc = 1;
    }
#line 326
    __nesc_atomic_end(__nesc_atomic); }
  tmp = NoCRCPacket$Receive$receive(tmp);
  if (tmp) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 329
        {
          NoCRCPacket$bufferPtrs[NoCRCPacket$bufferIndex ^ 1] = tmp;
        }
#line 331
        __nesc_atomic_end(__nesc_atomic); }
    }
}

static inline  
# 202 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
void FramerM$PacketUnknown(void)
#line 202
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 203
    {
      FramerM$gFlags |= FramerM$FLAGS_UNKNOWN;
    }
#line 205
    __nesc_atomic_end(__nesc_atomic); }

  FramerM$StartTx();
}

static inline  
# 246 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
TOS_MsgPtr AMStandard$UARTReceive$receive(TOS_MsgPtr packet)
#line 246
{


  packet->group = TOS_AM_GROUP;
  return received(packet);
}

# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
inline static  TOS_MsgPtr FramerAckM$ReceiveCombined$receive(TOS_MsgPtr arg_0xa7dbeb8){
#line 75
  struct TOS_Msg *result;
#line 75

#line 75
  result = AMStandard$UARTReceive$receive(arg_0xa7dbeb8);
#line 75

#line 75
  return result;
#line 75
}
#line 75
static inline  
# 91 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerAckM.nc"
TOS_MsgPtr FramerAckM$ReceiveMsg$receive(TOS_MsgPtr Msg)
#line 91
{
  TOS_MsgPtr pBuf;

  pBuf = FramerAckM$ReceiveCombined$receive(Msg);

  return pBuf;
}

# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
inline static  TOS_MsgPtr FramerM$ReceiveMsg$receive(TOS_MsgPtr arg_0xa7dbeb8){
#line 75
  struct TOS_Msg *result;
#line 75

#line 75
  result = FramerAckM$ReceiveMsg$receive(arg_0xa7dbeb8);
#line 75

#line 75
  return result;
#line 75
}
#line 75
static inline  
# 328 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
result_t FramerM$TokenReceiveMsg$ReflectToken(uint8_t Token)
#line 328
{
  result_t Result = SUCCESS;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 331
    {
      if (!(FramerM$gFlags & FramerM$FLAGS_TOKENPEND)) {
          FramerM$gFlags |= FramerM$FLAGS_TOKENPEND;
          FramerM$gTxTokenBuf = Token;
        }
      else {
          Result = FAIL;
        }
    }
#line 339
    __nesc_atomic_end(__nesc_atomic); }

  if (Result == SUCCESS) {
      Result = FramerM$StartTx();
    }

  return Result;
}

# 88 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/TokenReceiveMsg.nc"
inline static  result_t FramerAckM$TokenReceiveMsg$ReflectToken(uint8_t arg_0xaacc010){
#line 88
  unsigned char result;
#line 88

#line 88
  result = FramerM$TokenReceiveMsg$ReflectToken(arg_0xaacc010);
#line 88

#line 88
  return result;
#line 88
}
#line 88
static inline  
# 74 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerAckM.nc"
void FramerAckM$SendAckTask(void)
#line 74
{

  FramerAckM$TokenReceiveMsg$ReflectToken(FramerAckM$gTokenBuf);
}

static inline  TOS_MsgPtr FramerAckM$TokenReceiveMsg$receive(TOS_MsgPtr Msg, uint8_t token)
#line 79
{
  TOS_MsgPtr pBuf;

  FramerAckM$gTokenBuf = token;

  TOS_post(FramerAckM$SendAckTask);

  pBuf = FramerAckM$ReceiveCombined$receive(Msg);

  return pBuf;
}

# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/TokenReceiveMsg.nc"
inline static  TOS_MsgPtr FramerM$TokenReceiveMsg$receive(TOS_MsgPtr arg_0xaa89800, uint8_t arg_0xaa89948){
#line 75
  struct TOS_Msg *result;
#line 75

#line 75
  result = FramerAckM$TokenReceiveMsg$receive(arg_0xaa89800, arg_0xaa89948);
#line 75

#line 75
  return result;
#line 75
}
#line 75
static inline  
# 210 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
void FramerM$PacketRcvd(void)
#line 210
{
  FramerM$MsgRcvEntry_t *pRcv = &FramerM$gMsgRcvTbl[FramerM$gRxTailIndex];
  TOS_MsgPtr pBuf = pRcv->pMsg;


  if (pRcv->Length >= (size_t )& ((TOS_Msg *)0)->data) {

      switch (pRcv->Proto) {
          case FramerM$PROTO_ACK: 
            break;
          case FramerM$PROTO_PACKET_ACK: 
            pBuf->crc = 1;
          pBuf = FramerM$TokenReceiveMsg$receive(pBuf, pRcv->Token);
          break;
          case FramerM$PROTO_PACKET_NOACK: 
            pBuf->crc = 1;
          pBuf = FramerM$ReceiveMsg$receive(pBuf);
          break;
          default: 
            FramerM$gTxUnknownBuf = pRcv->Proto;
          TOS_post(FramerM$PacketUnknown);
          break;
        }
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 235
    {
      if (pBuf) {
          pRcv->pMsg = pBuf;
        }
      pRcv->Length = 0;
      pRcv->Token = 0;
      FramerM$gRxTailIndex++;
      FramerM$gRxTailIndex %= FramerM$HDLC_QUEUESIZE;
    }
#line 243
    __nesc_atomic_end(__nesc_atomic); }
}

# 96 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/HPLUART.nc"
inline static   result_t HPLUARTM$UART$putDone(void){
#line 96
  unsigned char result;
#line 96

#line 96
  result = UARTM$HPLUART$putDone();
#line 96

#line 96
  return result;
#line 96
}
#line 96
static inline   
# 94 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
result_t HPLUARTM$USARTData$txDone(void)
#line 94
{
  return HPLUARTM$UART$putDone();
}

# 46 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
inline static   result_t HPLUSART1M$USARTData$txDone(void){
#line 46
  unsigned char result;
#line 46

#line 46
  result = HPLUARTM$USARTData$txDone();
#line 46

#line 46
  return result;
#line 46
}
#line 46
# 60 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
void __attribute((interrupt(4))) __attribute((wakeup))  sig_UART1TX_VECTOR(void)
#line 60
{
  HPLUSART1M$USARTData$txDone();
}

static inline   
# 552 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
result_t FramerM$ByteComm$txDone(void)
#line 552
{

  if (FramerM$gTxState == FramerM$TXSTATE_FINISH) {
      FramerM$gTxState = FramerM$TXSTATE_IDLE;
      TOS_post(FramerM$PacketSent);
    }

  return SUCCESS;
}

static inline   
# 309 "C:/cygwin/opt/tinyos-1.x/tos/system/NoCRCPacket.nc"
result_t NoCRCPacket$ByteComm$txDone(void)
#line 309
{
  bool complete;

#line 311
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 311
    {
      complete = NoCRCPacket$txCount == NoCRCPacket$txLength;
    }
#line 313
    __nesc_atomic_end(__nesc_atomic); }
  if (complete) {
    NoCRCPacket$sendComplete(TRUE);
    }
#line 316
  return SUCCESS;
}

# 83 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ByteComm.nc"
inline static   result_t UARTM$ByteComm$txDone(void){
#line 83
  unsigned char result;
#line 83

#line 83
  result = NoCRCPacket$ByteComm$txDone();
#line 83
  result = rcombine(result, FramerM$ByteComm$txDone());
#line 83

#line 83
  return result;
#line 83
}
#line 83
static inline  
# 302 "BSN_RadioRangeM.nc"
result_t BSN_RadioRangeM$UARTSend$sendDone(TOS_MsgPtr data, result_t success)
#line 302
{
  return SUCCESS;
}

# 67 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
inline static  result_t NoCRCPacket$Send$sendDone(TOS_MsgPtr arg_0xa7b3e40, result_t arg_0xa7d6010){
#line 67
  unsigned char result;
#line 67

#line 67
  result = BSN_RadioRangeM$UARTSend$sendDone(arg_0xa7b3e40, arg_0xa7d6010);
#line 67

#line 67
  return result;
#line 67
}
#line 67
static inline  
# 200 "C:/cygwin/opt/tinyos-1.x/tos/system/NoCRCPacket.nc"
void NoCRCPacket$sendDoneSuccessTask(void)
#line 200
{
  TOS_MsgPtr msg;

#line 202
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 202
    {
      NoCRCPacket$txCount = 0;
      NoCRCPacket$state = NoCRCPacket$IDLE;
      msg = (TOS_MsgPtr )NoCRCPacket$sendPtr;
    }
#line 206
    __nesc_atomic_end(__nesc_atomic); }
  NoCRCPacket$Send$sendDone(msg, SUCCESS);
}

static inline  
#line 190
void NoCRCPacket$sendDoneFailTask(void)
#line 190
{
  TOS_MsgPtr msg;

#line 192
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 192
    {
      NoCRCPacket$txCount = 0;
      NoCRCPacket$state = NoCRCPacket$IDLE;
      msg = (TOS_MsgPtr )NoCRCPacket$sendPtr;
    }
#line 196
    __nesc_atomic_end(__nesc_atomic); }
  NoCRCPacket$Send$sendDone(msg, FAIL);
}

static inline   
#line 271
result_t NoCRCPacket$SendVarLenPacket$default$sendDone(uint8_t *packet, result_t success)
#line 271
{
  return success;
}

# 62 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SendVarLenPacket.nc"
inline static  result_t NoCRCPacket$SendVarLenPacket$sendDone(uint8_t *arg_0xab4f758, result_t arg_0xab4f8a8){
#line 62
  unsigned char result;
#line 62

#line 62
  result = NoCRCPacket$SendVarLenPacket$default$sendDone(arg_0xab4f758, arg_0xab4f8a8);
#line 62

#line 62
  return result;
#line 62
}
#line 62
static inline  
# 220 "C:/cygwin/opt/tinyos-1.x/tos/system/NoCRCPacket.nc"
void NoCRCPacket$sendVarLenSuccessTask(void)
#line 220
{
  uint8_t *buf;

#line 222
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 222
    {
      NoCRCPacket$txCount = 0;
      NoCRCPacket$state = NoCRCPacket$IDLE;
      buf = NoCRCPacket$sendPtr;
    }
#line 226
    __nesc_atomic_end(__nesc_atomic); }
  NoCRCPacket$SendVarLenPacket$sendDone(buf, SUCCESS);
}

static inline  
#line 210
void NoCRCPacket$sendVarLenFailTask(void)
#line 210
{
  uint8_t *buf;

#line 212
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 212
    {
      NoCRCPacket$txCount = 0;
      NoCRCPacket$state = NoCRCPacket$IDLE;
      buf = NoCRCPacket$sendPtr;
    }
#line 216
    __nesc_atomic_end(__nesc_atomic); }
  NoCRCPacket$SendVarLenPacket$sendDone(buf, FAIL);
}

# 55 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ByteComm.nc"
inline static   result_t FramerM$ByteComm$txByte(uint8_t arg_0xaacdaf0){
#line 55
  unsigned char result;
#line 55

#line 55
  result = UARTM$ByteComm$txByte(arg_0xaacdaf0);
#line 55

#line 55
  return result;
#line 55
}
#line 55
static inline   
# 482 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
result_t FramerM$ByteComm$txByteReady(bool LastByteSuccess)
#line 482
{
  result_t TxResult = SUCCESS;
  uint8_t nextByte;

  if (LastByteSuccess != TRUE) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 487
        FramerM$gTxState = FramerM$TXSTATE_ERROR;
#line 487
        __nesc_atomic_end(__nesc_atomic); }
      TOS_post(FramerM$PacketSent);
      return SUCCESS;
    }

  switch (FramerM$gTxState) {

      case FramerM$TXSTATE_PROTO: 
        FramerM$gTxState = FramerM$TXSTATE_INFO;
      FramerM$gTxRunningCRC = crcByte(FramerM$gTxRunningCRC, (uint8_t )(FramerM$gTxProto & 0x0FF));
      TxResult = FramerM$ByteComm$txByte((uint8_t )(FramerM$gTxProto & 0x0FF));
      break;

      case FramerM$TXSTATE_INFO: 
        nextByte = FramerM$gpTxBuf[FramerM$gTxByteCnt];

      FramerM$gTxRunningCRC = crcByte(FramerM$gTxRunningCRC, nextByte);
      FramerM$gTxByteCnt++;
      if (FramerM$gTxByteCnt >= FramerM$gTxLength) {
          FramerM$gTxState = FramerM$TXSTATE_FCS1;
        }

      TxResult = FramerM$TxArbitraryByte(nextByte);
      break;

      case FramerM$TXSTATE_ESC: 

        TxResult = FramerM$ByteComm$txByte(FramerM$gTxEscByte ^ 0x20);
      FramerM$gTxState = FramerM$gPrevTxState;
      break;

      case FramerM$TXSTATE_FCS1: 
        nextByte = (uint8_t )(FramerM$gTxRunningCRC & 0xff);
      FramerM$gTxState = FramerM$TXSTATE_FCS2;
      TxResult = FramerM$TxArbitraryByte(nextByte);
      break;

      case FramerM$TXSTATE_FCS2: 
        nextByte = (uint8_t )((FramerM$gTxRunningCRC >> 8) & 0xff);
      FramerM$gTxState = FramerM$TXSTATE_ENDFLAG;
      TxResult = FramerM$TxArbitraryByte(nextByte);
      break;

      case FramerM$TXSTATE_ENDFLAG: 
        FramerM$gTxState = FramerM$TXSTATE_FINISH;
      TxResult = FramerM$ByteComm$txByte(FramerM$HDLC_FLAG_BYTE);

      break;

      case FramerM$TXSTATE_FINISH: 
        case FramerM$TXSTATE_ERROR: 

          default: 
            break;
    }


  if (TxResult != SUCCESS) {
      FramerM$gTxState = FramerM$TXSTATE_ERROR;
      TOS_post(FramerM$PacketSent);
    }

  return SUCCESS;
}

static inline   
# 282 "C:/cygwin/opt/tinyos-1.x/tos/system/NoCRCPacket.nc"
result_t NoCRCPacket$ByteComm$txByteReady(bool success)
#line 282
{
  uint8_t txC;
  uint8_t txL;

#line 285
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 285
    {
      txC = NoCRCPacket$txCount;
      txL = NoCRCPacket$txLength;
    }
#line 288
    __nesc_atomic_end(__nesc_atomic); }
  if (txC > 0) {
      if (!success) {
          {
          }
#line 291
          ;
          NoCRCPacket$sendComplete(FAIL);
        }
      else {
#line 294
        if (txC < txL) {
            uint8_t byteToSend;

#line 296
            { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 296
              {
                byteToSend = NoCRCPacket$sendPtr[txC];
                NoCRCPacket$txCount++;
              }
#line 299
              __nesc_atomic_end(__nesc_atomic); }
            {
            }
#line 300
            ;

            if (!NoCRCPacket$ByteComm$txByte(byteToSend)) {
              NoCRCPacket$sendComplete(FAIL);
              }
          }
        }
    }
#line 306
  return SUCCESS;
}

# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ByteComm.nc"
inline static   result_t UARTM$ByteComm$txByteReady(bool arg_0xaac87e0){
#line 75
  unsigned char result;
#line 75

#line 75
  result = NoCRCPacket$ByteComm$txByteReady(arg_0xaac87e0);
#line 75
  result = rcombine(result, FramerM$ByteComm$txByteReady(arg_0xaac87e0));
#line 75

#line 75
  return result;
#line 75
}
#line 75
# 102 "C:/cygwin/opt/tinyos-1.x/tos/system/sched.c"
bool  TOS_post(void (*tp)(void))
#line 102
{
  __nesc_atomic_t fInterruptFlags;
  uint8_t tmp;



  fInterruptFlags = __nesc_atomic_start();

  tmp = TOSH_sched_free;

  if (TOSH_queue[tmp].tp == NULL) {
      TOSH_sched_free = (tmp + 1) & TOSH_TASK_BITMASK;
      TOSH_queue[tmp].tp = tp;
      __nesc_atomic_end(fInterruptFlags);

      return TRUE;
    }
  else {
      __nesc_atomic_end(fInterruptFlags);

      return FALSE;
    }
}

# 52 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MainM.nc"
int   main(void)
{
  MainM$hardwareInit();
  TOSH_sched_init();

  MainM$StdControl$init();
  MainM$StdControl$start();
  __nesc_enable_interrupt();

  for (; ; ) {
#line 61
      TOSH_run_task();
    }
}

static 
# 139 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
void MSP430ClockM$set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

static  
# 75 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
result_t TimerM$StdControl$init(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 77
    TimerM$m_hinow = 0;
#line 77
    __nesc_atomic_end(__nesc_atomic); }
  TimerM$m_head_short = TimerM$EMPTY_LIST;
  TimerM$m_head_long = TimerM$EMPTY_LIST;
  bzero(TimerM$m_timers, sizeof TimerM$m_timers);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 81
    TimerM$m_posted_checkShortTimers = FALSE;
#line 81
    __nesc_atomic_end(__nesc_atomic); }
  TimerM$AlarmControl$setControlAsCompare();
  TimerM$AlarmControl$disableEvents();
  return SUCCESS;
}

static 
# 268 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
void FramerM$HDLCInitialize(void)
#line 268
{
  int i;

#line 270
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 270
    {
      for (i = 0; i < FramerM$HDLC_QUEUESIZE; i++) {
          FramerM$gMsgRcvTbl[i].pMsg = &FramerM$gMsgRcvBuf[i];
          FramerM$gMsgRcvTbl[i].Length = 0;
          FramerM$gMsgRcvTbl[i].Token = 0;
        }
      FramerM$gTxState = FramerM$TXSTATE_IDLE;
      FramerM$gTxByteCnt = 0;
      FramerM$gTxLength = 0;
      FramerM$gTxRunningCRC = 0;
      FramerM$gpTxMsg = NULL;

      FramerM$gRxState = FramerM$RXSTATE_NOSYNC;
      FramerM$gRxHeadIndex = 0;
      FramerM$gRxTailIndex = 0;
      FramerM$gRxByteCnt = 0;
      FramerM$gRxRunningCRC = 0;
      FramerM$gpRxBuf = (uint8_t *)FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].pMsg;
    }
#line 288
    __nesc_atomic_end(__nesc_atomic); }
}

static   
# 225 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
void HPLUSART0M$USARTControl$setModeSPI(void)
#line 225
{

  if (HPLUSART0M$USARTControl$getMode() == USART_SPI) {
    return;
    }
  HPLUSART0M$USARTControl$disableUART();
  HPLUSART0M$USARTControl$disableI2C();

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 233
    {
      TOSH_SEL_SIMO0_MODFUNC();
      TOSH_SEL_SOMI0_MODFUNC();
      TOSH_SEL_UCLK0_MODFUNC();

      HPLUSART0M$IE1 &= ~((1 << 7) | (1 << 6));

      U0CTL = 0x01;
      U0CTL |= (0x10 | 0x04) | 0x02;
      U0CTL &= ~0x20;

      HPLUSART0M$U0TCTL = 0x02;
      HPLUSART0M$U0TCTL |= 0x80;

      if (HPLUSART0M$l_ssel & 0x80) {
          HPLUSART0M$U0TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
          HPLUSART0M$U0TCTL |= HPLUSART0M$l_ssel & 0x7F;
        }
      else {
          HPLUSART0M$U0TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
          HPLUSART0M$U0TCTL |= 0x20;
        }

      if (HPLUSART0M$l_br != 0) {
          U0BR0 = HPLUSART0M$l_br & 0x0FF;
          U0BR1 = (HPLUSART0M$l_br >> 8) & 0x0FF;
        }
      else {
          U0BR0 = 0x02;
          U0BR1 = 0x00;
        }
      U0MCTL = 0;

      HPLUSART0M$ME1 &= ~((1 << 7) | (1 << 6));
      HPLUSART0M$ME1 |= 1 << 6;
      U0CTL &= ~0x01;

      HPLUSART0M$IFG1 &= ~((1 << 7) | (1 << 6));
      HPLUSART0M$IE1 &= ~((1 << 7) | (1 << 6));
    }
#line 272
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

static   
# 50 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
result_t HPLUARTM$UART$init(void)
#line 50
{

  HPLUARTM$USARTControl$setModeUART();
#line 64
  HPLUARTM$USARTControl$setClockSource(0x20);
  HPLUARTM$USARTControl$setClockRate(UBR_SMCLK_57600, UMCTL_SMCLK_57600);
#line 77
  HPLUARTM$USARTControl$enableRxIntr();
  HPLUARTM$USARTControl$enableTxIntr();
  return SUCCESS;
}

static   
# 143 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420M.nc"
uint8_t HPLCC2420M$HPLCC2420$write(uint8_t addr, uint16_t data)
#line 143
{
  uint8_t status = 0;

#line 145
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {

      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(addr);
      while (!HPLCC2420M$USARTControl$isRxIntrPending()) ;
      status = HPLCC2420M$adjustStatusByte(HPLCC2420M$USARTControl$rx());
      HPLCC2420M$USARTControl$tx((data >> 8) & 0x0FF);
      while (!HPLCC2420M$USARTControl$isTxIntrPending()) ;
      HPLCC2420M$USARTControl$tx(data & 0x0FF);
      while (!HPLCC2420M$USARTControl$isTxEmpty()) ;
      TOSH_SET_RADIO_CSN_PIN();
#line 174
      HPLCC2420M$BusArbitration$releaseBus();
    }
  return status;
}

static   
# 94 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/BusArbitrationM.nc"
result_t BusArbitrationM$BusArbitration$getBus(uint8_t id)
#line 94
{
  bool gotbus = FALSE;

#line 96
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 96
    {
      if (BusArbitrationM$state == BusArbitrationM$BUS_IDLE) {
          BusArbitrationM$state = BusArbitrationM$BUS_BUSY;
          gotbus = TRUE;
          BusArbitrationM$busid = id;
        }
    }
#line 102
    __nesc_atomic_end(__nesc_atomic); }
  if (gotbus) {
    return SUCCESS;
    }
#line 105
  return FAIL;
}

static   result_t BusArbitrationM$BusArbitration$releaseBus(uint8_t id)
#line 108
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (BusArbitrationM$state == BusArbitrationM$BUS_BUSY && BusArbitrationM$busid == id) {
          BusArbitrationM$state = BusArbitrationM$BUS_IDLE;





          if (BusArbitrationM$isBusReleasedPending == FALSE && TOS_post(BusArbitrationM$busReleased) == TRUE) {
            BusArbitrationM$isBusReleasedPending = TRUE;
            }
        }
    }
#line 121
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

static   
# 110 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420M.nc"
uint8_t HPLCC2420M$HPLCC2420$cmd(uint8_t addr)
#line 110
{
  uint8_t status = 0;

#line 112
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {

      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(addr);
      while (!HPLCC2420M$USARTControl$isRxIntrPending()) ;
      status = HPLCC2420M$adjustStatusByte(HPLCC2420M$USARTControl$rx());
      TOSH_SET_RADIO_CSN_PIN();
#line 133
      HPLCC2420M$BusArbitration$releaseBus();
    }
  return status;
}

static  
# 440 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
result_t TimerM$Timer$start(uint8_t num, char type, uint32_t milli)
{
  switch (type) 
    {
      case TIMER_REPEAT: 
        return TimerM$setTimer(num, milli * 32, TRUE);

      case TIMER_ONE_SHOT: 
        return TimerM$setTimer(num, milli * 32, FALSE);
    }

  return FAIL;
}

static 
#line 314
result_t TimerM$setTimer(uint8_t num, int32_t jiffy, bool isperiodic)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      TimerM$Timer_t *timer = &TimerM$m_timers[num];
      int32_t now;

      if (timer->isset) {
        TimerM$removeTimer(num);
        }
      TimerM$m_period[num] = jiffy;
      timer->isperiodic = isperiodic;
      now = TimerM$LocalTime$read();
      timer->alarm = now + jiffy;
      TimerM$insertTimer(num, jiffy <= 0xffffL);
      TimerM$setNextShortEvent();
    }
#line 330
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

static   
#line 283
uint32_t TimerM$LocalTime$read(void)
{
  uint32_t now;

#line 286
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {



      uint16_t hinow = TimerM$m_hinow;
      uint16_t lonow = TimerM$AlarmTimer$read();

#line 293
      if (TimerM$AlarmTimer$isOverflowPending()) 
        {
          hinow++;
          lonow = TimerM$AlarmTimer$read();
        }
      now = ((uint32_t )TimerM$m_hinow << 16) | lonow;
    }
#line 299
    __nesc_atomic_end(__nesc_atomic); }
  return now;
}

static 
#line 97
void TimerM$insertTimer(uint8_t num, bool isshort)
{
  if (TimerM$m_timers[num].isqueued == FALSE) 
    {
      if (isshort) 
        {
          TimerM$m_timers[num].next = TimerM$m_head_short;
          TimerM$m_head_short = num;
        }
      else 
        {
          TimerM$m_timers[num].next = TimerM$m_head_long;
          TimerM$m_head_long = num;
        }

      TimerM$m_timers[num].isqueued = TRUE;
    }

  TimerM$m_timers[num].isset = TRUE;
}

static 
#line 204
void TimerM$setNextShortEvent(void)
{
  uint32_t now = TimerM$LocalTime$read();

  if (TimerM$m_head_short != TimerM$EMPTY_LIST) 
    {
      uint8_t head = TimerM$m_head_short;
      uint8_t soon = head;
      int32_t remaining = TimerM$m_timers[head].alarm - now;

#line 213
      head = TimerM$m_timers[head].next;

      while (head != TimerM$EMPTY_LIST) 
        {
          int32_t dt = TimerM$m_timers[head].alarm - now;

#line 218
          if (dt < remaining) 
            {
              remaining = dt;
              soon = head;
            }
          head = TimerM$m_timers[head].next;
        }

      now = TimerM$LocalTime$read();
      remaining = TimerM$m_timers[soon].alarm - now;

      if (remaining <= 0) 
        {

          TimerM$AlarmControl$disableEvents();
          TimerM$post_checkShortTimers();
        }
      else 
        {


          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
            {




              if (remaining > 2) {
                TimerM$AlarmCompare$setEventFromNow(remaining);
                }
              else {
#line 248
                TimerM$AlarmCompare$setEventFromNow(2);
                }
#line 249
              TimerM$AlarmControl$clearPendingInterrupt();
              TimerM$AlarmControl$enableEvents();
            }
#line 251
            __nesc_atomic_end(__nesc_atomic); }
        }
    }
  else 

    {

      TimerM$AlarmControl$disableEvents();
    }
}

static 
#line 146
void TimerM$executeTimers(uint8_t head)
{
  uint32_t now = TimerM$LocalTime$read();

#line 149
  while (head != TimerM$EMPTY_LIST) 
    {
      uint8_t num = head;
      bool signal_timer = FALSE;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          TimerM$Timer_t *timer = &TimerM$m_timers[num];

#line 157
          head = timer->next;

          timer->isqueued = FALSE;

          if (timer->isset) 
            {
              int32_t remaining = timer->alarm - now;

#line 164
              timer->isset = FALSE;
              if (remaining <= 0) 
                {


                  if (timer->isperiodic) 
                    {
                      timer->alarm += TimerM$m_period[num];
                      TimerM$insertTimer(num, (int32_t )(timer->alarm - now) <= 0xffffL);
                    }

                  signal_timer = TRUE;
                }
              else 
                {

                  TimerM$insertTimer(num, remaining <= 0xffffL);
                }
            }
        }
#line 183
        __nesc_atomic_end(__nesc_atomic); }

      if (signal_timer) {
        TimerM$signal_timer_fired(num);
        }
    }
}

static  
# 179 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
result_t AMStandard$SendMsg$send(uint8_t id, uint16_t addr, uint8_t length, TOS_MsgPtr data)
#line 179
{
  if (!AMStandard$state) {
      AMStandard$state = TRUE;
      if (length > DATA_LENGTH) {
          {
          }
#line 183
          ;
          AMStandard$state = FALSE;
          return FAIL;
        }
      if (!TOS_post(AMStandard$sendTask)) {
          {
          }
#line 188
          ;
          AMStandard$state = FALSE;
          return FAIL;
        }
      else {
          AMStandard$buffer = data;
          data->length = length;
          data->addr = addr;
          data->type = id;
          AMStandard$buffer->group = TOS_AM_GROUP;
          {
          }
#line 198
          ;
          AMStandard$dbgPacket(data);
        }
      return SUCCESS;
    }

  return FAIL;
}

static 
# 158 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
result_t FramerM$StartTx(void)
#line 158
{
  result_t Result = SUCCESS;
  bool fInitiate = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 162
    {
      if (FramerM$gTxState == FramerM$TXSTATE_IDLE) {
          if (FramerM$gFlags & FramerM$FLAGS_TOKENPEND) {
              FramerM$gpTxBuf = (uint8_t *)&FramerM$gTxTokenBuf;
              FramerM$gTxProto = FramerM$PROTO_ACK;
              FramerM$gTxLength = sizeof FramerM$gTxTokenBuf;
              fInitiate = TRUE;
              FramerM$gTxState = FramerM$TXSTATE_PROTO;
            }
          else {
#line 171
            if (FramerM$gFlags & FramerM$FLAGS_DATAPEND) {
                FramerM$gpTxBuf = (uint8_t *)FramerM$gpTxMsg;
                FramerM$gTxProto = FramerM$PROTO_PACKET_NOACK;
                FramerM$gTxLength = FramerM$gpTxMsg->length + (MSG_DATA_SIZE - DATA_LENGTH - 2);
                fInitiate = TRUE;
                FramerM$gTxState = FramerM$TXSTATE_PROTO;
              }
            else {
#line 178
              if (FramerM$gFlags & FramerM$FLAGS_UNKNOWN) {
                  FramerM$gpTxBuf = (uint8_t *)&FramerM$gTxUnknownBuf;
                  FramerM$gTxProto = FramerM$PROTO_UNKNOWN;
                  FramerM$gTxLength = sizeof FramerM$gTxUnknownBuf;
                  fInitiate = TRUE;
                  FramerM$gTxState = FramerM$TXSTATE_PROTO;
                }
              }
            }
        }
    }
#line 188
    __nesc_atomic_end(__nesc_atomic); }
#line 188
  if (fInitiate) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 189
        {
          FramerM$gTxRunningCRC = 0;
#line 190
          FramerM$gTxByteCnt = 0;
        }
#line 191
        __nesc_atomic_end(__nesc_atomic); }
      Result = FramerM$ByteComm$txByte(FramerM$HDLC_FLAG_BYTE);
      if (Result != SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 194
            FramerM$gTxState = FramerM$TXSTATE_ERROR;
#line 194
            __nesc_atomic_end(__nesc_atomic); }
          TOS_post(FramerM$PacketSent);
        }
    }

  return Result;
}

static   
# 110 "C:/cygwin/opt/tinyos-1.x/tos/system/UARTM.nc"
result_t UARTM$ByteComm$txByte(uint8_t data)
#line 110
{
  bool oldState;

  {
  }
#line 113
  ;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 115
    {
      oldState = UARTM$state;
      UARTM$state = TRUE;
    }
#line 118
    __nesc_atomic_end(__nesc_atomic); }
  if (oldState) {
    return FAIL;
    }
  UARTM$HPLUART$put(data);

  return SUCCESS;
}

static  
# 246 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
void FramerM$PacketSent(void)
#line 246
{
  result_t TxResult = SUCCESS;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 249
    {
      if (FramerM$gTxState == FramerM$TXSTATE_ERROR) {
          TxResult = FAIL;
          FramerM$gTxState = FramerM$TXSTATE_IDLE;
        }
    }
#line 254
    __nesc_atomic_end(__nesc_atomic); }
  if (FramerM$gTxProto == FramerM$PROTO_ACK) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 256
        FramerM$gFlags ^= FramerM$FLAGS_TOKENPEND;
#line 256
        __nesc_atomic_end(__nesc_atomic); }
    }
  else {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 259
        FramerM$gFlags ^= FramerM$FLAGS_DATAPEND;
#line 259
        __nesc_atomic_end(__nesc_atomic); }
      FramerM$BareSendMsg$sendDone((TOS_MsgPtr )FramerM$gpTxMsg, TxResult);
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 261
        FramerM$gpTxMsg = NULL;
#line 261
        __nesc_atomic_end(__nesc_atomic); }
    }


  FramerM$StartTx();
}

static 
# 143 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
result_t AMStandard$reportSendDone(TOS_MsgPtr msg, result_t success)
#line 143
{
  AMStandard$state = FALSE;
  AMStandard$SendMsg$sendDone(msg->type, msg, success);
  AMStandard$sendDone();

  return SUCCESS;
}

static   
# 70 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/TimerJiffyAsyncM.nc"
result_t TimerJiffyAsyncM$TimerJiffyAsync$setOneShot(uint32_t _jiffy)
{
  TimerJiffyAsyncM$AlarmControl$disableEvents();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      TimerJiffyAsyncM$jiffy = _jiffy;
      TimerJiffyAsyncM$bSet = TRUE;
    }
#line 76
    __nesc_atomic_end(__nesc_atomic); }
  if (_jiffy > 0xFFFF) {
      TimerJiffyAsyncM$AlarmCompare$setEventFromNow(0xFFFF);
    }
  else {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 81
        {




          if (_jiffy > 2) {
            TimerJiffyAsyncM$AlarmCompare$setEventFromNow(_jiffy);
            }
          else {
#line 89
            TimerJiffyAsyncM$AlarmCompare$setEventFromNow(2);
            }
        }
#line 91
        __nesc_atomic_end(__nesc_atomic); }
    }
#line 92
  TimerJiffyAsyncM$AlarmControl$clearPendingInterrupt();
  TimerJiffyAsyncM$AlarmControl$enableEvents();
  return SUCCESS;
}

static   
# 70 "C:/cygwin/opt/tinyos-1.x/tos/system/RandomLFSR.nc"
uint16_t RandomLFSR$Random$rand(void)
#line 70
{
  bool endbit;
  uint16_t tmpShiftReg;

#line 73
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      tmpShiftReg = RandomLFSR$shiftReg;
      endbit = (tmpShiftReg & 0x8000) != 0;
      tmpShiftReg <<= 1;
      if (endbit) {
        tmpShiftReg ^= 0x100b;
        }
#line 79
      tmpShiftReg++;
      RandomLFSR$shiftReg = tmpShiftReg;
      tmpShiftReg = tmpShiftReg ^ RandomLFSR$mask;
    }
#line 82
    __nesc_atomic_end(__nesc_atomic); }
  return tmpShiftReg;
}

static  
# 149 "C:/cygwin/opt/tinyos-1.x/tos/system/NoCRCPacket.nc"
result_t NoCRCPacket$Send$send(TOS_MsgPtr msg)
#line 149
{
  uint8_t oldState;
  uint8_t *packet;
  uint8_t sendNum;
  result_t rval = FAIL;

#line 154
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 154
    {
      oldState = NoCRCPacket$state;
      if (NoCRCPacket$state == NoCRCPacket$IDLE) {
          NoCRCPacket$state = NoCRCPacket$PACKET;
        }
      packet = (uint8_t *)msg;
      sendNum = TOS_MsgLength(msg->type);
    }
#line 161
    __nesc_atomic_end(__nesc_atomic); }
  if (oldState == NoCRCPacket$IDLE) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 163
        {
          msg->crc = 1;
          rval = NoCRCPacket$txBytes(packet, sendNum);
        }
#line 166
        __nesc_atomic_end(__nesc_atomic); }
    }
  return rval;
}

# 131 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void __attribute((interrupt(10))) __attribute((wakeup))  sig_TIMERA1_VECTOR(void)
{
  int n = TAIV;

#line 134
  switch (n) 
    {
      case 0: break;
      case 2: 
        if (MSP430TimerM$ControlA1$getControl().cap) {
          MSP430TimerM$CaptureA1$captured(MSP430TimerM$CaptureA1$getEvent());
          }
        else {
#line 141
          MSP430TimerM$CompareA1$fired();
          }
#line 142
      break;
      case 4: 
        if (MSP430TimerM$ControlA2$getControl().cap) {
          MSP430TimerM$CaptureA2$captured(MSP430TimerM$CaptureA2$getEvent());
          }
        else {
#line 147
          MSP430TimerM$CompareA2$fired();
          }
#line 148
      break;
      case 6: break;
      case 8: break;
      case 10: MSP430TimerM$TimerA$overflow();
#line 151
      break;
      case 12: break;
      case 14: break;
    }
}

#line 285
void __attribute((interrupt(24))) __attribute((wakeup))  sig_TIMERB1_VECTOR(void)
{
  int n = TBIV;

#line 288
  switch (n) 
    {
      case 0: break;
      case 2: 
        if (MSP430TimerM$ControlB1$getControl().cap) {
          MSP430TimerM$CaptureB1$captured(MSP430TimerM$CaptureB1$getEvent());
          }
        else {
#line 295
          MSP430TimerM$CompareB1$fired();
          }
#line 296
      break;
      case 4: 
        if (MSP430TimerM$ControlB2$getControl().cap) {
          MSP430TimerM$CaptureB2$captured(MSP430TimerM$CaptureB2$getEvent());
          }
        else {
#line 301
          MSP430TimerM$CompareB2$fired();
          }
#line 302
      break;
      case 6: 
        if (MSP430TimerM$ControlB3$getControl().cap) {
          MSP430TimerM$CaptureB3$captured(MSP430TimerM$CaptureB3$getEvent());
          }
        else {
#line 307
          MSP430TimerM$CompareB3$fired();
          }
#line 308
      break;
      case 8: 
        if (MSP430TimerM$ControlB4$getControl().cap) {
          MSP430TimerM$CaptureB4$captured(MSP430TimerM$CaptureB4$getEvent());
          }
        else {
#line 313
          MSP430TimerM$CompareB4$fired();
          }
#line 314
      break;
      case 10: 
        if (MSP430TimerM$ControlB5$getControl().cap) {
          MSP430TimerM$CaptureB5$captured(MSP430TimerM$CaptureB5$getEvent());
          }
        else {
#line 319
          MSP430TimerM$CompareB5$fired();
          }
#line 320
      break;
      case 12: 
        if (MSP430TimerM$ControlB6$getControl().cap) {
          MSP430TimerM$CaptureB6$captured(MSP430TimerM$CaptureB6$getEvent());
          }
        else {
#line 325
          MSP430TimerM$CompareB6$fired();
          }
#line 326
      break;
      case 14: MSP430TimerM$TimerB$overflow();
#line 327
      break;
    }
}

static   
# 185 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420InterruptM.nc"
result_t HPLCC2420InterruptM$SFD$enableCapture(bool low_to_high)
#line 185
{
  uint8_t _direction;

#line 187
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 187
    {
      TOSH_SEL_CC_SFD_MODFUNC();
      HPLCC2420InterruptM$SFDControl$disableEvents();
      if (low_to_high) {
#line 190
        _direction = MSP430TIMER_CM_RISING;
        }
      else {
#line 191
        _direction = MSP430TIMER_CM_FALLING;
        }
#line 192
      HPLCC2420InterruptM$SFDControl$setControlAsCapture(_direction);
      HPLCC2420InterruptM$SFDCapture$clearOverflow();
      HPLCC2420InterruptM$SFDControl$clearPendingInterrupt();
      HPLCC2420InterruptM$SFDControl$enableEvents();
    }
#line 196
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

static 
# 113 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
void CC2420RadioM$sendFailed(void)
#line 113
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 114
    CC2420RadioM$stateRadio = CC2420RadioM$IDLE_STATE;
#line 114
    __nesc_atomic_end(__nesc_atomic); }
  CC2420RadioM$txbufptr->length = CC2420RadioM$txbufptr->length - MSG_HEADER_SIZE - MSG_FOOTER_SIZE;
  CC2420RadioM$Send$sendDone(CC2420RadioM$txbufptr, FAIL);
}

static  
#line 168
void CC2420RadioM$PacketSent(void)
#line 168
{
  TOS_MsgPtr pBuf;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 171
    {
      CC2420RadioM$stateRadio = CC2420RadioM$IDLE_STATE;
      pBuf = CC2420RadioM$txbufptr;
      pBuf->length = pBuf->length - MSG_HEADER_SIZE - MSG_FOOTER_SIZE;
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }

  CC2420RadioM$Send$sendDone(pBuf, SUCCESS);
}

static  
#line 393
void CC2420RadioM$startSend(void)
#line 393
{

  if (!CC2420RadioM$HPLChipcon$cmd(0x09)) {
      CC2420RadioM$sendFailed();
      return;
    }

  if (!CC2420RadioM$HPLChipconFIFO$writeTXFIFO(CC2420RadioM$txlength + 1, (uint8_t *)CC2420RadioM$txbufptr)) {
      CC2420RadioM$sendFailed();
      return;
    }
}

static 



void CC2420RadioM$tryToSend(void)
#line 410
{
  uint8_t currentstate;

#line 412
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 412
    currentstate = CC2420RadioM$stateRadio;
#line 412
    __nesc_atomic_end(__nesc_atomic); }


  if (currentstate == CC2420RadioM$PRE_TX_STATE) {



      if (!TOSH_READ_CC_FIFO_PIN() && !TOSH_READ_CC_FIFOP_PIN()) {
          CC2420RadioM$flushRXFIFO();
        }

      if (TOSH_READ_RADIO_CCA_PIN()) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 424
            CC2420RadioM$stateRadio = CC2420RadioM$TX_STATE;
#line 424
            __nesc_atomic_end(__nesc_atomic); }
          CC2420RadioM$sendPacket();
        }
      else {



          if (CC2420RadioM$countRetry-- <= 0) {
              CC2420RadioM$flushRXFIFO();
              CC2420RadioM$countRetry = 8;
              if (!TOS_post(CC2420RadioM$startSend)) {
                CC2420RadioM$sendFailed();
                }
#line 436
              return;
            }
          if (!CC2420RadioM$setBackoffTimer(CC2420RadioM$MacBackoff$congestionBackoff(CC2420RadioM$txbufptr) * 10)) {
              CC2420RadioM$sendFailed();
            }
        }
    }
}

static 
#line 119
void CC2420RadioM$flushRXFIFO(void)
#line 119
{
  CC2420RadioM$FIFOP$disable();
  CC2420RadioM$HPLChipcon$read(0x3F);
  CC2420RadioM$HPLChipcon$cmd(0x08);
  CC2420RadioM$HPLChipcon$cmd(0x08);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 124
    CC2420RadioM$bPacketReceiving = FALSE;
#line 124
    __nesc_atomic_end(__nesc_atomic); }
  CC2420RadioM$FIFOP$startWait(FALSE);
}

static   
# 184 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420M.nc"
uint16_t HPLCC2420M$HPLCC2420$read(uint8_t addr)
#line 184
{
  uint16_t data = 0;

#line 186
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {

      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(addr | 0x40);
      while (!HPLCC2420M$USARTControl$isRxIntrPending()) ;
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(0);
      while (!HPLCC2420M$USARTControl$isRxIntrPending()) ;
      data = (HPLCC2420M$USARTControl$rx() << 8) & 0xFF00;
      HPLCC2420M$USARTControl$tx(0);
      while (!HPLCC2420M$USARTControl$isRxIntrPending()) ;
      data = data | (HPLCC2420M$USARTControl$rx() & 0x0FF);
      TOSH_SET_RADIO_CSN_PIN();
#line 219
      HPLCC2420M$BusArbitration$releaseBus();
    }
  return data;
}

static   
# 65 "C:/cygwin/opt/tinyos-1.x/tos/platform/bsn/HPLCC2420InterruptM.nc"
result_t HPLCC2420InterruptM$FIFOP$startWait(bool low_to_high)
#line 65
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 66
    {
      HPLCC2420InterruptM$FIFOPInterrupt$disable();
      HPLCC2420InterruptM$FIFOPInterrupt$clear();
      HPLCC2420InterruptM$FIFOPInterrupt$edge(low_to_high);
      HPLCC2420InterruptM$FIFOPInterrupt$enable();
    }
#line 71
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 215 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
TOS_MsgPtr   received(TOS_MsgPtr packet)
#line 215
{
  uint16_t addr = TOS_LOCAL_ADDRESS;

#line 217
  AMStandard$counter++;
  {
  }
#line 218
  ;


  if (
#line 220
  packet->crc == 1 && 
  packet->group == TOS_AM_GROUP && (
  packet->addr == TOS_BCAST_ADDR || 
  packet->addr == addr)) 
    {

      uint8_t type = packet->type;
      TOS_MsgPtr tmp;

      {
      }
#line 229
      ;
      AMStandard$dbgPacket(packet);
      {
      }
#line 231
      ;


      tmp = AMStandard$ReceiveMsg$receive(type, packet);
      if (tmp) {
        packet = tmp;
        }
    }
#line 238
  return packet;
}

static  
# 154 "BSN_RadioRangeM.nc"
void BSN_RadioRangeM$ReplyHeartBeat(void)
{
  struct UbiMonMsg *pack;

#line 157
  pack = (struct UbiMonMsg *)BSN_RadioRangeM$replymsg.data;
  pack->sourceID = TOS_LOCAL_ADDRESS;
  pack->destination = TOS_BCAST_ADDR;
  pack->data[0] = 0;
  pack->BSNcommand = 0x81;
  if (!BSN_RadioRangeM$heartbeatfromUART) 
    {
      BSN_RadioRangeM$ToRadio(TOS_BCAST_ADDR, &BSN_RadioRangeM$replymsg);
    }
  else {
      BSN_RadioRangeM$replymsg.addr = TOS_UART_ADDR;
      BSN_RadioRangeM$UARTSend$send(&BSN_RadioRangeM$replymsg);
    }
}

static 
# 149 "C:/cygwin/opt/tinyos-1.x/tos/system/tos.h"
void *nmemcpy(void *to, const void *from, size_t n)
{
  char *cto = to;
  const char *cfrom = from;

  while (n--) * cto++ = * cfrom++;

  return to;
}

# 56 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void __attribute((interrupt(8))) __attribute((wakeup))  sig_PORT1_VECTOR(void)
{
  volatile int n = MSP430InterruptM$P1IFG & MSP430InterruptM$P1IE;

  if (n & (1 << 0)) {
#line 60
      MSP430InterruptM$Port10$fired();
#line 60
      return;
    }
#line 61
  if (n & (1 << 1)) {
#line 61
      MSP430InterruptM$Port11$fired();
#line 61
      return;
    }
#line 62
  if (n & (1 << 2)) {
#line 62
      MSP430InterruptM$Port12$fired();
#line 62
      return;
    }
#line 63
  if (n & (1 << 3)) {
#line 63
      MSP430InterruptM$Port13$fired();
#line 63
      return;
    }
#line 64
  if (n & (1 << 4)) {
#line 64
      MSP430InterruptM$Port14$fired();
#line 64
      return;
    }
#line 65
  if (n & (1 << 5)) {
#line 65
      MSP430InterruptM$Port15$fired();
#line 65
      return;
    }
#line 66
  if (n & (1 << 6)) {
#line 66
      MSP430InterruptM$Port16$fired();
#line 66
      return;
    }
#line 67
  if (n & (1 << 7)) {
#line 67
      MSP430InterruptM$Port17$fired();
#line 67
      return;
    }
}

static 
# 540 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
void CC2420RadioM$delayedRXFIFO(void)
#line 540
{
  uint8_t len = MSG_DATA_SIZE;
  uint8_t _bPacketReceiving;

  if (!TOSH_READ_CC_FIFO_PIN() && !TOSH_READ_CC_FIFOP_PIN()) {
      CC2420RadioM$flushRXFIFO();
      return;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 549
    {
      _bPacketReceiving = CC2420RadioM$bPacketReceiving;

      if (_bPacketReceiving) {
          if (!TOS_post(CC2420RadioM$delayedRXFIFOtask)) {
            CC2420RadioM$flushRXFIFO();
            }
        }
      else 
#line 555
        {
          CC2420RadioM$bPacketReceiving = TRUE;
        }
    }
#line 558
    __nesc_atomic_end(__nesc_atomic); }





  if (!_bPacketReceiving) {
      if (!CC2420RadioM$HPLChipconFIFO$readRXFIFO(len, (uint8_t *)CC2420RadioM$rxbufptr)) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 566
            CC2420RadioM$bPacketReceiving = FALSE;
#line 566
            __nesc_atomic_end(__nesc_atomic); }
          if (!TOS_post(CC2420RadioM$delayedRXFIFOtask)) {
              CC2420RadioM$flushRXFIFO();
            }
          return;
        }
    }
  CC2420RadioM$flushRXFIFO();
}

# 71 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void __attribute((interrupt(2))) __attribute((wakeup))  sig_PORT2_VECTOR(void)
{
  volatile int n = MSP430InterruptM$P2IFG & MSP430InterruptM$P2IE;

  if (n & (1 << 0)) {
#line 75
      MSP430InterruptM$Port20$fired();
#line 75
      return;
    }
#line 76
  if (n & (1 << 1)) {
#line 76
      MSP430InterruptM$Port21$fired();
#line 76
      return;
    }
#line 77
  if (n & (1 << 2)) {
#line 77
      MSP430InterruptM$Port22$fired();
#line 77
      return;
    }
#line 78
  if (n & (1 << 3)) {
#line 78
      MSP430InterruptM$Port23$fired();
#line 78
      return;
    }
#line 79
  if (n & (1 << 4)) {
#line 79
      MSP430InterruptM$Port24$fired();
#line 79
      return;
    }
#line 80
  if (n & (1 << 5)) {
#line 80
      MSP430InterruptM$Port25$fired();
#line 80
      return;
    }
#line 81
  if (n & (1 << 6)) {
#line 81
      MSP430InterruptM$Port26$fired();
#line 81
      return;
    }
#line 82
  if (n & (1 << 7)) {
#line 82
      MSP430InterruptM$Port27$fired();
#line 82
      return;
    }
}

#line 85
void __attribute((interrupt(28))) __attribute((wakeup))  sig_NMI_VECTOR(void)
{
  volatile int n = IFG1;

#line 88
  if (n & (1 << 4)) {
#line 88
      MSP430InterruptM$NMI$fired();
#line 88
      return;
    }
#line 89
  if (n & (1 << 1)) {
#line 89
      MSP430InterruptM$OF$fired();
#line 89
      return;
    }
#line 90
  if (FCTL3 & 0x0004) {
#line 90
      MSP430InterruptM$ACCV$fired();
#line 90
      return;
    }
}

static   
# 337 "C:/cygwin/opt/tinyos-1.x/tos/system/NoCRCPacket.nc"
result_t NoCRCPacket$ByteComm$rxByteReady(uint8_t data, bool error, 
uint16_t strength)
#line 338
{
  bool rxDone;

  {
  }
#line 341
  ;
  if (error) 
    {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 344
        {
          NoCRCPacket$rxCount = 0;
        }
#line 346
        __nesc_atomic_end(__nesc_atomic); }
      return FAIL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 349
    {
      if (NoCRCPacket$rxCount == 0) {
        ((TOS_MsgPtr )NoCRCPacket$recPtr)->strength = strength;
        }
      if (NoCRCPacket$rxCount == (size_t )& ((TOS_Msg *)0)->type) {
        NoCRCPacket$rxLength = TOS_MsgLength(data);
        }
      NoCRCPacket$recPtr[NoCRCPacket$rxCount++] = data;

      rxDone = NoCRCPacket$rxCount == NoCRCPacket$rxLength;
    }
#line 359
    __nesc_atomic_end(__nesc_atomic); }

  if (rxDone) 
    {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 363
        {
          NoCRCPacket$bufferIndex = NoCRCPacket$bufferIndex ^ 1;
          NoCRCPacket$recPtr = (uint8_t *)NoCRCPacket$bufferPtrs[NoCRCPacket$bufferIndex];

          {
          }
#line 367
          ;
          NoCRCPacket$rxCount = 0;
        }
#line 369
        __nesc_atomic_end(__nesc_atomic); }
      TOS_post(NoCRCPacket$receiveTask);
      return FAIL;
    }

  return SUCCESS;
}

static  
# 139 "BSN_RadioRangeM.nc"
void BSN_RadioRangeM$ForwardRadio(void)
{
  struct UbiMonMsg *pack;

#line 142
  pack = (struct UbiMonMsg *)BSN_RadioRangeM$forwardmsg.data;

  BSN_RadioRangeM$ToRadio(BSN_RadioRangeM$forwardmsg.addr, &BSN_RadioRangeM$forwardmsg);
}

static   
# 348 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
result_t FramerM$ByteComm$rxByteReady(uint8_t data, bool error, uint16_t strength)
#line 348
{

  switch (FramerM$gRxState) {

      case FramerM$RXSTATE_NOSYNC: 
        if (data == FramerM$HDLC_FLAG_BYTE && FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Length == 0) {
            FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Token = 0;
            FramerM$gRxByteCnt = FramerM$gRxRunningCRC = 0;
            FramerM$gpRxBuf = (uint8_t *)FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].pMsg;
            FramerM$gRxState = FramerM$RXSTATE_PROTO;
          }
      break;

      case FramerM$RXSTATE_PROTO: 
        if (data == FramerM$HDLC_FLAG_BYTE) {
            break;
          }
      FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Proto = data;
      FramerM$gRxRunningCRC = crcByte(FramerM$gRxRunningCRC, data);
      switch (data) {
          case FramerM$PROTO_PACKET_ACK: 
            FramerM$gRxState = FramerM$RXSTATE_TOKEN;
          break;
          case FramerM$PROTO_PACKET_NOACK: 
            FramerM$gRxState = FramerM$RXSTATE_INFO;
          break;
          default: 
            FramerM$gRxState = FramerM$RXSTATE_NOSYNC;
          break;
        }
      break;

      case FramerM$RXSTATE_TOKEN: 
        if (data == FramerM$HDLC_FLAG_BYTE) {
            FramerM$gRxState = FramerM$RXSTATE_NOSYNC;
          }
        else {
#line 384
          if (data == FramerM$HDLC_CTLESC_BYTE) {
              FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Token = 0x20;
            }
          else {
              FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Token ^= data;
              FramerM$gRxRunningCRC = crcByte(FramerM$gRxRunningCRC, FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Token);
              FramerM$gRxState = FramerM$RXSTATE_INFO;
            }
          }
#line 392
      break;


      case FramerM$RXSTATE_INFO: 
        if (FramerM$gRxByteCnt > FramerM$HDLC_MTU) {
            FramerM$gRxByteCnt = FramerM$gRxRunningCRC = 0;
            FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Length = 0;
            FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Token = 0;
            FramerM$gRxState = FramerM$RXSTATE_NOSYNC;
          }
        else {
#line 402
          if (data == FramerM$HDLC_CTLESC_BYTE) {
              FramerM$gRxState = FramerM$RXSTATE_ESC;
            }
          else {
#line 405
            if (data == FramerM$HDLC_FLAG_BYTE) {
                if (FramerM$gRxByteCnt >= 2) {
                    uint16_t usRcvdCRC = FramerM$gpRxBuf[FramerM$gRxByteCnt - 1] & 0xff;

#line 408
                    usRcvdCRC = (usRcvdCRC << 8) | (FramerM$gpRxBuf[FramerM$gRxByteCnt - 2] & 0xff);
                    if (usRcvdCRC == FramerM$gRxRunningCRC) {
                        FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Length = FramerM$gRxByteCnt - 2;
                        TOS_post(FramerM$PacketRcvd);
                        FramerM$gRxHeadIndex++;
#line 412
                        FramerM$gRxHeadIndex %= FramerM$HDLC_QUEUESIZE;
                      }
                    else {
                        FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Length = 0;
                        FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Token = 0;
                      }
                    if (FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Length == 0) {
                        FramerM$gpRxBuf = (uint8_t *)FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].pMsg;
                        FramerM$gRxState = FramerM$RXSTATE_PROTO;
                      }
                    else {
                        FramerM$gRxState = FramerM$RXSTATE_NOSYNC;
                      }
                  }
                else {
                    FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Length = 0;
                    FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Token = 0;
                    FramerM$gRxState = FramerM$RXSTATE_NOSYNC;
                  }
                FramerM$gRxByteCnt = FramerM$gRxRunningCRC = 0;
              }
            else {
                FramerM$gpRxBuf[FramerM$gRxByteCnt] = data;
                if (FramerM$gRxByteCnt >= 2) {
                    FramerM$gRxRunningCRC = crcByte(FramerM$gRxRunningCRC, FramerM$gpRxBuf[FramerM$gRxByteCnt - 2]);
                  }
                FramerM$gRxByteCnt++;
              }
            }
          }
#line 440
      break;

      case FramerM$RXSTATE_ESC: 
        if (data == FramerM$HDLC_FLAG_BYTE) {

            FramerM$gRxByteCnt = FramerM$gRxRunningCRC = 0;
            FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Length = 0;
            FramerM$gMsgRcvTbl[FramerM$gRxHeadIndex].Token = 0;
            FramerM$gRxState = FramerM$RXSTATE_NOSYNC;
          }
        else {
            data = data ^ 0x20;
            FramerM$gpRxBuf[FramerM$gRxByteCnt] = data;
            if (FramerM$gRxByteCnt >= 2) {
                FramerM$gRxRunningCRC = crcByte(FramerM$gRxRunningCRC, FramerM$gpRxBuf[FramerM$gRxByteCnt - 2]);
              }
            FramerM$gRxByteCnt++;
            FramerM$gRxState = FramerM$RXSTATE_INFO;
          }
      break;

      default: 
        FramerM$gRxState = FramerM$RXSTATE_NOSYNC;
      break;
    }

  return SUCCESS;
}

static 
# 66 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/crc.h"
uint16_t crcByte(uint16_t fcs, uint8_t c)
{
  fcs = ccitt_crc16_table[((fcs >> 8) ^ c) & 0xffU] ^ (fcs << 8);
  return fcs;
}

static   
# 87 "C:/cygwin/opt/tinyos-1.x/tos/system/UARTM.nc"
result_t UARTM$HPLUART$putDone(void)
#line 87
{
  bool oldState;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 90
    {
      {
      }
#line 91
      ;
      oldState = UARTM$state;
      UARTM$state = FALSE;
    }
#line 94
    __nesc_atomic_end(__nesc_atomic); }








  if (oldState) {
      UARTM$ByteComm$txDone();
      UARTM$ByteComm$txByteReady(TRUE);
    }
  return SUCCESS;
}

static 
# 230 "C:/cygwin/opt/tinyos-1.x/tos/system/NoCRCPacket.nc"
void NoCRCPacket$sendComplete(result_t success)
#line 230
{
  uint8_t stateCopy;

#line 232
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 232
    {
      stateCopy = NoCRCPacket$state;
    }
#line 234
    __nesc_atomic_end(__nesc_atomic); }

  if (stateCopy == NoCRCPacket$PACKET) {





      if (success) {
          TOS_MsgPtr msg;

#line 244
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 244
            {
              msg = (TOS_MsgPtr )NoCRCPacket$sendPtr;
              msg->ack = TRUE;
            }
#line 247
            __nesc_atomic_end(__nesc_atomic); }
          TOS_post(NoCRCPacket$sendDoneSuccessTask);
        }
      else {
          TOS_post(NoCRCPacket$sendDoneFailTask);
        }
    }
  else {
#line 254
    if (stateCopy == NoCRCPacket$BYTES) {
        if (success) {
            TOS_post(NoCRCPacket$sendVarLenSuccessTask);
          }
        else {
            TOS_post(NoCRCPacket$sendVarLenFailTask);
          }
      }
    else {
        { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 263
          {
            NoCRCPacket$txCount = 0;
            NoCRCPacket$state = NoCRCPacket$IDLE;
          }
#line 266
          __nesc_atomic_end(__nesc_atomic); }
      }
    }
}

static 
# 469 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
result_t FramerM$TxArbitraryByte(uint8_t inByte)
#line 469
{
  if (inByte == FramerM$HDLC_FLAG_BYTE || inByte == FramerM$HDLC_CTLESC_BYTE) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 471
        {
          FramerM$gPrevTxState = FramerM$gTxState;
          FramerM$gTxState = FramerM$TXSTATE_ESC;
          FramerM$gTxEscByte = inByte;
        }
#line 475
        __nesc_atomic_end(__nesc_atomic); }
      inByte = FramerM$HDLC_CTLESC_BYTE;
    }

  return FramerM$ByteComm$txByte(inByte);
}

