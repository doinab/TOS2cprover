# 36 "/usr/local/mspgcc/lib/../msp430/include/sys/inttypes.h"
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
# 41 "/usr/local/mspgcc/lib/../msp430/include/sys/types.h"
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
# 56 "/usr/local/mspgcc/lib/../msp430/include/stdlib.h"
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;




typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;
# 122 "/usr/local/mspgcc/lib/../msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/local/mspgcc/lib/../msp430/include/sys/_types.h"
typedef long _off_t;
typedef long _ssize_t;
# 28 "/usr/local/mspgcc/lib/../msp430/include/sys/reent.h" 3
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
# 18 "/usr/local/mspgcc/lib/../msp430/include/math.h"
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
# 15 "/usr/local/mspgcc/lib/../msp430/include/msp430/iostructures.h"
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
# 84 "/usr/local/mspgcc/lib/../msp430/include/msp430/iostructures.h" 3
struct __nesc_unnamed4251 {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t ifg;
  ioregister_t ies;
  ioregister_t ie;
  ioregister_t sel;
};




struct __nesc_unnamed4252 {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t ifg;
  ioregister_t ies;
  ioregister_t ie;
  ioregister_t sel;
};




struct __nesc_unnamed4253 {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct __nesc_unnamed4254 {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct __nesc_unnamed4255 {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct __nesc_unnamed4256 {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};
# 107 "/usr/local/mspgcc/lib/../msp430/include/msp430/gpio.h" 3
volatile unsigned char P1IFG __asm ("0x0023");

volatile unsigned char P1IES __asm ("0x0024");

volatile unsigned char P1IE __asm ("0x0025");
#line 124
volatile unsigned char P2IFG __asm ("0x002B");



volatile unsigned char P2IE __asm ("0x002D");
# 85 "/usr/local/mspgcc/lib/../msp430/include/msp430/usart.h"
volatile unsigned char U0CTL __asm ("0x0070");

volatile unsigned char U0TCTL __asm ("0x0071");



volatile unsigned char U0MCTL __asm ("0x0073");

volatile unsigned char U0BR0 __asm ("0x0074");

volatile unsigned char U0BR1 __asm ("0x0075");

volatile unsigned char U0RXBUF __asm ("0x0076");

volatile unsigned char U0TXBUF __asm ("0x0077");
#line 121
volatile unsigned char UCTL0 __asm ("0x0070");
# 253 "/usr/local/mspgcc/lib/../msp430/include/msp430/usart.h" 3
volatile unsigned char U1CTL __asm ("0x0078");

volatile unsigned char U1TCTL __asm ("0x0079");

volatile unsigned char U1RCTL __asm ("0x007A");

volatile unsigned char U1MCTL __asm ("0x007B");

volatile unsigned char U1BR0 __asm ("0x007C");

volatile unsigned char U1BR1 __asm ("0x007D");

volatile unsigned char U1RXBUF __asm ("0x007E");

volatile unsigned char U1TXBUF __asm ("0x007F");


volatile unsigned char UCTL1 __asm ("0x0078");
# 18 "/usr/local/mspgcc/lib/../msp430/include/msp430/timera.h"
volatile unsigned int TAIV __asm ("0x012E");

volatile unsigned int TACTL __asm ("0x0160");

volatile unsigned int TACCTL0 __asm ("0x0162");

volatile unsigned int TACCTL1 __asm ("0x0164");

volatile unsigned int TACCTL2 __asm ("0x0166");

volatile unsigned int TAR __asm ("0x0170");
# 18 "/usr/local/mspgcc/lib/../msp430/include/msp430/timerb.h"
volatile unsigned int TBIV __asm ("0x011E");

volatile unsigned int TBCTL __asm ("0x0180");

volatile unsigned int TBCCTL0 __asm ("0x0182");

volatile unsigned int TBCCTL1 __asm ("0x0184");

volatile unsigned int TBCCTL2 __asm ("0x0186");

volatile unsigned int TBR __asm ("0x0190");










volatile unsigned int TBCCTL3 __asm ("0x0188");

volatile unsigned int TBCCTL4 __asm ("0x018A");

volatile unsigned int TBCCTL5 __asm ("0x018C");

volatile unsigned int TBCCTL6 __asm ("0x018E");
# 18 "/usr/local/mspgcc/lib/../msp430/include/msp430/basic_clock.h"
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 20 "/usr/local/mspgcc/lib/../msp430/include/msp430/adc12.h"
volatile unsigned int ADC12CTL1 __asm ("0x01A2");









typedef struct __nesc_unnamed4257 {
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

typedef struct __nesc_unnamed4258 {
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

typedef struct __nesc_unnamed4259 {
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


struct __nesc_unnamed4260 {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};
# 39 "/usr/local/mspgcc/lib/../msp430/include/msp430x14x.h"
volatile unsigned char IE1 __asm ("0x0000");
#line 56
volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char IE2 __asm ("0x0001");









volatile unsigned char ME2 __asm ("0x0005");
static 
# 130 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
__inline void TOSH_wait(void );
static 
#line 143
__inline void TOSH_uwait(uint16_t u);
static inline 
#line 165
void __nesc_disable_interrupt(void);
static inline 




void __nesc_enable_interrupt(void);
static inline 



bool are_interrupts_enabled(void);




typedef bool __nesc_atomic_t;
static inline 
__nesc_atomic_t __nesc_atomic_start(void );
static inline 





void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






bool LPMode_disabled = FALSE;
static 








__inline void TOSH_sleep(void);
# 69 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12.h"
typedef struct __nesc_unnamed4261 {

  unsigned int inputChannel : 4;
  unsigned int referenceVoltage : 3;
  unsigned int sampleHoldTime : 4;

  unsigned int refVolt2_5 : 1;
} 
__attribute((packed))  MSP430ADC12StandardSettings_t;


typedef union __nesc_unnamed4262 {
  uint16_t i;
  MSP430ADC12StandardSettings_t s;
} MSP430ADC12StandardSettings_ut;

typedef struct __nesc_unnamed4263 {

  unsigned int inputChannel : 4;
  unsigned int referenceVoltage : 3;
  unsigned int sampleHoldTime : 4;
  unsigned int clockSource : 2;
  unsigned int clockDiv : 3;
  unsigned int sampleHoldSource : 2;
  unsigned int refVolt2_5 : 1;
} 
__attribute((packed))  MSP430ADC12AdvancedSettings_t;


typedef union __nesc_unnamed4264 {
  uint32_t i;
  MSP430ADC12AdvancedSettings_t s;
} MSP430ADC12AdvancedSettings_ut;

typedef enum __nesc_unnamed4265 {

  ADC_FAIL = 0, 
  ADC_SUCCESS = 1, 
  ADC_QUEUED = 2
} adcResult_t;

enum sampleHoldSource_enum {


  HOLDSOURCE_TIMERA_OUT1 = 1, 
  HOLDSOURCE_TIMERB_OUT0 = 2, 
  HOLDSOURCE_TIMERB_OUT1 = 3
};

enum refVolt2_5_enum {

  REFVOLT_LEVEL_1_5 = 0, 
  REFVOLT_LEVEL_2_5 = 1
};


enum clockDiv_enum {

  CLOCK_DIV_1 = 0, 
  CLOCK_DIV_2 = 1, 
  CLOCK_DIV_3 = 2, 
  CLOCK_DIV_4 = 3, 
  CLOCK_DIV_5 = 4, 
  CLOCK_DIV_6 = 5, 
  CLOCK_DIV_7 = 6, 
  CLOCK_DIV_8 = 7
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

enum clockSource_enum {

  CLOCK_SOURCE_ADC12OSC = 0, 
  CLOCK_SOURCE_ACLK = 1, 
  CLOCK_SOURCE_MCLK = 2, 
  CLOCK_SOURCE_SMCLK = 3
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





enum __nesc_unnamed4266 {

  TOSH_ACTUAL_ADC_EXTERNAL_REFERENCE_VOLTAGE_PORT = (((REFVOLT_LEVEL_1_5 << 3) + REFERENCE_VREFplus_AVss) << 4) + EXTERNAL_REFERENCE_VOLTAGE, 

  TOSH_ACTUAL_ADC_REFERENCE_VOLTAGE_NEGATIVE_TERMINAL_PORT = (((REFVOLT_LEVEL_1_5 << 3) + REFERENCE_VREFplus_AVss) << 4) + REFERENCE_VOLTAGE_NEGATIVE_TERMINAL, 

  TOSH_ACTUAL_ADC_INTERNAL_TEMPERATURE_PORT = (((REFVOLT_LEVEL_1_5 << 3) + REFERENCE_VREFplus_AVss) << 4) + INTERNAL_TEMPERATURE, 

  TOSH_ACTUAL_ADC_INTERNAL_VOLTAGE_PORT = (((REFVOLT_LEVEL_1_5 << 3) + REFERENCE_VREFplus_AVss) << 4) + INTERNAL_VOLTAGE
};
#line 223
typedef enum __nesc_unnamed4267 {

  TIMERA_OUT1 = 1, 
  TIMERB_OUT0 = 2, 
  TIMERB_OUT1 = 3
} MSP430ADC12Timer;


typedef enum __nesc_unnamed4268 {

  SINGLE_CHANNEL_SINGLE_CONVERSION = 0, 
  SEQUENCE_OF_CHANNELS = 1, 
  REPEAT_SINGLE_CHANNEL = 2, 
  REPEAT_SEQUENCE_OF_CHANNELS = 3, 
  INTERNAL_CHANNEL = 4, 
  ADVANCED_SEQUENCE_OF_CHANNELS = 5, 
  ADVANCED_REPEAT_SINGLE_CHANNEL = 6, 
  ADVANCED_REPEAT_SEQUENCE_OF_CHANNELS = 7, 
  ADC_IDLE = 8
} MSP430ADC12ConversionMode_t;

typedef struct __nesc_unnamed4269 {

  volatile unsigned 
  inch : 4, 
  sref : 3, 
  eos : 1;
} __attribute((packed))  adc12memctl_t;

typedef struct __nesc_unnamed4270 {

  adc12memctl_t memctl;
  unsigned int sampleHoldTime : 4;
  unsigned int refVolt2_5 : 1;
  unsigned int queued : 1;
  unsigned int gotRefVolt : 1;
  volatile unsigned int locked : 1;
} __attribute((packed))  bSettings_t;

typedef struct __nesc_unnamed4271 {

  adc12memctl_t memctl;
  unsigned int sampleHoldTime : 4;
  unsigned int refVolt2_5 : 1;
  unsigned int gotRefVolt : 1;
  unsigned int clockSource : 2;
  unsigned int clockDiv : 3;
  unsigned int sampleHoldSource : 2;
  volatile unsigned int locked : 1;
} __attribute((packed))  aSettings_t;

typedef struct __nesc_unnamed4272 {

  uint16_t *dataDest;
  uint16_t jiffies;
  uint8_t length;
  volatile uint8_t type;
  uint8_t intf;
} aBuffer_t;
# 241 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420Const.h"
enum __nesc_unnamed4273 {
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
# 46 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/AM.h"
enum __nesc_unnamed4274 {
  TOS_BCAST_ADDR = 0xffff, 
  TOS_UART_ADDR = 0x007e
};





enum __nesc_unnamed4275 {
  TOS_DEFAULT_AM_GROUP = 0x7d
};

uint8_t TOS_AM_GROUP = TOS_DEFAULT_AM_GROUP;
#line 71
typedef struct TOS_Msg {


  uint8_t length;
  uint8_t fcfhi;
  uint8_t fcflo;
  uint8_t dsn;
  uint16_t destpan;
  uint16_t addr;
  uint8_t type;
  uint8_t group;
  int8_t data[28];







  uint8_t strength;
  uint8_t lqi;
  bool crc;
  bool ack;
  uint16_t time;
} __attribute((packed))  TOS_Msg;

enum __nesc_unnamed4276 {

  MSG_HEADER_SIZE = (size_t )& ((struct TOS_Msg *)0)->data - 1, 

  MSG_FOOTER_SIZE = 2, 

  MSG_DATA_SIZE = (size_t )& ((struct TOS_Msg *)0)->strength + sizeof(uint16_t ), 

  DATA_LENGTH = 28, 

  LENGTH_BYTE_NUMBER = (size_t )& ((struct TOS_Msg *)0)->length + 1
};

typedef TOS_Msg *TOS_MsgPtr;
static inline 
# 11 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/hardware.h"
void TOSH_SET_RED_LED_PIN(void);
static inline 
#line 11
void TOSH_MAKE_RED_LED_OUTPUT(void);
static inline void TOSH_SET_GREEN_LED_PIN(void);
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

uint8_t TOSH_READ_CC_FIFO_PIN(void);
static inline uint8_t TOSH_READ_CC_SFD_PIN(void);
static inline void TOSH_SET_CC_VREN_PIN(void);
static inline void TOSH_SET_CC_RSTN_PIN(void);
static inline 
#line 30
void TOSH_CLR_CC_RSTN_PIN(void);
static inline 

void TOSH_SEL_SOMI0_MODFUNC(void);
static inline void TOSH_SEL_SIMO0_MODFUNC(void);
static inline void TOSH_SEL_UCLK0_MODFUNC(void);
static inline 

void TOSH_CLR_UTXD1_PIN(void);
static inline 
#line 38
void TOSH_MAKE_UTXD1_OUTPUT(void);
static inline 
#line 38
void TOSH_SEL_UTXD1_MODFUNC(void);
static inline void TOSH_CLR_URXD1_PIN(void);
static inline 
#line 39
void TOSH_MAKE_URXD1_OUTPUT(void);
static inline 
#line 39
void TOSH_SEL_URXD1_MODFUNC(void);
static inline 
void TOSH_SET_SOMI1_PIN(void);
static inline 
#line 41
void TOSH_MAKE_SOMI1_INPUT(void);
static inline 








void TOSH_CLR_HUM_SDA_PIN(void);
static inline 
#line 51
void TOSH_MAKE_HUM_SDA_OUTPUT(void);
static inline void TOSH_CLR_HUM_SCL_PIN(void);
static inline 
#line 52
void TOSH_MAKE_HUM_SCL_OUTPUT(void);
static inline void TOSH_CLR_HUM_PWR_PIN(void);
static inline 
#line 53
void TOSH_MAKE_HUM_PWR_OUTPUT(void);
#line 72
enum __nesc_unnamed4277 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};
static inline 

void TOSH_SET_FLASH_PWR_PIN(void);
static inline 
#line 80
void TOSH_MAKE_FLASH_PWR_OUTPUT(void);
static inline void TOSH_SET_FLASH_CS_PIN(void);
static inline 
#line 81
void TOSH_MAKE_FLASH_CS_OUTPUT(void);
static inline 


void TOSH_MAKE_PROG_RX_INPUT(void);
static inline void TOSH_MAKE_PROG_TX_INPUT(void);
static inline 
void TOSH_SET_PIN_DIRECTIONS(void );
#line 143
enum __nesc_unnamed4278 {
  TOSH_ADC_PORTMAPSIZE = 4
};

enum __nesc_unnamed4279 {

  TOSH_ACTUAL_ADC_PAR_PORT = (((REFVOLT_LEVEL_1_5 << 3) + REFERENCE_VREFplus_AVss) << 4) + INPUT_CHANNEL_A4, 


  TOSH_ACTUAL_ADC_TSR_PORT = (((REFVOLT_LEVEL_1_5 << 3) + REFERENCE_VREFplus_AVss) << 4) + INPUT_CHANNEL_A5
};



enum __nesc_unnamed4280 {

  TOS_ADC_PAR_PORT, 
  TOS_ADC_TSR_PORT, 
  TOS_ADC_INTERNAL_TEMP_PORT, 
  TOS_ADC_INTERNAL_VOLTAGE_PORT
};
# 54 "C:/cygwin/opt/tinyos-1.x/tos/types/dbg_modes.h"
typedef long long TOS_dbg_mode;



enum __nesc_unnamed4281 {
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
typedef struct __nesc_unnamed4282 {
  void (*tp)(void);
} TOSH_sched_entry_T;

enum __nesc_unnamed4283 {






  TOSH_MAX_TASKS = 8, 

  TOSH_TASK_BITMASK = TOSH_MAX_TASKS - 1
};

volatile TOSH_sched_entry_T TOSH_queue[TOSH_MAX_TASKS];
uint8_t TOSH_sched_full;
volatile uint8_t TOSH_sched_free;
static 

void TOSH_wait(void );
static void TOSH_sleep(void );
static inline 
void TOSH_sched_init(void );
#line 100
bool  TOS_post(void (*tp)(void));
static inline 
#line 132
bool TOSH_run_next_task(void);
static inline 
#line 155
void TOSH_run_task(void);
# 28 "C:/cygwin/opt/tinyos-1.x/tos/system/Ident.h"
enum __nesc_unnamed4284 {

  IDENT_MAX_PROGRAM_NAME_LENGTH = 10
};

typedef struct __nesc_unnamed4285 {

  uint32_t unix_time;
  uint32_t user_hash;
  char program_name[IDENT_MAX_PROGRAM_NAME_LENGTH];
} Ident_t;
# 9 "UbiMonMsg.h"
enum __nesc_unnamed4286 {
  BUFFER_SIZE = 40
};

struct UbiMonMsg {

  uint16_t sourceMoteID;
  uint16_t data[BUFFER_SIZE];
};

struct UbiMonResetMsg {
};



enum __nesc_unnamed4287 {
  AM_OSCOPEMSG = 10, 
  AM_OSCOPERESETMSG = 10
};
# 28 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.h"
typedef struct __nesc_unnamed4288 {

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
# 39 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Timer.h"
enum __nesc_unnamed4289 {
  TIMER_REPEAT = 0, 
  TIMER_ONE_SHOT = 1, 
  NUM_TIMERS = 2
};
# 35 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ADC.h"
enum __nesc_unnamed4290 {
  TOS_ADCSample3750ns = 0, 
  TOS_ADCSample7500ns = 1, 
  TOS_ADCSample15us = 2, 
  TOS_ADCSample30us = 3, 
  TOS_ADCSample60us = 4, 
  TOS_ADCSample120us = 5, 
  TOS_ADCSample240us = 6, 
  TOS_ADCSample480us = 7
};
# 16 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVolt.h"
typedef enum __nesc_unnamed4291 {

  REFERENCE_1_5V, 
  REFERENCE_2_5V, 
  REFERENCE_UNSTABLE
} RefVolt_t;
static 
# 10 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/byteorder.h"
__inline int is_host_lsb(void);
static 




__inline uint16_t toLSB16(uint16_t a);
static 



__inline uint16_t fromLSB16(uint16_t a);
static 
# 34 "C:/cygwin/opt/tinyos-1.x/tos/system/crc.h"
uint16_t crcByte(uint16_t crc, uint8_t b);
# 20 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/msp430baudrates.h"
enum __nesc_unnamed4292 {

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
  UBR_SMCLK_230400 = 0x0004, UMCTL_SMCLK_230400 = 0x55
};
static  result_t HPLInitM$init(void);
static  void MSP430ClockM$MSP430ClockInit$default$initTimerB(void);
static  void MSP430ClockM$MSP430ClockInit$defaultInitTimerA(void);
static  void MSP430ClockM$MSP430ClockInit$default$initTimerA(void);
static  void MSP430ClockM$MSP430ClockInit$defaultInitTimerB(void);
static  void MSP430ClockM$MSP430ClockInit$default$initClocks(void);
static  void MSP430ClockM$MSP430ClockInit$defaultInitClocks(void);
static   void MSP430ClockM$ACLKCompare$fired(void);
static  result_t MSP430ClockM$StdControl$init(void);
static  result_t MSP430ClockM$StdControl$start(void);
static   void MSP430TimerM$CompareB5$default$fired(void);
static   void MSP430TimerM$CompareB6$default$fired(void);
static   void MSP430TimerM$CompareA1$setEventFromPrev(uint16_t arg_0xa41b240);
static   void MSP430TimerM$CompareA1$setEventFromNow(uint16_t arg_0xa41b658);
static   void MSP430TimerM$CompareA1$disableEvents(void);
static   void MSP430TimerM$CompareA1$setControl(MSP430CompareControl_t arg_0xa425b70);
static   void MSP430TimerM$CompareB3$enableEvents(void);
static   void MSP430TimerM$CompareB3$setControlAsTimer(void);
static   void MSP430TimerM$CompareB3$setEventFromNow(uint16_t arg_0xa41b658);
static   void MSP430TimerM$CompareB3$disableEvents(void);
static   void MSP430TimerM$CompareB3$clearPendingInterrupt(void);
static   void MSP430TimerM$CompareB1$setEventFromPrev(uint16_t arg_0xa41b240);
static   void MSP430TimerM$CompareB1$setEventFromNow(uint16_t arg_0xa41b658);
static   void MSP430TimerM$CompareB1$disableEvents(void);
static   void MSP430TimerM$CompareB1$setControl(MSP430CompareControl_t arg_0xa425b70);
static   void MSP430TimerM$TimerA$default$overflow(void);
static   void MSP430TimerM$CompareA2$default$fired(void);
static   void MSP430TimerM$CompareB4$enableEvents(void);
static   void MSP430TimerM$CompareB4$setControlAsTimer(void);
static   void MSP430TimerM$CompareB4$setEventFromNow(uint16_t arg_0xa41b658);
static   void MSP430TimerM$CompareB4$disableEvents(void);
static   void MSP430TimerM$CompareB4$clearPendingInterrupt(void);
static   void MSP430TimerM$CompareA0$default$fired(void);
static   void MSP430TimerM$CompareB2$enableEvents(void);
static   void MSP430TimerM$CompareB2$setEventFromPrev(uint16_t arg_0xa41b240);
static   void MSP430TimerM$CompareB2$setControlAsTimer(void);
static   void MSP430TimerM$CompareB2$setEventFromNow(uint16_t arg_0xa41b658);
static   uint16_t MSP430TimerM$TimerB$read(void);
static   void MSP430TimerM$TimerB$clearOverflow(void);
static   bool MSP430TimerM$TimerB$isOverflowPending(void);
static   void MSP430TimerM$CompareB0$setEventFromPrev(uint16_t arg_0xa41b240);
static   void MSP430TimerM$CompareB0$setEventFromNow(uint16_t arg_0xa41b658);
static   void MSP430TimerM$CompareB0$disableEvents(void);
static   void MSP430TimerM$CompareB0$setControl(MSP430CompareControl_t arg_0xa425b70);
static  result_t BSN_RSUM$DataMsg$sendDone(TOS_MsgPtr arg_0xa529ce8, result_t arg_0xa529e38);
static   result_t BSN_RSUM$AccelX$dataReady(uint16_t arg_0xa4fb2c8);
static   result_t BSN_RSUM$AccelY$dataReady(uint16_t arg_0xa4fb2c8);
static   result_t BSN_RSUM$Temp$dataReady(uint16_t arg_0xa4fb2c8);
static  TOS_MsgPtr BSN_RSUM$ResetCounterMsg$receive(TOS_MsgPtr arg_0xa511178);
static  result_t BSN_RSUM$StdControl$init(void);
static  result_t BSN_RSUM$StdControl$start(void);
static  result_t BSN_RSUM$Timer$fired(void);
static  result_t TimerM$TimerMilli$default$fired(uint8_t arg_0xa4e5e10);
static  result_t TimerM$TimerMilli$setOneShot(uint8_t arg_0xa4e5e10, int32_t arg_0xa538c18);
static   uint32_t TimerM$LocalTime$read(void);
static   void TimerM$AlarmCompare$fired(void);
static  result_t TimerM$TimerJiffy$default$fired(uint8_t arg_0xa54c850);
static   void TimerM$AlarmTimer$overflow(void);
static  result_t TimerM$StdControl$init(void);
static  result_t TimerM$StdControl$start(void);
static  result_t TimerM$Timer$default$fired(uint8_t arg_0xa4e5788);
static  result_t TimerM$Timer$start(uint8_t arg_0xa4e5788, char arg_0xa4efc10, uint32_t arg_0xa4efd68);
static   result_t LedsC$Leds$yellowOff(void);
static   result_t LedsC$Leds$yellowOn(void);
static   result_t LedsC$Leds$init(void);
static   result_t LedsC$Leds$greenOff(void);
static   result_t LedsC$Leds$redOff(void);
static   result_t LedsC$Leds$yellowToggle(void);
static  result_t AccelTempM$StdControl$init(void);
static  result_t AccelTempM$StdControl$start(void);
static  result_t ADCM$ADCControl$bindPort(uint8_t arg_0xa5a6618, uint8_t arg_0xa5a6760);
static  result_t ADCM$ADCControl$init(void);
static   result_t ADCM$ADC$getData(uint8_t arg_0xa59e788);
static   result_t ADCM$ADC$default$dataReady(uint8_t arg_0xa59e788, uint16_t arg_0xa4fb2c8);
static   void ADCM$MSP430ADC12Basic$dataReadySingle(uint16_t arg_0xa59d600);
static   void MSP430ADC12M$MSP430ADC12Advanced$default$dataReadySequence(uint8_t arg_0xa5c2898);
static   void MSP430ADC12M$MSP430ADC12Advanced$default$dataReadySingleRpt(uint8_t arg_0xa5c2898, uint16_t arg_0xa5c4cb8);
static   adcResult_t MSP430ADC12M$MSP430ADC12Advanced$getSingleDataRepeat(uint8_t arg_0xa5c2898, uint16_t arg_0xa5c6da0);
static   void MSP430ADC12M$MSP430ADC12Advanced$default$dataReadySequenceRpt(uint8_t arg_0xa5c2898);
static   void MSP430ADC12M$CompareB0$fired(void);
static   void MSP430ADC12M$CompareA1$fired(void);
static  void MSP430ADC12M$RefVolt$isStable(RefVolt_t arg_0xa5cea80);
static   void MSP430ADC12M$CompareB1$fired(void);
static   void MSP430ADC12M$HPLADC12$memOverflow(void);
static   void MSP430ADC12M$HPLADC12$converted(uint8_t arg_0xa5dafa0);
static   void MSP430ADC12M$HPLADC12$timeOverflow(void);
static   result_t MSP430ADC12M$MSP430ADC12Basic$bind(uint8_t arg_0xa5c2158, MSP430ADC12StandardSettings_ut arg_0xa59c690);
static   adcResult_t MSP430ADC12M$MSP430ADC12Basic$getSingleData(uint8_t arg_0xa5c2158);
static   void MSP430ADC12M$MSP430ADC12Basic$default$dataReadySingle(uint8_t arg_0xa5c2158, uint16_t arg_0xa59d600);
static   void HPLADC12M$HPLADC12$setRefOff(void);
static   void HPLADC12M$HPLADC12$resetIFGs(void);
static   bool HPLADC12M$HPLADC12$isBusy(void);
static   void HPLADC12M$HPLADC12$setControl1(adc12ctl1_t arg_0xa5bc198);
static   void HPLADC12M$HPLADC12$setRef2_5V(void);
static   void HPLADC12M$HPLADC12$disableConversion(void);
static   void HPLADC12M$HPLADC12$setControl0_IgnoreRef(adc12ctl0_t arg_0xa5bcb60);
static   void HPLADC12M$HPLADC12$setRefOn(void);
static   adc12memctl_t HPLADC12M$HPLADC12$getMemControl(uint8_t arg_0xa5bd4f0);
static   void HPLADC12M$HPLADC12$setRef1_5V(void);
static   void HPLADC12M$HPLADC12$startConversion(void);
static   uint16_t HPLADC12M$HPLADC12$getMem(uint8_t arg_0xa5bd908);
static   void HPLADC12M$HPLADC12$setIEFlags(uint16_t arg_0xa5bdd20);
static   void HPLADC12M$HPLADC12$setSHT(uint8_t arg_0xa5dba80);
static   void HPLADC12M$HPLADC12$setMemControl(uint8_t arg_0xa5bcf70, adc12memctl_t arg_0xa5bd0c8);
static   void HPLADC12M$HPLADC12$stopConversion(void);
static  result_t RefVoltM$SwitchOffTimer$fired(void);
static   RefVolt_t RefVoltM$RefVolt$getState(void);
static   result_t RefVoltM$RefVolt$release(void);
static   result_t RefVoltM$RefVolt$get(RefVolt_t arg_0xa5eb538);
static   void RefVoltM$HPLADC12$memOverflow(void);
static   void RefVoltM$HPLADC12$converted(uint8_t arg_0xa5dafa0);
static   void RefVoltM$HPLADC12$timeOverflow(void);
static  result_t RefVoltM$SwitchOnTimer$fired(void);
static  TOS_MsgPtr AMStandard$ReceiveMsg$default$receive(uint8_t arg_0xa684e80, TOS_MsgPtr arg_0xa511178);
static  result_t AMStandard$ActivityTimer$fired(void);
static  result_t AMStandard$UARTSend$sendDone(TOS_MsgPtr arg_0xa6a0e88, result_t arg_0xa6a0fd8);
static  TOS_MsgPtr AMStandard$RadioReceive$receive(TOS_MsgPtr arg_0xa511178);
static  result_t AMStandard$Control$init(void);
static  result_t AMStandard$Control$start(void);
static  result_t AMStandard$default$sendDone(void);
static  result_t AMStandard$RadioSend$sendDone(TOS_MsgPtr arg_0xa6a0e88, result_t arg_0xa6a0fd8);
static  result_t AMStandard$SendMsg$send(uint8_t arg_0xa6848c8, uint16_t arg_0xa529630, uint8_t arg_0xa529778, TOS_MsgPtr arg_0xa5298c8);
static  result_t AMStandard$SendMsg$default$sendDone(uint8_t arg_0xa6848c8, TOS_MsgPtr arg_0xa529ce8, result_t arg_0xa529e38);
static  TOS_MsgPtr AMStandard$UARTReceive$receive(TOS_MsgPtr arg_0xa511178);
static   result_t CC2420RadioM$BackoffTimerJiffy$fired(void);
static  result_t CC2420RadioM$Send$send(TOS_MsgPtr arg_0xa6a0970);
static   result_t CC2420RadioM$HPLChipcon$FIFOPIntr(void);
static   result_t CC2420RadioM$HPLChipconFIFO$TXFIFODone(uint8_t arg_0xa6fd6c8, uint8_t *arg_0xa6fd828);
static   result_t CC2420RadioM$HPLChipconFIFO$RXFIFODone(uint8_t arg_0xa6fd038, uint8_t *arg_0xa6fd198);
static  result_t CC2420RadioM$StdControl$init(void);
static  result_t CC2420RadioM$StdControl$start(void);
static   int16_t CC2420RadioM$MacBackoff$default$initialBackoff(TOS_MsgPtr arg_0xa6c9b18);
static   int16_t CC2420RadioM$MacBackoff$default$congestionBackoff(TOS_MsgPtr arg_0xa6c9f40);
static   result_t CC2420ControlM$HPLChipcon$FIFOPIntr(void);
static   result_t CC2420ControlM$HPLChipconRAM$writeDone(uint16_t arg_0xa71e8a8, uint8_t arg_0xa71e9f0, uint8_t *arg_0xa71eb50);
static  result_t CC2420ControlM$StdControl$init(void);
static  result_t CC2420ControlM$StdControl$start(void);
static   result_t CC2420ControlM$CC2420Control$VREFOn(void);
static  result_t CC2420ControlM$CC2420Control$TunePreset(uint8_t arg_0xa6d3178);
static   result_t CC2420ControlM$CC2420Control$RxMode(void);
static  result_t CC2420ControlM$CC2420Control$setShortAddress(uint16_t arg_0xa6ccae8);
static   result_t CC2420ControlM$CC2420Control$OscillatorOn(void);
static  result_t HPLCC2420M$BusArbitration$busFree(void);
static   result_t HPLCC2420M$HPLCC2420$enableFIFOP(void);
static   uint16_t HPLCC2420M$HPLCC2420$read(uint8_t arg_0xa6dcd48);
static   uint8_t HPLCC2420M$HPLCC2420$write(uint8_t arg_0xa6dc750, uint16_t arg_0xa6dc8a0);
static   uint8_t HPLCC2420M$HPLCC2420$cmd(uint8_t arg_0xa6dc2f8);
static   result_t HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(uint8_t arg_0xa6fc9b8, uint8_t *arg_0xa6fcb18);
static   result_t HPLCC2420M$HPLCC2420FIFO$readRXFIFO(uint8_t arg_0xa6fc280, uint8_t *arg_0xa6fc3e0);
static   result_t HPLCC2420M$HPLCC2420RAM$write(uint16_t arg_0xa71e180, uint8_t arg_0xa71e2c8, uint8_t *arg_0xa71e428);
static  result_t HPLCC2420M$StdControl$init(void);
static  result_t HPLCC2420M$StdControl$start(void);
static   void HPLCC2420M$FIFOPInterrupt$fired(void);
static   result_t HPLUSART0M$USARTData$default$rxDone(uint8_t arg_0xa7a28e8);
static   result_t HPLUSART0M$USARTData$default$txDone(void);
static   result_t HPLUSART0M$USARTControl$disableRxIntr(void);
static   void HPLUSART0M$USARTControl$setModeSPI(void);
static   result_t HPLUSART0M$USARTControl$disableTxIntr(void);
static   result_t HPLUSART0M$USARTControl$isTxIntrPending(void);
static   result_t HPLUSART0M$USARTControl$tx(uint8_t arg_0xa7709d0);
static   uint8_t HPLUSART0M$USARTControl$rx(void);
static   void MSP430InterruptM$Port14$clear(void);
static   void MSP430InterruptM$Port14$default$fired(void);
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
static   void MSP430InterruptM$Port15$clear(void);
static   void MSP430InterruptM$Port15$default$fired(void);
static   void MSP430InterruptM$Port27$clear(void);
static   void MSP430InterruptM$Port27$default$fired(void);
static   void MSP430InterruptM$Port10$clear(void);
static   void MSP430InterruptM$Port10$disable(void);
static   void MSP430InterruptM$Port10$edge(bool arg_0xa7797e0);
static   void MSP430InterruptM$Port10$enable(void);
static   void MSP430InterruptM$Port22$clear(void);
static   void MSP430InterruptM$Port22$default$fired(void);
static   void MSP430InterruptM$Port13$clear(void);
static   void MSP430InterruptM$Port13$default$fired(void);
static   void MSP430InterruptM$Port25$clear(void);
static   void MSP430InterruptM$Port25$default$fired(void);
static   void MSP430InterruptM$Port16$clear(void);
static   void MSP430InterruptM$Port16$default$fired(void);
static   void MSP430InterruptM$Port20$clear(void);
static   void MSP430InterruptM$Port20$default$fired(void);
static   void MSP430InterruptM$Port11$clear(void);
static   void MSP430InterruptM$Port11$default$fired(void);
static   void MSP430InterruptM$Port23$clear(void);
static   void MSP430InterruptM$Port23$default$fired(void);
static  result_t BusArbitrationM$BusArbitration$default$busFree(uint8_t arg_0xa820560);
static   result_t BusArbitrationM$BusArbitration$releaseBus(uint8_t arg_0xa820560);
static   result_t BusArbitrationM$BusArbitration$getBus(uint8_t arg_0xa820560);
static   uint16_t RandomLFSR$Random$rand(void);
static   result_t RandomLFSR$Random$init(void);
static   result_t TimerJiffyAsyncM$TimerJiffyAsync$setOneShot(uint32_t arg_0xa6ff210);
static   bool TimerJiffyAsyncM$TimerJiffyAsync$isSet(void);
static   result_t TimerJiffyAsyncM$TimerJiffyAsync$stop(void);
static   void TimerJiffyAsyncM$Alarm$fired(void);
static  result_t TimerJiffyAsyncM$StdControl$init(void);
static  result_t TimerJiffyAsyncM$StdControl$start(void);
static   result_t FramerM$ByteComm$txDone(void);
static   result_t FramerM$ByteComm$txByteReady(bool arg_0xa87d008);
static   result_t FramerM$ByteComm$rxByteReady(uint8_t arg_0xa87c838, bool arg_0xa87c980, uint16_t arg_0xa87cad8);
static  result_t FramerM$BareSendMsg$send(TOS_MsgPtr arg_0xa6a0970);
static  result_t FramerM$StdControl$init(void);
static  result_t FramerM$StdControl$start(void);
static  result_t FramerM$TokenReceiveMsg$ReflectToken(uint8_t arg_0xa84c810);
static  TOS_MsgPtr FramerAckM$ReceiveMsg$receive(TOS_MsgPtr arg_0xa511178);
static  TOS_MsgPtr FramerAckM$TokenReceiveMsg$receive(TOS_MsgPtr arg_0xa84c0b0, uint8_t arg_0xa84c1f8);
static   result_t UARTM$HPLUART$get(uint8_t arg_0xa8b48e0);
static   result_t UARTM$HPLUART$putDone(void);
static   result_t UARTM$ByteComm$txByte(uint8_t arg_0xa87c3a8);
static  result_t UARTM$Control$init(void);
static  result_t UARTM$Control$start(void);
static   result_t HPLUARTM$USARTData$rxDone(uint8_t arg_0xa7a28e8);
static   result_t HPLUARTM$USARTData$txDone(void);
static   result_t HPLUARTM$UART$init(void);
static   result_t HPLUARTM$UART$put(uint8_t arg_0xa8b43e0);
static   void HPLUSART1M$USARTControl$setClockRate(uint16_t arg_0xa772fb0, uint8_t arg_0xa7730f8);
static   void HPLUSART1M$USARTControl$setClockSource(uint8_t arg_0xa772b10);
static   result_t HPLUSART1M$USARTControl$enableRxIntr(void);
static   result_t HPLUSART1M$USARTControl$enableTxIntr(void);
static   result_t HPLUSART1M$USARTControl$tx(uint8_t arg_0xa7709d0);
static   void HPLUSART1M$USARTControl$setModeUART(void);
static   uint8_t HPLPowerManagementM$PowerManagement$adjustPower(void);
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
static   
# 36 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void MSP430ClockM$ACLKCompare$enableEvents(void);
static   




void MSP430ClockM$ACLKCompare$setEventFromPrev(uint16_t arg_0xa41b240);
static   
#line 34
void MSP430ClockM$ACLKCompare$setControlAsTimer(void);
static   







void MSP430ClockM$ACLKCompare$setEventFromNow(uint16_t arg_0xa41b658);
 
# 35 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
static volatile uint8_t MSP430ClockM$IE1 __asm ("0x0000");
 static volatile uint16_t MSP430ClockM$TACTL __asm ("0x0160");
 static volatile uint16_t MSP430ClockM$TAIV __asm ("0x012E");
 static volatile uint16_t MSP430ClockM$TBCTL __asm ("0x0180");
 static volatile uint16_t MSP430ClockM$TBIV __asm ("0x011E");

volatile  uint16_t MSP430ClockM$m_dco_curr;
volatile  uint16_t MSP430ClockM$m_dco_prev;
volatile  uint8_t MSP430ClockM$m_aclk_count;

enum MSP430ClockM$__nesc_unnamed4293 {

  MSP430ClockM$ACLK_CALIB_PERIOD = 128, 
  MSP430ClockM$ACLK_KHZ = 32, 
  MSP430ClockM$TARGET_DCO_KHZ = 4096, 
  MSP430ClockM$TARGET_DCO_DELTA = MSP430ClockM$TARGET_DCO_KHZ / MSP430ClockM$ACLK_KHZ * MSP430ClockM$ACLK_CALIB_PERIOD
};
static inline  
void MSP430ClockM$MSP430ClockInit$defaultInitClocks(void);
static inline  
#line 74
void MSP430ClockM$MSP430ClockInit$defaultInitTimerA(void);
static inline  
#line 89
void MSP430ClockM$MSP430ClockInit$defaultInitTimerB(void);
static inline   
#line 104
void MSP430ClockM$MSP430ClockInit$default$initClocks(void);
static inline   



void MSP430ClockM$MSP430ClockInit$default$initTimerA(void);
static inline   



void MSP430ClockM$MSP430ClockInit$default$initTimerB(void);
static inline 




void MSP430ClockM$startTimerA(void);
static inline 
#line 132
void MSP430ClockM$startTimerB(void);
static inline   
#line 145
void MSP430ClockM$ACLKCompare$fired(void);
static inline 
#line 157
void MSP430ClockM$set_calib(int calib);
static inline 




void MSP430ClockM$test_calib(int calib);
static inline 





uint16_t MSP430ClockM$busywait_delta(void);
static inline 




uint16_t MSP430ClockM$test_calib_busywait_delta(int calib);
static inline 





void MSP430ClockM$busyCalibrateDCO(void);
static inline 
#line 233
void MSP430ClockM$garnishedBusyCalibrateDCO(void);
static inline  








result_t MSP430ClockM$StdControl$init(void);
static inline  
#line 264
result_t MSP430ClockM$StdControl$start(void);
static   
# 45 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void MSP430TimerM$CompareB5$fired(void);
static   
#line 45
void MSP430TimerM$CompareB6$fired(void);
static   
#line 45
void MSP430TimerM$CompareA1$fired(void);
static   
#line 45
void MSP430TimerM$CompareB3$fired(void);
static   
#line 45
void MSP430TimerM$CompareB1$fired(void);
static   
# 32 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
void MSP430TimerM$TimerA$overflow(void);
static   
# 45 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void MSP430TimerM$CompareA2$fired(void);
static   
#line 45
void MSP430TimerM$CompareB4$fired(void);
static   
#line 45
void MSP430TimerM$CompareA0$fired(void);
static   
#line 45
void MSP430TimerM$CompareB2$fired(void);
static   
# 32 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
void MSP430TimerM$TimerB$overflow(void);
static   
# 45 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void MSP430TimerM$CompareB0$fired(void);
 
# 46 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
static volatile uint16_t MSP430TimerM$TACCTL1 __asm ("0x0164");
 


static volatile uint16_t MSP430TimerM$TACCR1 __asm ("0x0174");
 

static volatile uint16_t MSP430TimerM$TBCCTL0 __asm ("0x0182");
 static volatile uint16_t MSP430TimerM$TBCCTL1 __asm ("0x0184");
 static volatile uint16_t MSP430TimerM$TBCCTL2 __asm ("0x0186");
 static volatile uint16_t MSP430TimerM$TBCCTL3 __asm ("0x0188");
 static volatile uint16_t MSP430TimerM$TBCCTL4 __asm ("0x018A");
 


static volatile uint16_t MSP430TimerM$TBCCR0 __asm ("0x0192");
 static volatile uint16_t MSP430TimerM$TBCCR1 __asm ("0x0194");
 static volatile uint16_t MSP430TimerM$TBCCR2 __asm ("0x0196");
 static volatile uint16_t MSP430TimerM$TBCCR3 __asm ("0x0198");
 static volatile uint16_t MSP430TimerM$TBCCR4 __asm ("0x019A");



typedef MSP430CompareControl_t MSP430TimerM$CC_t;
static inline 
uint16_t MSP430TimerM$CC2int(MSP430TimerM$CC_t cc);
static inline 





uint16_t MSP430TimerM$timerControl(void);
#line 90
void __attribute((interrupt(12))) __attribute((wakeup))   sig_TIMERA0_VECTOR(void);




void __attribute((interrupt(10))) __attribute((wakeup))   sig_TIMERA1_VECTOR(void);
static inline    
#line 111
void MSP430TimerM$CompareA0$default$fired(void);
static inline    
void MSP430TimerM$CompareA2$default$fired(void);
static inline    void MSP430TimerM$TimerA$default$overflow(void);
static inline   

uint16_t MSP430TimerM$TimerB$read(void);
static inline   

bool MSP430TimerM$TimerB$isOverflowPending(void);
static inline   

void MSP430TimerM$TimerB$clearOverflow(void);
static inline   
#line 138
void MSP430TimerM$CompareA1$setControl(MSP430TimerM$CC_t x);
static inline   
#line 150
void MSP430TimerM$CompareA1$disableEvents(void);
static inline   
#line 166
void MSP430TimerM$CompareA1$setEventFromPrev(uint16_t x);
static inline   


void MSP430TimerM$CompareA1$setEventFromNow(uint16_t x);



void __attribute((interrupt(26))) __attribute((wakeup))   sig_TIMERB0_VECTOR(void);




void __attribute((interrupt(24))) __attribute((wakeup))   sig_TIMERB1_VECTOR(void);
static inline    
#line 201
void MSP430TimerM$CompareB5$default$fired(void);
static inline    void MSP430TimerM$CompareB6$default$fired(void);
static inline   
#line 224
void MSP430TimerM$CompareB3$clearPendingInterrupt(void);
static inline   void MSP430TimerM$CompareB4$clearPendingInterrupt(void);
static inline   


void MSP430TimerM$CompareB0$setControl(MSP430TimerM$CC_t x);
static inline   void MSP430TimerM$CompareB1$setControl(MSP430TimerM$CC_t x);
static inline   







void MSP430TimerM$CompareB2$setControlAsTimer(void);
static inline   void MSP430TimerM$CompareB3$setControlAsTimer(void);
static inline   void MSP430TimerM$CompareB4$setControlAsTimer(void);
static inline   




void MSP430TimerM$CompareB2$enableEvents(void);
static inline   void MSP430TimerM$CompareB3$enableEvents(void);
static inline   void MSP430TimerM$CompareB4$enableEvents(void);
static inline   


void MSP430TimerM$CompareB0$disableEvents(void);
static inline   void MSP430TimerM$CompareB1$disableEvents(void);
static inline   
void MSP430TimerM$CompareB3$disableEvents(void);
static inline   void MSP430TimerM$CompareB4$disableEvents(void);
static inline   
#line 285
void MSP430TimerM$CompareB0$setEventFromPrev(uint16_t x);
static inline   void MSP430TimerM$CompareB1$setEventFromPrev(uint16_t x);
static inline   void MSP430TimerM$CompareB2$setEventFromPrev(uint16_t x);
static inline   




void MSP430TimerM$CompareB0$setEventFromNow(uint16_t x);
static inline   void MSP430TimerM$CompareB1$setEventFromNow(uint16_t x);
static inline   void MSP430TimerM$CompareB2$setEventFromNow(uint16_t x);
static inline   void MSP430TimerM$CompareB3$setEventFromNow(uint16_t x);
static inline   void MSP430TimerM$CompareB4$setEventFromNow(uint16_t x);
static  
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
result_t BSN_RSUM$SensorControl$init(void);
static  





result_t BSN_RSUM$SensorControl$start(void);
static  
# 48 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SendMsg.nc"
result_t BSN_RSUM$DataMsg$send(uint16_t arg_0xa529630, uint8_t arg_0xa529778, TOS_MsgPtr arg_0xa5298c8);
static   
# 52 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ADC.nc"
result_t BSN_RSUM$AccelX$getData(void);
static   
#line 52
result_t BSN_RSUM$AccelY$getData(void);
static   
# 122 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Leds.nc"
result_t BSN_RSUM$Leds$yellowOff(void);
static   
#line 56
result_t BSN_RSUM$Leds$init(void);
static   
#line 97
result_t BSN_RSUM$Leds$greenOff(void);
static   
#line 72
result_t BSN_RSUM$Leds$redOff(void);
static   
#line 131
result_t BSN_RSUM$Leds$yellowToggle(void);
static   
# 52 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ADC.nc"
result_t BSN_RSUM$Temp$getData(void);
static  
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
result_t BSN_RSUM$CommControl$init(void);
static  





result_t BSN_RSUM$CommControl$start(void);
static  
# 59 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Timer.nc"
result_t BSN_RSUM$Timer$start(char arg_0xa4efc10, uint32_t arg_0xa4efd68);
# 68 "BSN_RSUM.nc"
uint8_t BSN_RSUM$packetReadingNumber;
uint16_t BSN_RSUM$readingNumber;
TOS_Msg BSN_RSUM$msg[2];
uint8_t BSN_RSUM$currentMsg;
uint8_t BSN_RSUM$readwhich;
static 
char BSN_RSUM$SendData(uint16_t data, int channel);
static inline  



result_t BSN_RSUM$StdControl$init(void);
static inline  
#line 103
result_t BSN_RSUM$StdControl$start(void);
static inline  
#line 122
void BSN_RSUM$dataTask(void);
static 
#line 175
char BSN_RSUM$SendData(uint16_t data, int channel);
static inline   
#line 200
result_t BSN_RSUM$Temp$dataReady(uint16_t data);
static inline   




result_t BSN_RSUM$AccelX$dataReady(uint16_t data);
static inline   





result_t BSN_RSUM$AccelY$dataReady(uint16_t data);
static inline  
#line 225
result_t BSN_RSUM$DataMsg$sendDone(TOS_MsgPtr sent, result_t success);
static inline  






result_t BSN_RSUM$Timer$fired(void);
static inline  
#line 249
TOS_MsgPtr BSN_RSUM$ResetCounterMsg$receive(TOS_MsgPtr m);
static  
# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
result_t TimerM$TimerMilli$fired(
# 32 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
uint8_t arg_0xa4e5e10);
static   
# 36 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void TimerM$AlarmCompare$enableEvents(void);
static   
#line 34
void TimerM$AlarmCompare$setControlAsTimer(void);
static   







void TimerM$AlarmCompare$setEventFromNow(uint16_t arg_0xa41b658);
static   
#line 37
void TimerM$AlarmCompare$disableEvents(void);
static   
#line 31
void TimerM$AlarmCompare$clearPendingInterrupt(void);
static  
# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerJiffy.nc"
result_t TimerM$TimerJiffy$fired(
# 33 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
uint8_t arg_0xa54c850);
static   
# 29 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
uint16_t TimerM$AlarmTimer$read(void);
static   
void TimerM$AlarmTimer$clearOverflow(void);
static   
#line 30
bool TimerM$AlarmTimer$isOverflowPending(void);
static  
# 73 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Timer.nc"
result_t TimerM$Timer$fired(
# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
uint8_t arg_0xa4e5788);







enum TimerM$__nesc_unnamed4294 {

  TimerM$COUNT_TIMER_OLD = 2, 
  TimerM$COUNT_TIMER_MILLI = 2, 
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
  int _reserved_flags : 6;
  uint8_t _reserved_byte;
} TimerM$Timer_t;

TimerM$Timer_t TimerM$m_timers[TimerM$NUM_TIMERS];
int32_t TimerM$m_period[TimerM$NUM_TIMERS];
uint16_t TimerM$m_hinow;
uint8_t TimerM$m_head_short;
uint8_t TimerM$m_head_long;
static  
result_t TimerM$StdControl$init(void);
static inline  








result_t TimerM$StdControl$start(void);
static 








void TimerM$insertTimer(uint8_t num, bool isshort);
static inline 
#line 105
void TimerM$removeTimer(uint8_t num);
static inline 
#line 145
void TimerM$signal_timer_fired(uint8_t num);
static 
#line 168
void TimerM$executeTimers(uint8_t head);
static inline  
#line 201
void TimerM$checkShortTimers(void);
static 
void TimerM$setNextShortEvent(void);
static inline  
#line 252
void TimerM$checkShortTimers(void);
static inline  






void TimerM$checkLongTimers(void);
static   






uint32_t TimerM$LocalTime$read(void);
static inline   
#line 296
void TimerM$AlarmCompare$fired(void);
static inline   



void TimerM$AlarmTimer$overflow(void);
static 




result_t TimerM$setTimer(uint8_t num, int32_t jiffy, bool isperiodic);
static inline   
#line 375
result_t TimerM$TimerJiffy$default$fired(uint8_t num);
static inline 






uint8_t TimerM$fromNumMilli(uint8_t num);
static inline  



result_t TimerM$TimerMilli$setOneShot(uint8_t num, int32_t milli);
static inline   
#line 424
result_t TimerM$TimerMilli$default$fired(uint8_t num);
static  






result_t TimerM$Timer$start(uint8_t num, char type, uint32_t milli);
static inline   
#line 452
result_t TimerM$Timer$default$fired(uint8_t num);
# 50 "C:/cygwin/opt/tinyos-1.x/tos/system/LedsC.nc"
uint8_t LedsC$ledsOn;

enum LedsC$__nesc_unnamed4295 {
  LedsC$RED_BIT = 1, 
  LedsC$GREEN_BIT = 2, 
  LedsC$YELLOW_BIT = 4
};
static inline   
result_t LedsC$Leds$init(void);
static inline   
#line 81
result_t LedsC$Leds$redOff(void);
static inline   
#line 110
result_t LedsC$Leds$greenOff(void);
static inline   
#line 130
result_t LedsC$Leds$yellowOn(void);
static inline   







result_t LedsC$Leds$yellowOff(void);
static inline   







result_t LedsC$Leds$yellowToggle(void);
static  
# 89 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ADCControl.nc"
result_t AccelTempM$ADCControl$bindPort(uint8_t arg_0xa5a6618, uint8_t arg_0xa5a6760);
static  
#line 50
result_t AccelTempM$ADCControl$init(void);
static inline  
# 19 "AccelTempM.nc"
result_t AccelTempM$StdControl$init(void);
static inline  





result_t AccelTempM$StdControl$start(void);
static   
# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ADC.nc"
result_t ADCM$ADC$dataReady(
# 46 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/ADCM.nc"
uint8_t arg_0xa59e788, 
# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ADC.nc"
uint16_t arg_0xa4fb2c8);
static   
# 69 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12Basic.nc"
result_t ADCM$MSP430ADC12Basic$bind(MSP430ADC12StandardSettings_ut arg_0xa59c690);
static   
#line 93
adcResult_t ADCM$MSP430ADC12Basic$getSingleData(void);
 
# 62 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/ADCM.nc"
uint8_t ADCM$TOSH_adc_portmap[TOSH_ADC_PORTMAPSIZE];
 uint8_t ADCM$samplingRate;
 bool ADCM$continuousData;
 uint8_t ADCM$owner;
bool ADCM$initialized;
volatile bool ADCM$busy;
static inline  
#line 90
result_t ADCM$ADCControl$init(void);
static inline  
#line 115
result_t ADCM$ADCControl$bindPort(uint8_t port, uint8_t adcPort);
static inline 




result_t ADCM$triggerConversion(uint8_t port);
static   
#line 136
result_t ADCM$ADC$getData(uint8_t port);
static inline   
#line 170
void ADCM$MSP430ADC12Basic$dataReadySingle(uint16_t data);
static inline    







result_t ADCM$ADC$default$dataReady(uint8_t port, uint16_t data);
static   
# 151 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12Advanced.nc"
void MSP430ADC12M$MSP430ADC12Advanced$dataReadySequence(
# 49 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
uint8_t arg_0xa5c2898);
static   
# 145 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12Advanced.nc"
void MSP430ADC12M$MSP430ADC12Advanced$dataReadySingleRpt(
# 49 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
uint8_t arg_0xa5c2898, 
# 145 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12Advanced.nc"
uint16_t arg_0xa5c4cb8);
static   
#line 157
void MSP430ADC12M$MSP430ADC12Advanced$dataReadySequenceRpt(
# 49 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
uint8_t arg_0xa5c2898);
static   
# 42 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void MSP430ADC12M$CompareB0$setEventFromPrev(uint16_t arg_0xa41b240);
static   void MSP430ADC12M$CompareB0$setEventFromNow(uint16_t arg_0xa41b658);
static   
#line 37
void MSP430ADC12M$CompareB0$disableEvents(void);
static   
#line 33
void MSP430ADC12M$CompareB0$setControl(MSP430CompareControl_t arg_0xa425b70);
static   







void MSP430ADC12M$CompareA1$setEventFromPrev(uint16_t arg_0xa41b240);
static   void MSP430ADC12M$CompareA1$setEventFromNow(uint16_t arg_0xa41b658);
static   
#line 37
void MSP430ADC12M$CompareA1$disableEvents(void);
static   
#line 33
void MSP430ADC12M$CompareA1$setControl(MSP430CompareControl_t arg_0xa425b70);
static   
# 118 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVolt.nc"
RefVolt_t MSP430ADC12M$RefVolt$getState(void);
static   
#line 109
result_t MSP430ADC12M$RefVolt$release(void);
static   
#line 93
result_t MSP430ADC12M$RefVolt$get(RefVolt_t arg_0xa5eb538);
static   
# 42 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void MSP430ADC12M$CompareB1$setEventFromPrev(uint16_t arg_0xa41b240);
static   void MSP430ADC12M$CompareB1$setEventFromNow(uint16_t arg_0xa41b658);
static   
#line 37
void MSP430ADC12M$CompareB1$disableEvents(void);
static   
#line 33
void MSP430ADC12M$CompareB1$setControl(MSP430CompareControl_t arg_0xa425b70);
static   
# 58 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
void MSP430ADC12M$HPLADC12$resetIFGs(void);
static   
#line 43
void MSP430ADC12M$HPLADC12$setControl1(adc12ctl1_t arg_0xa5bc198);
static   
#line 80
void MSP430ADC12M$HPLADC12$disableConversion(void);
static   
#line 48
void MSP430ADC12M$HPLADC12$setControl0_IgnoreRef(adc12ctl0_t arg_0xa5bcb60);
static   

adc12memctl_t MSP430ADC12M$HPLADC12$getMemControl(uint8_t arg_0xa5bd4f0);
static   
#line 81
void MSP430ADC12M$HPLADC12$startConversion(void);
static   
#line 52
uint16_t MSP430ADC12M$HPLADC12$getMem(uint8_t arg_0xa5bd908);
static   

void MSP430ADC12M$HPLADC12$setIEFlags(uint16_t arg_0xa5bdd20);
static   
#line 69
void MSP430ADC12M$HPLADC12$setSHT(uint8_t arg_0xa5dba80);
static   
#line 50
void MSP430ADC12M$HPLADC12$setMemControl(uint8_t arg_0xa5bcf70, adc12memctl_t arg_0xa5bd0c8);
static   
#line 82
void MSP430ADC12M$HPLADC12$stopConversion(void);
static   
# 99 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12Basic.nc"
void MSP430ADC12M$MSP430ADC12Basic$dataReadySingle(
# 48 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
uint8_t arg_0xa5c2158, 
# 99 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12Basic.nc"
uint16_t arg_0xa59d600);
 
# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
MSP430ADC12ConversionMode_t MSP430ADC12M$cmode;
 uint16_t *MSP430ADC12M$seqResultPtr;
 uint8_t MSP430ADC12M$owner;
 bool MSP430ADC12M$cancelled;
uint8_t MSP430ADC12M$queueOffset;
 aBuffer_t MSP430ADC12M$aCmdBuffer;
 uint16_t MSP430ADC12M$advancedInterval;
 MSP430ADC12Timer MSP430ADC12M$advancedTimer;
 

aSettings_t MSP430ADC12M$aSettings[0];
 
bSettings_t MSP430ADC12M$bSettings[1];
static 
#line 106
__inline bool MSP430ADC12M$getLockBasic(bSettings_t *settings);
static 









__inline void MSP430ADC12M$releaseLockBasic(bSettings_t *settings);
static inline   



result_t MSP430ADC12M$MSP430ADC12Basic$bind(uint8_t num, MSP430ADC12StandardSettings_ut settings);
static   
#line 162
adcResult_t MSP430ADC12M$MSP430ADC12Basic$getSingleData(uint8_t num);
static 
#line 241
__inline bool MSP430ADC12M$getLockAdvanced(aSettings_t *settings);
static 









__inline void MSP430ADC12M$releaseLockAdvanced(aSettings_t *settings);
static 
#line 278
__inline adcResult_t MSP430ADC12M$getRefVoltAdvanced(uint8_t num);
static 
#line 306
__inline result_t MSP430ADC12M$releaseRefVoltAdvanced(uint8_t num);
static 
#line 319
__inline void MSP430ADC12M$setTimerAdvanced(uint8_t shs);
static inline   
#line 366
adcResult_t MSP430ADC12M$MSP430ADC12Advanced$getSingleDataRepeat(uint8_t num, uint16_t jiffies);
static 
#line 455
adcResult_t MSP430ADC12M$internalGetSequenceData(bool repeat, uint8_t num, uint16_t dataDest[], uint8_t length, uint16_t jiffies);
static inline   
#line 589
void MSP430ADC12M$CompareA1$fired(void);
static   
#line 610
void MSP430ADC12M$CompareB0$fired(void);
static inline   
#line 631
void MSP430ADC12M$CompareB1$fired(void);
static  
#line 663
void MSP430ADC12M$checkQueue(void);
static inline    
void MSP430ADC12M$MSP430ADC12Basic$default$dataReadySingle(uint8_t num, uint16_t x);
static inline    
void MSP430ADC12M$MSP430ADC12Advanced$default$dataReadySequence(uint8_t num);
static inline    void MSP430ADC12M$MSP430ADC12Advanced$default$dataReadySingleRpt(uint8_t num, uint16_t data);
static inline    void MSP430ADC12M$MSP430ADC12Advanced$default$dataReadySequenceRpt(uint8_t num);
static inline  
void MSP430ADC12M$RefVolt$isStable(RefVolt_t vref);
static  



void MSP430ADC12M$checkQueue(void);
static 
#line 758
__inline void MSP430ADC12M$disableTimers(void);
static inline  
#line 772
void MSP430ADC12M$conversionCancelled(void);
static inline   
#line 786
void MSP430ADC12M$HPLADC12$converted(uint8_t number);
static inline   
#line 860
void MSP430ADC12M$HPLADC12$memOverflow(void);
static inline   void MSP430ADC12M$HPLADC12$timeOverflow(void);
static   
# 61 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
void HPLADC12M$HPLADC12$memOverflow(void);
static   
void HPLADC12M$HPLADC12$converted(uint8_t arg_0xa5dafa0);
static   
#line 62
void HPLADC12M$HPLADC12$timeOverflow(void);
 
# 44 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
static volatile uint16_t HPLADC12M$ADC12CTL0 __asm ("0x01A0");
 static volatile uint16_t HPLADC12M$ADC12CTL1 __asm ("0x01A2");
 static volatile uint16_t HPLADC12M$ADC12IFG __asm ("0x01A4");
 static volatile uint16_t HPLADC12M$ADC12IE __asm ("0x01A6");
 static volatile uint16_t HPLADC12M$ADC12IV __asm ("0x01A8");
static inline   




void HPLADC12M$HPLADC12$setControl1(adc12ctl1_t control1);
static   


void HPLADC12M$HPLADC12$setControl0_IgnoreRef(adc12ctl0_t control0);
static   
#line 73
void HPLADC12M$HPLADC12$setMemControl(uint8_t i, adc12memctl_t memControl);
static   






adc12memctl_t HPLADC12M$HPLADC12$getMemControl(uint8_t i);
static inline   








uint16_t HPLADC12M$HPLADC12$getMem(uint8_t i);
static inline   


void HPLADC12M$HPLADC12$setIEFlags(uint16_t mask);
static   

void HPLADC12M$HPLADC12$resetIFGs(void);
static inline   
#line 112
bool HPLADC12M$HPLADC12$isBusy(void);
static inline   

void HPLADC12M$HPLADC12$disableConversion(void);
static inline   void HPLADC12M$HPLADC12$startConversion(void);
static inline   void HPLADC12M$HPLADC12$stopConversion(void);
static inline   
#line 137
void HPLADC12M$HPLADC12$setRefOn(void);
static inline   void HPLADC12M$HPLADC12$setRefOff(void);
static inline   
void HPLADC12M$HPLADC12$setRef1_5V(void);
static inline   void HPLADC12M$HPLADC12$setRef2_5V(void);
static   

void HPLADC12M$HPLADC12$setSHT(uint8_t sht);
#line 163
void __attribute((interrupt(14))) __attribute((wakeup))   sig_ADC_VECTOR(void);
static  
# 28 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
result_t RefVoltM$SwitchOffTimer$setOneShot(int32_t arg_0xa538c18);
static  
# 127 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVolt.nc"
void RefVoltM$RefVolt$isStable(RefVolt_t arg_0xa5cea80);
static   
# 73 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
void RefVoltM$HPLADC12$setRefOff(void);
static   
#line 65
bool RefVoltM$HPLADC12$isBusy(void);
static   









void RefVoltM$HPLADC12$setRef2_5V(void);
static   


void RefVoltM$HPLADC12$disableConversion(void);
static   
#line 72
void RefVoltM$HPLADC12$setRefOn(void);
static   

void RefVoltM$HPLADC12$setRef1_5V(void);
static  
# 28 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
result_t RefVoltM$SwitchOnTimer$setOneShot(int32_t arg_0xa538c18);
# 84 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVoltM.nc"
enum RefVoltM$__nesc_unnamed4296 {

  RefVoltM$REFERENCE_OFF, 
  RefVoltM$REFERENCE_1_5V_PENDING, 
  RefVoltM$REFERENCE_2_5V_PENDING, 
  RefVoltM$REFERENCE_1_5V_STABLE, 
  RefVoltM$REFERENCE_2_5V_STABLE
};
 
uint8_t RefVoltM$semaCount;
 uint8_t RefVoltM$state;
 bool RefVoltM$switchOff;
static 
__inline void RefVoltM$switchRefOn(uint8_t vref);
static __inline void RefVoltM$switchRefOff(void);
static __inline void RefVoltM$switchToRefStable(uint8_t vref);
static __inline void RefVoltM$switchToRefPending(uint8_t vref);
static inline  
void RefVoltM$switchOnDelay(void);
static inline  void RefVoltM$switchOffDelay(void);
static inline  void RefVoltM$switchOffRetry(void);
static   
result_t RefVoltM$RefVolt$get(RefVolt_t vref);
static 
#line 140
__inline void RefVoltM$switchRefOn(uint8_t vref);
static 
#line 154
__inline void RefVoltM$switchToRefPending(uint8_t vref);
static 


__inline void RefVoltM$switchToRefStable(uint8_t vref);
static inline  


void RefVoltM$switchOnDelay(void);
static inline  


result_t RefVoltM$SwitchOnTimer$fired(void);
static   
#line 180
result_t RefVoltM$RefVolt$release(void);
static 
#line 205
__inline void RefVoltM$switchRefOff(void);
static inline  
#line 225
void RefVoltM$switchOffDelay(void);
static inline  



void RefVoltM$switchOffRetry(void);
static inline  



result_t RefVoltM$SwitchOffTimer$fired(void);
static   



RefVolt_t RefVoltM$RefVolt$getState(void);
static inline   






void RefVoltM$HPLADC12$memOverflow(void);
static inline   void RefVoltM$HPLADC12$timeOverflow(void);
static inline   void RefVoltM$HPLADC12$converted(uint8_t number);
static  
# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
TOS_MsgPtr AMStandard$ReceiveMsg$receive(
# 56 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
uint8_t arg_0xa684e80, 
# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
TOS_MsgPtr arg_0xa511178);
static  
# 59 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Timer.nc"
result_t AMStandard$ActivityTimer$start(char arg_0xa4efc10, uint32_t arg_0xa4efd68);
static  
# 58 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
result_t AMStandard$UARTSend$send(TOS_MsgPtr arg_0xa6a0970);
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
result_t AMStandard$RadioSend$send(TOS_MsgPtr arg_0xa6a0970);
static  
# 49 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SendMsg.nc"
result_t AMStandard$SendMsg$sendDone(
# 55 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
uint8_t arg_0xa6848c8, 
# 49 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SendMsg.nc"
TOS_MsgPtr arg_0xa529ce8, result_t arg_0xa529e38);
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
static inline  
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
# 6 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/TimerJiffyAsync.nc"
result_t CC2420RadioM$BackoffTimerJiffy$setOneShot(uint32_t arg_0xa6ff210);
static   


bool CC2420RadioM$BackoffTimerJiffy$isSet(void);
static   
#line 8
result_t CC2420RadioM$BackoffTimerJiffy$stop(void);
static  
# 67 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
result_t CC2420RadioM$Send$sendDone(TOS_MsgPtr arg_0xa6a0e88, result_t arg_0xa6a0fd8);
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
TOS_MsgPtr CC2420RadioM$Receive$receive(TOS_MsgPtr arg_0xa511178);
static   
# 42 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
result_t CC2420RadioM$HPLChipcon$enableFIFOP(void);
static   
#line 66
uint16_t CC2420RadioM$HPLChipcon$read(uint8_t arg_0xa6dcd48);
static   
#line 52
uint8_t CC2420RadioM$HPLChipcon$cmd(uint8_t arg_0xa6dc2f8);
static  
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
result_t CC2420RadioM$CC2420StdControl$init(void);
static  





result_t CC2420RadioM$CC2420StdControl$start(void);
static   
# 27 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
result_t CC2420RadioM$HPLChipconFIFO$writeTXFIFO(uint8_t arg_0xa6fc9b8, uint8_t *arg_0xa6fcb18);
static   
#line 17
result_t CC2420RadioM$HPLChipconFIFO$readRXFIFO(uint8_t arg_0xa6fc280, uint8_t *arg_0xa6fc3e0);
static   
# 144 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420Control.nc"
result_t CC2420RadioM$CC2420Control$RxMode(void);
static   
# 74 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/MacBackoff.nc"
int16_t CC2420RadioM$MacBackoff$initialBackoff(TOS_MsgPtr arg_0xa6c9b18);
static   int16_t CC2420RadioM$MacBackoff$congestionBackoff(TOS_MsgPtr arg_0xa6c9f40);
# 73 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
enum CC2420RadioM$__nesc_unnamed4297 {
  CC2420RadioM$DISABLED_STATE = 0, 
  CC2420RadioM$IDLE_STATE, 
  CC2420RadioM$TX_STATE, 
  CC2420RadioM$PRE_TX_STATE, 
  CC2420RadioM$POST_TX_STATE, 
  CC2420RadioM$POST_TX_ACK_STATE, 
  CC2420RadioM$RX_STATE, 
  CC2420RadioM$POWER_DOWN_STATE, 

  CC2420RadioM$TIMER_INITIAL = 0, 
  CC2420RadioM$TIMER_BACKOFF, 
  CC2420RadioM$TIMER_ACK
};
 


uint8_t CC2420RadioM$countRetry;
uint8_t CC2420RadioM$stateRadio;
 uint8_t CC2420RadioM$stateTimer;
 uint8_t CC2420RadioM$currentDSN;
 bool CC2420RadioM$bAckEnable;
uint16_t CC2420RadioM$txlength;
uint16_t CC2420RadioM$rxlength;
 TOS_MsgPtr CC2420RadioM$txbufptr;
 TOS_MsgPtr CC2420RadioM$rxbufptr;
TOS_Msg CC2420RadioM$RxBuf;








volatile uint16_t CC2420RadioM$LocalAddr;
static 




void CC2420RadioM$sendFailed(void);
static 




__inline result_t CC2420RadioM$setInitialTimer(uint16_t jiffy);
static 



__inline result_t CC2420RadioM$setBackoffTimer(uint16_t jiffy);
static 



__inline result_t CC2420RadioM$setAckTimer(uint16_t jiffy);
static inline  







void CC2420RadioM$PacketRcvd(void);
static  
#line 154
void CC2420RadioM$PacketSent(void);
static inline  
#line 171
result_t CC2420RadioM$StdControl$init(void);
static inline  
#line 198
result_t CC2420RadioM$StdControl$start(void);
static inline 
#line 219
void CC2420RadioM$sendPacket(void);
static inline  
#line 245
void CC2420RadioM$startSend(void);
static 









void CC2420RadioM$tryToSend(void);
static inline   
#line 282
result_t CC2420RadioM$BackoffTimerJiffy$fired(void);
static inline  
#line 315
result_t CC2420RadioM$Send$send(TOS_MsgPtr pMsg);
static inline  
#line 348
void CC2420RadioM$delayedRXFIFO(void);
static inline   
#line 368
result_t CC2420RadioM$HPLChipcon$FIFOPIntr(void);
static inline   
#line 393
result_t CC2420RadioM$HPLChipconFIFO$RXFIFODone(uint8_t length, uint8_t *data);
static inline   
#line 439
result_t CC2420RadioM$HPLChipconFIFO$TXFIFODone(uint8_t length, uint8_t *data);
static inline    
#line 460
int16_t CC2420RadioM$MacBackoff$default$initialBackoff(TOS_MsgPtr m);
static inline    




int16_t CC2420RadioM$MacBackoff$default$congestionBackoff(TOS_MsgPtr m);
static   
# 66 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
uint16_t CC2420ControlM$HPLChipcon$read(uint8_t arg_0xa6dcd48);
static   
#line 59
uint8_t CC2420ControlM$HPLChipcon$write(uint8_t arg_0xa6dc750, uint16_t arg_0xa6dc8a0);
static   
#line 52
uint8_t CC2420ControlM$HPLChipcon$cmd(uint8_t arg_0xa6dc2f8);
static  
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
result_t CC2420ControlM$HPLChipconControl$init(void);
static  





result_t CC2420ControlM$HPLChipconControl$start(void);
static   
# 47 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
result_t CC2420ControlM$HPLChipconRAM$write(uint16_t arg_0xa71e180, uint8_t arg_0xa71e2c8, uint8_t *arg_0xa71e428);
 
# 62 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
uint16_t CC2420ControlM$gCurrentParameters[14];
static inline 






bool CC2420ControlM$SetRegs(void);
static inline  
#line 102
result_t CC2420ControlM$StdControl$init(void);
static inline  
#line 174
result_t CC2420ControlM$StdControl$start(void);
static inline  
#line 205
result_t CC2420ControlM$CC2420Control$TunePreset(uint8_t chnl);
static inline   
#line 257
result_t CC2420ControlM$CC2420Control$RxMode(void);
static inline   
#line 282
result_t CC2420ControlM$CC2420Control$OscillatorOn(void);
static inline   
#line 307
result_t CC2420ControlM$CC2420Control$VREFOn(void);
static inline  
#line 338
result_t CC2420ControlM$CC2420Control$setShortAddress(uint16_t addr);
static inline   



result_t CC2420ControlM$HPLChipcon$FIFOPIntr(void);
static inline   






result_t CC2420ControlM$HPLChipconRAM$writeDone(uint16_t addr, uint8_t length, uint8_t *buffer);
static   
# 38 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/BusArbitration.nc"
result_t HPLCC2420M$BusArbitration$releaseBus(void);
static   
#line 37
result_t HPLCC2420M$BusArbitration$getBus(void);
static   
# 45 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
result_t HPLCC2420M$HPLCC2420$FIFOPIntr(void);
static   
# 48 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
result_t HPLCC2420M$HPLCC2420FIFO$TXFIFODone(uint8_t arg_0xa6fd6c8, uint8_t *arg_0xa6fd828);
static   
#line 37
result_t HPLCC2420M$HPLCC2420FIFO$RXFIFODone(uint8_t arg_0xa6fd038, uint8_t *arg_0xa6fd198);
static   
# 112 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
result_t HPLCC2420M$USARTControl$disableRxIntr(void);
static   
#line 87
void HPLCC2420M$USARTControl$setModeSPI(void);
static   
#line 113
result_t HPLCC2420M$USARTControl$disableTxIntr(void);
static   





result_t HPLCC2420M$USARTControl$isTxIntrPending(void);
static   
#line 142
result_t HPLCC2420M$USARTControl$tx(uint8_t arg_0xa7709d0);
static   





uint8_t HPLCC2420M$USARTControl$rx(void);
static   
# 49 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
result_t HPLCC2420M$HPLCC2420RAM$writeDone(uint16_t arg_0xa71e8a8, uint8_t arg_0xa71e9f0, uint8_t *arg_0xa71eb50);
static   
# 29 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
void HPLCC2420M$FIFOPInterrupt$clear(void);
static   
#line 28
void HPLCC2420M$FIFOPInterrupt$disable(void);
static   
void HPLCC2420M$FIFOPInterrupt$edge(bool arg_0xa7797e0);
static   
#line 27
void HPLCC2420M$FIFOPInterrupt$enable(void);
 
# 58 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/HPLCC2420M.nc"
uint8_t *HPLCC2420M$rxbuf;
 uint8_t *HPLCC2420M$txbuf;
 uint8_t *HPLCC2420M$rambuf;
 
uint8_t HPLCC2420M$txlen;
 uint8_t HPLCC2420M$rxlen;
 uint8_t HPLCC2420M$ramlen;
 uint16_t HPLCC2420M$ramaddr;
static inline  


result_t HPLCC2420M$StdControl$init(void);
static inline  







result_t HPLCC2420M$StdControl$start(void);
static inline   
#line 96
result_t HPLCC2420M$HPLCC2420$enableFIFOP(void);
static   
#line 125
uint8_t HPLCC2420M$HPLCC2420$cmd(uint8_t addr);
static   
#line 146
uint8_t HPLCC2420M$HPLCC2420$write(uint8_t addr, uint16_t data);
static   
#line 171
uint16_t HPLCC2420M$HPLCC2420$read(uint8_t addr);
static inline  
#line 232
void HPLCC2420M$signalRAMWr(void);
static inline   


result_t HPLCC2420M$HPLCC2420RAM$write(uint16_t addr, uint8_t length, uint8_t *buffer);
static inline  
#line 264
void HPLCC2420M$signalRXFIFO(void);
static inline   
#line 279
result_t HPLCC2420M$HPLCC2420FIFO$readRXFIFO(uint8_t length, uint8_t *data);
static inline  
#line 320
void HPLCC2420M$signalTXFIFO(void);
static inline   
#line 332
result_t HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(uint8_t length, uint8_t *data);
static inline   
#line 358
void HPLCC2420M$FIFOPInterrupt$fired(void);
static inline  




result_t HPLCC2420M$BusArbitration$busFree(void);
static   
# 53 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
result_t HPLUSART0M$USARTData$rxDone(uint8_t arg_0xa7a28e8);
static   
#line 46
result_t HPLUSART0M$USARTData$txDone(void);
 
# 46 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
static volatile uint8_t HPLUSART0M$ME1 __asm ("0x0004");
 static volatile uint8_t HPLUSART0M$IFG1 __asm ("0x0002");
 static volatile uint8_t HPLUSART0M$U0TCTL __asm ("0x0071");

uint16_t HPLUSART0M$l_br;

uint8_t HPLUSART0M$l_ssel;

void __attribute((interrupt(18))) __attribute((wakeup))   sig_UART0RX_VECTOR(void);




void __attribute((interrupt(16))) __attribute((wakeup))   sig_UART0TX_VECTOR(void);
static   
#line 102
void HPLUSART0M$USARTControl$setModeSPI(void);
static inline   
#line 232
result_t HPLUSART0M$USARTControl$isTxIntrPending(void);
static inline   
#line 255
result_t HPLUSART0M$USARTControl$disableRxIntr(void);
static inline   



result_t HPLUSART0M$USARTControl$disableTxIntr(void);
static inline   
#line 281
result_t HPLUSART0M$USARTControl$tx(uint8_t data);
static inline   



uint8_t HPLUSART0M$USARTControl$rx(void);
static inline    




result_t HPLUSART0M$USARTData$default$txDone(void);
static inline    
result_t HPLUSART0M$USARTData$default$rxDone(uint8_t data);
static   
# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
void MSP430InterruptM$Port14$fired(void);
static   
#line 31
void MSP430InterruptM$Port26$fired(void);
static   
#line 31
void MSP430InterruptM$Port17$fired(void);
static   
#line 31
void MSP430InterruptM$Port21$fired(void);
static   
#line 31
void MSP430InterruptM$Port12$fired(void);
static   
#line 31
void MSP430InterruptM$Port24$fired(void);
static   
#line 31
void MSP430InterruptM$Port15$fired(void);
static   
#line 31
void MSP430InterruptM$Port27$fired(void);
static   
#line 31
void MSP430InterruptM$Port10$fired(void);
static   
#line 31
void MSP430InterruptM$Port22$fired(void);
static   
#line 31
void MSP430InterruptM$Port13$fired(void);
static   
#line 31
void MSP430InterruptM$Port25$fired(void);
static   
#line 31
void MSP430InterruptM$Port16$fired(void);
static   
#line 31
void MSP430InterruptM$Port20$fired(void);
static   
#line 31
void MSP430InterruptM$Port11$fired(void);
static   
#line 31
void MSP430InterruptM$Port23$fired(void);
# 47 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void __attribute((interrupt(8))) __attribute((wakeup))   sig_PORT1_VECTOR(void);
#line 62
void __attribute((interrupt(2))) __attribute((wakeup))   sig_PORT2_VECTOR(void);
static inline    
#line 77
void MSP430InterruptM$Port11$default$fired(void);
static inline    void MSP430InterruptM$Port12$default$fired(void);
static inline    void MSP430InterruptM$Port13$default$fired(void);
static inline    void MSP430InterruptM$Port14$default$fired(void);
static inline    void MSP430InterruptM$Port15$default$fired(void);
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
void MSP430InterruptM$Port10$enable(void);
static inline   
#line 112
void MSP430InterruptM$Port10$disable(void);
static inline   
#line 130
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
void MSP430InterruptM$Port10$edge(bool l2h);
static  
# 39 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/BusArbitration.nc"
result_t BusArbitrationM$BusArbitration$busFree(
# 38 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/BusArbitrationM.nc"
uint8_t arg_0xa820560);





uint8_t BusArbitrationM$state;
uint8_t BusArbitrationM$busid;
bool BusArbitrationM$isBusReleasedPending;
enum BusArbitrationM$__nesc_unnamed4298 {
#line 47
  BusArbitrationM$BUS_IDLE, BusArbitrationM$BUS_BUSY, BusArbitrationM$BUS_OFF
};
static inline  void BusArbitrationM$busReleased(void);
static   
#line 85
result_t BusArbitrationM$BusArbitration$getBus(uint8_t id);
static   
#line 99
result_t BusArbitrationM$BusArbitration$releaseBus(uint8_t id);
static inline   
#line 116
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
# 12 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/TimerJiffyAsync.nc"
result_t TimerJiffyAsyncM$TimerJiffyAsync$fired(void);
static   
# 36 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
void TimerJiffyAsyncM$Alarm$enableEvents(void);
static   
#line 34
void TimerJiffyAsyncM$Alarm$setControlAsTimer(void);
static   







void TimerJiffyAsyncM$Alarm$setEventFromNow(uint16_t arg_0xa41b658);
static   
#line 37
void TimerJiffyAsyncM$Alarm$disableEvents(void);
static   
#line 31
void TimerJiffyAsyncM$Alarm$clearPendingInterrupt(void);
# 12 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/TimerJiffyAsyncM.nc"
uint32_t TimerJiffyAsyncM$jiffy;
bool TimerJiffyAsyncM$bSet;
static inline  
result_t TimerJiffyAsyncM$StdControl$init(void);
static inline  





result_t TimerJiffyAsyncM$StdControl$start(void);
static inline   
#line 40
void TimerJiffyAsyncM$Alarm$fired(void);
static   
#line 55
result_t TimerJiffyAsyncM$TimerJiffyAsync$setOneShot(uint32_t _jiffy);
static inline   
#line 73
bool TimerJiffyAsyncM$TimerJiffyAsync$isSet(void);
static inline   



result_t TimerJiffyAsyncM$TimerJiffyAsync$stop(void);
static  
# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
TOS_MsgPtr FramerM$ReceiveMsg$receive(TOS_MsgPtr arg_0xa511178);
static   
# 55 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ByteComm.nc"
result_t FramerM$ByteComm$txByte(uint8_t arg_0xa87c3a8);
static  
# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
result_t FramerM$ByteControl$init(void);
static  





result_t FramerM$ByteControl$start(void);
static  
# 67 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
result_t FramerM$BareSendMsg$sendDone(TOS_MsgPtr arg_0xa6a0e88, result_t arg_0xa6a0fd8);
static  
# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/TokenReceiveMsg.nc"
TOS_MsgPtr FramerM$TokenReceiveMsg$receive(TOS_MsgPtr arg_0xa84c0b0, uint8_t arg_0xa84c1f8);
# 82 "C:/cygwin/opt/tinyos-1.x/tos/system/FramerM.nc"
enum FramerM$__nesc_unnamed4299 {
  FramerM$HDLC_QUEUESIZE = 2, 
  FramerM$HDLC_MTU = sizeof(TOS_Msg ), 
  FramerM$HDLC_FLAG_BYTE = 0x7e, 
  FramerM$HDLC_CTLESC_BYTE = 0x7d, 
  FramerM$PROTO_ACK = 64, 
  FramerM$PROTO_PACKET_ACK = 65, 
  FramerM$PROTO_PACKET_NOACK = 66, 
  FramerM$PROTO_UNKNOWN = 255
};

enum FramerM$__nesc_unnamed4300 {
  FramerM$RXSTATE_NOSYNC, 
  FramerM$RXSTATE_PROTO, 
  FramerM$RXSTATE_TOKEN, 
  FramerM$RXSTATE_INFO, 
  FramerM$RXSTATE_ESC
};

enum FramerM$__nesc_unnamed4301 {
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

enum FramerM$__nesc_unnamed4302 {
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
TOS_MsgPtr FramerAckM$ReceiveCombined$receive(TOS_MsgPtr arg_0xa511178);
static  
# 88 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/TokenReceiveMsg.nc"
result_t FramerAckM$TokenReceiveMsg$ReflectToken(uint8_t arg_0xa84c810);
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
result_t UARTM$HPLUART$put(uint8_t arg_0xa8b43e0);
static   
# 83 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ByteComm.nc"
result_t UARTM$ByteComm$txDone(void);
static   
#line 75
result_t UARTM$ByteComm$txByteReady(bool arg_0xa87d008);
static   
#line 66
result_t UARTM$ByteComm$rxByteReady(uint8_t arg_0xa87c838, bool arg_0xa87c980, uint16_t arg_0xa87cad8);
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
result_t HPLUARTM$UART$get(uint8_t arg_0xa8b48e0);
static   






result_t HPLUARTM$UART$putDone(void);
static   
# 109 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
void HPLUARTM$USARTControl$setClockRate(uint16_t arg_0xa772fb0, uint8_t arg_0xa7730f8);
static   
#line 107
void HPLUARTM$USARTControl$setClockSource(uint8_t arg_0xa772b10);
static   





result_t HPLUARTM$USARTControl$enableRxIntr(void);
static   result_t HPLUARTM$USARTControl$enableTxIntr(void);
static   
#line 142
result_t HPLUARTM$USARTControl$tx(uint8_t arg_0xa7709d0);
static   
#line 105
void HPLUARTM$USARTControl$setModeUART(void);
static inline   
# 50 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
result_t HPLUARTM$UART$init(void);
static inline   
#line 72
result_t HPLUARTM$USARTData$rxDone(uint8_t b);
static inline   


result_t HPLUARTM$USARTData$txDone(void);
static inline   


result_t HPLUARTM$UART$put(uint8_t data);
static   
# 53 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
result_t HPLUSART1M$USARTData$rxDone(uint8_t arg_0xa7a28e8);
static   
#line 46
result_t HPLUSART1M$USARTData$txDone(void);
 
# 46 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
static volatile uint8_t HPLUSART1M$ME2 __asm ("0x0005");
 static volatile uint8_t HPLUSART1M$IFG2 __asm ("0x0003");
 static volatile uint8_t HPLUSART1M$U1TCTL __asm ("0x0079");

uint16_t HPLUSART1M$l_br;
uint8_t HPLUSART1M$l_mctl;
uint8_t HPLUSART1M$l_ssel;

void __attribute((interrupt(6))) __attribute((wakeup))   sig_UART1RX_VECTOR(void);




void __attribute((interrupt(4))) __attribute((wakeup))   sig_UART1TX_VECTOR(void);
static inline 
#line 146
void HPLUSART1M$setUARTModeCommon(void);
static inline   
#line 205
void HPLUSART1M$USARTControl$setModeUART(void);
static inline   







void HPLUSART1M$USARTControl$setClockSource(uint8_t source);
static inline   






void HPLUSART1M$USARTControl$setClockRate(uint16_t baudrate, uint8_t mctl);
static inline   
#line 265
result_t HPLUSART1M$USARTControl$enableRxIntr(void);
static inline   






result_t HPLUSART1M$USARTControl$enableTxIntr(void);
static inline   






result_t HPLUSART1M$USARTControl$tx(uint8_t data);
static inline   
# 54 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLPowerManagementM.nc"
uint8_t HPLPowerManagementM$PowerManagement$adjustPower(void);
static inline 
# 132 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
void MSP430ClockM$startTimerB(void)
{

  MSP430ClockM$TBCTL = 0x0020 | (MSP430ClockM$TBCTL & ~(0x0020 | 0x0010));
}

static inline 
#line 120
void MSP430ClockM$startTimerA(void)
{

  MSP430ClockM$TACTL = 0x0020 | (MSP430ClockM$TACTL & ~(0x0020 | 0x0010));
}

static inline  
#line 264
result_t MSP430ClockM$StdControl$start(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      MSP430ClockM$startTimerA();
      MSP430ClockM$startTimerB();
    }
#line 270
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
# 89 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
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
# 74 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
void MSP430ClockM$MSP430ClockInit$defaultInitTimerA(void)
{
  TAR = 0;









  MSP430ClockM$TACTL = 0x0200 | 0x0002;
}

static inline   
#line 109
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
# 53 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
void MSP430ClockM$MSP430ClockInit$defaultInitClocks(void)
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  MSP430ClockM$IE1 &= ~(1 << 1);
}

static inline   
#line 104
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
# 170 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
uint16_t MSP430ClockM$busywait_delta(void)
{
  while (MSP430ClockM$m_aclk_count != 0) {
    }
#line 173
  return MSP430ClockM$m_dco_curr - MSP430ClockM$m_dco_prev;
}

static inline   
# 295 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB2$setEventFromNow(uint16_t x)
#line 295
{
#line 295
  MSP430TimerM$TBCCR2 = TBR + x;
}

# 43 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ClockM$ACLKCompare$setEventFromNow(uint16_t arg_0xa41b658){
#line 43
  MSP430TimerM$CompareB2$setEventFromNow(arg_0xa41b658);
#line 43
}
#line 43
static inline 
# 157 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
void MSP430ClockM$set_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

static inline void MSP430ClockM$test_calib(int calib)
{
  MSP430ClockM$set_calib(calib);
  MSP430ClockM$m_aclk_count = 2;
  MSP430ClockM$ACLKCompare$setEventFromNow(MSP430ClockM$ACLK_CALIB_PERIOD);
}

static inline 





uint16_t MSP430ClockM$test_calib_busywait_delta(int calib)
{
  MSP430ClockM$test_calib(calib);
  return MSP430ClockM$busywait_delta();
}

static inline   
# 247 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB2$enableEvents(void)
#line 247
{
#line 247
  MSP430TimerM$TBCCTL2 |= 0x0010;
}

# 36 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ClockM$ACLKCompare$enableEvents(void){
#line 36
  MSP430TimerM$CompareB2$enableEvents();
#line 36
}
#line 36
static inline 
# 71 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$CC2int(MSP430TimerM$CC_t cc)
{
  typedef union __nesc_unnamed4303 {
#line 73
    MSP430TimerM$CC_t x;
#line 73
    uint16_t i;
  } 
#line 73
  convert_t;
  convert_t a = { .x = cc };

#line 75
  return a.i;
}

static inline uint16_t MSP430TimerM$timerControl(void)
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
#line 239
void MSP430TimerM$CompareB2$setControlAsTimer(void)
#line 239
{
#line 239
  MSP430TimerM$TBCCTL2 = MSP430TimerM$timerControl();
}

# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ClockM$ACLKCompare$setControlAsTimer(void){
#line 34
  MSP430TimerM$CompareB2$setControlAsTimer();
#line 34
}
#line 34
static inline 
# 183 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
void MSP430ClockM$busyCalibrateDCO(void)
{

  int calib;
  int step;





  MSP430ClockM$m_aclk_count = 0;
  MSP430ClockM$TACTL = 0x0200 | 0x0020;
  MSP430ClockM$TBCTL = 0x0100 | 0x0020;
  MSP430ClockM$IE1 &= ~(1 << 1);
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TACCTL0 = 0;
  TACCTL1 = 0;
  TACCTL2 = 0;
  TBCCTL0 = 0;
  TBCCTL1 = 0;
  TBCCTL2 = 0;
  TBCCTL3 = 0;
  TBCCTL4 = 0;
  TBCCTL5 = 0;
  TBCCTL6 = 0;
  MSP430ClockM$TBCTL |= 0x0002;
  MSP430ClockM$ACLKCompare$setControlAsTimer();
  MSP430ClockM$ACLKCompare$enableEvents();






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (MSP430ClockM$test_calib_busywait_delta(calib | step) <= MSP430ClockM$TARGET_DCO_DELTA) {
        calib |= step;
        }
    }



  MSP430ClockM$TACTL = 0;
  MSP430ClockM$TBCTL = 0;
  TBCCTL2 = 0;
}

static inline 
# 176 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
bool are_interrupts_enabled(void)
{
  return (({
#line 178
    uint16_t __x;

#line 178
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 178
   & 0x0008) != 0;
}

static inline 
# 233 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
void MSP430ClockM$garnishedBusyCalibrateDCO(void)
{
  bool do_dint;

#line 236
  do_dint = !are_interrupts_enabled();
   __asm volatile ("eint");
  MSP430ClockM$busyCalibrateDCO();
  if (do_dint) {
     __asm volatile ("dint");
    }
}

static inline  
#line 243
result_t MSP430ClockM$StdControl$init(void)
{


  MSP430ClockM$TACTL = 0x0004;
  MSP430ClockM$TAIV = 0;
  MSP430ClockM$TBCTL = 0x0004;
  MSP430ClockM$TBIV = 0;

  MSP430ClockM$garnishedBusyCalibrateDCO();

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      MSP430ClockM$MSP430ClockInit$initClocks();
      MSP430ClockM$MSP430ClockInit$initTimerA();
      MSP430ClockM$MSP430ClockInit$initTimerB();
    }
#line 259
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
# 53 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/hardware.h"
void TOSH_CLR_HUM_PWR_PIN(void)
#line 53
{
   
#line 53
  static volatile uint8_t r __asm ("0x0021");

#line 53
  r &= ~(1 << 7);
}

static inline 
#line 51
void TOSH_CLR_HUM_SDA_PIN(void)
#line 51
{
   
#line 51
  static volatile uint8_t r __asm ("0x0021");

#line 51
  r &= ~(1 << 5);
}

static inline 
#line 52
void TOSH_CLR_HUM_SCL_PIN(void)
#line 52
{
   
#line 52
  static volatile uint8_t r __asm ("0x0021");

#line 52
  r &= ~(1 << 6);
}

static inline 
#line 53
void TOSH_MAKE_HUM_PWR_OUTPUT(void)
#line 53
{
   
#line 53
  static volatile uint8_t r __asm ("0x0022");

#line 53
  r |= 1 << 7;
}

static inline 
#line 51
void TOSH_MAKE_HUM_SDA_OUTPUT(void)
#line 51
{
   
#line 51
  static volatile uint8_t r __asm ("0x0022");

#line 51
  r |= 1 << 5;
}

static inline 
#line 52
void TOSH_MAKE_HUM_SCL_OUTPUT(void)
#line 52
{
   
#line 52
  static volatile uint8_t r __asm ("0x0022");

#line 52
  r |= 1 << 6;
}

static inline 
#line 81
void TOSH_SET_FLASH_CS_PIN(void)
#line 81
{
   
#line 81
  static volatile uint8_t r __asm ("0x001D");

#line 81
  r |= 1 << 4;
}

static inline 
#line 81
void TOSH_MAKE_FLASH_CS_OUTPUT(void)
#line 81
{
   
#line 81
  static volatile uint8_t r __asm ("0x001E");

#line 81
  r |= 1 << 4;
}

static inline 
#line 80
void TOSH_SET_FLASH_PWR_PIN(void)
#line 80
{
   
#line 80
  static volatile uint8_t r __asm ("0x001D");

#line 80
  r |= 1 << 3;
}

static inline 
#line 80
void TOSH_MAKE_FLASH_PWR_OUTPUT(void)
#line 80
{
   
#line 80
  static volatile uint8_t r __asm ("0x001E");

#line 80
  r |= 1 << 3;
}

static inline 


void TOSH_MAKE_PROG_TX_INPUT(void)
#line 86
{
   
#line 86
  static volatile uint8_t r __asm ("0x002A");

#line 86
  r &= ~(1 << 2);
}

static inline 
#line 85
void TOSH_MAKE_PROG_RX_INPUT(void)
#line 85
{
   
#line 85
  static volatile uint8_t r __asm ("0x0022");

#line 85
  r &= ~(1 << 1);
}

static inline 
#line 39
void TOSH_CLR_URXD1_PIN(void)
#line 39
{
   
#line 39
  static volatile uint8_t r __asm ("0x0019");

#line 39
  r &= ~(1 << 7);
}

static inline 
#line 38
void TOSH_CLR_UTXD1_PIN(void)
#line 38
{
   
#line 38
  static volatile uint8_t r __asm ("0x0019");

#line 38
  r &= ~(1 << 6);
}

static inline 
#line 39
void TOSH_MAKE_URXD1_OUTPUT(void)
#line 39
{
   
#line 39
  static volatile uint8_t r __asm ("0x001A");

#line 39
  r |= 1 << 7;
}

static inline 
#line 38
void TOSH_MAKE_UTXD1_OUTPUT(void)
#line 38
{
   
#line 38
  static volatile uint8_t r __asm ("0x001A");

#line 38
  r |= 1 << 6;
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
#line 41
void TOSH_SET_SOMI1_PIN(void)
#line 41
{
   
#line 41
  static volatile uint8_t r __asm ("0x0031");

#line 41
  r |= 1 << 2;
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
#line 88
void TOSH_SET_PIN_DIRECTIONS(void )
{

  TOSH_SET_RED_LED_PIN();
  TOSH_SET_GREEN_LED_PIN();
  TOSH_SET_YELLOW_LED_PIN();
  TOSH_MAKE_RED_LED_OUTPUT();
  TOSH_MAKE_GREEN_LED_OUTPUT();
  TOSH_MAKE_YELLOW_LED_OUTPUT();



  TOSH_SET_SOMI1_PIN();
  TOSH_MAKE_SOMI1_INPUT();


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


  TOSH_MAKE_UTXD1_OUTPUT();
  TOSH_MAKE_URXD1_OUTPUT();
  TOSH_CLR_UTXD1_PIN();
  TOSH_CLR_URXD1_PIN();


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
# 83 "C:/cygwin/opt/tinyos-1.x/tos/system/sched.c"
void TOSH_sched_init(void )
{
  TOSH_sched_free = 0;
  TOSH_sched_full = 0;
}

static inline 
# 120 "C:/cygwin/opt/tinyos-1.x/tos/system/tos.h"
result_t rcombine(result_t r1, result_t r2)



{
  return r1 == FAIL ? FAIL : r2;
}

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
# 257 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB4$disableEvents(void)
#line 257
{
#line 257
  MSP430TimerM$TBCCTL4 &= ~0x0010;
}

# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerJiffyAsyncM$Alarm$disableEvents(void){
#line 37
  MSP430TimerM$CompareB4$disableEvents();
#line 37
}
#line 37
static inline   
# 241 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB4$setControlAsTimer(void)
#line 241
{
#line 241
  MSP430TimerM$TBCCTL4 = MSP430TimerM$timerControl();
}

# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerJiffyAsyncM$Alarm$setControlAsTimer(void){
#line 34
  MSP430TimerM$CompareB4$setControlAsTimer();
#line 34
}
#line 34
static inline  
# 15 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/TimerJiffyAsyncM.nc"
result_t TimerJiffyAsyncM$StdControl$init(void)
{
  TimerJiffyAsyncM$Alarm$setControlAsTimer();
  TimerJiffyAsyncM$Alarm$disableEvents();
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
# 260 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
result_t HPLUSART0M$USARTControl$disableTxIntr(void)
#line 260
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 261
    IE1 &= ~(1 << 7);
#line 261
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 113 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$disableTxIntr(void){
#line 113
  unsigned char result;
#line 113

#line 113
  result = HPLUSART0M$USARTControl$disableTxIntr();
#line 113

#line 113
  return result;
#line 113
}
#line 113
static inline   
# 255 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
result_t HPLUSART0M$USARTControl$disableRxIntr(void)
#line 255
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 256
    IE1 &= ~(1 << 6);
#line 256
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 112 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$disableRxIntr(void){
#line 112
  unsigned char result;
#line 112

#line 112
  result = HPLUSART0M$USARTControl$disableRxIntr();
#line 112

#line 112
  return result;
#line 112
}
#line 112
#line 87
inline static   void HPLCC2420M$USARTControl$setModeSPI(void){
#line 87
  HPLUSART0M$USARTControl$setModeSPI();
#line 87
}
#line 87
static inline  
# 69 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/HPLCC2420M.nc"
result_t HPLCC2420M$StdControl$init(void)
#line 69
{
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
# 102 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
result_t CC2420ControlM$StdControl$init(void)
#line 102
{

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
  1 << 5)) | (0x1f << 0);

  CC2420ControlM$gCurrentParameters[CP_RXCTRL0] = (((((1 << 12) | (
  2 << 8)) | (3 << 6)) | (
  2 << 4)) | (1 << 2)) | (
  1 << 0);

  CC2420ControlM$gCurrentParameters[CP_RXCTRL1] = (((((1 << 11) | (
  1 << 9)) | (1 << 6)) | (
  1 << 4)) | (1 << 2)) | (
  2 << 0);

  CC2420ControlM$gCurrentParameters[CP_FSCTRL] = (1 << 14) | ((
  357 + 5 * (11 - 11)) << 0);

  CC2420ControlM$gCurrentParameters[CP_SECCTRL0] = (((1 << 8) | (
  1 << 7)) | (1 << 6)) | (
  1 << 2);

  CC2420ControlM$gCurrentParameters[CP_SECCTRL1] = 0;
  CC2420ControlM$gCurrentParameters[CP_BATTMON] = 0;



  CC2420ControlM$gCurrentParameters[CP_IOCFG0] = (127 << 0) | (
  1 << 9);

  CC2420ControlM$gCurrentParameters[CP_IOCFG1] = 0;

  return SUCCESS;
}

# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t CC2420RadioM$CC2420StdControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = CC2420ControlM$StdControl$init();
#line 63

#line 63
  return result;
#line 63
}
#line 63
static inline  
# 171 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$StdControl$init(void)
#line 171
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 173
    {
      CC2420RadioM$stateRadio = CC2420RadioM$DISABLED_STATE;
      CC2420RadioM$currentDSN = 0;
      CC2420RadioM$bAckEnable = FALSE;
      CC2420RadioM$rxbufptr = &CC2420RadioM$RxBuf;
      CC2420RadioM$rxbufptr->length = 0;
      CC2420RadioM$rxlength = MSG_DATA_SIZE - 2;
    }
#line 180
    __nesc_atomic_end(__nesc_atomic); }

  CC2420RadioM$CC2420StdControl$init();
  CC2420RadioM$TimerControl$init();
  CC2420RadioM$Random$init();
  CC2420RadioM$LocalAddr = TOS_LOCAL_ADDRESS;

  return SUCCESS;
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
inline static  result_t BSN_RSUM$CommControl$init(void){
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
# 90 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/ADCM.nc"
result_t ADCM$ADCControl$init(void)
{
  if (!ADCM$initialized) {
      ADCM$samplingRate = 0xFF;
      ADCM$initialized = 1;
    }
  return SUCCESS;
}

# 50 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ADCControl.nc"
inline static  result_t AccelTempM$ADCControl$init(void){
#line 50
  unsigned char result;
#line 50

#line 50
  result = ADCM$ADCControl$init();
#line 50

#line 50
  return result;
#line 50
}
#line 50
static inline  
# 115 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/ADCM.nc"
result_t ADCM$ADCControl$bindPort(uint8_t port, uint8_t adcPort)
{
  ADCM$TOSH_adc_portmap[port] = adcPort;
  return SUCCESS;
}

# 89 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ADCControl.nc"
inline static  result_t AccelTempM$ADCControl$bindPort(uint8_t arg_0xa5a6618, uint8_t arg_0xa5a6760){
#line 89
  unsigned char result;
#line 89

#line 89
  result = ADCM$ADCControl$bindPort(arg_0xa5a6618, arg_0xa5a6760);
#line 89

#line 89
  return result;
#line 89
}
#line 89
static inline  
# 19 "AccelTempM.nc"
result_t AccelTempM$StdControl$init(void)
#line 19
{
  AccelTempM$ADCControl$bindPort(3, 3);
  AccelTempM$ADCControl$bindPort(2, 2);
  AccelTempM$ADCControl$bindPort(1, 1);
  {
  }
#line 23
  ;
  return AccelTempM$ADCControl$init();
}

# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t BSN_RSUM$SensorControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = AccelTempM$StdControl$init();
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
inline static   result_t BSN_RSUM$Leds$greenOff(void){
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
inline static   result_t BSN_RSUM$Leds$redOff(void){
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
inline static   result_t BSN_RSUM$Leds$yellowOff(void){
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
inline static   result_t BSN_RSUM$Leds$init(void){
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
# 79 "BSN_RSUM.nc"
result_t BSN_RSUM$StdControl$init(void)
#line 79
{
  BSN_RSUM$Leds$init();
  BSN_RSUM$Leds$yellowOff();
#line 81
  BSN_RSUM$Leds$redOff();
#line 81
  BSN_RSUM$Leds$greenOff();


  BSN_RSUM$SensorControl$init();

  BSN_RSUM$CommControl$init();

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 88
    {
      BSN_RSUM$currentMsg = 0;
      BSN_RSUM$packetReadingNumber = 0;
      BSN_RSUM$readingNumber = 0;
      BSN_RSUM$readwhich = 0;
    }
#line 93
    __nesc_atomic_end(__nesc_atomic); }

  {
  }
#line 95
  ;
  return SUCCESS;
}

# 63 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t MainM$StdControl$init(void){
#line 63
  unsigned char result;
#line 63

#line 63
  result = TimerM$StdControl$init();
#line 63
  result = rcombine(result, BSN_RSUM$StdControl$init());
#line 63
  result = rcombine(result, TimerM$StdControl$init());
#line 63

#line 63
  return result;
#line 63
}
#line 63
static inline   
# 240 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB3$setControlAsTimer(void)
#line 240
{
#line 240
  MSP430TimerM$TBCCTL3 = MSP430TimerM$timerControl();
}

# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerM$AlarmCompare$setControlAsTimer(void){
#line 34
  MSP430TimerM$CompareB3$setControlAsTimer();
#line 34
}
#line 34
static inline   
# 256 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB3$disableEvents(void)
#line 256
{
#line 256
  MSP430TimerM$TBCCTL3 &= ~0x0010;
}

# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerM$AlarmCompare$disableEvents(void){
#line 37
  MSP430TimerM$CompareB3$disableEvents();
#line 37
}
#line 37
static inline 
# 34 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/hardware.h"
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
# 79 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
result_t TimerM$StdControl$start(void)
{
  return SUCCESS;
}

static inline 
# 133 "C:/cygwin/opt/tinyos-1.x/tos/system/tos.h"
result_t rcombine4(result_t r1, result_t r2, result_t r3, 
result_t r4)
{
  return rcombine(r1, rcombine(r2, rcombine(r3, r4)));
}

static inline   
# 54 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLPowerManagementM.nc"
uint8_t HPLPowerManagementM$PowerManagement$adjustPower(void)
#line 54
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
inline static  result_t AMStandard$ActivityTimer$start(char arg_0xa4efc10, uint32_t arg_0xa4efd68){
#line 59
  unsigned char result;
#line 59

#line 59
  result = TimerM$Timer$start(1, arg_0xa4efc10, arg_0xa4efd68);
#line 59

#line 59
  return result;
#line 59
}
#line 59
static inline   
# 94 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port10$enable(void)
#line 94
{
#line 94
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 94
    P1IE |= 1 << 0;
#line 94
    __nesc_atomic_end(__nesc_atomic); }
}

# 27 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420M$FIFOPInterrupt$enable(void){
#line 27
  MSP430InterruptM$Port10$enable();
#line 27
}
#line 27
static inline   
# 148 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port10$edge(bool l2h)
#line 148
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 149
    {
      if (l2h) {
#line 150
        P1IES &= ~(1 << 0);
        }
      else {
#line 151
        P1IES |= 1 << 0;
        }
    }
#line 153
    __nesc_atomic_end(__nesc_atomic); }
}

# 30 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420M$FIFOPInterrupt$edge(bool arg_0xa7797e0){
#line 30
  MSP430InterruptM$Port10$edge(arg_0xa7797e0);
#line 30
}
#line 30
static inline   
# 130 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port10$clear(void)
#line 130
{
#line 130
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 130
    P1IFG &= ~(1 << 0);
#line 130
    __nesc_atomic_end(__nesc_atomic); }
}

# 29 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420M$FIFOPInterrupt$clear(void){
#line 29
  MSP430InterruptM$Port10$clear();
#line 29
}
#line 29
static inline   
# 112 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port10$disable(void)
#line 112
{
#line 112
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    P1IE &= ~(1 << 0);
#line 112
    __nesc_atomic_end(__nesc_atomic); }
}

# 28 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void HPLCC2420M$FIFOPInterrupt$disable(void){
#line 28
  MSP430InterruptM$Port10$disable();
#line 28
}
#line 28
static inline   
# 96 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/HPLCC2420M.nc"
result_t HPLCC2420M$HPLCC2420$enableFIFOP(void)
#line 96
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 98
    {
      HPLCC2420M$FIFOPInterrupt$disable();
      HPLCC2420M$FIFOPInterrupt$clear();
      HPLCC2420M$FIFOPInterrupt$edge(FALSE);
      HPLCC2420M$FIFOPInterrupt$enable();
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 42 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static   result_t CC2420RadioM$HPLChipcon$enableFIFOP(void){
#line 42
  unsigned char result;
#line 42

#line 42
  result = HPLCC2420M$HPLCC2420$enableFIFOP();
#line 42

#line 42
  return result;
#line 42
}
#line 42










inline static   uint8_t CC2420ControlM$HPLChipcon$cmd(uint8_t arg_0xa6dc2f8){
#line 52
  unsigned char result;
#line 52

#line 52
  result = HPLCC2420M$HPLCC2420$cmd(arg_0xa6dc2f8);
#line 52

#line 52
  return result;
#line 52
}
#line 52
static inline   
# 257 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
result_t CC2420ControlM$CC2420Control$RxMode(void)
#line 257
{
  CC2420ControlM$HPLChipcon$cmd(0x03);
  return SUCCESS;
}

# 144 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420Control.nc"
inline static   result_t CC2420RadioM$CC2420Control$RxMode(void){
#line 144
  unsigned char result;
#line 144

#line 144
  result = CC2420ControlM$CC2420Control$RxMode();
#line 144

#line 144
  return result;
#line 144
}
#line 144
# 59 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static   uint8_t CC2420ControlM$HPLChipcon$write(uint8_t arg_0xa6dc750, uint16_t arg_0xa6dc8a0){
#line 59
  unsigned char result;
#line 59

#line 59
  result = HPLCC2420M$HPLCC2420$write(arg_0xa6dc750, arg_0xa6dc8a0);
#line 59

#line 59
  return result;
#line 59
}
#line 59
static inline  
# 205 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
result_t CC2420ControlM$CC2420Control$TunePreset(uint8_t chnl)
#line 205
{
  int fsctrl;

  fsctrl = 357 + 5 * (chnl - 11);
  CC2420ControlM$gCurrentParameters[CP_FSCTRL] = (CC2420ControlM$gCurrentParameters[CP_FSCTRL] & 0xfc00) | (fsctrl << 0);
  CC2420ControlM$HPLChipcon$write(0x18, CC2420ControlM$gCurrentParameters[CP_FSCTRL]);
  return SUCCESS;
}

static inline   
#line 351
result_t CC2420ControlM$HPLChipconRAM$writeDone(uint16_t addr, uint8_t length, uint8_t *buffer)
#line 351
{
  return SUCCESS;
}

# 49 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
inline static   result_t HPLCC2420M$HPLCC2420RAM$writeDone(uint16_t arg_0xa71e8a8, uint8_t arg_0xa71e9f0, uint8_t *arg_0xa71eb50){
#line 49
  unsigned char result;
#line 49

#line 49
  result = CC2420ControlM$HPLChipconRAM$writeDone(arg_0xa71e8a8, arg_0xa71e9f0, arg_0xa71eb50);
#line 49

#line 49
  return result;
#line 49
}
#line 49
static inline  
# 232 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/HPLCC2420M.nc"
void HPLCC2420M$signalRAMWr(void)
#line 232
{
  HPLCC2420M$HPLCC2420RAM$writeDone(HPLCC2420M$ramaddr, HPLCC2420M$ramlen, HPLCC2420M$rambuf);
}

# 38 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/BusArbitration.nc"
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
static 
# 143 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
__inline void TOSH_uwait(uint16_t u)
{
  uint16_t i;

#line 146
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
# 281 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
result_t HPLUSART0M$USARTControl$tx(uint8_t data)
#line 281
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 282
    U0TXBUF = data;
#line 282
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 142 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$tx(uint8_t arg_0xa7709d0){
#line 142
  unsigned char result;
#line 142

#line 142
  result = HPLUSART0M$USARTControl$tx(arg_0xa7709d0);
#line 142

#line 142
  return result;
#line 142
}
#line 142
static inline   
# 286 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
uint8_t HPLUSART0M$USARTControl$rx(void)
#line 286
{
  uint8_t value;

#line 288
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 288
    value = U0RXBUF;
#line 288
    __nesc_atomic_end(__nesc_atomic); }
  return value;
}

# 149 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   uint8_t HPLCC2420M$USARTControl$rx(void){
#line 149
  unsigned char result;
#line 149

#line 149
  result = HPLUSART0M$USARTControl$rx();
#line 149

#line 149
  return result;
#line 149
}
#line 149
static inline   
# 232 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
result_t HPLUSART0M$USARTControl$isTxIntrPending(void)
#line 232
{
  if (HPLUSART0M$IFG1 & (1 << 7)) {
      HPLUSART0M$IFG1 &= ~(1 << 7);
      return SUCCESS;
    }
  return FAIL;
}

# 120 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLCC2420M$USARTControl$isTxIntrPending(void){
#line 120
  unsigned char result;
#line 120

#line 120
  result = HPLUSART0M$USARTControl$isTxIntrPending();
#line 120

#line 120
  return result;
#line 120
}
#line 120
static inline 
# 16 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/hardware.h"
void TOSH_CLR_RADIO_CSN_PIN(void)
#line 16
{
   
#line 16
  static volatile uint8_t r __asm ("0x001D");

#line 16
  r &= ~(1 << 2);
}

# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/BusArbitration.nc"
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
# 236 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/HPLCC2420M.nc"
result_t HPLCC2420M$HPLCC2420RAM$write(uint16_t addr, uint8_t length, uint8_t *buffer)
#line 236
{
  uint8_t i = 0;

#line 238
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 239
        {
          HPLCC2420M$ramaddr = addr;
          HPLCC2420M$ramlen = length;
          HPLCC2420M$rambuf = buffer;
        }
#line 243
        __nesc_atomic_end(__nesc_atomic); }
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx((HPLCC2420M$ramaddr & 0x7F) | 0x80);
      TOSH_uwait(20);
      HPLCC2420M$USARTControl$tx((HPLCC2420M$ramaddr >> 1) & 0xC0);
      TOSH_uwait(20);
      for (i = 0; i < HPLCC2420M$ramlen; i++) {
          HPLCC2420M$USARTControl$tx(HPLCC2420M$rambuf[i]);
          TOSH_uwait(20);
        }
      TOSH_uwait(20);
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$BusArbitration$releaseBus();
      return TOS_post(HPLCC2420M$signalRAMWr);
    }
  return FAIL;
}

# 47 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420RAM.nc"
inline static   result_t CC2420ControlM$HPLChipconRAM$write(uint16_t arg_0xa71e180, uint8_t arg_0xa71e2c8, uint8_t *arg_0xa71e428){
#line 47
  unsigned char result;
#line 47

#line 47
  result = HPLCC2420M$HPLCC2420RAM$write(arg_0xa71e180, arg_0xa71e2c8, arg_0xa71e428);
#line 47

#line 47
  return result;
#line 47
}
#line 47
static 
# 10 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/byteorder.h"
__inline int is_host_lsb(void)
{
  const uint8_t n[2] = { 1, 0 };

#line 13
  return * (uint16_t *)n == 1;
}

static __inline uint16_t toLSB16(uint16_t a)
{
  return is_host_lsb() ? a : (a << 8) | (a >> 8);
}

static inline  
# 338 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
result_t CC2420ControlM$CC2420Control$setShortAddress(uint16_t addr)
#line 338
{
  addr = toLSB16(addr);
  return CC2420ControlM$HPLChipconRAM$write(0x16A, 2, (uint8_t *)&addr);
}

# 66 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static   uint16_t CC2420ControlM$HPLChipcon$read(uint8_t arg_0xa6dcd48){
#line 66
  unsigned int result;
#line 66

#line 66
  result = HPLCC2420M$HPLCC2420$read(arg_0xa6dcd48);
#line 66

#line 66
  return result;
#line 66
}
#line 66
static inline 
# 70 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
bool CC2420ControlM$SetRegs(void)
#line 70
{
  uint16_t data;

  CC2420ControlM$HPLChipcon$write(0x10, CC2420ControlM$gCurrentParameters[CP_MAIN]);
  CC2420ControlM$HPLChipcon$write(0x11, CC2420ControlM$gCurrentParameters[CP_MDMCTRL0]);
  data = CC2420ControlM$HPLChipcon$read(0x11);
  if (data != CC2420ControlM$gCurrentParameters[CP_MDMCTRL0]) {
#line 76
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
#line 282
result_t CC2420ControlM$CC2420Control$OscillatorOn(void)
#line 282
{
  uint16_t i;
  uint8_t status;
  bool bXoscOn = FALSE;

  i = 0;

  CC2420ControlM$HPLChipcon$cmd(0x01);

  while (i < 200 && bXoscOn == FALSE) {
      status = CC2420ControlM$HPLChipcon$cmd(0x00);
      status = status & (1 << 6);
      if (status) {
#line 294
        bXoscOn = TRUE;
        }
#line 295
      i++;
    }

  if (!bXoscOn) {
#line 298
    return FAIL;
    }
#line 299
  return SUCCESS;
}

static 
# 130 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
__inline void TOSH_wait(void )
{
   __asm volatile ("nop"); __asm volatile ("nop");}

static inline 
# 30 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/hardware.h"
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

static inline 
#line 29
void TOSH_SET_CC_VREN_PIN(void)
#line 29
{
   
#line 29
  static volatile uint8_t r __asm ("0x001D");

#line 29
  r |= 1 << 5;
}

static inline   
# 307 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
result_t CC2420ControlM$CC2420Control$VREFOn(void)
#line 307
{
  TOSH_SET_CC_VREN_PIN();
  TOSH_uwait(600);
  return SUCCESS;
}

static inline  
# 78 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/HPLCC2420M.nc"
result_t HPLCC2420M$StdControl$start(void)
#line 78
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
# 174 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
result_t CC2420ControlM$StdControl$start(void)
#line 174
{
  result_t status;

  CC2420ControlM$HPLChipconControl$start();

  CC2420ControlM$CC2420Control$VREFOn();

  TOSH_CLR_CC_RSTN_PIN();
  TOSH_wait();
  TOSH_SET_CC_RSTN_PIN();
  TOSH_wait();


  status = CC2420ControlM$CC2420Control$OscillatorOn();

  status = CC2420ControlM$SetRegs() && status;
  status = status && CC2420ControlM$CC2420Control$setShortAddress(TOS_LOCAL_ADDRESS);
  status = status && CC2420ControlM$CC2420Control$TunePreset(11);

  return status;
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t CC2420RadioM$CC2420StdControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = CC2420ControlM$StdControl$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
static inline  
# 22 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/TimerJiffyAsyncM.nc"
result_t TimerJiffyAsyncM$StdControl$start(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 24
    {
      TimerJiffyAsyncM$bSet = FALSE;
      TimerJiffyAsyncM$Alarm$disableEvents();
    }
#line 27
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
# 198 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$StdControl$start(void)
#line 198
{
  uint8_t chkstateRadio;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 201
    chkstateRadio = CC2420RadioM$stateRadio;
#line 201
    __nesc_atomic_end(__nesc_atomic); }

  if (chkstateRadio == CC2420RadioM$DISABLED_STATE) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 204
        {
          CC2420RadioM$countRetry = 0;
          CC2420RadioM$rxbufptr->length = 0;

          CC2420RadioM$TimerControl$start();
          CC2420RadioM$CC2420StdControl$start();
          CC2420RadioM$CC2420Control$RxMode();
          CC2420RadioM$HPLChipcon$enableFIFOP();

          CC2420RadioM$stateRadio = CC2420RadioM$IDLE_STATE;
        }
#line 214
        __nesc_atomic_end(__nesc_atomic); }
    }
  return SUCCESS;
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
static inline   
# 273 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
result_t HPLUSART1M$USARTControl$enableTxIntr(void)
#line 273
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 274
    {
      HPLUSART1M$IFG2 &= ~(1 << 5);
      IE2 |= 1 << 5;
    }
#line 277
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 115 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLUARTM$USARTControl$enableTxIntr(void){
#line 115
  unsigned char result;
#line 115

#line 115
  result = HPLUSART1M$USARTControl$enableTxIntr();
#line 115

#line 115
  return result;
#line 115
}
#line 115
static inline   
# 265 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
result_t HPLUSART1M$USARTControl$enableRxIntr(void)
#line 265
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 266
    {
      HPLUSART1M$IFG2 &= ~(1 << 4);
      IE2 |= 1 << 4;
    }
#line 269
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 114 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLUARTM$USARTControl$enableRxIntr(void){
#line 114
  unsigned char result;
#line 114

#line 114
  result = HPLUSART1M$USARTControl$enableRxIntr();
#line 114

#line 114
  return result;
#line 114
}
#line 114
static inline   
# 222 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
void HPLUSART1M$USARTControl$setClockRate(uint16_t baudrate, uint8_t mctl)
#line 222
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 223
    {
      HPLUSART1M$l_br = baudrate;
      HPLUSART1M$l_mctl = mctl;
      U1BR0 = baudrate & 0x0FF;
      U1BR1 = (baudrate >> 8) & 0x0FF;
      U1MCTL = mctl;
    }
#line 229
    __nesc_atomic_end(__nesc_atomic); }
}

# 109 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   void HPLUARTM$USARTControl$setClockRate(uint16_t arg_0xa772fb0, uint8_t arg_0xa7730f8){
#line 109
  HPLUSART1M$USARTControl$setClockRate(arg_0xa772fb0, arg_0xa7730f8);
#line 109
}
#line 109
static inline   
# 214 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
void HPLUSART1M$USARTControl$setClockSource(uint8_t source)
#line 214
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 215
    {
      HPLUSART1M$l_ssel = source | 0x80;
      HPLUSART1M$U1TCTL &= ~(((0x00 | 0x10) | 0x20) | 0x30);
      HPLUSART1M$U1TCTL |= HPLUSART1M$l_ssel & 0x7F;
    }
#line 219
    __nesc_atomic_end(__nesc_atomic); }
}

# 107 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   void HPLUARTM$USARTControl$setClockSource(uint8_t arg_0xa772b10){
#line 107
  HPLUSART1M$USARTControl$setClockSource(arg_0xa772b10);
#line 107
}
#line 107
static inline 
# 146 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
void HPLUSART1M$setUARTModeCommon(void)
#line 146
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 147
    {
      UCTL1 = 0x01;
      UCTL1 |= 0x10;

      U1RCTL &= ~0x08;

      UCTL1 = 0x01;
      UCTL1 |= 0x10;

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
#line 183
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

static inline 
# 39 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/hardware.h"
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
# 205 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
void HPLUSART1M$USARTControl$setModeUART(void)
#line 205
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 206
    {
      TOSH_SEL_UTXD1_MODFUNC();
      TOSH_SEL_URXD1_MODFUNC();
      HPLUSART1M$setUARTModeCommon();
    }
#line 210
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 105 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   void HPLUARTM$USARTControl$setModeUART(void){
#line 105
  HPLUSART1M$USARTControl$setModeUART();
#line 105
}
#line 105
static inline   
# 50 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
result_t HPLUARTM$UART$init(void)
#line 50
{

  HPLUARTM$USARTControl$setModeUART();

  HPLUARTM$USARTControl$setClockSource(0x20);

  HPLUARTM$USARTControl$setClockRate(UBR_SMCLK_19200, UMCTL_SMCLK_19200);


  HPLUARTM$USARTControl$enableRxIntr();
  HPLUARTM$USARTControl$enableTxIntr();
  return SUCCESS;
}

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
inline static  result_t BSN_RSUM$CommControl$start(void){
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
# 59 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Timer.nc"
inline static  result_t BSN_RSUM$Timer$start(char arg_0xa4efc10, uint32_t arg_0xa4efd68){
#line 59
  unsigned char result;
#line 59

#line 59
  result = TimerM$Timer$start(0, arg_0xa4efc10, arg_0xa4efd68);
#line 59

#line 59
  return result;
#line 59
}
#line 59
static inline  
# 26 "AccelTempM.nc"
result_t AccelTempM$StdControl$start(void)
{
  return SUCCESS;
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t BSN_RSUM$SensorControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = AccelTempM$StdControl$start();
#line 70

#line 70
  return result;
#line 70
}
#line 70
static inline  
# 103 "BSN_RSUM.nc"
result_t BSN_RSUM$StdControl$start(void)
#line 103
{
  BSN_RSUM$SensorControl$start();

  BSN_RSUM$Timer$start(TIMER_REPEAT, 5);
  BSN_RSUM$CommControl$start();
  return SUCCESS;
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/StdControl.nc"
inline static  result_t MainM$StdControl$start(void){
#line 70
  unsigned char result;
#line 70

#line 70
  result = TimerM$StdControl$start();
#line 70
  result = rcombine(result, BSN_RSUM$StdControl$start());
#line 70
  result = rcombine(result, TimerM$StdControl$start());
#line 70

#line 70
  return result;
#line 70
}
#line 70
static inline 
# 105 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
void TimerM$removeTimer(uint8_t num)
{
  uint8_t prev;
  uint8_t head;

  TimerM$m_timers[num].isset = FALSE;

  prev = TimerM$EMPTY_LIST;
  head = TimerM$m_head_short;
  while (head != TimerM$EMPTY_LIST) 
    {
      if (head == num) 
        {
          if (prev == TimerM$EMPTY_LIST) {
            TimerM$m_head_short = TimerM$m_timers[head].next;
            }
          else {
#line 121
            TimerM$m_timers[prev].next = TimerM$m_timers[head].next;
            }
#line 122
          return;
        }
      prev = head;
      head = TimerM$m_timers[head].next;
    }

  prev = TimerM$EMPTY_LIST;
  head = TimerM$m_head_long;
  while (head != TimerM$EMPTY_LIST) 
    {
      if (head == num) 
        {
          if (prev == TimerM$EMPTY_LIST) {
            TimerM$m_head_long = TimerM$m_timers[head].next;
            }
          else {
#line 137
            TimerM$m_timers[prev].next = TimerM$m_timers[head].next;
            }
#line 138
          return;
        }
      prev = head;
      head = TimerM$m_timers[head].next;
    }
}

static inline   
# 117 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
uint16_t MSP430TimerM$TimerB$read(void)
#line 117
{
#line 117
  return TBR;
}

# 29 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   uint16_t TimerM$AlarmTimer$read(void){
#line 29
  unsigned int result;
#line 29

#line 29
  result = MSP430TimerM$TimerB$read();
#line 29

#line 29
  return result;
#line 29
}
#line 29
static inline   
# 120 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
bool MSP430TimerM$TimerB$isOverflowPending(void)
#line 120
{
#line 120
  return TBCTL & 0x0001;
}

# 30 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   bool TimerM$AlarmTimer$isOverflowPending(void){
#line 30
  unsigned char result;
#line 30

#line 30
  result = MSP430TimerM$TimerB$isOverflowPending();
#line 30

#line 30
  return result;
#line 30
}
#line 30
static inline   
# 123 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$TimerB$clearOverflow(void)
#line 123
{
#line 123
  TBCTL &= ~0x0001;
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void TimerM$AlarmTimer$clearOverflow(void){
#line 31
  MSP430TimerM$TimerB$clearOverflow();
#line 31
}
#line 31
# 52 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ADC.nc"
inline static   result_t BSN_RSUM$AccelY$getData(void){
#line 52
  unsigned char result;
#line 52

#line 52
  result = ADCM$ADC$getData(3);
#line 52

#line 52
  return result;
#line 52
}
#line 52
# 52 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ADC.nc"
inline static   result_t BSN_RSUM$AccelX$getData(void){
#line 52
  unsigned char result;
#line 52

#line 52
  result = ADCM$ADC$getData(2);
#line 52

#line 52
  return result;
#line 52
}
#line 52
# 52 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ADC.nc"
inline static   result_t BSN_RSUM$Temp$getData(void){
#line 52
  unsigned char result;
#line 52

#line 52
  result = ADCM$ADC$getData(1);
#line 52

#line 52
  return result;
#line 52
}
#line 52
static inline  
# 233 "BSN_RSUM.nc"
result_t BSN_RSUM$Timer$fired(void)
#line 233
{

  switch (BSN_RSUM$readwhich) 
    {
      case 0: BSN_RSUM$Temp$getData();
#line 237
      BSN_RSUM$readwhich = 1;
#line 237
      break;
      case 1: BSN_RSUM$AccelX$getData();
#line 238
      BSN_RSUM$readwhich = 2;
#line 238
      break;
      case 2: BSN_RSUM$AccelY$getData();
#line 239
      BSN_RSUM$readwhich = 0;
#line 239
      break;
      default: BSN_RSUM$readwhich = 0;
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
# 452 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
result_t TimerM$Timer$default$fired(uint8_t num)
{
  return SUCCESS;
}

# 73 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Timer.nc"
inline static  result_t TimerM$Timer$fired(uint8_t arg_0xa4e5788){
#line 73
  unsigned char result;
#line 73

#line 73
  switch (arg_0xa4e5788) {
#line 73
    case 0:
#line 73
      result = BSN_RSUM$Timer$fired();
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
      result = TimerM$Timer$default$fired(arg_0xa4e5788);
#line 73
    }
#line 73

#line 73
  return result;
#line 73
}
#line 73
static inline 
# 383 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
uint8_t TimerM$fromNumMilli(uint8_t num)
{
  return num + TimerM$OFFSET_TIMER_MILLI;
}

static inline  result_t TimerM$TimerMilli$setOneShot(uint8_t num, int32_t milli)
{
  return TimerM$setTimer(TimerM$fromNumMilli(num), milli * 32, FALSE);
}

# 28 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
inline static  result_t RefVoltM$SwitchOffTimer$setOneShot(int32_t arg_0xa538c18){
#line 28
  unsigned char result;
#line 28

#line 28
  result = TimerM$TimerMilli$setOneShot(1, arg_0xa538c18);
#line 28

#line 28
  return result;
#line 28
}
#line 28
static inline  
# 230 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVoltM.nc"
void RefVoltM$switchOffRetry(void)
#line 230
{
  if (RefVoltM$switchOff == TRUE) {
    RefVoltM$SwitchOffTimer$setOneShot(5);
    }
}

static inline   
# 138 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
void HPLADC12M$HPLADC12$setRefOff(void)
#line 138
{
#line 138
  HPLADC12M$ADC12CTL0 &= ~0x0020;
}

# 73 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   void RefVoltM$HPLADC12$setRefOff(void){
#line 73
  HPLADC12M$HPLADC12$setRefOff();
#line 73
}
#line 73
static inline   
# 115 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
void HPLADC12M$HPLADC12$disableConversion(void)
#line 115
{
#line 115
  HPLADC12M$ADC12CTL0 &= ~0x0002;
}

# 80 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   void RefVoltM$HPLADC12$disableConversion(void){
#line 80
  HPLADC12M$HPLADC12$disableConversion();
#line 80
}
#line 80
static inline   
# 112 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
bool HPLADC12M$HPLADC12$isBusy(void)
#line 112
{
#line 112
  return HPLADC12M$ADC12CTL1 & 1;
}

# 65 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   bool RefVoltM$HPLADC12$isBusy(void){
#line 65
  unsigned char result;
#line 65

#line 65
  result = HPLADC12M$HPLADC12$isBusy();
#line 65

#line 65
  return result;
#line 65
}
#line 65
static 
# 205 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVoltM.nc"
__inline void RefVoltM$switchRefOff(void)
#line 205
{
  result_t result;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 208
    {
      if (RefVoltM$switchOff == FALSE) {
        result = FAIL;
        }
      else {
#line 211
        if (RefVoltM$HPLADC12$isBusy()) {
            result = FAIL;
          }
        else {
            RefVoltM$HPLADC12$disableConversion();
            RefVoltM$HPLADC12$setRefOff();
            RefVoltM$state = RefVoltM$REFERENCE_OFF;
            result = SUCCESS;
          }
        }
    }
#line 221
    __nesc_atomic_end(__nesc_atomic); }
#line 221
  if (RefVoltM$switchOff == TRUE && result == FAIL) {
    TOS_post(RefVoltM$switchOffRetry);
    }
}

static inline  








result_t RefVoltM$SwitchOffTimer$fired(void)
#line 235
{
  RefVoltM$switchRefOff();
  return SUCCESS;
}

static inline  
# 671 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
void MSP430ADC12M$RefVolt$isStable(RefVolt_t vref)
{
  TOS_post(MSP430ADC12M$checkQueue);
}

# 127 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVolt.nc"
inline static  void RefVoltM$RefVolt$isStable(RefVolt_t arg_0xa5cea80){
#line 127
  MSP430ADC12M$RefVolt$isStable(arg_0xa5cea80);
#line 127
}
#line 127
static inline  
# 166 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVoltM.nc"
result_t RefVoltM$SwitchOnTimer$fired(void)
#line 166
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 167
    {
      if (RefVoltM$state == RefVoltM$REFERENCE_1_5V_PENDING) {
        RefVoltM$state = RefVoltM$REFERENCE_1_5V_STABLE;
        }
#line 170
      if (RefVoltM$state == RefVoltM$REFERENCE_2_5V_PENDING) {
        RefVoltM$state = RefVoltM$REFERENCE_2_5V_STABLE;
        }
    }
#line 173
    __nesc_atomic_end(__nesc_atomic); }
#line 173
  if (RefVoltM$state == RefVoltM$REFERENCE_1_5V_STABLE) {
    RefVoltM$RefVolt$isStable(REFERENCE_1_5V);
    }
#line 175
  if (RefVoltM$state == RefVoltM$REFERENCE_2_5V_STABLE) {
    RefVoltM$RefVolt$isStable(REFERENCE_2_5V);
    }
#line 177
  return SUCCESS;
}

static inline   
# 424 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
result_t TimerM$TimerMilli$default$fired(uint8_t num)
{
  return SUCCESS;
}

# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
inline static  result_t TimerM$TimerMilli$fired(uint8_t arg_0xa4e5e10){
#line 37
  unsigned char result;
#line 37

#line 37
  switch (arg_0xa4e5e10) {
#line 37
    case 0:
#line 37
      result = RefVoltM$SwitchOnTimer$fired();
#line 37
      break;
#line 37
    case 1:
#line 37
      result = RefVoltM$SwitchOffTimer$fired();
#line 37
      break;
#line 37
    default:
#line 37
      result = TimerM$TimerMilli$default$fired(arg_0xa4e5e10);
#line 37
    }
#line 37

#line 37
  return result;
#line 37
}
#line 37
static inline   
# 375 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
result_t TimerM$TimerJiffy$default$fired(uint8_t num)
{
  return SUCCESS;
}

# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerJiffy.nc"
inline static  result_t TimerM$TimerJiffy$fired(uint8_t arg_0xa54c850){
#line 37
  unsigned char result;
#line 37

#line 37
    result = TimerM$TimerJiffy$default$fired(arg_0xa54c850);
#line 37

#line 37
  return result;
#line 37
}
#line 37
static inline 
#line 145
void TimerM$signal_timer_fired(uint8_t num)
{



  const int16_t num16 = num;

  if (TimerM$COUNT_TIMER_JIFFY > 0 && num16 >= TimerM$OFFSET_TIMER_JIFFY) 
    {
      TimerM$TimerJiffy$fired(num - TimerM$OFFSET_TIMER_JIFFY);
    }
  else {
#line 156
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

# 109 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVolt.nc"
inline static   result_t MSP430ADC12M$RefVolt$release(void){
#line 109
  unsigned char result;
#line 109

#line 109
  result = RefVoltM$RefVolt$release();
#line 109

#line 109
  return result;
#line 109
}
#line 109
static 
# 306 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
__inline result_t MSP430ADC12M$releaseRefVoltAdvanced(uint8_t num)
{
  adc12memctl_t memctl = MSP430ADC12M$aSettings[num].memctl;

  if (memctl.sref == REFERENCE_VREFplus_AVss || 
  memctl.sref == REFERENCE_VREFplus_VREFnegterm) 
    {
      MSP430ADC12M$RefVolt$release();
      MSP430ADC12M$aSettings[num].gotRefVolt = 0;
    }
  return SUCCESS;
}

static 
#line 252
__inline void MSP430ADC12M$releaseLockAdvanced(aSettings_t *settings)
{
  settings->locked = 0;
}

static inline   
# 230 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB1$setControl(MSP430TimerM$CC_t x)
#line 230
{
#line 230
  MSP430TimerM$TBCCTL1 = MSP430TimerM$CC2int(x);
}

# 33 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ADC12M$CompareB1$setControl(MSP430CompareControl_t arg_0xa425b70){
#line 33
  MSP430TimerM$CompareB1$setControl(arg_0xa425b70);
#line 33
}
#line 33
static inline   
# 294 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB1$setEventFromNow(uint16_t x)
#line 294
{
#line 294
  MSP430TimerM$TBCCR1 = TBR + x;
}

# 43 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ADC12M$CompareB1$setEventFromNow(uint16_t arg_0xa41b658){
#line 43
  MSP430TimerM$CompareB1$setEventFromNow(arg_0xa41b658);
#line 43
}
#line 43
static inline   
# 229 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB0$setControl(MSP430TimerM$CC_t x)
#line 229
{
#line 229
  MSP430TimerM$TBCCTL0 = MSP430TimerM$CC2int(x);
}

# 33 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ADC12M$CompareB0$setControl(MSP430CompareControl_t arg_0xa425b70){
#line 33
  MSP430TimerM$CompareB0$setControl(arg_0xa425b70);
#line 33
}
#line 33
static inline   
# 293 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB0$setEventFromNow(uint16_t x)
#line 293
{
#line 293
  MSP430TimerM$TBCCR0 = TBR + x;
}

# 43 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ADC12M$CompareB0$setEventFromNow(uint16_t arg_0xa41b658){
#line 43
  MSP430TimerM$CompareB0$setEventFromNow(arg_0xa41b658);
#line 43
}
#line 43
static inline   
# 138 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareA1$setControl(MSP430TimerM$CC_t x)
#line 138
{
#line 138
  MSP430TimerM$TACCTL1 = MSP430TimerM$CC2int(x);
}

# 33 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ADC12M$CompareA1$setControl(MSP430CompareControl_t arg_0xa425b70){
#line 33
  MSP430TimerM$CompareA1$setControl(arg_0xa425b70);
#line 33
}
#line 33
static inline   
# 170 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareA1$setEventFromNow(uint16_t x)
#line 170
{
#line 170
  MSP430TimerM$TACCR1 = TAR + x;
}

# 43 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ADC12M$CompareA1$setEventFromNow(uint16_t arg_0xa41b658){
#line 43
  MSP430TimerM$CompareA1$setEventFromNow(arg_0xa41b658);
#line 43
}
#line 43
static 
# 319 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
__inline void MSP430ADC12M$setTimerAdvanced(uint8_t shs)
{
  MSP430CompareControl_t ccSetSHI = { 
  .ccifg = 0, .cov = 0, .out = 1, .cci = 0, .ccie = 0, 
  .outmod = 0, 
  .cap = 0, .clld = 0, 
  .scs = 0, .ccis = 0, .cm = 0 };

  MSP430CompareControl_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, 
  .cap = 0, .clld = 0, 
  .scs = 0, .ccis = 0, .cm = 0 };

  MSP430CompareControl_t ccEnableSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 1, 
  .outmod = 1, 
  .cap = 0, .clld = 0, 
  .scs = 0, .ccis = 0, .cm = 0 };

  switch (shs) 
    {

      case TIMERA_OUT1: MSP430ADC12M$advancedTimer = TIMERA_OUT1;
      MSP430ADC12M$CompareA1$setEventFromNow(MSP430ADC12M$advancedInterval);
      MSP430ADC12M$CompareA1$setControl(ccResetSHI);
      MSP430ADC12M$CompareA1$setControl(ccSetSHI);
      MSP430ADC12M$CompareA1$setControl(ccResetSHI);
      MSP430ADC12M$CompareA1$setControl(ccEnableSHI);
      break;
      case TIMERB_OUT0: MSP430ADC12M$advancedTimer = TIMERB_OUT0;
      MSP430ADC12M$CompareB0$setEventFromNow(MSP430ADC12M$advancedInterval);
      MSP430ADC12M$CompareB0$setControl(ccResetSHI);
      MSP430ADC12M$CompareB0$setControl(ccSetSHI);
      MSP430ADC12M$CompareB0$setControl(ccResetSHI);
      MSP430ADC12M$CompareB0$setControl(ccEnableSHI);
      break;
      case TIMERB_OUT1: MSP430ADC12M$advancedTimer = TIMERB_OUT1;
      MSP430ADC12M$CompareB1$setEventFromNow(MSP430ADC12M$advancedInterval);
      MSP430ADC12M$CompareB1$setControl(ccResetSHI);
      MSP430ADC12M$CompareB1$setControl(ccSetSHI);
      MSP430ADC12M$CompareB1$setControl(ccResetSHI);
      MSP430ADC12M$CompareB1$setControl(ccEnableSHI);
      break;
    }
}

static inline   
# 116 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
void HPLADC12M$HPLADC12$startConversion(void)
#line 116
{
#line 116
  HPLADC12M$ADC12CTL0 |= 0x0001 + 0x0002;
}

# 81 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   void MSP430ADC12M$HPLADC12$startConversion(void){
#line 81
  HPLADC12M$HPLADC12$startConversion();
#line 81
}
#line 81
static inline   
# 95 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
void HPLADC12M$HPLADC12$setIEFlags(uint16_t mask)
#line 95
{
#line 95
  HPLADC12M$ADC12IE = mask;
}

# 55 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   void MSP430ADC12M$HPLADC12$setIEFlags(uint16_t arg_0xa5bdd20){
#line 55
  HPLADC12M$HPLADC12$setIEFlags(arg_0xa5bdd20);
#line 55
}
#line 55
#line 69
inline static   void MSP430ADC12M$HPLADC12$setSHT(uint8_t arg_0xa5dba80){
#line 69
  HPLADC12M$HPLADC12$setSHT(arg_0xa5dba80);
#line 69
}
#line 69
#line 50
inline static   void MSP430ADC12M$HPLADC12$setMemControl(uint8_t arg_0xa5bcf70, adc12memctl_t arg_0xa5bd0c8){
#line 50
  HPLADC12M$HPLADC12$setMemControl(arg_0xa5bcf70, arg_0xa5bd0c8);
#line 50
}
#line 50
static inline   
# 54 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
void HPLADC12M$HPLADC12$setControl1(adc12ctl1_t control1)
#line 54
{
  HPLADC12M$ADC12CTL1 = * (uint16_t *)&control1;
}

# 43 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   void MSP430ADC12M$HPLADC12$setControl1(adc12ctl1_t arg_0xa5bc198){
#line 43
  HPLADC12M$HPLADC12$setControl1(arg_0xa5bc198);
#line 43
}
#line 43





inline static   void MSP430ADC12M$HPLADC12$setControl0_IgnoreRef(adc12ctl0_t arg_0xa5bcb60){
#line 48
  HPLADC12M$HPLADC12$setControl0_IgnoreRef(arg_0xa5bcb60);
#line 48
}
#line 48
#line 80
inline static   void MSP430ADC12M$HPLADC12$disableConversion(void){
#line 80
  HPLADC12M$HPLADC12$disableConversion();
#line 80
}
#line 80
# 118 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVolt.nc"
inline static   RefVolt_t MSP430ADC12M$RefVolt$getState(void){
#line 118
  enum __nesc_unnamed4291 result;
#line 118

#line 118
  result = RefVoltM$RefVolt$getState();
#line 118

#line 118
  return result;
#line 118
}
#line 118
#line 93
inline static   result_t MSP430ADC12M$RefVolt$get(RefVolt_t arg_0xa5eb538){
#line 93
  unsigned char result;
#line 93

#line 93
  result = RefVoltM$RefVolt$get(arg_0xa5eb538);
#line 93

#line 93
  return result;
#line 93
}
#line 93
static 
# 278 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
__inline adcResult_t MSP430ADC12M$getRefVoltAdvanced(uint8_t num)
{
  adcResult_t adcResult = ADC_SUCCESS;
  result_t refVresult;
  adc12memctl_t memctl = MSP430ADC12M$aSettings[num].memctl;

  if (memctl.sref == REFERENCE_VREFplus_AVss || 
  memctl.sref == REFERENCE_VREFplus_VREFnegterm) 
    {
      if (MSP430ADC12M$aSettings[num].gotRefVolt == 0) {
          if (MSP430ADC12M$aSettings[num].refVolt2_5) {
            refVresult = MSP430ADC12M$RefVolt$get(REFERENCE_2_5V);
            }
          else {
#line 291
            refVresult = MSP430ADC12M$RefVolt$get(REFERENCE_1_5V);
            }
        }
      else {
#line 293
        refVresult = SUCCESS;
        }
#line 294
      if (refVresult != SUCCESS) 
        {
          adcResult = ADC_FAIL;
        }
      else 
#line 297
        {
          MSP430ADC12M$aSettings[num].gotRefVolt = 1;
          if (MSP430ADC12M$RefVolt$getState() == REFERENCE_UNSTABLE) {
            adcResult = ADC_QUEUED;
            }
        }
    }
#line 303
  return adcResult;
}

static 
#line 241
__inline bool MSP430ADC12M$getLockAdvanced(aSettings_t *settings)
{
  bool result;

#line 244
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      result = settings->locked == 0;
      settings->locked = 1;
    }
#line 248
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

static inline   
#line 366
adcResult_t MSP430ADC12M$MSP430ADC12Advanced$getSingleDataRepeat(uint8_t num, uint16_t jiffies)
{
  bool access = FALSE;
  bool buffered = FALSE;




  const int16_t num16 = num;

  if (num16 >= 0 || !MSP430ADC12M$getLockAdvanced(&MSP430ADC12M$aSettings[num])) {
    return ADC_FAIL;
    }
  switch (MSP430ADC12M$getRefVoltAdvanced(num)) 
    {
      case ADC_FAIL: 
        MSP430ADC12M$releaseLockAdvanced(&MSP430ADC12M$aSettings[num]);
      break;
      case ADC_SUCCESS: 
        { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 385
          {
            if (MSP430ADC12M$cmode == ADC_IDLE) {
                MSP430ADC12M$cmode = ADVANCED_REPEAT_SINGLE_CHANNEL;
                MSP430ADC12M$owner = num;
                access = TRUE;
                MSP430ADC12M$cancelled = FALSE;
              }
          }
#line 392
          __nesc_atomic_end(__nesc_atomic); }
      if (access) 
        {
          MSP430ADC12M$advancedInterval = jiffies;
          MSP430ADC12M$HPLADC12$disableConversion();
          if (jiffies != 0) 
            {

              uint16_t ctl0 = (0x0000 | 0x0010) & ~0x0080;
              adc12ctl1_t ctl1 = { .adc12busy = 0, .conseq = 2, .adc12ssel = MSP430ADC12M$aSettings[num].clockSource, 
              .adc12div = MSP430ADC12M$aSettings[num].clockDiv, .issh = 0, .shp = 1, 
              .shs = MSP430ADC12M$aSettings[num].sampleHoldSource, .cstartadd = 0 };

#line 404
              MSP430ADC12M$HPLADC12$setControl0_IgnoreRef(* (adc12ctl0_t *)&ctl0);
              MSP430ADC12M$HPLADC12$setControl1(ctl1);
            }
          else 
#line 406
            {


              uint16_t ctl0 = (0x0000 | 0x0010) | 0x0080;
              uint16_t ctl1 = (0x0218 & ~0x02) | 0x04;

#line 411
              MSP430ADC12M$HPLADC12$setControl0_IgnoreRef(* (adc12ctl0_t *)&ctl0);
              MSP430ADC12M$HPLADC12$setControl1(* (adc12ctl1_t *)&ctl1);
            }
          MSP430ADC12M$HPLADC12$setMemControl(0, MSP430ADC12M$aSettings[num].memctl);
          MSP430ADC12M$HPLADC12$setSHT(MSP430ADC12M$aSettings[num].sampleHoldTime);
          MSP430ADC12M$HPLADC12$setIEFlags(1);
          MSP430ADC12M$HPLADC12$startConversion();

          if (jiffies != 0) {
            MSP430ADC12M$setTimerAdvanced(MSP430ADC12M$aSettings[num].sampleHoldSource);
            }
#line 421
          return SUCCESS;
        }

      case ADC_QUEUED: 



        { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
          {
            if (MSP430ADC12M$aCmdBuffer.type == ADC_IDLE) 
              {
                MSP430ADC12M$aCmdBuffer.dataDest = 0;
                MSP430ADC12M$aCmdBuffer.jiffies = jiffies;
                MSP430ADC12M$aCmdBuffer.length = 0;
                MSP430ADC12M$aCmdBuffer.type = ADVANCED_REPEAT_SINGLE_CHANNEL;
                MSP430ADC12M$aCmdBuffer.intf = num;
                buffered = TRUE;
              }
          }
#line 439
          __nesc_atomic_end(__nesc_atomic); }
      if (buffered) {


        return ADC_QUEUED;
        }
      else {

          MSP430ADC12M$releaseLockAdvanced(&MSP430ADC12M$aSettings[num]);
          MSP430ADC12M$releaseRefVoltAdvanced(num);
        }
      break;
    }
  return ADC_FAIL;
}

# 28 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerMilli.nc"
inline static  result_t RefVoltM$SwitchOnTimer$setOneShot(int32_t arg_0xa538c18){
#line 28
  unsigned char result;
#line 28

#line 28
  result = TimerM$TimerMilli$setOneShot(0, arg_0xa538c18);
#line 28

#line 28
  return result;
#line 28
}
#line 28
static inline  
# 162 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVoltM.nc"
void RefVoltM$switchOnDelay(void)
#line 162
{
  RefVoltM$SwitchOnTimer$setOneShot(17);
}

static inline   
# 141 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
void HPLADC12M$HPLADC12$setRef2_5V(void)
#line 141
{
#line 141
  HPLADC12M$ADC12CTL0 |= 0x0040;
}

# 76 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   void RefVoltM$HPLADC12$setRef2_5V(void){
#line 76
  HPLADC12M$HPLADC12$setRef2_5V();
#line 76
}
#line 76
static inline   
# 140 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
void HPLADC12M$HPLADC12$setRef1_5V(void)
#line 140
{
#line 140
  HPLADC12M$ADC12CTL0 &= ~0x0040;
}

# 75 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   void RefVoltM$HPLADC12$setRef1_5V(void){
#line 75
  HPLADC12M$HPLADC12$setRef1_5V();
#line 75
}
#line 75
static inline   
# 137 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
void HPLADC12M$HPLADC12$setRefOn(void)
#line 137
{
#line 137
  HPLADC12M$ADC12CTL0 |= 0x0020;
}

# 72 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   void RefVoltM$HPLADC12$setRefOn(void){
#line 72
  HPLADC12M$HPLADC12$setRefOn();
#line 72
}
#line 72
static 
# 140 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVoltM.nc"
__inline void RefVoltM$switchRefOn(uint8_t vref)
#line 140
{
  RefVoltM$HPLADC12$disableConversion();
  RefVoltM$HPLADC12$setRefOn();
  if (vref == REFERENCE_1_5V) {
      RefVoltM$HPLADC12$setRef1_5V();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 145
        RefVoltM$state = RefVoltM$REFERENCE_1_5V_PENDING;
#line 145
        __nesc_atomic_end(__nesc_atomic); }
    }
  else {
      RefVoltM$HPLADC12$setRef2_5V();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 149
        RefVoltM$state = RefVoltM$REFERENCE_2_5V_PENDING;
#line 149
        __nesc_atomic_end(__nesc_atomic); }
    }
  TOS_post(RefVoltM$switchOnDelay);
}

static __inline void RefVoltM$switchToRefPending(uint8_t vref)
#line 154
{
  RefVoltM$switchRefOn(vref);
}

static __inline void RefVoltM$switchToRefStable(uint8_t vref)
#line 158
{
  RefVoltM$switchRefOn(vref);
}

static inline  
#line 225
void RefVoltM$switchOffDelay(void)
#line 225
{
  if (RefVoltM$switchOff == TRUE) {
    RefVoltM$SwitchOffTimer$setOneShot(100);
    }
}

# 93 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12Basic.nc"
inline static   adcResult_t ADCM$MSP430ADC12Basic$getSingleData(void){
#line 93
  enum __nesc_unnamed4265 result;
#line 93

#line 93
  result = MSP430ADC12M$MSP430ADC12Basic$getSingleData(0);
#line 93

#line 93
  return result;
#line 93
}
#line 93
static 
# 117 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
__inline void MSP430ADC12M$releaseLockBasic(bSettings_t *settings)
{
  settings->locked = 0;
}

static 
#line 106
__inline bool MSP430ADC12M$getLockBasic(bSettings_t *settings)
{
  bool result;

#line 109
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      result = settings->locked == 0;
      settings->locked = 1;
    }
#line 113
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

static inline   




result_t MSP430ADC12M$MSP430ADC12Basic$bind(uint8_t num, MSP430ADC12StandardSettings_ut settings)
{
  adc12memctl_t memctl = { .inch = settings.s.inputChannel, 
  .sref = settings.s.referenceVoltage, 
  .eos = 0 };

#line 127
  if (num < 1 && MSP430ADC12M$getLockBasic(&MSP430ADC12M$bSettings[num])) 
    {
      MSP430ADC12M$bSettings[num].memctl = memctl;
      MSP430ADC12M$bSettings[num].sampleHoldTime = settings.s.sampleHoldTime;
      MSP430ADC12M$bSettings[num].refVolt2_5 = settings.s.refVolt2_5;
      MSP430ADC12M$bSettings[num].queued = 0;
      MSP430ADC12M$bSettings[num].gotRefVolt = 0;
      MSP430ADC12M$releaseLockBasic(&MSP430ADC12M$bSettings[num]);
      return SUCCESS;
    }
  else {
    return FAIL;
    }
}

# 69 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12Basic.nc"
inline static   result_t ADCM$MSP430ADC12Basic$bind(MSP430ADC12StandardSettings_ut arg_0xa59c690){
#line 69
  unsigned char result;
#line 69

#line 69
  result = MSP430ADC12M$MSP430ADC12Basic$bind(0, arg_0xa59c690);
#line 69

#line 69
  return result;
#line 69
}
#line 69
static inline 
# 121 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/ADCM.nc"
result_t ADCM$triggerConversion(uint8_t port)
#line 121
{
  MSP430ADC12StandardSettings_ut settings;

#line 123
  settings.s.inputChannel = ADCM$TOSH_adc_portmap[port] & 0x0F;
  settings.s.referenceVoltage = (ADCM$TOSH_adc_portmap[port] & 0x70) >> 4;
  settings.s.sampleHoldTime = ADCM$samplingRate;
  settings.s.refVolt2_5 = (ADCM$TOSH_adc_portmap[port] & 0x80) >> 7;
  ADCM$MSP430ADC12Basic$bind(settings);
  if (ADCM$MSP430ADC12Basic$getSingleData() != ADC_FAIL) {
      ADCM$owner = port;
      return SUCCESS;
    }
  else {
#line 132
    { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 132
      ADCM$busy = FALSE;
#line 132
      __nesc_atomic_end(__nesc_atomic); }
    }
#line 133
  return FAIL;
}

static inline   
# 296 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB3$setEventFromNow(uint16_t x)
#line 296
{
#line 296
  MSP430TimerM$TBCCR3 = TBR + x;
}

# 43 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerM$AlarmCompare$setEventFromNow(uint16_t arg_0xa41b658){
#line 43
  MSP430TimerM$CompareB3$setEventFromNow(arg_0xa41b658);
#line 43
}
#line 43
static inline   
# 224 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB3$clearPendingInterrupt(void)
#line 224
{
#line 224
  MSP430TimerM$TBCCTL3 &= ~0x0001;
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerM$AlarmCompare$clearPendingInterrupt(void){
#line 31
  MSP430TimerM$CompareB3$clearPendingInterrupt();
#line 31
}
#line 31
static inline   
# 248 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB3$enableEvents(void)
#line 248
{
#line 248
  MSP430TimerM$TBCCTL3 |= 0x0010;
}

# 36 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerM$AlarmCompare$enableEvents(void){
#line 36
  MSP430TimerM$CompareB3$enableEvents();
#line 36
}
#line 36
static inline  
# 364 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/HPLCC2420M.nc"
result_t HPLCC2420M$BusArbitration$busFree(void)
#line 364
{
  return SUCCESS;
}

static inline   
# 116 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/BusArbitrationM.nc"
result_t BusArbitrationM$BusArbitration$default$busFree(uint8_t id)
#line 116
{
  return SUCCESS;
}

# 39 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/BusArbitration.nc"
inline static  result_t BusArbitrationM$BusArbitration$busFree(uint8_t arg_0xa820560){
#line 39
  unsigned char result;
#line 39

#line 39
  switch (arg_0xa820560) {
#line 39
    case 0:
#line 39
      result = HPLCC2420M$BusArbitration$busFree();
#line 39
      break;
#line 39
    default:
#line 39
      result = BusArbitrationM$BusArbitration$default$busFree(arg_0xa820560);
#line 39
    }
#line 39

#line 39
  return result;
#line 39
}
#line 39
static inline  
# 49 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/BusArbitrationM.nc"
void BusArbitrationM$busReleased(void)
#line 49
{
  uint8_t i;
  uint8_t currentstate;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 53
    BusArbitrationM$isBusReleasedPending = FALSE;
#line 53
    __nesc_atomic_end(__nesc_atomic); }
  for (i = 0; i < 1; i++) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 55
        currentstate = BusArbitrationM$state;
#line 55
        __nesc_atomic_end(__nesc_atomic); }
      if (currentstate == BusArbitrationM$BUS_IDLE) {
        BusArbitrationM$BusArbitration$busFree(i);
        }
    }
}

static inline 
# 171 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/msp430hardware.h"
void __nesc_enable_interrupt(void)
{
   __asm volatile ("eint");}

static inline 
#line 190
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

static inline 
#line 165
void __nesc_disable_interrupt(void)
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

static inline 
#line 183
__nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = are_interrupts_enabled();

#line 186
  __nesc_disable_interrupt();
  return result;
}

static 
#line 207
__inline void TOSH_sleep(void)
#line 207
{







  extern uint8_t TOSH_sched_full;
  extern volatile uint8_t TOSH_sched_free;
  __nesc_atomic_t fInterruptFlags;
  uint16_t LPMode_bits = 0;

  fInterruptFlags = __nesc_atomic_start();

  if (LPMode_disabled || TOSH_sched_full != TOSH_sched_free) {
      __nesc_atomic_end(fInterruptFlags);
      return;
    }
  else 
#line 225
    {
      LPMode_bits = 0x0080 + 0x0040 + 0x0010;




      if (((((
#line 228
      TACCTL0 & 0x0010 || TACCTL1 & 0x0010) || TACCTL2 & 0x0010)
       && (TACTL & (3 << 8)) == 2 << 8)
       || (ME1 & ((1 << 7) | (1 << 6)) && U0TCTL & 0x20))
       || (ME2 & ((1 << 5) | (1 << 4)) && U1TCTL & 0x20)) {
        LPMode_bits = 0x0040 + 0x0010;
        }
      if (ADC12CTL1 & 1) {
        switch (ADC12CTL1 & (3 << 3)) {
            case 2 << 3: LPMode_bits = 0;
#line 236
            break;
            case 3 << 3: LPMode_bits = 0x0040 + 0x0010;
#line 237
            break;
          }
        }
#line 239
      LPMode_bits |= 0x0008;
       __asm volatile ("bis  %0, r2" :  : "m"((uint16_t )LPMode_bits));}
}

static inline 
# 132 "C:/cygwin/opt/tinyos-1.x/tos/system/sched.c"
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
      __nesc_atomic_end(fInterruptFlags);
      return 0;
    }

  TOSH_queue[old_full].tp = NULL;
  TOSH_sched_full = (old_full + 1) & TOSH_TASK_BITMASK;
  __nesc_atomic_end(fInterruptFlags);
  func();

  return 1;
}

static inline void TOSH_run_task(void)
#line 155
{
  while (TOSH_run_next_task()) 
    ;
  TOSH_sleep();
  TOSH_wait();
}

static inline    
# 111 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareA0$default$fired(void)
#line 111
{
}

# 45 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareA0$fired(void){
#line 45
  MSP430TimerM$CompareA0$default$fired();
#line 45
}
#line 45
# 90 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void __attribute((interrupt(12))) __attribute((wakeup))   sig_TIMERA0_VECTOR(void)
{
  MSP430TimerM$CompareA0$fired();
}

static inline   
#line 166
void MSP430TimerM$CompareA1$setEventFromPrev(uint16_t x)
#line 166
{
#line 166
  MSP430TimerM$TACCR1 += x;
}

# 42 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ADC12M$CompareA1$setEventFromPrev(uint16_t arg_0xa41b240){
#line 42
  MSP430TimerM$CompareA1$setEventFromPrev(arg_0xa41b240);
#line 42
}
#line 42
static inline   
# 589 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
void MSP430ADC12M$CompareA1$fired(void)
#line 589
{
  MSP430CompareControl_t control1 = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, 
  .cap = 0, .clld = 0, 
  .scs = 0, .ccis = 0, .cm = 0 };

  MSP430CompareControl_t control2 = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 1, 
  .outmod = 1, 
  .cap = 0, .clld = 0, 
  .scs = 0, .ccis = 0, .cm = 0 };

  if (MSP430ADC12M$advancedTimer == TIMERA_OUT1) 
    {
      MSP430ADC12M$CompareA1$setControl(control1);
      MSP430ADC12M$CompareA1$setControl(control2);
      MSP430ADC12M$CompareA1$setEventFromPrev(MSP430ADC12M$advancedInterval);
    }
}

# 45 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareA1$fired(void){
#line 45
  MSP430ADC12M$CompareA1$fired();
#line 45
}
#line 45
static inline    
# 113 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareA2$default$fired(void)
#line 113
{
}

# 45 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareA2$fired(void){
#line 45
  MSP430TimerM$CompareA2$default$fired();
#line 45
}
#line 45
static inline    
# 114 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$TimerA$default$overflow(void)
#line 114
{
}

# 32 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void MSP430TimerM$TimerA$overflow(void){
#line 32
  MSP430TimerM$TimerA$default$overflow();
#line 32
}
#line 32
# 45 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB0$fired(void){
#line 45
  MSP430ADC12M$CompareB0$fired();
#line 45
}
#line 45
# 174 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void __attribute((interrupt(26))) __attribute((wakeup))   sig_TIMERB0_VECTOR(void)
{
  MSP430TimerM$CompareB0$fired();
}

static inline   
#line 285
void MSP430TimerM$CompareB0$setEventFromPrev(uint16_t x)
#line 285
{
#line 285
  MSP430TimerM$TBCCR0 += x;
}

# 42 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ADC12M$CompareB0$setEventFromPrev(uint16_t arg_0xa41b240){
#line 42
  MSP430TimerM$CompareB0$setEventFromPrev(arg_0xa41b240);
#line 42
}
#line 42
static inline   
# 286 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB1$setEventFromPrev(uint16_t x)
#line 286
{
#line 286
  MSP430TimerM$TBCCR1 += x;
}

# 42 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ADC12M$CompareB1$setEventFromPrev(uint16_t arg_0xa41b240){
#line 42
  MSP430TimerM$CompareB1$setEventFromPrev(arg_0xa41b240);
#line 42
}
#line 42
static inline   
# 631 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
void MSP430ADC12M$CompareB1$fired(void)
#line 631
{
  MSP430CompareControl_t control1 = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, 
  .cap = 0, .clld = 0, 
  .scs = 0, .ccis = 0, .cm = 0 };

  MSP430CompareControl_t control2 = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 1, 
  .outmod = 1, 
  .cap = 0, .clld = 0, 
  .scs = 0, .ccis = 0, .cm = 0 };

  if (MSP430ADC12M$advancedTimer == TIMERB_OUT1) 
    {
      MSP430ADC12M$CompareB1$setControl(control1);
      MSP430ADC12M$CompareB1$setControl(control2);
      MSP430ADC12M$CompareB1$setEventFromPrev(MSP430ADC12M$advancedInterval);
    }
}

# 45 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB1$fired(void){
#line 45
  MSP430ADC12M$CompareB1$fired();
#line 45
}
#line 45
static inline   
# 287 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB2$setEventFromPrev(uint16_t x)
#line 287
{
#line 287
  MSP430TimerM$TBCCR2 += x;
}

# 42 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ClockM$ACLKCompare$setEventFromPrev(uint16_t arg_0xa41b240){
#line 42
  MSP430TimerM$CompareB2$setEventFromPrev(arg_0xa41b240);
#line 42
}
#line 42
static inline   
# 145 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ClockM.nc"
void MSP430ClockM$ACLKCompare$fired(void)
{
  if (MSP430ClockM$m_aclk_count > 0) 
    {
      MSP430ClockM$m_dco_prev = MSP430ClockM$m_dco_curr;
      MSP430ClockM$m_dco_curr = TAR;
      if (MSP430ClockM$m_aclk_count > 1) {
        MSP430ClockM$ACLKCompare$setEventFromPrev(MSP430ClockM$ACLK_CALIB_PERIOD);
        }
#line 153
      MSP430ClockM$m_aclk_count--;
    }
}

# 45 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB2$fired(void){
#line 45
  MSP430ClockM$ACLKCompare$fired();
#line 45
}
#line 45
static inline  
# 252 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
void TimerM$checkShortTimers(void)
{
  uint8_t head = TimerM$m_head_short;

#line 255
  TimerM$m_head_short = TimerM$EMPTY_LIST;
  TimerM$executeTimers(head);
  TimerM$setNextShortEvent();
}

static inline   
#line 296
void TimerM$AlarmCompare$fired(void)
{
  TOS_post(TimerM$checkShortTimers);
}

# 45 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB3$fired(void){
#line 45
  TimerM$AlarmCompare$fired();
#line 45
}
#line 45
static inline   
# 249 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB4$enableEvents(void)
#line 249
{
#line 249
  MSP430TimerM$TBCCTL4 |= 0x0010;
}

# 36 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerJiffyAsyncM$Alarm$enableEvents(void){
#line 36
  MSP430TimerM$CompareB4$enableEvents();
#line 36
}
#line 36
static inline   
# 225 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB4$clearPendingInterrupt(void)
#line 225
{
#line 225
  MSP430TimerM$TBCCTL4 &= ~0x0001;
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerJiffyAsyncM$Alarm$clearPendingInterrupt(void){
#line 31
  MSP430TimerM$CompareB4$clearPendingInterrupt();
#line 31
}
#line 31
static inline   
# 297 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB4$setEventFromNow(uint16_t x)
#line 297
{
#line 297
  MSP430TimerM$TBCCR4 = TBR + x;
}

# 43 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void TimerJiffyAsyncM$Alarm$setEventFromNow(uint16_t arg_0xa41b658){
#line 43
  MSP430TimerM$CompareB4$setEventFromNow(arg_0xa41b658);
#line 43
}
#line 43
static inline   
# 439 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$HPLChipconFIFO$TXFIFODone(uint8_t length, uint8_t *data)
#line 439
{
  CC2420RadioM$tryToSend();
  return SUCCESS;
}

# 48 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
inline static   result_t HPLCC2420M$HPLCC2420FIFO$TXFIFODone(uint8_t arg_0xa6fd6c8, uint8_t *arg_0xa6fd828){
#line 48
  unsigned char result;
#line 48

#line 48
  result = CC2420RadioM$HPLChipconFIFO$TXFIFODone(arg_0xa6fd6c8, arg_0xa6fd828);
#line 48

#line 48
  return result;
#line 48
}
#line 48
static inline  
# 320 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/HPLCC2420M.nc"
void HPLCC2420M$signalTXFIFO(void)
#line 320
{
  HPLCC2420M$HPLCC2420FIFO$TXFIFODone(HPLCC2420M$txlen, HPLCC2420M$txbuf);
}

static inline   







result_t HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(uint8_t length, uint8_t *data)
#line 332
{
  uint8_t i = 0;

#line 334
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 335
        {
          HPLCC2420M$txlen = length;
          HPLCC2420M$txbuf = data;
        }
#line 338
        __nesc_atomic_end(__nesc_atomic); }
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(0x3E);
      TOSH_uwait(20);
      for (i = 0; i < HPLCC2420M$txlen; i++) {
          HPLCC2420M$USARTControl$tx(HPLCC2420M$txbuf[i]);
          TOSH_uwait(20);
        }
      TOSH_uwait(20);
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$BusArbitration$releaseBus();
      return TOS_post(HPLCC2420M$signalTXFIFO);
    }
  return FAIL;
}

# 27 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
inline static   result_t CC2420RadioM$HPLChipconFIFO$writeTXFIFO(uint8_t arg_0xa6fc9b8, uint8_t *arg_0xa6fcb18){
#line 27
  unsigned char result;
#line 27

#line 27
  result = HPLCC2420M$HPLCC2420FIFO$writeTXFIFO(arg_0xa6fc9b8, arg_0xa6fcb18);
#line 27

#line 27
  return result;
#line 27
}
#line 27
# 52 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static   uint8_t CC2420RadioM$HPLChipcon$cmd(uint8_t arg_0xa6dc2f8){
#line 52
  unsigned char result;
#line 52

#line 52
  result = HPLCC2420M$HPLCC2420$cmd(arg_0xa6dc2f8);
#line 52

#line 52
  return result;
#line 52
}
#line 52
static inline  
# 245 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
void CC2420RadioM$startSend(void)
#line 245
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

static inline   
#line 282
result_t CC2420RadioM$BackoffTimerJiffy$fired(void)
#line 282
{
  uint8_t currentstate;

#line 284
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 284
    currentstate = CC2420RadioM$stateRadio;
#line 284
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
            CC2420RadioM$txbufptr->ack = 0;
            TOS_post(CC2420RadioM$PacketSent);
          }
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
# 40 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/TimerJiffyAsyncM.nc"
void TimerJiffyAsyncM$Alarm$fired(void)
{
  if (TimerJiffyAsyncM$jiffy < 0xFFFF) {
      TimerJiffyAsyncM$Alarm$disableEvents();
      TimerJiffyAsyncM$bSet = FALSE;
      TimerJiffyAsyncM$TimerJiffyAsync$fired();
    }
  else {
      TimerJiffyAsyncM$jiffy = TimerJiffyAsyncM$jiffy >> 16;
      TimerJiffyAsyncM$Alarm$setEventFromNow(TimerJiffyAsyncM$jiffy);
      TimerJiffyAsyncM$Alarm$clearPendingInterrupt();
      TimerJiffyAsyncM$Alarm$enableEvents();
    }
}

# 45 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB4$fired(void){
#line 45
  TimerJiffyAsyncM$Alarm$fired();
#line 45
}
#line 45
static inline  
# 210 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
result_t AMStandard$RadioSend$sendDone(TOS_MsgPtr msg, result_t success)
#line 210
{
  return AMStandard$reportSendDone(msg, success);
}

# 67 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
inline static  result_t CC2420RadioM$Send$sendDone(TOS_MsgPtr arg_0xa6a0e88, result_t arg_0xa6a0fd8){
#line 67
  unsigned char result;
#line 67

#line 67
  result = AMStandard$RadioSend$sendDone(arg_0xa6a0e88, arg_0xa6a0fd8);
#line 67

#line 67
  return result;
#line 67
}
#line 67
static inline  
# 225 "BSN_RSUM.nc"
result_t BSN_RSUM$DataMsg$sendDone(TOS_MsgPtr sent, result_t success)
#line 225
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
inline static  result_t AMStandard$SendMsg$sendDone(uint8_t arg_0xa6848c8, TOS_MsgPtr arg_0xa529ce8, result_t arg_0xa529e38){
#line 49
  unsigned char result;
#line 49

#line 49
  switch (arg_0xa6848c8) {
#line 49
    case AM_OSCOPEMSG:
#line 49
      result = BSN_RSUM$DataMsg$sendDone(arg_0xa529ce8, arg_0xa529e38);
#line 49
      break;
#line 49
    default:
#line 49
      result = AMStandard$SendMsg$default$sendDone(arg_0xa6848c8, arg_0xa529ce8, arg_0xa529e38);
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
# 24 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/hardware.h"
uint8_t TOSH_READ_RADIO_CCA_PIN(void)
#line 24
{
   
#line 24
  static volatile uint8_t r __asm ("0x0020");

#line 24
  return r & (1 << 4);
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
# 466 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
int16_t CC2420RadioM$MacBackoff$default$congestionBackoff(TOS_MsgPtr m)
#line 466
{
  return (CC2420RadioM$Random$rand() & 0xF) + 1;
}

# 75 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/MacBackoff.nc"
inline static   int16_t CC2420RadioM$MacBackoff$congestionBackoff(TOS_MsgPtr arg_0xa6c9f40){
#line 75
  int result;
#line 75

#line 75
  result = CC2420RadioM$MacBackoff$default$congestionBackoff(arg_0xa6c9f40);
#line 75

#line 75
  return result;
#line 75
}
#line 75
# 6 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/TimerJiffyAsync.nc"
inline static   result_t CC2420RadioM$BackoffTimerJiffy$setOneShot(uint32_t arg_0xa6ff210){
#line 6
  unsigned char result;
#line 6

#line 6
  result = TimerJiffyAsyncM$TimerJiffyAsync$setOneShot(arg_0xa6ff210);
#line 6

#line 6
  return result;
#line 6
}
#line 6
static 
# 125 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
__inline result_t CC2420RadioM$setBackoffTimer(uint16_t jiffy)
#line 125
{
  CC2420RadioM$stateTimer = CC2420RadioM$TIMER_BACKOFF;
  return CC2420RadioM$BackoffTimerJiffy$setOneShot(jiffy);
}

static __inline result_t CC2420RadioM$setAckTimer(uint16_t jiffy)
#line 130
{
  CC2420RadioM$stateTimer = CC2420RadioM$TIMER_ACK;
  return CC2420RadioM$BackoffTimerJiffy$setOneShot(jiffy);
}

static inline 
# 28 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/hardware.h"
uint8_t TOSH_READ_CC_SFD_PIN(void)
#line 28
{
   
#line 28
  static volatile uint8_t r __asm ("0x001C");

#line 28
  return r & (1 << 1);
}

static inline 
# 219 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
void CC2420RadioM$sendPacket(void)
#line 219
{
  uint8_t status;

  CC2420RadioM$HPLChipcon$cmd(0x05);
  status = CC2420RadioM$HPLChipcon$cmd(0x00);
  if ((status >> 3) & 0x01) {
      TOSH_uwait(450);
      while (TOSH_READ_CC_SFD_PIN()) {
        }
#line 226
      ;
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 227
        CC2420RadioM$stateRadio = CC2420RadioM$POST_TX_STATE;
#line 227
        __nesc_atomic_end(__nesc_atomic); }
      if (CC2420RadioM$bAckEnable) {
          if (!CC2420RadioM$setAckTimer(75)) {
            CC2420RadioM$sendFailed();
            }
        }
      else 
#line 232
        {
          if (!TOS_post(CC2420RadioM$PacketSent)) {
            CC2420RadioM$sendFailed();
            }
        }
    }
  else 
#line 237
    {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 238
        CC2420RadioM$stateRadio = CC2420RadioM$PRE_TX_STATE;
#line 238
        __nesc_atomic_end(__nesc_atomic); }
      if (!CC2420RadioM$setBackoffTimer(CC2420RadioM$MacBackoff$congestionBackoff(CC2420RadioM$txbufptr) * 10)) {
          CC2420RadioM$sendFailed();
        }
    }
}

static inline    
# 201 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB5$default$fired(void)
#line 201
{
}

# 45 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB5$fired(void){
#line 45
  MSP430TimerM$CompareB5$default$fired();
#line 45
}
#line 45
static inline    
# 202 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB6$default$fired(void)
#line 202
{
}

# 45 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430TimerM$CompareB6$fired(void){
#line 45
  MSP430TimerM$CompareB6$default$fired();
#line 45
}
#line 45
static inline  
# 260 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
void TimerM$checkLongTimers(void)
{
  uint8_t head = TimerM$m_head_long;

#line 263
  TimerM$m_head_long = TimerM$EMPTY_LIST;
  TimerM$executeTimers(head);
  TimerM$setNextShortEvent();
}

static inline   
#line 301
void TimerM$AlarmTimer$overflow(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 303
    TimerM$m_hinow++;
#line 303
    __nesc_atomic_end(__nesc_atomic); }
  TOS_post(TimerM$checkLongTimers);
}

# 32 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Timer.nc"
inline static   void MSP430TimerM$TimerB$overflow(void){
#line 32
  TimerM$AlarmTimer$overflow();
#line 32
}
#line 32
static inline   
# 860 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
void MSP430ADC12M$HPLADC12$memOverflow(void)
#line 860
{
}

static inline   
# 248 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVoltM.nc"
void RefVoltM$HPLADC12$memOverflow(void)
#line 248
{
}

# 61 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   void HPLADC12M$HPLADC12$memOverflow(void){
#line 61
  RefVoltM$HPLADC12$memOverflow();
#line 61
  MSP430ADC12M$HPLADC12$memOverflow();
#line 61
}
#line 61
static inline   
# 861 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
void MSP430ADC12M$HPLADC12$timeOverflow(void)
#line 861
{
}

static inline   
# 249 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVoltM.nc"
void RefVoltM$HPLADC12$timeOverflow(void)
#line 249
{
}

# 62 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   void HPLADC12M$HPLADC12$timeOverflow(void){
#line 62
  RefVoltM$HPLADC12$timeOverflow();
#line 62
  MSP430ADC12M$HPLADC12$timeOverflow();
#line 62
}
#line 62
static inline    
# 669 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
void MSP430ADC12M$MSP430ADC12Advanced$default$dataReadySequenceRpt(uint8_t num)
#line 669
{
}

# 157 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12Advanced.nc"
inline static   void MSP430ADC12M$MSP430ADC12Advanced$dataReadySequenceRpt(uint8_t arg_0xa5c2898){
#line 157
    MSP430ADC12M$MSP430ADC12Advanced$default$dataReadySequenceRpt(arg_0xa5c2898);
#line 157
}
#line 157
static inline   
# 91 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
uint16_t HPLADC12M$HPLADC12$getMem(uint8_t i)
#line 91
{
  return *((uint16_t *)(int *)0x0140 + i);
}

# 52 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   uint16_t MSP430ADC12M$HPLADC12$getMem(uint8_t arg_0xa5bd908){
#line 52
  unsigned int result;
#line 52

#line 52
  result = HPLADC12M$HPLADC12$getMem(arg_0xa5bd908);
#line 52

#line 52
  return result;
#line 52
}
#line 52
#line 51
inline static   adc12memctl_t MSP430ADC12M$HPLADC12$getMemControl(uint8_t arg_0xa5bd4f0){
#line 51
  struct __nesc_unnamed4269 result;
#line 51

#line 51
  result = HPLADC12M$HPLADC12$getMemControl(arg_0xa5bd4f0);
#line 51

#line 51
  return result;
#line 51
}
#line 51
static inline    
# 667 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
void MSP430ADC12M$MSP430ADC12Advanced$default$dataReadySequence(uint8_t num)
#line 667
{
}

# 151 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12Advanced.nc"
inline static   void MSP430ADC12M$MSP430ADC12Advanced$dataReadySequence(uint8_t arg_0xa5c2898){
#line 151
    MSP430ADC12M$MSP430ADC12Advanced$default$dataReadySequence(arg_0xa5c2898);
#line 151
}
#line 151
# 58 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   void MSP430ADC12M$HPLADC12$resetIFGs(void){
#line 58
  HPLADC12M$HPLADC12$resetIFGs();
#line 58
}
#line 58
static inline   
# 117 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
void HPLADC12M$HPLADC12$stopConversion(void)
#line 117
{
  HPLADC12M$ADC12CTL1 &= ~((1 << 1) | (3 << 1));
  HPLADC12M$ADC12CTL0 &= ~0x0002;
}

# 82 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   void MSP430ADC12M$HPLADC12$stopConversion(void){
#line 82
  HPLADC12M$HPLADC12$stopConversion();
#line 82
}
#line 82
static inline   
# 254 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB1$disableEvents(void)
#line 254
{
#line 254
  MSP430TimerM$TBCCTL1 &= ~0x0010;
}

# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ADC12M$CompareB1$disableEvents(void){
#line 37
  MSP430TimerM$CompareB1$disableEvents();
#line 37
}
#line 37
static inline   
# 253 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareB0$disableEvents(void)
#line 253
{
#line 253
  MSP430TimerM$TBCCTL0 &= ~0x0010;
}

# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ADC12M$CompareB0$disableEvents(void){
#line 37
  MSP430TimerM$CompareB0$disableEvents();
#line 37
}
#line 37
static inline   
# 150 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void MSP430TimerM$CompareA1$disableEvents(void)
#line 150
{
#line 150
  MSP430TimerM$TACCTL1 &= ~0x0010;
}

# 37 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Compare.nc"
inline static   void MSP430ADC12M$CompareA1$disableEvents(void){
#line 37
  MSP430TimerM$CompareA1$disableEvents();
#line 37
}
#line 37
static 
# 758 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
__inline void MSP430ADC12M$disableTimers(void)
#line 758
{

  if ((
#line 759
  MSP430ADC12M$cmode == ADVANCED_SEQUENCE_OF_CHANNELS || 
  MSP430ADC12M$cmode == ADVANCED_REPEAT_SINGLE_CHANNEL) || 
  MSP430ADC12M$cmode == ADVANCED_REPEAT_SEQUENCE_OF_CHANNELS) 
    {
      switch (MSP430ADC12M$advancedTimer) 
        {
          case TIMERA_OUT1: MSP430ADC12M$CompareA1$disableEvents();
#line 765
          break;
          case TIMERB_OUT0: MSP430ADC12M$CompareB0$disableEvents();
#line 766
          break;
          case TIMERB_OUT1: MSP430ADC12M$CompareB1$disableEvents();
#line 767
          break;
        }
    }
}

static inline    
#line 668
void MSP430ADC12M$MSP430ADC12Advanced$default$dataReadySingleRpt(uint8_t num, uint16_t data)
#line 668
{
}

# 145 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12Advanced.nc"
inline static   void MSP430ADC12M$MSP430ADC12Advanced$dataReadySingleRpt(uint8_t arg_0xa5c2898, uint16_t arg_0xa5c4cb8){
#line 145
    MSP430ADC12M$MSP430ADC12Advanced$default$dataReadySingleRpt(arg_0xa5c2898, arg_0xa5c4cb8);
#line 145
}
#line 145
static inline   
# 200 "BSN_RSUM.nc"
result_t BSN_RSUM$Temp$dataReady(uint16_t data)
{
  BSN_RSUM$SendData(data, 0);
  return SUCCESS;
}

static inline   






result_t BSN_RSUM$AccelY$dataReady(uint16_t data)
{
  BSN_RSUM$SendData(data, 2);
  return SUCCESS;
}

static inline   
#line 206
result_t BSN_RSUM$AccelX$dataReady(uint16_t data)
{
  BSN_RSUM$SendData(data, 1);
  return SUCCESS;
}

static inline    
# 179 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/ADCM.nc"
result_t ADCM$ADC$default$dataReady(uint8_t port, uint16_t data)
#line 179
{
#line 179
  return FAIL;
}

# 70 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ADC.nc"
inline static   result_t ADCM$ADC$dataReady(uint8_t arg_0xa59e788, uint16_t arg_0xa4fb2c8){
#line 70
  unsigned char result;
#line 70

#line 70
  switch (arg_0xa59e788) {
#line 70
    case 1:
#line 70
      result = BSN_RSUM$Temp$dataReady(arg_0xa4fb2c8);
#line 70
      break;
#line 70
    case 2:
#line 70
      result = BSN_RSUM$AccelX$dataReady(arg_0xa4fb2c8);
#line 70
      break;
#line 70
    case 3:
#line 70
      result = BSN_RSUM$AccelY$dataReady(arg_0xa4fb2c8);
#line 70
      break;
#line 70
    default:
#line 70
      result = ADCM$ADC$default$dataReady(arg_0xa59e788, arg_0xa4fb2c8);
#line 70
    }
#line 70

#line 70
  return result;
#line 70
}
#line 70
static inline   
# 170 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/ADCM.nc"
void ADCM$MSP430ADC12Basic$dataReadySingle(uint16_t data)
{
  result_t result = ADCM$ADC$dataReady(ADCM$owner, data);

#line 173
  if (result == SUCCESS && ADCM$continuousData) {
    if (ADCM$MSP430ADC12Basic$getSingleData() == SUCCESS) {
      return;
      }
    }
#line 176
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 176
    ADCM$busy = FALSE;
#line 176
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
# 665 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
void MSP430ADC12M$MSP430ADC12Basic$default$dataReadySingle(uint8_t num, uint16_t x)
#line 665
{
}

# 99 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12Basic.nc"
inline static   void MSP430ADC12M$MSP430ADC12Basic$dataReadySingle(uint8_t arg_0xa5c2158, uint16_t arg_0xa59d600){
#line 99
  switch (arg_0xa5c2158) {
#line 99
    case 0:
#line 99
      ADCM$MSP430ADC12Basic$dataReadySingle(arg_0xa59d600);
#line 99
      break;
#line 99
    default:
#line 99
      MSP430ADC12M$MSP430ADC12Basic$default$dataReadySingle(arg_0xa5c2158, arg_0xa59d600);
#line 99
    }
#line 99
}
#line 99
static inline  
# 772 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
void MSP430ADC12M$conversionCancelled(void)
{
  if (MSP430ADC12M$advancedInterval > 0) {
    MSP430ADC12M$disableTimers();
    }
#line 776
  if (MSP430ADC12M$aSettings[MSP430ADC12M$owner].gotRefVolt == 1) 
    {
      MSP430ADC12M$RefVolt$release();
      MSP430ADC12M$aSettings[MSP430ADC12M$owner].gotRefVolt = 0;
    }
  MSP430ADC12M$releaseLockAdvanced(&MSP430ADC12M$aSettings[MSP430ADC12M$owner]);
  MSP430ADC12M$cmode = ADC_IDLE;
  TOS_post(MSP430ADC12M$checkQueue);
}

static inline   void MSP430ADC12M$HPLADC12$converted(uint8_t number)
#line 786
{

  volatile uint16_t data = MSP430ADC12M$HPLADC12$getMem(number);

#line 789
  if (MSP430ADC12M$cancelled == TRUE) {
      MSP430ADC12M$HPLADC12$stopConversion();
      MSP430ADC12M$HPLADC12$disableConversion();
      MSP430ADC12M$HPLADC12$setIEFlags(0);
      MSP430ADC12M$HPLADC12$resetIFGs();
      TOS_post(MSP430ADC12M$conversionCancelled);
    }
  else 
#line 795
    {
      switch (MSP430ADC12M$cmode) 
        {
          case SINGLE_CHANNEL_SINGLE_CONVERSION: 
            {
              volatile uint8_t ownerTmp = MSP430ADC12M$owner;

#line 801
              data = MSP430ADC12M$HPLADC12$getMem(0);
              if (MSP430ADC12M$bSettings[ownerTmp].gotRefVolt) {

                  MSP430ADC12M$RefVolt$release();
                  MSP430ADC12M$bSettings[ownerTmp].gotRefVolt = 0;
                }
              MSP430ADC12M$bSettings[ownerTmp].queued = 0;
              MSP430ADC12M$releaseLockBasic(&MSP430ADC12M$bSettings[ownerTmp]);
              MSP430ADC12M$cmode = ADC_IDLE;
              TOS_post(MSP430ADC12M$checkQueue);
              MSP430ADC12M$MSP430ADC12Basic$dataReadySingle(ownerTmp, data);
              break;
            }
          case ADVANCED_REPEAT_SINGLE_CHANNEL: 
            MSP430ADC12M$MSP430ADC12Advanced$dataReadySingleRpt(MSP430ADC12M$owner, data);
          break;
          case ADVANCED_SEQUENCE_OF_CHANNELS: 
            {
              volatile uint8_t ownerTmp = MSP430ADC12M$owner;
              uint8_t i = 0;
              adc12memctl_t memctl;

#line 822
              if (MSP430ADC12M$advancedInterval > 0) {
                MSP430ADC12M$disableTimers();
                }
#line 824
              MSP430ADC12M$HPLADC12$stopConversion();
              MSP430ADC12M$HPLADC12$setIEFlags(0);
              MSP430ADC12M$HPLADC12$resetIFGs();
              if (MSP430ADC12M$aSettings[ownerTmp].gotRefVolt) {

                  MSP430ADC12M$releaseRefVoltAdvanced(ownerTmp);
                  MSP430ADC12M$aSettings[ownerTmp].gotRefVolt = 0;
                }
              do {
                  memctl = MSP430ADC12M$HPLADC12$getMemControl(i);
                  * MSP430ADC12M$seqResultPtr++ = MSP430ADC12M$HPLADC12$getMem(i++);
                }
              while (
#line 835
              ! memctl.eos);
              MSP430ADC12M$releaseLockAdvanced(&MSP430ADC12M$aSettings[ownerTmp]);
              MSP430ADC12M$cmode = ADC_IDLE;
              TOS_post(MSP430ADC12M$checkQueue);
              MSP430ADC12M$MSP430ADC12Advanced$dataReadySequence(ownerTmp);
            }
          break;
          case ADVANCED_REPEAT_SEQUENCE_OF_CHANNELS: 
            {
              uint8_t i = 0;
              adc12memctl_t memctl;

#line 846
              do {
                  memctl = MSP430ADC12M$HPLADC12$getMemControl(i);
                  * MSP430ADC12M$seqResultPtr++ = MSP430ADC12M$HPLADC12$getMem(i++);
                }
              while (
#line 849
              ! memctl.eos);
              MSP430ADC12M$seqResultPtr -= i;
              MSP430ADC12M$MSP430ADC12Advanced$dataReadySequenceRpt(MSP430ADC12M$owner);
            }
          break;
          default: 
            break;
        }
    }
}

static inline   
# 250 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVoltM.nc"
void RefVoltM$HPLADC12$converted(uint8_t number)
#line 250
{
}

# 63 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12.nc"
inline static   void HPLADC12M$HPLADC12$converted(uint8_t arg_0xa5dafa0){
#line 63
  RefVoltM$HPLADC12$converted(arg_0xa5dafa0);
#line 63
  MSP430ADC12M$HPLADC12$converted(arg_0xa5dafa0);
#line 63
}
#line 63
static inline 
# 13 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/hardware.h"
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

static inline   








result_t LedsC$Leds$yellowToggle(void)
#line 148
{
  result_t rval;

#line 150
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 150
    {
      if (LedsC$ledsOn & LedsC$YELLOW_BIT) {
        rval = LedsC$Leds$yellowOff();
        }
      else {
#line 154
        rval = LedsC$Leds$yellowOn();
        }
    }
#line 156
    __nesc_atomic_end(__nesc_atomic); }
#line 156
  return rval;
}

# 131 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/Leds.nc"
inline static   result_t BSN_RSUM$Leds$yellowToggle(void){
#line 131
  unsigned char result;
#line 131

#line 131
  result = LedsC$Leds$yellowToggle();
#line 131

#line 131
  return result;
#line 131
}
#line 131
static inline 
# 132 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
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
# 460 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
int16_t CC2420RadioM$MacBackoff$default$initialBackoff(TOS_MsgPtr m)
#line 460
{
  return (CC2420RadioM$Random$rand() & 0xF) + 1;
}

# 74 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/MacBackoff.nc"
inline static   int16_t CC2420RadioM$MacBackoff$initialBackoff(TOS_MsgPtr arg_0xa6c9b18){
#line 74
  int result;
#line 74

#line 74
  result = CC2420RadioM$MacBackoff$default$initialBackoff(arg_0xa6c9b18);
#line 74

#line 74
  return result;
#line 74
}
#line 74
static 
# 120 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
__inline result_t CC2420RadioM$setInitialTimer(uint16_t jiffy)
#line 120
{
  CC2420RadioM$stateTimer = CC2420RadioM$TIMER_INITIAL;
  return CC2420RadioM$BackoffTimerJiffy$setOneShot(jiffy);
}

static inline  
#line 315
result_t CC2420RadioM$Send$send(TOS_MsgPtr pMsg)
#line 315
{

  uint8_t currentstate;

#line 318
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 318
    currentstate = CC2420RadioM$stateRadio;
#line 318
    __nesc_atomic_end(__nesc_atomic); }

  if (currentstate == CC2420RadioM$IDLE_STATE) {

      pMsg->fcflo = 0x08;
      if (CC2420RadioM$bAckEnable) {
        pMsg->fcfhi = 0x21;
        }
      else {
#line 326
        pMsg->fcfhi = 0x01;
        }
      pMsg->destpan = TOS_BCAST_ADDR;

      pMsg->addr = toLSB16(pMsg->addr);

      pMsg->length = pMsg->length + MSG_HEADER_SIZE + MSG_FOOTER_SIZE;

      pMsg->dsn = ++CC2420RadioM$currentDSN;

      CC2420RadioM$txlength = pMsg->length - MSG_FOOTER_SIZE;
      CC2420RadioM$txbufptr = pMsg;
      CC2420RadioM$countRetry = 8;
      if (CC2420RadioM$setInitialTimer(CC2420RadioM$MacBackoff$initialBackoff(CC2420RadioM$txbufptr) * 10)) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 340
            CC2420RadioM$stateRadio = CC2420RadioM$PRE_TX_STATE;
#line 340
            __nesc_atomic_end(__nesc_atomic); }
          return SUCCESS;
        }
    }
  return FAIL;
}

# 58 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/BareSendMsg.nc"
inline static  result_t AMStandard$RadioSend$send(TOS_MsgPtr arg_0xa6a0970){
#line 58
  unsigned char result;
#line 58

#line 58
  result = CC2420RadioM$Send$send(arg_0xa6a0970);
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
inline static  result_t AMStandard$UARTSend$send(TOS_MsgPtr arg_0xa6a0970){
#line 58
  unsigned char result;
#line 58

#line 58
  result = FramerM$BareSendMsg$send(arg_0xa6a0970);
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

static inline  result_t AMStandard$SendMsg$send(uint8_t id, uint16_t addr, uint8_t length, TOS_MsgPtr data)
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

# 48 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/SendMsg.nc"
inline static  result_t BSN_RSUM$DataMsg$send(uint16_t arg_0xa529630, uint8_t arg_0xa529778, TOS_MsgPtr arg_0xa5298c8){
#line 48
  unsigned char result;
#line 48

#line 48
  result = AMStandard$SendMsg$send(AM_OSCOPEMSG, arg_0xa529630, arg_0xa529778, arg_0xa5298c8);
#line 48

#line 48
  return result;
#line 48
}
#line 48
static inline  
# 122 "BSN_RSUM.nc"
void BSN_RSUM$dataTask(void)
#line 122
{

  struct UbiMonMsg *pack;

#line 125
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 125
    {

      pack = (struct UbiMonMsg *)BSN_RSUM$msg[BSN_RSUM$currentMsg].data;
      BSN_RSUM$packetReadingNumber = 0;
    }
#line 129
    __nesc_atomic_end(__nesc_atomic); }



  pack->sourceMoteID = TOS_LOCAL_ADDRESS;






  if (BSN_RSUM$DataMsg$send(TOS_BCAST_ADDR, sizeof(struct UbiMonMsg ), 
  &BSN_RSUM$msg[BSN_RSUM$currentMsg])) 
    {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 143
        {
          BSN_RSUM$currentMsg ^= 0x1;
        }
#line 145
        __nesc_atomic_end(__nesc_atomic); }
      BSN_RSUM$Leds$yellowToggle();
    }
}

static inline   
# 281 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
result_t HPLUSART1M$USARTControl$tx(uint8_t data)
#line 281
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 282
    U1TXBUF = data;
#line 282
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 142 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTControl.nc"
inline static   result_t HPLUARTM$USARTControl$tx(uint8_t arg_0xa7709d0){
#line 142
  unsigned char result;
#line 142

#line 142
  result = HPLUSART1M$USARTControl$tx(arg_0xa7709d0);
#line 142

#line 142
  return result;
#line 142
}
#line 142
static inline   
# 80 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
result_t HPLUARTM$UART$put(uint8_t data)
#line 80
{
  return HPLUARTM$USARTControl$tx(data);
}

# 80 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/HPLUART.nc"
inline static   result_t UARTM$HPLUART$put(uint8_t arg_0xa8b43e0){
#line 80
  unsigned char result;
#line 80

#line 80
  result = HPLUARTM$UART$put(arg_0xa8b43e0);
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
inline static  result_t FramerM$BareSendMsg$sendDone(TOS_MsgPtr arg_0xa6a0e88, result_t arg_0xa6a0fd8){
#line 67
  unsigned char result;
#line 67

#line 67
  result = AMStandard$UARTSend$sendDone(arg_0xa6a0e88, arg_0xa6a0fd8);
#line 67

#line 67
  return result;
#line 67
}
#line 67
static inline  
# 249 "BSN_RSUM.nc"
TOS_MsgPtr BSN_RSUM$ResetCounterMsg$receive(TOS_MsgPtr m)
#line 249
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 250
    {
      BSN_RSUM$readingNumber = 0;
    }
#line 252
    __nesc_atomic_end(__nesc_atomic); }
  return m;
}

static inline   
# 242 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
TOS_MsgPtr AMStandard$ReceiveMsg$default$receive(uint8_t id, TOS_MsgPtr msg)
#line 242
{
  return msg;
}

# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
inline static  TOS_MsgPtr AMStandard$ReceiveMsg$receive(uint8_t arg_0xa684e80, TOS_MsgPtr arg_0xa511178){
#line 75
  struct TOS_Msg *result;
#line 75

#line 75
  switch (arg_0xa684e80) {
#line 75
    case AM_OSCOPERESETMSG:
#line 75
      result = BSN_RSUM$ResetCounterMsg$receive(arg_0xa511178);
#line 75
      break;
#line 75
    default:
#line 75
      result = AMStandard$ReceiveMsg$default$receive(arg_0xa684e80, arg_0xa511178);
#line 75
    }
#line 75

#line 75
  return result;
#line 75
}
#line 75
static inline    
# 294 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
result_t HPLUSART0M$USARTData$default$rxDone(uint8_t data)
#line 294
{
#line 294
  return SUCCESS;
}

# 53 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
inline static   result_t HPLUSART0M$USARTData$rxDone(uint8_t arg_0xa7a28e8){
#line 53
  unsigned char result;
#line 53

#line 53
  result = HPLUSART0M$USARTData$default$rxDone(arg_0xa7a28e8);
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 54 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
void __attribute((interrupt(18))) __attribute((wakeup))   sig_UART0RX_VECTOR(void)
#line 54
{
  uint8_t temp = U0RXBUF;

#line 56
  HPLUSART0M$USARTData$rxDone(temp);
}

static inline    
#line 292
result_t HPLUSART0M$USARTData$default$txDone(void)
#line 292
{
#line 292
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
# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
void __attribute((interrupt(16))) __attribute((wakeup))   sig_UART0TX_VECTOR(void)
#line 59
{
  HPLUSART0M$USARTData$txDone();
}

static inline   
# 343 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420ControlM.nc"
result_t CC2420ControlM$HPLChipcon$FIFOPIntr(void)
#line 343
{
  return SUCCESS;
}

static inline  
# 252 "C:/cygwin/opt/tinyos-1.x/tos/system/AMStandard.nc"
TOS_MsgPtr AMStandard$RadioReceive$receive(TOS_MsgPtr packet)
#line 252
{
  return received(packet);
}

# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ReceiveMsg.nc"
inline static  TOS_MsgPtr CC2420RadioM$Receive$receive(TOS_MsgPtr arg_0xa511178){
#line 75
  struct TOS_Msg *result;
#line 75

#line 75
  result = AMStandard$RadioReceive$receive(arg_0xa511178);
#line 75

#line 75
  return result;
#line 75
}
#line 75
static inline  
# 139 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
void CC2420RadioM$PacketRcvd(void)
#line 139
{
  TOS_MsgPtr pBuf;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 142
    {
      CC2420RadioM$rxbufptr->time = 0;
      pBuf = CC2420RadioM$rxbufptr;
    }
#line 145
    __nesc_atomic_end(__nesc_atomic); }
  pBuf = CC2420RadioM$Receive$receive((TOS_MsgPtr )pBuf);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 147
    {
      if (pBuf) {
#line 148
        CC2420RadioM$rxbufptr = pBuf;
        }
#line 149
      CC2420RadioM$rxbufptr->length = 0;
    }
#line 150
    __nesc_atomic_end(__nesc_atomic); }
}

static 
# 21 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/byteorder.h"
__inline uint16_t fromLSB16(uint16_t a)
{
  return is_host_lsb() ? a : (a << 8) | (a >> 8);
}

# 66 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static   uint16_t CC2420RadioM$HPLChipcon$read(uint8_t arg_0xa6dcd48){
#line 66
  unsigned int result;
#line 66

#line 66
  result = HPLCC2420M$HPLCC2420$read(arg_0xa6dcd48);
#line 66

#line 66
  return result;
#line 66
}
#line 66
static inline 
# 27 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/hardware.h"
uint8_t TOSH_READ_CC_FIFO_PIN(void)
#line 27
{
   
#line 27
  static volatile uint8_t r __asm ("0x0020");

#line 27
  return r & (1 << 3);
}

static inline   
# 393 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$HPLChipconFIFO$RXFIFODone(uint8_t length, uint8_t *data)
#line 393
{
  uint8_t currentstate;

#line 395
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 395
    currentstate = CC2420RadioM$stateRadio;
#line 395
    __nesc_atomic_end(__nesc_atomic); }

  if (TOSH_READ_CC_FIFO_PIN()) {
      CC2420RadioM$HPLChipcon$read(0x3F);
      CC2420RadioM$HPLChipcon$cmd(0x08);
    }


  if (length > MSG_DATA_SIZE) {
    return SUCCESS;
    }
  CC2420RadioM$rxbufptr = (TOS_MsgPtr )data;


  if (
#line 408
  CC2420RadioM$bAckEnable && currentstate == CC2420RadioM$POST_TX_STATE && (
  CC2420RadioM$rxbufptr->fcfhi & 0x03) == 0x02 && 
  CC2420RadioM$rxbufptr->dsn == CC2420RadioM$currentDSN) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 411
        {
          CC2420RadioM$txbufptr->ack = 1;
          CC2420RadioM$txbufptr->strength = data[length - 2];
          CC2420RadioM$txbufptr->lqi = data[length - 1] & 0x7F;
          currentstate = CC2420RadioM$POST_TX_ACK_STATE;
        }
#line 416
        __nesc_atomic_end(__nesc_atomic); }
      TOS_post(CC2420RadioM$PacketSent);
    }

  if ((CC2420RadioM$rxbufptr->fcfhi & 0x03) != 0x01) {
    return SUCCESS;
    }
  CC2420RadioM$rxbufptr->length = CC2420RadioM$rxbufptr->length - MSG_HEADER_SIZE - MSG_FOOTER_SIZE;

  CC2420RadioM$rxbufptr->addr = fromLSB16(CC2420RadioM$rxbufptr->addr);


  CC2420RadioM$rxbufptr->crc = data[length - 1] >> 7;

  CC2420RadioM$rxbufptr->strength = data[length - 2];

  CC2420RadioM$rxbufptr->lqi = data[length - 1] & 0x7F;

  TOS_post(CC2420RadioM$PacketRcvd);

  return SUCCESS;
}

# 37 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
inline static   result_t HPLCC2420M$HPLCC2420FIFO$RXFIFODone(uint8_t arg_0xa6fd038, uint8_t *arg_0xa6fd198){
#line 37
  unsigned char result;
#line 37

#line 37
  result = CC2420RadioM$HPLChipconFIFO$RXFIFODone(arg_0xa6fd038, arg_0xa6fd198);
#line 37

#line 37
  return result;
#line 37
}
#line 37
static inline  
# 264 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/HPLCC2420M.nc"
void HPLCC2420M$signalRXFIFO(void)
#line 264
{
  HPLCC2420M$HPLCC2420FIFO$RXFIFODone(HPLCC2420M$rxlen, HPLCC2420M$rxbuf);
}

static inline   
#line 279
result_t HPLCC2420M$HPLCC2420FIFO$readRXFIFO(uint8_t length, uint8_t *data)
#line 279
{
  uint8_t i;

#line 281
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 282
        HPLCC2420M$rxbuf = data;
#line 282
        __nesc_atomic_end(__nesc_atomic); }
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(0x3F | 0x40);
      TOSH_uwait(20);
      HPLCC2420M$rxlen = HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(0);
      TOSH_uwait(20);

      HPLCC2420M$rxlen = HPLCC2420M$USARTControl$rx();
      if (HPLCC2420M$rxlen > 0) {
          HPLCC2420M$rxbuf[0] = HPLCC2420M$rxlen;

          HPLCC2420M$rxlen++;

          if (HPLCC2420M$rxlen > length) {
#line 299
            HPLCC2420M$rxlen = length;
            }
#line 300
          for (i = 1; i < HPLCC2420M$rxlen; i++) {
              HPLCC2420M$USARTControl$tx(0);
              TOSH_uwait(20);
              HPLCC2420M$rxbuf[i] = HPLCC2420M$USARTControl$rx();
            }
        }
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$BusArbitration$releaseBus();
    }
  else {
      return FAIL;
    }
  if (HPLCC2420M$rxlen > 0) {
      return TOS_post(HPLCC2420M$signalRXFIFO);
    }
  else {
      return FAIL;
    }
}

# 17 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420FIFO.nc"
inline static   result_t CC2420RadioM$HPLChipconFIFO$readRXFIFO(uint8_t arg_0xa6fc280, uint8_t *arg_0xa6fc3e0){
#line 17
  unsigned char result;
#line 17

#line 17
  result = HPLCC2420M$HPLCC2420FIFO$readRXFIFO(arg_0xa6fc280, arg_0xa6fc3e0);
#line 17

#line 17
  return result;
#line 17
}
#line 17
static inline  
# 348 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
void CC2420RadioM$delayedRXFIFO(void)
#line 348
{
  uint8_t len = MSG_DATA_SIZE;

#line 350
  CC2420RadioM$HPLChipconFIFO$readRXFIFO(len, (uint8_t *)CC2420RadioM$rxbufptr);
}

static inline   
# 78 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/TimerJiffyAsyncM.nc"
result_t TimerJiffyAsyncM$TimerJiffyAsync$stop(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 80
    {
      TimerJiffyAsyncM$bSet = FALSE;
      TimerJiffyAsyncM$Alarm$disableEvents();
    }
#line 83
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
# 73 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/TimerJiffyAsyncM.nc"
bool TimerJiffyAsyncM$TimerJiffyAsync$isSet(void)
{
  return TimerJiffyAsyncM$bSet;
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
# 368 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
result_t CC2420RadioM$HPLChipcon$FIFOPIntr(void)
#line 368
{




  if (CC2420RadioM$bAckEnable && CC2420RadioM$stateRadio == CC2420RadioM$PRE_TX_STATE) {
      if (CC2420RadioM$BackoffTimerJiffy$isSet()) {
          CC2420RadioM$BackoffTimerJiffy$stop();
          CC2420RadioM$BackoffTimerJiffy$setOneShot(CC2420RadioM$MacBackoff$congestionBackoff(CC2420RadioM$txbufptr) * 10 + 75);
        }
    }



  if (!TOSH_READ_CC_FIFO_PIN()) {
      CC2420RadioM$HPLChipcon$read(0x3F);
      CC2420RadioM$HPLChipcon$cmd(0x08);
      return SUCCESS;
    }

  TOS_post(CC2420RadioM$delayedRXFIFO);

  return SUCCESS;
}

# 45 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/HPLCC2420.nc"
inline static   result_t HPLCC2420M$HPLCC2420$FIFOPIntr(void){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420RadioM$HPLChipcon$FIFOPIntr();
#line 45
  result = rcombine(result, CC2420ControlM$HPLChipcon$FIFOPIntr());
#line 45

#line 45
  return result;
#line 45
}
#line 45
static inline   
# 358 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/HPLCC2420M.nc"
void HPLCC2420M$FIFOPInterrupt$fired(void)
#line 358
{
  HPLCC2420M$HPLCC2420$FIFOPIntr();
  HPLCC2420M$FIFOPInterrupt$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port10$fired(void){
#line 31
  HPLCC2420M$FIFOPInterrupt$fired();
#line 31
}
#line 31
static inline   
# 131 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port11$clear(void)
#line 131
{
#line 131
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 131
    P1IFG &= ~(1 << 1);
#line 131
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 77
void MSP430InterruptM$Port11$default$fired(void)
#line 77
{
#line 77
  MSP430InterruptM$Port11$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port11$fired(void){
#line 31
  MSP430InterruptM$Port11$default$fired();
#line 31
}
#line 31
static inline   
# 132 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port12$clear(void)
#line 132
{
#line 132
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 132
    P1IFG &= ~(1 << 2);
#line 132
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 78
void MSP430InterruptM$Port12$default$fired(void)
#line 78
{
#line 78
  MSP430InterruptM$Port12$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port12$fired(void){
#line 31
  MSP430InterruptM$Port12$default$fired();
#line 31
}
#line 31
static inline   
# 133 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port13$clear(void)
#line 133
{
#line 133
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 133
    P1IFG &= ~(1 << 3);
#line 133
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 79
void MSP430InterruptM$Port13$default$fired(void)
#line 79
{
#line 79
  MSP430InterruptM$Port13$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port13$fired(void){
#line 31
  MSP430InterruptM$Port13$default$fired();
#line 31
}
#line 31
static inline   
# 134 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port14$clear(void)
#line 134
{
#line 134
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 134
    P1IFG &= ~(1 << 4);
#line 134
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 80
void MSP430InterruptM$Port14$default$fired(void)
#line 80
{
#line 80
  MSP430InterruptM$Port14$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port14$fired(void){
#line 31
  MSP430InterruptM$Port14$default$fired();
#line 31
}
#line 31
static inline   
# 135 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port15$clear(void)
#line 135
{
#line 135
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 135
    P1IFG &= ~(1 << 5);
#line 135
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 81
void MSP430InterruptM$Port15$default$fired(void)
#line 81
{
#line 81
  MSP430InterruptM$Port15$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port15$fired(void){
#line 31
  MSP430InterruptM$Port15$default$fired();
#line 31
}
#line 31
static inline   
# 136 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port16$clear(void)
#line 136
{
#line 136
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 136
    P1IFG &= ~(1 << 6);
#line 136
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 82
void MSP430InterruptM$Port16$default$fired(void)
#line 82
{
#line 82
  MSP430InterruptM$Port16$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port16$fired(void){
#line 31
  MSP430InterruptM$Port16$default$fired();
#line 31
}
#line 31
static inline   
# 137 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port17$clear(void)
#line 137
{
#line 137
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 137
    P1IFG &= ~(1 << 7);
#line 137
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 83
void MSP430InterruptM$Port17$default$fired(void)
#line 83
{
#line 83
  MSP430InterruptM$Port17$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port17$fired(void){
#line 31
  MSP430InterruptM$Port17$default$fired();
#line 31
}
#line 31
static inline   
# 139 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port20$clear(void)
#line 139
{
#line 139
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 139
    P2IFG &= ~(1 << 0);
#line 139
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 85
void MSP430InterruptM$Port20$default$fired(void)
#line 85
{
#line 85
  MSP430InterruptM$Port20$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port20$fired(void){
#line 31
  MSP430InterruptM$Port20$default$fired();
#line 31
}
#line 31
static inline   
# 140 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port21$clear(void)
#line 140
{
#line 140
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 140
    P2IFG &= ~(1 << 1);
#line 140
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 86
void MSP430InterruptM$Port21$default$fired(void)
#line 86
{
#line 86
  MSP430InterruptM$Port21$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port21$fired(void){
#line 31
  MSP430InterruptM$Port21$default$fired();
#line 31
}
#line 31
static inline   
# 141 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port22$clear(void)
#line 141
{
#line 141
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 141
    P2IFG &= ~(1 << 2);
#line 141
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 87
void MSP430InterruptM$Port22$default$fired(void)
#line 87
{
#line 87
  MSP430InterruptM$Port22$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port22$fired(void){
#line 31
  MSP430InterruptM$Port22$default$fired();
#line 31
}
#line 31
static inline   
# 142 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port23$clear(void)
#line 142
{
#line 142
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 142
    P2IFG &= ~(1 << 3);
#line 142
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 88
void MSP430InterruptM$Port23$default$fired(void)
#line 88
{
#line 88
  MSP430InterruptM$Port23$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port23$fired(void){
#line 31
  MSP430InterruptM$Port23$default$fired();
#line 31
}
#line 31
static inline   
# 143 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port24$clear(void)
#line 143
{
#line 143
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 143
    P2IFG &= ~(1 << 4);
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 89
void MSP430InterruptM$Port24$default$fired(void)
#line 89
{
#line 89
  MSP430InterruptM$Port24$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port24$fired(void){
#line 31
  MSP430InterruptM$Port24$default$fired();
#line 31
}
#line 31
static inline   
# 144 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port25$clear(void)
#line 144
{
#line 144
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 144
    P2IFG &= ~(1 << 5);
#line 144
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 90
void MSP430InterruptM$Port25$default$fired(void)
#line 90
{
#line 90
  MSP430InterruptM$Port25$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port25$fired(void){
#line 31
  MSP430InterruptM$Port25$default$fired();
#line 31
}
#line 31
static inline   
# 145 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port26$clear(void)
#line 145
{
#line 145
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 145
    P2IFG &= ~(1 << 6);
#line 145
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 91
void MSP430InterruptM$Port26$default$fired(void)
#line 91
{
#line 91
  MSP430InterruptM$Port26$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port26$fired(void){
#line 31
  MSP430InterruptM$Port26$default$fired();
#line 31
}
#line 31
static inline   
# 146 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void MSP430InterruptM$Port27$clear(void)
#line 146
{
#line 146
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 146
    P2IFG &= ~(1 << 7);
#line 146
    __nesc_atomic_end(__nesc_atomic); }
}

static inline    
#line 92
void MSP430InterruptM$Port27$default$fired(void)
#line 92
{
#line 92
  MSP430InterruptM$Port27$clear();
}

# 31 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430Interrupt.nc"
inline static   void MSP430InterruptM$Port27$fired(void){
#line 31
  MSP430InterruptM$Port27$default$fired();
#line 31
}
#line 31
# 66 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ByteComm.nc"
inline static   result_t UARTM$ByteComm$rxByteReady(uint8_t arg_0xa87c838, bool arg_0xa87c980, uint16_t arg_0xa87cad8){
#line 66
  unsigned char result;
#line 66

#line 66
  result = FramerM$ByteComm$rxByteReady(arg_0xa87c838, arg_0xa87c980, arg_0xa87cad8);
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
inline static   result_t HPLUARTM$UART$get(uint8_t arg_0xa8b48e0){
#line 88
  unsigned char result;
#line 88

#line 88
  result = UARTM$HPLUART$get(arg_0xa8b48e0);
#line 88

#line 88
  return result;
#line 88
}
#line 88
static inline   
# 72 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
result_t HPLUARTM$USARTData$rxDone(uint8_t b)
#line 72
{
  return HPLUARTM$UART$get(b);
}

# 53 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSARTFeedback.nc"
inline static   result_t HPLUSART1M$USARTData$rxDone(uint8_t arg_0xa7a28e8){
#line 53
  unsigned char result;
#line 53

#line 53
  result = HPLUARTM$USARTData$rxDone(arg_0xa7a28e8);
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 54 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
void __attribute((interrupt(6))) __attribute((wakeup))   sig_UART1RX_VECTOR(void)
#line 54
{
  uint8_t temp = U1RXBUF;

#line 56
  HPLUSART1M$USARTData$rxDone(temp);
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
inline static  TOS_MsgPtr FramerAckM$ReceiveCombined$receive(TOS_MsgPtr arg_0xa511178){
#line 75
  struct TOS_Msg *result;
#line 75

#line 75
  result = AMStandard$UARTReceive$receive(arg_0xa511178);
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
inline static  TOS_MsgPtr FramerM$ReceiveMsg$receive(TOS_MsgPtr arg_0xa511178){
#line 75
  struct TOS_Msg *result;
#line 75

#line 75
  result = FramerAckM$ReceiveMsg$receive(arg_0xa511178);
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
inline static  result_t FramerAckM$TokenReceiveMsg$ReflectToken(uint8_t arg_0xa84c810){
#line 88
  unsigned char result;
#line 88

#line 88
  result = FramerM$TokenReceiveMsg$ReflectToken(arg_0xa84c810);
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
inline static  TOS_MsgPtr FramerM$TokenReceiveMsg$receive(TOS_MsgPtr arg_0xa84c0b0, uint8_t arg_0xa84c1f8){
#line 75
  struct TOS_Msg *result;
#line 75

#line 75
  result = FramerAckM$TokenReceiveMsg$receive(arg_0xa84c0b0, arg_0xa84c1f8);
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


  if (pRcv->Length >= (size_t )& ((struct TOS_Msg *)0)->data) {

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
# 76 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUARTM.nc"
result_t HPLUARTM$USARTData$txDone(void)
#line 76
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
# 59 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART1M.nc"
void __attribute((interrupt(4))) __attribute((wakeup))   sig_UART1TX_VECTOR(void)
#line 59
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

# 83 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ByteComm.nc"
inline static   result_t UARTM$ByteComm$txDone(void){
#line 83
  unsigned char result;
#line 83

#line 83
  result = FramerM$ByteComm$txDone();
#line 83

#line 83
  return result;
#line 83
}
#line 83
#line 55
inline static   result_t FramerM$ByteComm$txByte(uint8_t arg_0xa87c3a8){
#line 55
  unsigned char result;
#line 55

#line 55
  result = UARTM$ByteComm$txByte(arg_0xa87c3a8);
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

# 75 "C:/cygwin/opt/tinyos-1.x/tos/interfaces/ByteComm.nc"
inline static   result_t UARTM$ByteComm$txByteReady(bool arg_0xa87d008){
#line 75
  unsigned char result;
#line 75

#line 75
  result = FramerM$ByteComm$txByteReady(arg_0xa87d008);
#line 75

#line 75
  return result;
#line 75
}
#line 75
# 100 "C:/cygwin/opt/tinyos-1.x/tos/system/sched.c"
bool  TOS_post(void (*tp)(void))
#line 100
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
# 69 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
result_t TimerM$StdControl$init(void)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 71
    TimerM$m_hinow = 0;
#line 71
    __nesc_atomic_end(__nesc_atomic); }
  TimerM$m_head_short = TimerM$EMPTY_LIST;
  TimerM$m_head_long = TimerM$EMPTY_LIST;
  TimerM$AlarmCompare$setControlAsTimer();
  TimerM$AlarmCompare$disableEvents();
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
# 102 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLUSART0M.nc"
void HPLUSART0M$USARTControl$setModeSPI(void)
#line 102
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 103
    {
      TOSH_SEL_SIMO0_MODFUNC();
      TOSH_SEL_SOMI0_MODFUNC();
      TOSH_SEL_UCLK0_MODFUNC();

      IE1 &= ~((1 << 7) | (1 << 6));

      U0CTL |= 0x01;
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
      UCTL0 &= ~0x01;

      HPLUSART0M$IFG1 &= ~((1 << 7) | (1 << 6));
      IE1 &= ~((1 << 7) | (1 << 6));
    }
#line 142
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

static  
# 432 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
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
#line 307
result_t TimerM$setTimer(uint8_t num, int32_t jiffy, bool isperiodic)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      TimerM$Timer_t *timer = &TimerM$m_timers[num];
      int32_t now;

#line 313
      if (timer->isset) {
        TimerM$removeTimer(num);
        }
#line 315
      TimerM$m_period[num] = jiffy;
      timer->isperiodic = isperiodic;
      now = TimerM$LocalTime$read();
      if (isperiodic) {
#line 318
        timer->alarm = now;
        }
      else {
#line 319
        timer->alarm = now + jiffy;
        }
#line 320
      TimerM$insertTimer(num, jiffy <= 0xffffL);
      TimerM$setNextShortEvent();
    }
#line 322
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

static   
#line 268
uint32_t TimerM$LocalTime$read(void)
{
  uint32_t now;
  bool overflow;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {



      uint16_t lonow = TimerM$AlarmTimer$read();

#line 279
      overflow = TimerM$AlarmTimer$isOverflowPending();
      if (overflow) 
        {
          TimerM$AlarmTimer$clearOverflow();
          TimerM$m_hinow++;
          lonow = TimerM$AlarmTimer$read();
        }
      now = ((uint32_t )TimerM$m_hinow << 16) | lonow;
    }
#line 287
    __nesc_atomic_end(__nesc_atomic); }


  if (overflow) {
    TOS_post(TimerM$checkLongTimers);
    }
  return now;
}

static 
#line 168
void TimerM$executeTimers(uint8_t head)
{
  uint32_t now = TimerM$LocalTime$read();

#line 171
  while (head != TimerM$EMPTY_LIST) 
    {
      uint8_t num = head;
      TimerM$Timer_t *timer = &TimerM$m_timers[num];

#line 175
      head = timer->next;

      if (timer->isset) 
        {
          int32_t remaining = timer->alarm - now;

#line 180
          timer->isset = FALSE;
          if (remaining <= 0) 
            {


              if (timer->isperiodic) 
                {
                  timer->alarm += TimerM$m_period[num];
                  TimerM$insertTimer(num, (int32_t )(timer->alarm - now) <= 0xffffL);
                }
              TimerM$signal_timer_fired(num);
            }
          else 
            {

              TimerM$insertTimer(num, remaining <= 0xffffL);
            }
        }
    }
}

static 
#line 89
void TimerM$insertTimer(uint8_t num, bool isshort)
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

  TimerM$m_timers[num].isset = TRUE;
}

static  
# 676 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
void MSP430ADC12M$checkQueue(void)
{
  int16_t i;
  bool access = FALSE;

  if (MSP430ADC12M$cmode != ADC_IDLE) {
    return;
    }
  if (1 > 0) 
    {

      for (i = 0; i < 1; i++) 
        {
          if (MSP430ADC12M$bSettings[MSP430ADC12M$queueOffset].queued && (! MSP430ADC12M$bSettings[MSP430ADC12M$queueOffset].gotRefVolt || (
          MSP430ADC12M$bSettings[MSP430ADC12M$queueOffset].gotRefVolt && MSP430ADC12M$RefVolt$getState() != REFERENCE_UNSTABLE))) {
            break;
            }
#line 692
          MSP430ADC12M$queueOffset = (MSP430ADC12M$queueOffset + 1) % (1 != 0 ? 1 : 1);
        }

      if (i != 1) 
        {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
            {
              if (MSP430ADC12M$cmode == ADC_IDLE) 
                {
                  MSP430ADC12M$bSettings[MSP430ADC12M$queueOffset].queued = 0;
                  MSP430ADC12M$cmode = SINGLE_CHANNEL_SINGLE_CONVERSION;
                  MSP430ADC12M$owner = MSP430ADC12M$queueOffset;
                  MSP430ADC12M$cancelled = FALSE;
                  access = TRUE;
                }
            }
#line 707
            __nesc_atomic_end(__nesc_atomic); }
          if (access) 
            {
              uint16_t ctl0 = 0x0000 | 0x0010;
              uint16_t ctl1 = 0x0218 & ~0x06;

              MSP430ADC12M$HPLADC12$disableConversion();
              MSP430ADC12M$HPLADC12$setControl0_IgnoreRef(* (adc12ctl0_t *)&ctl0);
              MSP430ADC12M$HPLADC12$setControl1(* (adc12ctl1_t *)&ctl1);
              MSP430ADC12M$HPLADC12$setSHT(MSP430ADC12M$bSettings[MSP430ADC12M$queueOffset].sampleHoldTime);
              MSP430ADC12M$HPLADC12$setMemControl(0, MSP430ADC12M$bSettings[MSP430ADC12M$queueOffset].memctl);
              MSP430ADC12M$HPLADC12$setIEFlags(1);
              MSP430ADC12M$HPLADC12$startConversion();
            }
          MSP430ADC12M$queueOffset = (MSP430ADC12M$queueOffset + 1) % (1 != 0 ? 1 : 1);
          if (access) {
            return;
            }
        }
    }
  if (0 > 0) 
    {

      switch (MSP430ADC12M$aCmdBuffer.type) 
        {
          case ADVANCED_REPEAT_SINGLE_CHANNEL: 
            MSP430ADC12M$releaseLockAdvanced(&MSP430ADC12M$aSettings[MSP430ADC12M$aCmdBuffer.intf]);

          if (
#line 734
          MSP430ADC12M$MSP430ADC12Advanced$getSingleDataRepeat(MSP430ADC12M$aCmdBuffer.intf, 
          MSP430ADC12M$aCmdBuffer.jiffies) == ADC_SUCCESS) {
            MSP430ADC12M$aCmdBuffer.type = ADC_IDLE;
            }
          else {
#line 737
            TOS_post(MSP430ADC12M$checkQueue);
            }
#line 738
          break;
          case ADVANCED_SEQUENCE_OF_CHANNELS: 
            MSP430ADC12M$releaseLockAdvanced(&MSP430ADC12M$aSettings[MSP430ADC12M$aCmdBuffer.intf]);

          if (
#line 741
          MSP430ADC12M$internalGetSequenceData(FALSE, MSP430ADC12M$aCmdBuffer.intf, MSP430ADC12M$aCmdBuffer.dataDest, 
          MSP430ADC12M$aCmdBuffer.length, MSP430ADC12M$aCmdBuffer.jiffies) == ADC_SUCCESS) {
            MSP430ADC12M$aCmdBuffer.type = ADC_IDLE;
            }
          else {
#line 744
            TOS_post(MSP430ADC12M$checkQueue);
            }
#line 745
          break;
          case ADVANCED_REPEAT_SEQUENCE_OF_CHANNELS: 
            MSP430ADC12M$releaseLockAdvanced(&MSP430ADC12M$aSettings[MSP430ADC12M$aCmdBuffer.intf]);

          if (
#line 748
          MSP430ADC12M$internalGetSequenceData(TRUE, MSP430ADC12M$aCmdBuffer.intf, MSP430ADC12M$aCmdBuffer.dataDest, 
          MSP430ADC12M$aCmdBuffer.length, MSP430ADC12M$aCmdBuffer.jiffies) == ADC_SUCCESS) {
            MSP430ADC12M$aCmdBuffer.type = ADC_IDLE;
            }
          else {
#line 751
            TOS_post(MSP430ADC12M$checkQueue);
            }
#line 752
          break;
          default: break;
        }
    }
}

static   
# 240 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVoltM.nc"
RefVolt_t RefVoltM$RefVolt$getState(void)
#line 240
{
  if (RefVoltM$state == RefVoltM$REFERENCE_2_5V_STABLE) {
    return REFERENCE_2_5V;
    }
#line 243
  if (RefVoltM$state == RefVoltM$REFERENCE_1_5V_STABLE) {
    return REFERENCE_1_5V;
    }
#line 245
  return REFERENCE_UNSTABLE;
}

static   
# 58 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
void HPLADC12M$HPLADC12$setControl0_IgnoreRef(adc12ctl0_t control0)
#line 58
{
  adc12ctl0_t oldControl0 = * (adc12ctl0_t *)&HPLADC12M$ADC12CTL0;

#line 60
  control0.refon = oldControl0.refon;
  control0.r2_5v = oldControl0.r2_5v;
  HPLADC12M$ADC12CTL0 = * (uint16_t *)&control0;
}

static   
#line 144
void HPLADC12M$HPLADC12$setSHT(uint8_t sht)
#line 144
{
  uint16_t ctl0 = HPLADC12M$ADC12CTL0;
  uint16_t shttemp = sht & 0x0F;

#line 147
  ctl0 &= 0x00FF;
  ctl0 |= shttemp << 8;
  ctl0 |= shttemp << 12;
  HPLADC12M$ADC12CTL0 = ctl0;
}

static   
#line 73
void HPLADC12M$HPLADC12$setMemControl(uint8_t i, adc12memctl_t memControl)
#line 73
{
  uint8_t *memCtlPtr = (uint8_t *)(char *)0x0080;

#line 75
  if (i < 16) {
      memCtlPtr += i;
      *memCtlPtr = * (uint16_t *)&memControl;
    }
}

static   
# 106 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/RefVoltM.nc"
result_t RefVoltM$RefVolt$get(RefVolt_t vref)
#line 106
{
  result_t result = SUCCESS;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (RefVoltM$semaCount == 0) {
          if (RefVoltM$HPLADC12$isBusy()) {
            result = FAIL;
            }
          else 
#line 113
            {
              if (RefVoltM$state == RefVoltM$REFERENCE_OFF) {
                RefVoltM$switchRefOn(vref);
                }
              else {
#line 116
                if ((RefVoltM$state == RefVoltM$REFERENCE_1_5V_PENDING && vref == REFERENCE_2_5V) || (
                RefVoltM$state == RefVoltM$REFERENCE_2_5V_PENDING && vref == REFERENCE_1_5V)) {
                  RefVoltM$switchToRefPending(vref);
                  }
                else {
#line 119
                  if ((RefVoltM$state == RefVoltM$REFERENCE_1_5V_STABLE && vref == REFERENCE_2_5V) || (
                  RefVoltM$state == RefVoltM$REFERENCE_2_5V_STABLE && vref == REFERENCE_1_5V)) {
                    RefVoltM$switchToRefStable(vref);
                    }
                  }
                }
#line 122
              RefVoltM$semaCount++;
              RefVoltM$switchOff = FALSE;
              result = SUCCESS;
            }
        }
      else {

        if ((((
#line 127
        RefVoltM$state == RefVoltM$REFERENCE_1_5V_PENDING && vref == REFERENCE_1_5V) || (
        RefVoltM$state == RefVoltM$REFERENCE_2_5V_PENDING && vref == REFERENCE_2_5V)) || (
        RefVoltM$state == RefVoltM$REFERENCE_1_5V_STABLE && vref == REFERENCE_1_5V)) || (
        RefVoltM$state == RefVoltM$REFERENCE_2_5V_STABLE && vref == REFERENCE_2_5V)) {
            RefVoltM$semaCount++;
            RefVoltM$switchOff = FALSE;
            result = SUCCESS;
          }
        else {
#line 135
          result = FAIL;
          }
        }
    }
#line 138
    __nesc_atomic_end(__nesc_atomic); }
#line 137
  return result;
}

static   
#line 180
result_t RefVoltM$RefVolt$release(void)
#line 180
{
  result_t result = FAIL;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 183
    {
      if (RefVoltM$semaCount <= 0) {
        result = FAIL;
        }
      else 
#line 186
        {
          RefVoltM$semaCount--;
          if (RefVoltM$semaCount == 0) {
              if (RefVoltM$state == RefVoltM$REFERENCE_1_5V_PENDING || 
              RefVoltM$state == RefVoltM$REFERENCE_2_5V_PENDING) {
                  RefVoltM$switchOff = TRUE;
                  RefVoltM$switchRefOff();
                }
              else {
                  RefVoltM$switchOff = TRUE;
                  TOS_post(RefVoltM$switchOffDelay);
                }
              result = SUCCESS;
            }
        }
    }
#line 201
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

static 
# 455 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
adcResult_t MSP430ADC12M$internalGetSequenceData(bool repeat, uint8_t num, uint16_t dataDest[], uint8_t length, uint16_t jiffies)
{
  bool access = FALSE;
  bool buffered = FALSE;




  const int16_t num16 = num;


  if (((
#line 465
  num16 >= 0 || !length) || length > 16)
   || !MSP430ADC12M$getLockAdvanced(&MSP430ADC12M$aSettings[num])) {
    return ADC_FAIL;
    }
  switch (MSP430ADC12M$getRefVoltAdvanced(num)) 
    {
      case ADC_FAIL: 
        MSP430ADC12M$releaseLockAdvanced(&MSP430ADC12M$aSettings[num]);
      break;
      case ADC_SUCCESS: 
        { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
          {
            if (MSP430ADC12M$cmode == ADC_IDLE) {
                MSP430ADC12M$owner = num;
                if (repeat) {
                  MSP430ADC12M$cmode = ADVANCED_REPEAT_SEQUENCE_OF_CHANNELS;
                  }
                else {
#line 482
                  MSP430ADC12M$cmode = ADVANCED_SEQUENCE_OF_CHANNELS;
                  }
#line 483
                access = TRUE;
                MSP430ADC12M$cancelled = FALSE;
              }
          }
#line 486
          __nesc_atomic_end(__nesc_atomic); }
      if (access) {
          uint8_t i;
          uint16_t mask = 1;
          adc12memctl_t tmpMemctl;

#line 491
          MSP430ADC12M$advancedInterval = jiffies;
          MSP430ADC12M$seqResultPtr = dataDest;


          MSP430ADC12M$HPLADC12$disableConversion();
          for (i = 0; i < length - 1; i++) 
            MSP430ADC12M$HPLADC12$setMemControl(i, MSP430ADC12M$aSettings[num].memctl);
          tmpMemctl = MSP430ADC12M$aSettings[num].memctl;
          tmpMemctl.eos = 1;
          MSP430ADC12M$HPLADC12$setMemControl(i, tmpMemctl);
          if (jiffies != 0) 
            {

              uint16_t ctl0 = (0x0000 | 0x0010) & ~0x0080;
              adc12ctl1_t ctl1 = { .adc12busy = 0, .conseq = 1, .adc12ssel = MSP430ADC12M$aSettings[num].clockSource, 
              .adc12div = MSP430ADC12M$aSettings[num].clockDiv, .issh = 0, .shp = 1, 
              .shs = MSP430ADC12M$aSettings[num].sampleHoldSource, .cstartadd = 0 };

#line 508
              if (repeat) {
                ctl1.conseq = 3;
                }
#line 510
              MSP430ADC12M$HPLADC12$setControl0_IgnoreRef(* (adc12ctl0_t *)&ctl0);
              MSP430ADC12M$HPLADC12$setControl1(ctl1);
            }
          else 
#line 512
            {


              uint16_t ctl0 = (0x0000 | 0x0010) | 0x0080;
              uint16_t ctl1 = ((0x0218 & ~0x06) | 0x02) | 0x0080;

#line 517
              if (repeat) {
                ctl1 = 0x0218 | 0x06;
                }
#line 519
              MSP430ADC12M$HPLADC12$setControl0_IgnoreRef(* (adc12ctl0_t *)&ctl0);
              MSP430ADC12M$HPLADC12$setControl1(* (adc12ctl1_t *)&ctl1);
            }
          MSP430ADC12M$HPLADC12$setSHT(MSP430ADC12M$aSettings[num].sampleHoldTime);
          MSP430ADC12M$HPLADC12$setIEFlags(mask << i);
          MSP430ADC12M$HPLADC12$startConversion();
          if (jiffies != 0) {
            MSP430ADC12M$setTimerAdvanced(MSP430ADC12M$aSettings[num].sampleHoldSource);
            }
#line 527
          return ADC_SUCCESS;
        }

      case ADC_QUEUED: 



        { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
          {
            if (MSP430ADC12M$aCmdBuffer.type == ADC_IDLE) 
              {
                MSP430ADC12M$aCmdBuffer.dataDest = dataDest;
                MSP430ADC12M$aCmdBuffer.jiffies = jiffies;
                MSP430ADC12M$aCmdBuffer.length = length;
                if (repeat) {
                  MSP430ADC12M$aCmdBuffer.type = ADVANCED_REPEAT_SEQUENCE_OF_CHANNELS;
                  }
                else {
#line 544
                  MSP430ADC12M$aCmdBuffer.type = ADVANCED_SEQUENCE_OF_CHANNELS;
                  }
#line 545
                MSP430ADC12M$aCmdBuffer.intf = num;
                buffered = TRUE;
              }
          }
#line 548
          __nesc_atomic_end(__nesc_atomic); }
      if (buffered) {


        return ADC_QUEUED;
        }
      else {

          MSP430ADC12M$releaseLockAdvanced(&MSP430ADC12M$aSettings[num]);
          MSP430ADC12M$releaseRefVoltAdvanced(num);
        }
    }
  return ADC_FAIL;
}

static   
# 136 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/ADCM.nc"
result_t ADCM$ADC$getData(uint8_t port)
{
  bool oldBusy;

#line 139
  if (port >= TOSH_ADC_PORTMAPSIZE) {
    return FAIL;
    }
#line 141
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 141
    {
      oldBusy = ADCM$busy;
      ADCM$busy = TRUE;
    }
#line 144
    __nesc_atomic_end(__nesc_atomic); }
  if (!oldBusy) {
      ADCM$continuousData = FALSE;
      return ADCM$triggerConversion(port);
    }
  return FAIL;
}

static   
# 162 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
adcResult_t MSP430ADC12M$MSP430ADC12Basic$getSingleData(uint8_t num)
{
  bool access = FALSE;
  adc12memctl_t memctl;
  result_t refVresult;
  adcResult_t adcResult = ADC_SUCCESS;
  uint16_t ctl0 = 0x0000 | 0x0010;
  uint16_t ctl1 = 0x0218 & ~0x06;

  if (num >= 1 || !MSP430ADC12M$getLockBasic(&MSP430ADC12M$bSettings[num])) {
    return ADC_FAIL;
    }

  MSP430ADC12M$bSettings[num].queued = 1;
  memctl = MSP430ADC12M$bSettings[num].memctl;


  if (memctl.sref == REFERENCE_VREFplus_AVss || 
  memctl.sref == REFERENCE_VREFplus_VREFnegterm) 
    {
      if (MSP430ADC12M$bSettings[num].refVolt2_5) {
        refVresult = MSP430ADC12M$RefVolt$get(REFERENCE_2_5V);
        }
      else {
#line 185
        refVresult = MSP430ADC12M$RefVolt$get(REFERENCE_1_5V);
        }
#line 186
      if (refVresult != SUCCESS) 
        {
          MSP430ADC12M$bSettings[num].queued = 0;
          MSP430ADC12M$releaseLockBasic(&MSP430ADC12M$bSettings[num]);
          adcResult = ADC_FAIL;
        }
      else 
#line 191
        {
          MSP430ADC12M$bSettings[num].gotRefVolt = 1;
          if (MSP430ADC12M$RefVolt$getState() == REFERENCE_UNSTABLE) {
            adcResult = ADC_QUEUED;
            }
        }
    }
  if (adcResult == ADC_SUCCESS) 
    {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 201
        {
          if (MSP430ADC12M$cmode == ADC_IDLE) 
            {
              MSP430ADC12M$cmode = SINGLE_CHANNEL_SINGLE_CONVERSION;
              MSP430ADC12M$owner = num;
              access = TRUE;
              MSP430ADC12M$cancelled = FALSE;
            }
        }
#line 209
        __nesc_atomic_end(__nesc_atomic); }
      if (access) 
        {

          MSP430ADC12M$HPLADC12$disableConversion();
          MSP430ADC12M$HPLADC12$setControl0_IgnoreRef(* (adc12ctl0_t *)&ctl0);
          MSP430ADC12M$HPLADC12$setControl1(* (adc12ctl1_t *)&ctl1);
          MSP430ADC12M$HPLADC12$setSHT(MSP430ADC12M$bSettings[num].sampleHoldTime);
          MSP430ADC12M$HPLADC12$setMemControl(0, memctl);
          MSP430ADC12M$HPLADC12$setIEFlags(1);
          MSP430ADC12M$HPLADC12$startConversion();
          adcResult = ADC_SUCCESS;
        }
      else {
#line 222
        adcResult = ADC_QUEUED;
        }
    }

  return adcResult;
}

static 
# 203 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/TimerM.nc"
void TimerM$setNextShortEvent(void)
{
  uint32_t now = TimerM$LocalTime$read();

#line 206
  if (TimerM$m_head_short != TimerM$EMPTY_LIST) 
    {
      uint8_t head = TimerM$m_head_short;
      uint8_t soon = head;
      int32_t remaining = TimerM$m_timers[head].alarm - now;

#line 211
      head = TimerM$m_timers[head].next;
      while (head != TimerM$EMPTY_LIST) 
        {
          int32_t dt = TimerM$m_timers[head].alarm - now;

#line 215
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

          TimerM$AlarmCompare$disableEvents();
          TOS_post(TimerM$checkShortTimers);
        }
      else 
        {


          TimerM$AlarmCompare$setEventFromNow(remaining);
          TimerM$AlarmCompare$clearPendingInterrupt();
          TimerM$AlarmCompare$enableEvents();
        }
    }
  else 
    {

      TimerM$AlarmCompare$disableEvents();
    }
}

static   
# 125 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/HPLCC2420M.nc"
uint8_t HPLCC2420M$HPLCC2420$cmd(uint8_t addr)
#line 125
{
  uint8_t status = 0;

#line 127
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(addr);
      TOSH_uwait(20);
      status = HPLCC2420M$USARTControl$rx();
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$BusArbitration$releaseBus();
    }
  return status;
}

static   
# 85 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/BusArbitrationM.nc"
result_t BusArbitrationM$BusArbitration$getBus(uint8_t id)
#line 85
{
  bool gotbus = FALSE;

#line 87
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 87
    {
      if (BusArbitrationM$state == BusArbitrationM$BUS_IDLE) {
          BusArbitrationM$state = BusArbitrationM$BUS_BUSY;
          gotbus = TRUE;
          BusArbitrationM$busid = id;
        }
    }
#line 93
    __nesc_atomic_end(__nesc_atomic); }
  if (gotbus) {
    return SUCCESS;
    }
#line 96
  return FAIL;
}

static   result_t BusArbitrationM$BusArbitration$releaseBus(uint8_t id)
#line 99
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 100
    {
      if (BusArbitrationM$state == BusArbitrationM$BUS_BUSY && BusArbitrationM$busid == id) {
          BusArbitrationM$state = BusArbitrationM$BUS_IDLE;





          if (BusArbitrationM$isBusReleasedPending == FALSE && TOS_post(BusArbitrationM$busReleased) == TRUE) {
            BusArbitrationM$isBusReleasedPending = TRUE;
            }
        }
    }
#line 112
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

static   
# 146 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/HPLCC2420M.nc"
uint8_t HPLCC2420M$HPLCC2420$write(uint8_t addr, uint16_t data)
#line 146
{
  uint8_t status = 0;

#line 148
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(addr);
      TOSH_uwait(20);
      status = HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx((data >> 8) & 0x0FF);
      TOSH_uwait(20);
      HPLCC2420M$USARTControl$tx(data & 0x0FF);
      TOSH_uwait(20);
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$BusArbitration$releaseBus();
    }
  return status;
}

static   




uint16_t HPLCC2420M$HPLCC2420$read(uint8_t addr)
#line 171
{
  uint16_t data = 0;

#line 173
  if (HPLCC2420M$BusArbitration$getBus() == SUCCESS) {
      TOSH_CLR_RADIO_CSN_PIN();

      HPLCC2420M$USARTControl$isTxIntrPending();
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(addr | 0x40);
      TOSH_uwait(20);
      HPLCC2420M$USARTControl$rx();
      HPLCC2420M$USARTControl$tx(0);
      TOSH_uwait(20);
      data = (HPLCC2420M$USARTControl$rx() << 8) & 0xFF00;
      HPLCC2420M$USARTControl$tx(0);
      TOSH_uwait(20);
      data = data | (HPLCC2420M$USARTControl$rx() & 0x0FF);
      TOSH_SET_RADIO_CSN_PIN();
      HPLCC2420M$BusArbitration$releaseBus();
    }
  return data;
}

# 95 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void __attribute((interrupt(10))) __attribute((wakeup))   sig_TIMERA1_VECTOR(void)
{
  int n = TAIV;

#line 98
  switch (n) 
    {
      case 0: break;
      case 2: MSP430TimerM$CompareA1$fired();
#line 101
      break;
      case 4: MSP430TimerM$CompareA2$fired();
#line 102
      break;
      case 6: break;
      case 8: break;
      case 10: MSP430TimerM$TimerA$overflow();
#line 105
      break;
      case 12: break;
      case 14: break;
    }
}

static   
# 610 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430ADC12M.nc"
void MSP430ADC12M$CompareB0$fired(void)
#line 610
{
  MSP430CompareControl_t control1 = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, 
  .cap = 0, .clld = 0, 
  .scs = 0, .ccis = 0, .cm = 0 };

  MSP430CompareControl_t control2 = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 1, 
  .outmod = 1, 
  .cap = 0, .clld = 0, 
  .scs = 0, .ccis = 0, .cm = 0 };

  if (MSP430ADC12M$advancedTimer == TIMERB_OUT0) 
    {
      MSP430ADC12M$CompareB0$setControl(control1);
      MSP430ADC12M$CompareB0$setControl(control2);
      MSP430ADC12M$CompareB0$setEventFromPrev(MSP430ADC12M$advancedInterval);
    }
}

# 179 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430TimerM.nc"
void __attribute((interrupt(24))) __attribute((wakeup))   sig_TIMERB1_VECTOR(void)
{
  int n = TBIV;

#line 182
  switch (n) 
    {
      case 0: break;
      case 2: MSP430TimerM$CompareB1$fired();
#line 185
      break;
      case 4: MSP430TimerM$CompareB2$fired();
#line 186
      break;
      case 6: MSP430TimerM$CompareB3$fired();
#line 187
      break;
      case 8: MSP430TimerM$CompareB4$fired();
#line 188
      break;
      case 10: MSP430TimerM$CompareB5$fired();
#line 189
      break;
      case 12: MSP430TimerM$CompareB6$fired();
#line 190
      break;
      case 14: MSP430TimerM$TimerB$overflow();
#line 191
      break;
    }
}

static 
# 114 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
void CC2420RadioM$sendFailed(void)
#line 114
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 115
    CC2420RadioM$stateRadio = CC2420RadioM$IDLE_STATE;
#line 115
    __nesc_atomic_end(__nesc_atomic); }
  CC2420RadioM$txbufptr->length = CC2420RadioM$txbufptr->length - MSG_HEADER_SIZE - MSG_FOOTER_SIZE;
  CC2420RadioM$Send$sendDone(CC2420RadioM$txbufptr, FAIL);
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
# 256 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
void CC2420RadioM$tryToSend(void)
#line 256
{
  uint8_t currentstate;

#line 258
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 258
    currentstate = CC2420RadioM$stateRadio;
#line 258
    __nesc_atomic_end(__nesc_atomic); }


  if (currentstate == CC2420RadioM$PRE_TX_STATE) {
      if (TOSH_READ_RADIO_CCA_PIN()) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 263
            CC2420RadioM$stateRadio = CC2420RadioM$TX_STATE;
#line 263
            __nesc_atomic_end(__nesc_atomic); }
          CC2420RadioM$sendPacket();
        }
      else {
          if (CC2420RadioM$countRetry-- <= 0) {
              CC2420RadioM$HPLChipcon$read(0x3F);
              CC2420RadioM$HPLChipcon$cmd(0x08);
              CC2420RadioM$HPLChipcon$read(0x3F);
              CC2420RadioM$HPLChipcon$cmd(0x08);
              CC2420RadioM$sendFailed();
              return;
            }
          if (!CC2420RadioM$setBackoffTimer(CC2420RadioM$MacBackoff$congestionBackoff(CC2420RadioM$txbufptr) * 10)) {
              CC2420RadioM$sendFailed();
            }
        }
    }
}

static   
# 55 "C:/cygwin/opt/tinyos-1.x/tos/platform/telos/TimerJiffyAsyncM.nc"
result_t TimerJiffyAsyncM$TimerJiffyAsync$setOneShot(uint32_t _jiffy)
{
  TimerJiffyAsyncM$Alarm$disableEvents();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 58
    {
      TimerJiffyAsyncM$jiffy = _jiffy;
      TimerJiffyAsyncM$bSet = TRUE;
    }
#line 61
    __nesc_atomic_end(__nesc_atomic); }
  if (_jiffy > 0xFFFF) {
      TimerJiffyAsyncM$Alarm$setEventFromNow(0xFFFF);
    }
  else {
      TimerJiffyAsyncM$Alarm$setEventFromNow(_jiffy);
    }
  TimerJiffyAsyncM$Alarm$clearPendingInterrupt();
  TimerJiffyAsyncM$Alarm$enableEvents();
  return SUCCESS;
}

static  
# 154 "C:/cygwin/opt/tinyos-1.x/tos/lib/CC2420Radio/CC2420RadioM.nc"
void CC2420RadioM$PacketSent(void)
#line 154
{
  TOS_MsgPtr pBuf;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 157
    {
      CC2420RadioM$stateRadio = CC2420RadioM$IDLE_STATE;
      CC2420RadioM$txbufptr->time = 0;
      pBuf = CC2420RadioM$txbufptr;
      pBuf->length = pBuf->length - MSG_HEADER_SIZE - MSG_FOOTER_SIZE;
    }
#line 162
    __nesc_atomic_end(__nesc_atomic); }

  CC2420RadioM$Send$sendDone(pBuf, SUCCESS);
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

# 163 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
void __attribute((interrupt(14))) __attribute((wakeup))   sig_ADC_VECTOR(void)
#line 163
{
  uint16_t iv = HPLADC12M$ADC12IV;

#line 165
  switch (iv) 
    {
      case 2: HPLADC12M$HPLADC12$memOverflow();
#line 167
      return;
      case 4: HPLADC12M$HPLADC12$timeOverflow();
#line 168
      return;
    }
  iv >>= 1;
  if (iv && iv < 19) {
    HPLADC12M$HPLADC12$converted(iv - 3);
    }
}

static   
#line 98
void HPLADC12M$HPLADC12$resetIFGs(void)
#line 98
{

  if (!HPLADC12M$ADC12IFG) {
    return;
    }
  else 
#line 102
    {
      uint8_t i;
      volatile uint16_t mud;

#line 105
      for (i = 0; i < 16; i++) 
        mud = HPLADC12M$HPLADC12$getMem(i);
    }
}

static 
# 175 "BSN_RSUM.nc"
char BSN_RSUM$SendData(uint16_t data, int channel)
{
  struct UbiMonMsg *pack = (struct UbiMonMsg *)BSN_RSUM$msg[BSN_RSUM$currentMsg].data;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 179
    {
      pack->data[BSN_RSUM$packetReadingNumber++] = data;
      BSN_RSUM$readingNumber++;
      if (BSN_RSUM$packetReadingNumber == BUFFER_SIZE) 

        {
          TOS_post(BSN_RSUM$dataTask);
        }
    }
#line 187
    __nesc_atomic_end(__nesc_atomic); }









  return 0;
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
# 81 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/HPLADC12M.nc"
adc12memctl_t HPLADC12M$HPLADC12$getMemControl(uint8_t i)
#line 81
{
  adc12memctl_t x = { .inch = 0, .sref = 0, .eos = 0 };
  uint8_t *memCtlPtr = (uint8_t *)(char *)0x0080;

#line 84
  if (i < 16) {
      memCtlPtr += i;
      x = * (adc12memctl_t *)memCtlPtr;
    }
  return x;
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

# 47 "C:/cygwin/opt/tinyos-1.x/tos/platform/msp430/MSP430InterruptM.nc"
void __attribute((interrupt(8))) __attribute((wakeup))   sig_PORT1_VECTOR(void)
{
  volatile int n = P1IFG & P1IE;

  if (n & (1 << 0)) {
#line 51
      MSP430InterruptM$Port10$fired();
#line 51
      return;
    }
#line 52
  if (n & (1 << 1)) {
#line 52
      MSP430InterruptM$Port11$fired();
#line 52
      return;
    }
#line 53
  if (n & (1 << 2)) {
#line 53
      MSP430InterruptM$Port12$fired();
#line 53
      return;
    }
#line 54
  if (n & (1 << 3)) {
#line 54
      MSP430InterruptM$Port13$fired();
#line 54
      return;
    }
#line 55
  if (n & (1 << 4)) {
#line 55
      MSP430InterruptM$Port14$fired();
#line 55
      return;
    }
#line 56
  if (n & (1 << 5)) {
#line 56
      MSP430InterruptM$Port15$fired();
#line 56
      return;
    }
#line 57
  if (n & (1 << 6)) {
#line 57
      MSP430InterruptM$Port16$fired();
#line 57
      return;
    }
#line 58
  if (n & (1 << 7)) {
#line 58
      MSP430InterruptM$Port17$fired();
#line 58
      return;
    }
}

void __attribute((interrupt(2))) __attribute((wakeup))   sig_PORT2_VECTOR(void)
{
  volatile int n = P2IFG & P2IE;

  if (n & (1 << 0)) {
#line 66
      MSP430InterruptM$Port20$fired();
#line 66
      return;
    }
#line 67
  if (n & (1 << 1)) {
#line 67
      MSP430InterruptM$Port21$fired();
#line 67
      return;
    }
#line 68
  if (n & (1 << 2)) {
#line 68
      MSP430InterruptM$Port22$fired();
#line 68
      return;
    }
#line 69
  if (n & (1 << 3)) {
#line 69
      MSP430InterruptM$Port23$fired();
#line 69
      return;
    }
#line 70
  if (n & (1 << 4)) {
#line 70
      MSP430InterruptM$Port24$fired();
#line 70
      return;
    }
#line 71
  if (n & (1 << 5)) {
#line 71
      MSP430InterruptM$Port25$fired();
#line 71
      return;
    }
#line 72
  if (n & (1 << 6)) {
#line 72
      MSP430InterruptM$Port26$fired();
#line 72
      return;
    }
#line 73
  if (n & (1 << 7)) {
#line 73
      MSP430InterruptM$Port27$fired();
#line 73
      return;
    }
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
# 34 "C:/cygwin/opt/tinyos-1.x/tos/system/crc.h"
uint16_t crcByte(uint16_t crc, uint8_t b)
{
  uint8_t i;

  crc = crc ^ (b << 8);
  i = 8;
  do 
    if (crc & 0x8000) {
      crc = (crc << 1) ^ 0x1021;
      }
    else {
#line 44
      crc = crc << 1;
      }
  while (
#line 45
  --i);

  return crc;
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

