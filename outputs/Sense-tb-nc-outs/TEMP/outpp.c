#define nx_struct struct
#define nx_union union
/*#define dbg(mode, format, ...) ((void)0) */
/*(#define dbg_clear(mode, format, ...) ((void)0)*/
/*#define dbg_active(mode) 0*/
/* # 151 "/opt/local/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3 */
typedef int ptrdiff_t;
/* #line 213 */
typedef unsigned int size_t;
/* #line 325 */
typedef int wchar_t;
/* # 8 "/opt/local/lib/ncc/deputy_nodeputy.h" */
struct __nesc_attr_nonnull {
}  ;
/* #line 9 */
struct __nesc_attr_bnd {
/* #line 9 */
  void *lo, *hi;
}  ;
/* #line 10 */
struct __nesc_attr_bnd_nok {
/* #line 10 */
  void *lo, *hi;
}  ;
/* #line 11 */
struct __nesc_attr_count {
/* #line 11 */
  int n;
}  ;
/* #line 12 */
struct __nesc_attr_count_nok {
/* #line 12 */
  int n;
}  ;
/* #line 13 */
struct __nesc_attr_one {
}  ;
/* #line 14 */
struct __nesc_attr_one_nok {
}  ;
/* #line 15 */
struct __nesc_attr_dmemset {
/* #line 15 */
  int a1, a2, a3;
}  ;
/* #line 16 */
struct __nesc_attr_dmemcpy {
/* #line 16 */
  int a1, a2, a3;
}  ;
/* #line 17 */
struct __nesc_attr_nts {
}  ;
/* # 38 "/opt/local/msp430/include/sys/inttypes.h" 3 */
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
/* # 385 "/opt/local/lib/ncc/nesc_nx.h" */
typedef struct { unsigned char data[1]; } __attribute_packed nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute_packed nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute_packed nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute_packed nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute_packed nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute_packed nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute_packed nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute_packed nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char data[1]; } __attribute_packed nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute_packed nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute_packed nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute_packed nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute_packed nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute_packed nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute_packed nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute_packed nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
/* # 41 "/opt/local/msp430/include/sys/types.h" 3 */
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
/* # 42 "/opt/local/msp430/include/string.h" 3 */
extern void *memset(void *arg_0x702690, int arg_0x7027e8, size_t arg_0x702980);
/* #line 63 */
extern void *memset(void *arg_0x1045650, int arg_0x10457a8, size_t arg_0x1045940);
/* # 59 "/opt/local/msp430/include/stdlib.h" 3 */
/* #line 56 */
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;







/* #line 64 */
typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;
/* # 122 "/opt/local/msp430/include/sys/config.h" 3 */
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
/* # 12 "/opt/local/msp430/include/sys/_types.h" 3 */
typedef long _off_t;
typedef long _ssize_t;
/* # 28 "/opt/local/msp430/include/sys/reent.h" 3 */
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
/* #line 116 */
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
/* #line 174 */
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

  void (*__cleanup)(struct _reent *arg_0x1073ab8);


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


  void (**_sig_func)(int arg_0x1077c98);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
/* #line 273 */
struct _reent;
/* # 18 "/opt/local/msp430/include/math.h" 3 */
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
/* #line 208 */
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
/* #line 261 */
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
/* # 23 "/Users/doina/tinyos-2.x/tos/system/tos.h" */
typedef uint8_t bool;
enum __nesc_unnamed4247 {
/* #line 24 */
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;







struct __nesc_attr_atmostonce {
};
/* #line 35 */
struct __nesc_attr_atleastonce {
};
/* #line 36 */
struct __nesc_attr_exactlyonce {
};
/* # 40 "/Users/doina/tinyos-2.x/tos/types/TinyError.h" */
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2);
/* # 39 "/opt/local/msp430/include/msp430/iostructures.h" 3 */
/* #line 27 */
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4249 {
    unsigned char __p0 : 1, 
    __p1 : 1, 
    __p2 : 1, 
    __p3 : 1, 
    __p4 : 1, 
    __p5 : 1, 
    __p6 : 1, 
    __p7 : 1;
  } __pin;
} __attribute_packed  ioregister_t;
/* #line 108 */
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
/* # 116 "/opt/local/msp430/include/msp430/gpio.h" 3 */
volatile unsigned char P1OUT __asm ("0x0021");

volatile unsigned char P1DIR __asm ("0x0022");





volatile unsigned char P1IE __asm ("0x0025");

volatile unsigned char P1SEL __asm ("0x0026");










volatile unsigned char P2OUT __asm ("0x0029");

volatile unsigned char P2DIR __asm ("0x002A");





volatile unsigned char P2IE __asm ("0x002D");

volatile unsigned char P2SEL __asm ("0x002E");










volatile unsigned char P3OUT __asm ("0x0019");

volatile unsigned char P3DIR __asm ("0x001A");

volatile unsigned char P3SEL __asm ("0x001B");










volatile unsigned char P4OUT __asm ("0x001D");

volatile unsigned char P4DIR __asm ("0x001E");

volatile unsigned char P4SEL __asm ("0x001F");










volatile unsigned char P5OUT __asm ("0x0031");

volatile unsigned char P5DIR __asm ("0x0032");

volatile unsigned char P5SEL __asm ("0x0033");










volatile unsigned char P6OUT __asm ("0x0035");

volatile unsigned char P6DIR __asm ("0x0036");

volatile unsigned char P6SEL __asm ("0x0037");
/* # 94 "/opt/local/msp430/include/msp430/usart.h" 3 */
volatile unsigned char U0TCTL __asm ("0x0071");
/* #line 277 */
volatile unsigned char U1TCTL __asm ("0x0079");
/* # 27 "/opt/local/msp430/include/msp430/timera.h" 3 */
volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");


volatile unsigned int TA0CCTL0 __asm ("0x0162");

volatile unsigned int TA0CCTL1 __asm ("0x0164");
/* #line 70 */
volatile unsigned int TA0CCTL2 __asm ("0x0166");
/* #line 127 */
/* #line 118 */
typedef struct __nesc_unnamed4250 {
  volatile unsigned 
  taifg : 1, 
  taie : 1, 
  taclr : 1, 
  dummy : 1, 
  tamc : 2, 
  taid : 2, 
  tassel : 2;
} __attribute_packed  tactl_t;
/* #line 143 */
/* #line 129 */
typedef struct __nesc_unnamed4251 {
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
} __attribute_packed  tacctl_t;


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
/* # 26 "/opt/local/msp430/include/msp430/timerb.h" 3 */
volatile unsigned int TBR __asm ("0x0190");


volatile unsigned int TBCCTL0 __asm ("0x0182");





volatile unsigned int TBCCR0 __asm ("0x0192");
/* #line 76 */
/* #line 64 */
typedef struct __nesc_unnamed4252 {
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
} __attribute_packed  tbctl_t;
/* #line 91 */
/* #line 78 */
typedef struct __nesc_unnamed4253 {
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
} __attribute_packed  tbcctl_t;


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
/* # 20 "/opt/local/msp430/include/msp430/basic_clock.h" 3 */
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
/* # 18 "/opt/local/msp430/include/msp430/adc12.h" 3 */
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");
/* #line 42 */
/* #line 30 */
typedef struct __nesc_unnamed4254 {
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
} __attribute_packed  adc12ctl0_t;
/* #line 54 */
/* #line 44 */
typedef struct __nesc_unnamed4255 {
  volatile unsigned 
  adc12busy : 1, 
  conseq : 2, 
  adc12ssel : 2, 
  adc12div : 3, 
  issh : 1, 
  shp : 1, 
  shs : 2, 
  cstartadd : 4;
} __attribute_packed  adc12ctl1_t;
/* #line 74 */
/* #line 56 */
typedef struct __nesc_unnamed4256 {
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
} __attribute_packed  adc12xflg_t;


struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};




struct adc12_t;
/* # 83 "/opt/local/msp430/include/msp430x16x.h" 3 */
volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
/* # 158 "/Users/doina/tinyos-2.x/tos/chips/msp430/msp430hardware.h" */
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");
/* #line 193 */
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4257 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
/* #line 248 */
typedef struct { unsigned char data[4]; } __attribute_packed nx_float;typedef float __nesc_nxbase_nx_float  ;
/* # 33 "/Users/doina/tinyos-2.x/tos/platforms/telosb/hardware.h" */
static inline void TOSH_SET_SIMO0_PIN()  ;
/* #line 33 */
static inline void TOSH_CLR_SIMO0_PIN()  ;
/* #line 33 */
static inline void TOSH_MAKE_SIMO0_OUTPUT()  ;
static inline void TOSH_SET_UCLK0_PIN()  ;
/* #line 34 */
static inline void TOSH_CLR_UCLK0_PIN()  ;
/* #line 34 */
static inline void TOSH_MAKE_UCLK0_OUTPUT()  ;
/* #line 76 */
enum __nesc_unnamed4258 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN()  ;
/* #line 85 */
static inline void TOSH_CLR_FLASH_CS_PIN()  ;
/* #line 85 */
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()  ;
static inline void TOSH_SET_FLASH_HOLD_PIN()  ;
/* #line 86 */
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()  ;
/* # 29 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.h" */
typedef struct __nesc_unnamed4259 {
/* #line 29 */
  int notUsed;
} 
/* #line 29 */
TMilli;
typedef struct __nesc_unnamed4260 {
/* #line 30 */
  int notUsed;
} 
/* #line 30 */
T32khz;
typedef struct __nesc_unnamed4261 {
/* #line 31 */
  int notUsed;
} 
/* #line 31 */
TMicro;
/* # 32 "/Users/doina/tinyos-2.x/tos/types/Leds.h" */
enum __nesc_unnamed4262 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.h" */
enum __nesc_unnamed4263 {
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
/* #line 64 */
/* #line 51 */
typedef struct __nesc_unnamed4264 {

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
} msp430_compare_control_t;
/* #line 76 */
/* #line 66 */
typedef struct __nesc_unnamed4265 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
/* #line 91 */
/* #line 78 */
typedef struct __nesc_unnamed4266 {

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
} msp430_timer_b_control_t;
/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12.h" */
/* #line 48 */
typedef struct __nesc_unnamed4267 {

  unsigned int inch : 4;
  unsigned int sref : 3;
  unsigned int ref2_5v : 1;
  unsigned int adc12ssel : 2;
  unsigned int adc12div : 3;
  unsigned int sht : 4;
  unsigned int sampcon_ssel : 2;
  unsigned int sampcon_id : 2;
/*  unsigned int  : 0; */
} msp430adc12_channel_config_t;








/* #line 61 */
typedef struct __nesc_unnamed4268 {


  volatile unsigned 
  inch : 4, 
  sref : 3, 
  eos : 1;
} __attribute_packed  adc12memctl_t;

enum inch_enum {


  INPUT_CHANNEL_A0 = 0, 
  INPUT_CHANNEL_A1 = 1, 
  INPUT_CHANNEL_A2 = 2, 
  INPUT_CHANNEL_A3 = 3, 
  INPUT_CHANNEL_A4 = 4, 
  INPUT_CHANNEL_A5 = 5, 
  INPUT_CHANNEL_A6 = 6, 
  INPUT_CHANNEL_A7 = 7, 
  EXTERNAL_REF_VOLTAGE_CHANNEL = 8, 
  REF_VOLTAGE_NEG_TERMINAL_CHANNEL = 9, 
  TEMPERATURE_DIODE_CHANNEL = 10, 
  SUPPLY_VOLTAGE_HALF_CHANNEL = 11, 
  INPUT_CHANNEL_NONE = 12
};

enum sref_enum {

  REFERENCE_AVcc_AVss = 0, 
  REFERENCE_VREFplus_AVss = 1, 
  REFERENCE_VeREFplus_AVss = 2, 
  REFERENCE_AVcc_VREFnegterm = 4, 
  REFERENCE_VREFplus_VREFnegterm = 5, 
  REFERENCE_VeREFplus_VREFnegterm = 6
};

enum ref2_5v_enum {

  REFVOLT_LEVEL_1_5 = 0, 
  REFVOLT_LEVEL_2_5 = 1, 
  REFVOLT_LEVEL_NONE = 0
};

enum adc12ssel_enum {

  SHT_SOURCE_ADC12OSC = 0, 
  SHT_SOURCE_ACLK = 1, 
  SHT_SOURCE_MCLK = 2, 
  SHT_SOURCE_SMCLK = 3
};

enum adc12div_enum {

  SHT_CLOCK_DIV_1 = 0, 
  SHT_CLOCK_DIV_2 = 1, 
  SHT_CLOCK_DIV_3 = 2, 
  SHT_CLOCK_DIV_4 = 3, 
  SHT_CLOCK_DIV_5 = 4, 
  SHT_CLOCK_DIV_6 = 5, 
  SHT_CLOCK_DIV_7 = 6, 
  SHT_CLOCK_DIV_8 = 7
};

enum sht_enum {

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

enum sampcon_ssel_enum {

  SAMPCON_SOURCE_TACLK = 0, 
  SAMPCON_SOURCE_ACLK = 1, 
  SAMPCON_SOURCE_SMCLK = 2, 
  SAMPCON_SOURCE_INCLK = 3
};

enum sampcon_id_enum {

  SAMPCON_CLOCK_DIV_1 = 0, 
  SAMPCON_CLOCK_DIV_2 = 1, 
  SAMPCON_CLOCK_DIV_3 = 2, 
  SAMPCON_CLOCK_DIV_4 = 3
};
/* # 33 "/Users/doina/tinyos-2.x/tos/types/Resource.h" */
typedef uint8_t resource_client_id_t;
typedef uint16_t SenseC_Read_val_t;
typedef TMilli SenseC_Timer_precision_tag;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC_0___nesc_unnamed4269 {
  Msp430Timer32khzC_0_ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC_0_to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC_0_to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC_0_from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC_0_from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC_0_upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC_0_from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC_0_from_size_type /*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC_0_to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC_0_Counter_precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC_0_to_size_type /*CounterMilli32C.Transform*/TransformCounterC_0_Counter_size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Counter_precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Counter_size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_0_precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_0_precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_0_LocalTime_precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_0_precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_0_Counter_precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_0_Counter_size_type;
typedef uint16_t AdcP_Read_val_t;
typedef uint16_t AdcP_ReadNow_val_t;
typedef const msp430adc12_channel_config_t *AdcP_Config_adc_config_t;
typedef TMilli Msp430RefVoltGeneratorP_SwitchOffTimer_precision_tag;
typedef TMilli Msp430RefVoltGeneratorP_SwitchOnTimer_precision_tag;
typedef const msp430adc12_channel_config_t *Msp430RefVoltArbiterImplP_Config_adc_config_t;
enum /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0___nesc_unnamed4270 {
  Msp430Adc12ClientAutoRVGC_0_ID = 0U
};
typedef const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_0_ConfSub_adc_config_t;
typedef const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_0_ConfUp_adc_config_t;
enum /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC_0___nesc_unnamed4271 {
  AdcReadClientC_0_CLIENT = 0U
};
typedef TMilli AdcStreamP_Alarm_precision_tag;
typedef uint32_t AdcStreamP_Alarm_size_type;
typedef const msp430adc12_channel_config_t *AdcStreamP_AdcConfigure_adc_config_t;
typedef uint16_t AdcStreamP_ReadStream_val_t;
enum /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Timer*/Msp430Timer32khzC_1___nesc_unnamed4272 {
  Msp430Timer32khzC_1_ALARM_ID = 1U
};
typedef T32khz /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_frequency_tag;
typedef /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_frequency_tag /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_precision_tag;
typedef uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_size_type;
typedef TMilli /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_precision_tag;
typedef uint32_t /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type;
typedef T32khz /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_from_precision_tag;
typedef uint16_t /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_from_size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_from_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_AlarmFrom_precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_from_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_AlarmFrom_size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Counter_precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Counter_size_type;
typedef uint16_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_val_t;
typedef /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_val_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Service_val_t;
typedef /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_val_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_val_t;
enum /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1___nesc_unnamed4273 {
  Msp430Adc12ClientAutoRVGC_1_ID = 1U
};
typedef const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_1_ConfSub_adc_config_t;
typedef const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_1_ConfUp_adc_config_t;
enum /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC_0___nesc_unnamed4274 {
  AdcReadStreamClientC_0_RSCLIENT = 0U
};
typedef const msp430adc12_channel_config_t *Msp430InternalVoltageP_AdcConfigure_adc_config_t;
enum /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_2___nesc_unnamed4275 {
  Msp430Adc12ClientAutoRVGC_2_ID = 2U
};
typedef const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_2_ConfSub_adc_config_t;
typedef const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_2_ConfUp_adc_config_t;
enum /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC_0___nesc_unnamed4276 {
  AdcReadNowClientC_0_CLIENT = 1U
};
/* # 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc" */
static void SenseC_Boot_booted(void );
/* # 63 "/Users/doina/tinyos-2.x/tos/interfaces/Read.nc" */
static void SenseC_Read_readDone(error_t result, SenseC_Read_val_t val);
/* # 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void SenseC_Timer_fired(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t PlatformP_Init_init(void );
/* #line 51 */
static error_t MotePlatformC_Init_init(void );
/* # 35 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc" */
static void Msp430ClockP_Msp430ClockInit_defaultInitClocks(void );
/* #line 32 */
static void Msp430ClockP_Msp430ClockInit_default_initTimerB(void );



static void Msp430ClockP_Msp430ClockInit_defaultInitTimerA(void );
/* #line 31 */
static void Msp430ClockP_Msp430ClockInit_default_initTimerA(void );





static void Msp430ClockP_Msp430ClockInit_defaultInitTimerB(void );
/* #line 34 */
static void Msp430ClockP_Msp430ClockInit_defaultSetupDcoCalibrate(void );
/* #line 29 */
static void Msp430ClockP_Msp430ClockInit_default_setupDcoCalibrate(void );
static void Msp430ClockP_Msp430ClockInit_default_initClocks(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t Msp430ClockP_Init_init(void );
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_VectorTimerX0_fired(void );
/* #line 28 */
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Overflow_fired(void );
/* #line 28 */
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_VectorTimerX1_fired(void );
/* #line 28 */
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Event_default_fired(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
uint8_t arg_0x15734f0);
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_clear(void );


static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_setClockSource(uint16_t clockSource);
/* #line 43 */
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_disableEvents(void );
/* #line 39 */
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_setMode(int mode);





static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_setInputDivider(uint16_t inputDivider);
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_VectorTimerX0_fired(void );
/* #line 28 */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Overflow_fired(void );
/* #line 28 */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_VectorTimerX1_fired(void );
/* #line 28 */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_default_fired(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
uint8_t arg_0x15734f0);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_isOverflowPending(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_getEvent(void );
/* #line 75 */
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_default_captured(uint16_t time);
/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Control_getControl(void );



static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Control_setControl(msp430_compare_control_t control);
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Event_fired(void );
/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Compare_setEvent(uint16_t time);
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Timer_overflow(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_getEvent(void );
/* #line 75 */
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_default_captured(uint16_t time);
/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Control_getControl(void );



static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Control_setControl(msp430_compare_control_t control);
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Event_fired(void );
/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Compare_setEvent(uint16_t time);
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Timer_overflow(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Capture_getEvent(void );
/* #line 75 */
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Capture_default_captured(uint16_t time);
/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Control_getControl(void );
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Event_fired(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Compare_default_fired(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Timer_overflow(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Capture_getEvent(void );
/* #line 75 */
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Capture_default_captured(uint16_t time);
/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_getControl(void );
/* #line 46 */
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_enableEvents(void );
/* #line 36 */
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_disableEvents(void );
/* #line 33 */
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_clearPendingInterrupt(void );
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Event_fired(void );
/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Compare_setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Compare_setEventFromNow(uint16_t delta);
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Timer_overflow(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_getEvent(void );
/* #line 75 */
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_default_captured(uint16_t time);
/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_getControl(void );
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Event_fired(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Compare_default_fired(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Timer_overflow(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Capture_getEvent(void );
/* #line 75 */
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Capture_default_captured(uint16_t time);
/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_getControl(void );
/* #line 46 */
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_disableEvents(void );
/* #line 33 */
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_clearPendingInterrupt(void );
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Event_fired(void );
/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_setEventFromNow(uint16_t delta);
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Timer_overflow(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Capture_getEvent(void );
/* #line 75 */
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Capture_default_captured(uint16_t time);
/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Control_getControl(void );
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Event_fired(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Compare_default_fired(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Timer_overflow(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Capture_getEvent(void );
/* #line 75 */
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Capture_default_captured(uint16_t time);
/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Control_getControl(void );
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Event_fired(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Compare_default_fired(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Timer_overflow(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Capture_getEvent(void );
/* #line 75 */
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Capture_default_captured(uint16_t time);
/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Control_getControl(void );
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Event_fired(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Compare_default_fired(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Timer_overflow(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Capture_getEvent(void );
/* #line 75 */
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Capture_default_captured(uint16_t time);
/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Control_getControl(void );
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Event_fired(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Compare_default_fired(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Timer_overflow(void );
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t SchedulerBasicP_TaskBasic_postTask(
/* # 45 "/Users/doina/tinyos-2.x/tos/system/SchedulerBasicP.nc" */
uint8_t arg_0x147bb38);
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void SchedulerBasicP_TaskBasic_default_runTask(
/* # 45 "/Users/doina/tinyos-2.x/tos/system/SchedulerBasicP.nc" */
uint8_t arg_0x147bb38);
/* # 46 "/Users/doina/tinyos-2.x/tos/interfaces/Scheduler.nc" */
static void SchedulerBasicP_Scheduler_init(void );
/* #line 61 */
static void SchedulerBasicP_Scheduler_taskLoop(void );
/* #line 54 */
static bool SchedulerBasicP_Scheduler_runNextTask(void );
/* # 54 "/Users/doina/tinyos-2.x/tos/interfaces/McuPowerOverride.nc" */
static mcu_power_t McuSleepC_McuPowerOverride_default_lowestState(void );
/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/McuSleep.nc" */
static void McuSleepC_McuSleep_sleep(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t LedsP_Init_init(void );
/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
static void LedsP_Leds_led0Off(void );










static void LedsP_Leds_led1On(void );




static void LedsP_Leds_led1Off(void );
/* #line 83 */
static void LedsP_Leds_led2Off(void );
/* #line 45 */
static void LedsP_Leds_led0On(void );
/* #line 78 */
static void LedsP_Leds_led2On(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_makeOutput(void );
/* #line 34 */
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_set(void );




static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_clr(void );
/* #line 71 */
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_makeOutput(void );
/* #line 34 */
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_set(void );




static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_clr(void );
/* #line 71 */
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_makeOutput(void );
/* #line 34 */
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_set(void );




static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_clr(void );
/* #line 64 */
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP_40_IO_makeInput(void );
/* #line 85 */
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP_40_IO_selectIOFunc(void );
/* #line 78 */
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP_40_IO_selectModuleFunc(void );
/* #line 64 */
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP_41_IO_makeInput(void );
/* #line 85 */
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP_41_IO_selectIOFunc(void );
/* #line 78 */
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP_41_IO_selectModuleFunc(void );
/* #line 64 */
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP_42_IO_makeInput(void );
/* #line 85 */
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP_42_IO_selectIOFunc(void );
/* #line 78 */
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP_42_IO_selectModuleFunc(void );
/* #line 64 */
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP_43_IO_makeInput(void );
/* #line 85 */
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP_43_IO_selectIOFunc(void );
/* #line 78 */
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP_43_IO_selectModuleFunc(void );
/* #line 64 */
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP_44_IO_makeInput(void );
/* #line 85 */
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP_44_IO_selectIOFunc(void );
/* #line 78 */
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP_44_IO_selectModuleFunc(void );
/* #line 64 */
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP_45_IO_makeInput(void );
/* #line 85 */
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP_45_IO_selectIOFunc(void );
/* #line 78 */
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP_45_IO_selectModuleFunc(void );
/* #line 64 */
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP_46_IO_makeInput(void );
/* #line 85 */
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP_46_IO_selectIOFunc(void );
/* #line 78 */
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP_46_IO_selectModuleFunc(void );
/* #line 64 */
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP_47_IO_makeInput(void );
/* #line 85 */
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP_47_IO_selectIOFunc(void );
/* #line 78 */
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP_47_IO_selectModuleFunc(void );
/* # 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_makeOutput(void );
/* #line 29 */
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_set(void );
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_clr(void );




static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_makeOutput(void );
/* #line 29 */
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_set(void );
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_clr(void );




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_makeOutput(void );
/* #line 29 */
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_set(void );
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_clr(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_fired(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Timer_overflow(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_size_type dt);
/* #line 62 */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_stop(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Init_init(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Msp430Timer_overflow(void );
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_overflow(void );
/* #line 53 */
static /*CounterMilli32C.Transform*/TransformCounterC_0_Counter_size_type /*CounterMilli32C.Transform*/TransformCounterC_0_Counter_get(void );
/* # 98 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_getNow(void );
/* #line 92 */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_size_type dt);
/* #line 105 */
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_getAlarm(void );
/* #line 62 */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_fired(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Counter_overflow(void );
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_fired_runTask(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_fired(void );
/* # 125 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_getNow(void );
/* #line 118 */
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_startOneShotAt(uint32_t t0, uint32_t dt);
/* #line 67 */
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_stop(void );
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer_runTask(void );
/* # 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_fired(void );
/* #line 72 */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_default_fired(
/* # 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
uint8_t arg_0x18923c8);
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startPeriodic(
/* # 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
uint8_t arg_0x18923c8, 
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
uint32_t dt);








static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startOneShot(
/* # 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
uint8_t arg_0x18923c8, 
/* # 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_stop(
/* # 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
uint8_t arg_0x18923c8);
/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_0_Counter_overflow(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void AdcP_SubResourceReadNow_granted(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x18fe908);
/* # 55 "/Users/doina/tinyos-2.x/tos/interfaces/Read.nc" */
static error_t AdcP_Read_read(
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x1903338);
/* # 63 "/Users/doina/tinyos-2.x/tos/interfaces/Read.nc" */
static void AdcP_Read_default_readDone(
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x1903338, 
/* # 63 "/Users/doina/tinyos-2.x/tos/interfaces/Read.nc" */
error_t result, AdcP_Read_val_t val);
/* # 66 "/Users/doina/tinyos-2.x/tos/interfaces/ReadNow.nc" */
static void AdcP_ReadNow_default_readDone(
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x1901350, 
/* # 66 "/Users/doina/tinyos-2.x/tos/interfaces/ReadNow.nc" */
error_t result, AdcP_ReadNow_val_t val);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void AdcP_ResourceReadNow_default_granted(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x18ff330);
/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
static AdcP_Config_adc_config_t AdcP_Config_default_getConfiguration(
/* # 48 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x18fc648);
/* # 189 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
static error_t AdcP_SingleChannel_default_getData(
/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x191ee60);
/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
static error_t AdcP_SingleChannel_default_configureSingle(
/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x191ee60, 
/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
const msp430adc12_channel_config_t * config);
/* #line 227 */
static uint16_t * AdcP_SingleChannel_multipleDataReady(
/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x191ee60, 
/* # 227 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
uint16_t * buffer, uint16_t numSamples);
/* #line 206 */
static error_t AdcP_SingleChannel_singleDataReady(
/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x191ee60, 
/* # 206 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
uint16_t data);
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t AdcP_ResourceRead_default_release(
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x18ffe18);
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t AdcP_ResourceRead_default_request(
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x18ffe18);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void AdcP_ResourceRead_granted(
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x18ffe18);
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void AdcP_readDone_runTask(void );
/* # 105 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc" */
static void Msp430Adc12ImplP_MultiChannel_default_dataReady(
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
uint8_t arg_0x1972108, 
/* # 105 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc" */
uint16_t *buffer, uint16_t numSamples);
/* # 112 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
static void Msp430Adc12ImplP_HplAdc12_conversionDone(uint16_t iv);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void Msp430Adc12ImplP_CompareA1_fired(void );
/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc" */
static void Msp430Adc12ImplP_Overflow_default_memOverflow(
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
uint8_t arg_0x19729f8);
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc" */
static void Msp430Adc12ImplP_Overflow_default_conversionTimeOverflow(
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
uint8_t arg_0x19729f8);
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t Msp430Adc12ImplP_Init_init(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void Msp430Adc12ImplP_TimerA_overflow(void );
/* # 189 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
static error_t Msp430Adc12ImplP_SingleChannel_getData(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
uint8_t arg_0x19743f0);
/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
static error_t Msp430Adc12ImplP_SingleChannel_configureSingle(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
uint8_t arg_0x19743f0, 
/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
const msp430adc12_channel_config_t * config);
/* #line 227 */
static uint16_t * Msp430Adc12ImplP_SingleChannel_default_multipleDataReady(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
uint8_t arg_0x19743f0, 
/* # 227 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
uint16_t * buffer, uint16_t numSamples);
/* #line 138 */
static error_t Msp430Adc12ImplP_SingleChannel_configureMultiple(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
uint8_t arg_0x19743f0, 
/* # 138 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
/* #line 206 */
static error_t Msp430Adc12ImplP_SingleChannel_default_singleDataReady(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
uint8_t arg_0x19743f0, 
/* # 206 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
uint16_t data);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void Msp430Adc12ImplP_CompareA0_fired(void );
/* # 63 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
static adc12ctl0_t HplAdc12P_HplAdc12_getCtl0(void );
/* #line 82 */
static adc12memctl_t HplAdc12P_HplAdc12_getMCtl(uint8_t idx);
/* #line 106 */
static void HplAdc12P_HplAdc12_resetIFGs(void );
/* #line 118 */
static bool HplAdc12P_HplAdc12_isBusy(void );
/* #line 75 */
static void HplAdc12P_HplAdc12_setMCtl(uint8_t idx, adc12memctl_t memControl);
/* #line 128 */
static void HplAdc12P_HplAdc12_startConversion(void );
/* #line 51 */
static void HplAdc12P_HplAdc12_setCtl0(adc12ctl0_t control0);
/* #line 89 */
static uint16_t HplAdc12P_HplAdc12_getMem(uint8_t idx);





static void HplAdc12P_HplAdc12_setIEFlags(uint16_t mask);
/* #line 123 */
static void HplAdc12P_HplAdc12_stopConversion(void );
/* #line 57 */
static void HplAdc12P_HplAdc12_setCtl1(adc12ctl1_t control1);
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_Init_init(void );
/* # 69 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceQueue.nc" */
static error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_enqueue(resource_client_id_t id);
/* #line 43 */
static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_isEmpty(void );








static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_dequeue(void );
/* # 43 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceRequested.nc" */
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceRequested_default_requested(
/* # 52 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
uint8_t arg_0x1a813f0);
/* # 55 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceConfigure.nc" */
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceConfigure_default_unconfigure(
/* # 56 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
uint8_t arg_0x1a7f030);
/* # 49 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceConfigure.nc" */
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceConfigure_default_configure(
/* # 56 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
uint8_t arg_0x1a7f030);
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_release(
/* # 51 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
uint8_t arg_0x1a82930);
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_request(
/* # 51 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
uint8_t arg_0x1a82930);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_default_granted(
/* # 51 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
uint8_t arg_0x1a82930);
/* # 88 "/Users/doina/tinyos-2.x/tos/interfaces/ArbiterInfo.nc" */
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ArbiterInfo_userId(void );
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_grantedTask_runTask(void );
/* # 112 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
static void Msp430RefVoltGeneratorP_HplAdc12_conversionDone(uint16_t iv);
/* # 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void Msp430RefVoltGeneratorP_SwitchOffTimer_fired(void );
/* # 83 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
static error_t Msp430RefVoltGeneratorP_RefVolt_2_5V_start(void );
/* #line 83 */
static error_t Msp430RefVoltGeneratorP_RefVolt_1_5V_start(void );
/* #line 109 */
static error_t Msp430RefVoltGeneratorP_RefVolt_1_5V_stop(void );
/* # 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void Msp430RefVoltGeneratorP_SwitchOnTimer_fired(void );
/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
static Msp430RefVoltArbiterImplP_Config_adc_config_t Msp430RefVoltArbiterImplP_Config_default_getConfiguration(
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
uint8_t arg_0x1af6638);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
static void Msp430RefVoltArbiterImplP_RefVolt_2_5V_startDone(error_t error);
/* #line 117 */
static void Msp430RefVoltArbiterImplP_RefVolt_2_5V_stopDone(error_t error);
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t Msp430RefVoltArbiterImplP_AdcResource_default_release(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
uint8_t arg_0x1af9ac8);
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t Msp430RefVoltArbiterImplP_AdcResource_default_request(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
uint8_t arg_0x1af9ac8);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void Msp430RefVoltArbiterImplP_AdcResource_granted(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
uint8_t arg_0x1af9ac8);
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t Msp430RefVoltArbiterImplP_ClientResource_release(
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
uint8_t arg_0x1af9010);
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t Msp430RefVoltArbiterImplP_ClientResource_request(
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
uint8_t arg_0x1af9010);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void Msp430RefVoltArbiterImplP_ClientResource_default_granted(
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
uint8_t arg_0x1af9010);
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void Msp430RefVoltArbiterImplP_switchOff_runTask(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
static void Msp430RefVoltArbiterImplP_RefVolt_1_5V_startDone(error_t error);
/* #line 117 */
static void Msp430RefVoltArbiterImplP_RefVolt_1_5V_stopDone(error_t error);
/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_0_ConfSub_adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_0_ConfSub_getConfiguration(void );
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void AdcStreamP_bufferDone_runTask(void );
/* #line 64 */
static void AdcStreamP_readStreamDone_runTask(void );
/* #line 64 */
static void AdcStreamP_readStreamFail_runTask(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void AdcStreamP_Alarm_fired(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t AdcStreamP_Init_init(void );
/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
static AdcStreamP_AdcConfigure_adc_config_t AdcStreamP_AdcConfigure_default_getConfiguration(
/* # 53 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
uint8_t arg_0x1b5cde0);
/* # 189 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
static error_t AdcStreamP_SingleChannel_default_getData(
/* # 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
uint8_t arg_0x1b5c010);
/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
static error_t AdcStreamP_SingleChannel_default_configureSingle(
/* # 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
uint8_t arg_0x1b5c010, 
/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
const msp430adc12_channel_config_t * config);
/* #line 227 */
static uint16_t * AdcStreamP_SingleChannel_multipleDataReady(
/* # 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
uint8_t arg_0x1b5c010, 
/* # 227 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
uint16_t * buffer, uint16_t numSamples);
/* #line 138 */
static error_t AdcStreamP_SingleChannel_default_configureMultiple(
/* # 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
uint8_t arg_0x1b5c010, 
/* # 138 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
/* #line 206 */
static error_t AdcStreamP_SingleChannel_singleDataReady(
/* # 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
uint8_t arg_0x1b5c010, 
/* # 206 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
uint16_t data);
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
static error_t AdcStreamP_ReadStream_read(
/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
uint8_t arg_0x1b5f340, 
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
uint32_t usPeriod);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_fired(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Timer_overflow(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_startAt(/*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_size_type t0, /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_size_type dt);





static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_getNow(void );
/* #line 92 */
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_size_type dt);
/* #line 67 */
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_AlarmFrom_fired(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Counter_overflow(void );
/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Service_bufferDone(
/* # 26 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
uint8_t arg_0x1bc37c8, 
/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
error_t result, 
/* #line 86 */
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Service_val_t * buf, 



uint16_t count);
/* #line 102 */
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Service_readDone(
/* # 26 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
uint8_t arg_0x1bc37c8, 
/* # 102 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
error_t result, uint32_t usActualPeriod);
/* #line 89 */
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_default_bufferDone(
/* # 24 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
uint8_t arg_0x1bc7bb0, 
/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
error_t result, 
/* #line 86 */
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_val_t * buf, 



uint16_t count);
/* #line 102 */
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_default_readDone(
/* # 24 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
uint8_t arg_0x1bc7bb0, 
/* # 102 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
error_t result, uint32_t usActualPeriod);
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Resource_default_release(
/* # 27 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
uint8_t arg_0x1bbf278);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Resource_granted(
/* # 27 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
uint8_t arg_0x1bbf278);
/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_1_ConfSub_adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_1_ConfSub_getConfiguration(void );
/* #line 58 */
static Msp430InternalVoltageP_AdcConfigure_adc_config_t Msp430InternalVoltageP_AdcConfigure_getConfiguration(void );
/* #line 58 */
static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_2_ConfSub_adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_2_ConfSub_getConfiguration(void );
/* # 55 "/Users/doina/tinyos-2.x/tos/interfaces/Read.nc" */
static error_t SenseC_Read_read(void );
/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
static void SenseC_Leds_led0Off(void );










static void SenseC_Leds_led1On(void );




static void SenseC_Leds_led1Off(void );
/* #line 83 */
static void SenseC_Leds_led2Off(void );
/* #line 45 */
static void SenseC_Leds_led0On(void );
/* #line 78 */
static void SenseC_Leds_led2On(void );
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void SenseC_Timer_startPeriodic(uint32_t dt);
/* # 61 "SenseC.nc" */
static inline void SenseC_Boot_booted(void );



static inline void SenseC_Timer_fired(void );




static void SenseC_Read_readDone(error_t result, uint16_t data);
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t PlatformP_MoteInit_init(void );
/* #line 51 */
static error_t PlatformP_MoteClockInit_init(void );
/* #line 51 */
static error_t PlatformP_LedsInit_init(void );
/* # 10 "/Users/doina/tinyos-2.x/tos/platforms/telosa/PlatformP.nc" */
static inline error_t PlatformP_Init_init(void );
/* # 6 "/Users/doina/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc" */
static __inline void MotePlatformC_uwait(uint16_t u);




static __inline void MotePlatformC_TOSH_wait(void );




static void MotePlatformC_TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC_TOSH_FLASH_M25P_DP(void );
/* #line 56 */
static inline error_t MotePlatformC_Init_init(void );
/* # 32 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc" */
static void Msp430ClockP_Msp430ClockInit_initTimerB(void );
/* #line 31 */
static void Msp430ClockP_Msp430ClockInit_initTimerA(void );
/* #line 29 */
static void Msp430ClockP_Msp430ClockInit_setupDcoCalibrate(void );
static void Msp430ClockP_Msp430ClockInit_initClocks(void );
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc" */
static volatile uint8_t Msp430ClockP_IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP_TA0CTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP_TA0IV __asm ("0x012E");
static volatile uint16_t Msp430ClockP_TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP_TBIV __asm ("0x011E");

enum Msp430ClockP___nesc_unnamed4277 {

  Msp430ClockP_ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP_TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP_ACLK_CALIB_PERIOD
};


static inline void Msp430ClockP_Msp430ClockInit_defaultSetupDcoCalibrate(void );
/* #line 64 */
static inline void Msp430ClockP_Msp430ClockInit_defaultInitClocks(void );
/* #line 85 */
static inline void Msp430ClockP_Msp430ClockInit_defaultInitTimerA(void );
/* #line 100 */
static inline void Msp430ClockP_Msp430ClockInit_defaultInitTimerB(void );
/* #line 115 */
static inline void Msp430ClockP_Msp430ClockInit_default_setupDcoCalibrate(void );




static inline void Msp430ClockP_Msp430ClockInit_default_initClocks(void );




static inline void Msp430ClockP_Msp430ClockInit_default_initTimerA(void );




static inline void Msp430ClockP_Msp430ClockInit_default_initTimerB(void );





static inline void Msp430ClockP_startTimerA(void );
/* #line 148 */
static inline void Msp430ClockP_startTimerB(void );
/* #line 160 */
static void Msp430ClockP_set_dco_calib(int calib);





static inline uint16_t Msp430ClockP_test_calib_busywait_delta(int calib);
/* #line 189 */
static inline void Msp430ClockP_busyCalibrateDco(void );
/* #line 214 */
static inline error_t Msp430ClockP_Init_init(void );
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Event_fired(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
uint8_t arg_0x15734f0);
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_overflow(void );
/* # 80 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_setMode(int mode);









static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_clear(void );









static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_disableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_setClockSource(uint16_t clockSource);




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_setInputDivider(uint16_t inputDivider);




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_VectorTimerX0_fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_VectorTimerX1_fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Overflow_fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Event_default_fired(uint8_t n);
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_fired(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
uint8_t arg_0x15734f0);
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_overflow(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_get(void );
/* #line 70 */
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_isOverflowPending(void );
/* #line 115 */
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_VectorTimerX0_fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_VectorTimerX1_fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Overflow_fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_default_fired(uint8_t n);
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_captured(uint16_t time);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Compare_fired(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_int2CC(uint16_t x)  ;
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Control_getControl(void );
/* #line 89 */
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Control_setControl(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_cc_t x);
/* #line 139 */
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Compare_setEvent(uint16_t x);
/* #line 169 */
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Event_fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_default_captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Timer_overflow(void );
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_captured(uint16_t time);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Compare_fired(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_int2CC(uint16_t x)  ;
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Control_getControl(void );
/* #line 89 */
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Control_setControl(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t x);
/* #line 139 */
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Compare_setEvent(uint16_t x);
/* #line 169 */
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Event_fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_default_captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Timer_overflow(void );
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Capture_captured(uint16_t time);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Compare_fired(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_int2CC(uint16_t x)  ;
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Control_getControl(void );
/* #line 139 */
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Capture_getEvent(void );
/* #line 169 */
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Event_fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Capture_default_captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Compare_default_fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Timer_overflow(void );
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Capture_captured(uint16_t time);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Compare_fired(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Timer_get(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_compareControl(void );
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_setControlAsCompare(void );
/* #line 119 */
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_disableEvents(void );
/* #line 139 */
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Capture_getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Compare_setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Compare_setEventFromNow(uint16_t x);
/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Event_fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Capture_default_captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Timer_overflow(void );
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_captured(uint16_t time);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Compare_fired(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_cc_t;


static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_int2CC(uint16_t x)  ;
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_getControl(void );
/* #line 139 */
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_getEvent(void );
/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Event_fired(void );







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_default_captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Compare_default_fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Timer_overflow(void );
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Capture_captured(uint16_t time);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_fired(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Timer_get(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_int2CC(uint16_t x)  ;
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_clearPendingInterrupt(void );
/* #line 119 */
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_disableEvents(void );
/* #line 139 */
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Capture_getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_setEventFromNow(uint16_t x);
/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Event_fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Capture_default_captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Timer_overflow(void );
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Capture_captured(uint16_t time);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Compare_fired(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_int2CC(uint16_t x)  ;
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Control_getControl(void );
/* #line 139 */
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Capture_getEvent(void );
/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Event_fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Capture_default_captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Compare_default_fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Timer_overflow(void );
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Capture_captured(uint16_t time);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Compare_fired(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_int2CC(uint16_t x)  ;
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Control_getControl(void );
/* #line 139 */
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Capture_getEvent(void );
/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Event_fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Capture_default_captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Compare_default_fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Timer_overflow(void );
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Capture_captured(uint16_t time);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Compare_fired(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_int2CC(uint16_t x)  ;
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Control_getControl(void );
/* #line 139 */
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Capture_getEvent(void );
/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Event_fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Capture_default_captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Compare_default_fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Timer_overflow(void );
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Capture_captured(uint16_t time);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Compare_fired(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_int2CC(uint16_t x)  ;
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Control_getControl(void );
/* #line 139 */
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Capture_getEvent(void );
/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Event_fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Capture_default_captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Compare_default_fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Timer_overflow(void );
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void Msp430TimerCommonP_VectorTimerB1_fired(void );
/* #line 28 */
static void Msp430TimerCommonP_VectorTimerA0_fired(void );
/* #line 28 */
static void Msp430TimerCommonP_VectorTimerA1_fired(void );
/* #line 28 */
static void Msp430TimerCommonP_VectorTimerB0_fired(void );
/* # 11 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc" */
void sig_TIMERA0_VECTOR(void )   ;
void sig_TIMERA1_VECTOR(void )   ;
void sig_TIMERB0_VECTOR(void )   ;
void sig_TIMERB1_VECTOR(void )   ;
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t RealMainP_SoftwareInit_init(void );
/* # 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc" */
static void RealMainP_Boot_booted(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t RealMainP_PlatformInit_init(void );
/* # 46 "/Users/doina/tinyos-2.x/tos/interfaces/Scheduler.nc" */
static void RealMainP_Scheduler_init(void );
/* #line 61 */
static void RealMainP_Scheduler_taskLoop(void );
/* #line 54 */
static bool RealMainP_Scheduler_runNextTask(void );
/* # 52 "/Users/doina/tinyos-2.x/tos/system/RealMainP.nc" */
int main(void )   ;
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void SchedulerBasicP_TaskBasic_runTask(
/* # 45 "/Users/doina/tinyos-2.x/tos/system/SchedulerBasicP.nc" */
uint8_t arg_0x147bb38);
/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/McuSleep.nc" */
static void SchedulerBasicP_McuSleep_sleep(void );
/* # 50 "/Users/doina/tinyos-2.x/tos/system/SchedulerBasicP.nc" */
enum SchedulerBasicP___nesc_unnamed4278 {

  SchedulerBasicP_NUM_TASKS = 8U, 
  SchedulerBasicP_NO_TASK = 255
};

uint8_t SchedulerBasicP_m_head;
uint8_t SchedulerBasicP_m_tail;
uint8_t SchedulerBasicP_m_next[SchedulerBasicP_NUM_TASKS];








static __inline uint8_t SchedulerBasicP_popTask(void );
/* #line 86 */
static inline bool SchedulerBasicP_isWaiting(uint8_t id);




static inline bool SchedulerBasicP_pushTask(uint8_t id);
/* #line 113 */
static inline void SchedulerBasicP_Scheduler_init(void );









static bool SchedulerBasicP_Scheduler_runNextTask(void );
/* #line 138 */
static inline void SchedulerBasicP_Scheduler_taskLoop(void );
/* #line 159 */
static error_t SchedulerBasicP_TaskBasic_postTask(uint8_t id);




static void SchedulerBasicP_TaskBasic_default_runTask(uint8_t id);
/* # 54 "/Users/doina/tinyos-2.x/tos/interfaces/McuPowerOverride.nc" */
static mcu_power_t McuSleepC_McuPowerOverride_lowestState(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/chips/msp430/McuSleepC.nc" */
bool McuSleepC_dirty = TRUE;
mcu_power_t McuSleepC_powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC_msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC_getPowerState(void );
/* #line 104 */
static inline void McuSleepC_computePowerState(void );




static inline void McuSleepC_McuSleep_sleep(void );
/* #line 126 */
static inline mcu_power_t McuSleepC_McuPowerOverride_default_lowestState(void );
/* # 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
static void LedsP_Led0_makeOutput(void );
/* #line 29 */
static void LedsP_Led0_set(void );
static void LedsP_Led0_clr(void );




static void LedsP_Led1_makeOutput(void );
/* #line 29 */
static void LedsP_Led1_set(void );
static void LedsP_Led1_clr(void );




static void LedsP_Led2_makeOutput(void );
/* #line 29 */
static void LedsP_Led2_set(void );
static void LedsP_Led2_clr(void );
/* # 45 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc" */
static inline error_t LedsP_Init_init(void );
/* #line 63 */
static inline void LedsP_Leds_led0On(void );




static inline void LedsP_Leds_led0Off(void );









static inline void LedsP_Leds_led1On(void );




static inline void LedsP_Leds_led1Off(void );









static inline void LedsP_Leds_led2On(void );




static inline void LedsP_Leds_led2Off(void );
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_set(void );
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_clr(void );





static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_makeOutput(void );
/* #line 45 */
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_set(void );
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_clr(void );





static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_makeOutput(void );
/* #line 45 */
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_set(void );
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_clr(void );





static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_makeOutput(void );
/* #line 50 */
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP_40_IO_makeInput(void );



static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP_40_IO_selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP_40_IO_selectIOFunc(void );
/* #line 50 */
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP_41_IO_makeInput(void );



static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP_41_IO_selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP_41_IO_selectIOFunc(void );
/* #line 50 */
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP_42_IO_makeInput(void );



static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP_42_IO_selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP_42_IO_selectIOFunc(void );
/* #line 50 */
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP_43_IO_makeInput(void );



static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP_43_IO_selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP_43_IO_selectIOFunc(void );
/* #line 50 */
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP_44_IO_makeInput(void );



static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP_44_IO_selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP_44_IO_selectIOFunc(void );
/* #line 50 */
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP_45_IO_makeInput(void );



static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP_45_IO_selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP_45_IO_selectIOFunc(void );
/* #line 50 */
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP_46_IO_makeInput(void );



static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP_46_IO_selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP_46_IO_selectIOFunc(void );
/* #line 50 */
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP_47_IO_makeInput(void );



static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP_47_IO_selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP_47_IO_selectIOFunc(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_HplGeneralIO_makeOutput(void );
/* #line 34 */
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_HplGeneralIO_set(void );




static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_HplGeneralIO_clr(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_set(void );
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_clr(void );




static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_makeOutput(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_HplGeneralIO_makeOutput(void );
/* #line 34 */
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_HplGeneralIO_set(void );




static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_HplGeneralIO_clr(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_set(void );
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_clr(void );




static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_makeOutput(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_HplGeneralIO_makeOutput(void );
/* #line 34 */
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_HplGeneralIO_set(void );




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_HplGeneralIO_clr(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_set(void );
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_clr(void );




static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_makeOutput(void );
/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_setEventFromNow(uint16_t delta);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Timer_get(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_fired(void );
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_enableEvents(void );
/* #line 36 */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_disableEvents(void );
/* #line 33 */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_clearPendingInterrupt(void );
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Init_init(void );
/* #line 54 */
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_startAt(uint16_t t0, uint16_t dt);
/* #line 103 */
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Timer_overflow(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Msp430Timer_get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Msp430Timer_isOverflowPending(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_overflow(void );
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc" */
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Msp430Timer_overflow(void );
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static /*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_size_type /*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC_0_Counter_overflow(void );
/* # 56 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformCounterC.nc" */
/*CounterMilli32C.Transform*/TransformCounterC_0_upper_count_type /*CounterMilli32C.Transform*/TransformCounterC_0_m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC_0___nesc_unnamed4279 {

  TransformCounterC_0_LOW_SHIFT_RIGHT = 5, 
  TransformCounterC_0_HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC_0_from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC_0_LOW_SHIFT_RIGHT, 
  TransformCounterC_0_NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC_0_to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC_0_from_size_type ) + 5, 



  TransformCounterC_0_OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC_0_NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC_0_upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC_0_NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC_0_to_size_type /*CounterMilli32C.Transform*/TransformCounterC_0_Counter_get(void );
/* #line 122 */
static inline void /*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_overflow(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_fired(void );
/* #line 92 */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_size_type dt);
/* #line 62 */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_stop(void );
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Counter_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Counter_get(void );
/* # 66 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0___nesc_unnamed4280 {

  TransformAlarmC_0_MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_from_size_type ) - 1 - 5, 
  TransformAlarmC_0_MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_set_alarm(void );
/* #line 136 */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type dt);
/* #line 151 */
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_fired(void );
/* #line 166 */
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Counter_overflow(void );
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_fired_postTask(void );
/* # 98 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_getNow(void );
/* #line 92 */
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_size_type dt);
/* #line 105 */
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_getAlarm(void );
/* #line 62 */
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_stop(void );
/* # 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_fired(void );
/* # 63 "/Users/doina/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc" */
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0___nesc_unnamed4281 {
/* #line 63 */
  AlarmToTimerC_0_fired = 0U
};
/* #line 63 */
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0___nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_fired];
/* #line 44 */
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_start(uint32_t t0, uint32_t dt, bool oneshot);
/* #line 60 */
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_fired_runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_fired(void );
/* #line 82 */
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_getNow(void );
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer_postTask(void );
/* # 125 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_getNow(void );
/* #line 118 */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_startOneShotAt(uint32_t t0, uint32_t dt);
/* #line 67 */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_fired(
/* # 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
uint8_t arg_0x18923c8);
/* #line 60 */
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0___nesc_unnamed4282 {
/* #line 60 */
  VirtualizeTimerC_0_updateFromTimer = 1U
};
/* #line 60 */
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0___nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer];
/* #line 42 */
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0___nesc_unnamed4283 {

  VirtualizeTimerC_0_NUM_TIMERS = 3U, 
  VirtualizeTimerC_0_END_OF_LIST = 255
};








/* #line 48 */
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0___nesc_unnamed4284 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_fireTimers(uint32_t now);
/* #line 89 */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer_runTask(void );
/* #line 128 */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_fired(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startPeriodic(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_stop(uint8_t num);
/* #line 193 */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_default_fired(uint8_t num);
/* # 47 "/Users/doina/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc" */
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_0_Counter_overflow(void );
/* # 63 "/Users/doina/tinyos-2.x/tos/interfaces/Read.nc" */
static void AdcP_Read_readDone(
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x1903338, 
/* # 63 "/Users/doina/tinyos-2.x/tos/interfaces/Read.nc" */
error_t result, AdcP_Read_val_t val);
/* # 66 "/Users/doina/tinyos-2.x/tos/interfaces/ReadNow.nc" */
static void AdcP_ReadNow_readDone(
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x1901350, 
/* # 66 "/Users/doina/tinyos-2.x/tos/interfaces/ReadNow.nc" */
error_t result, AdcP_ReadNow_val_t val);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void AdcP_ResourceReadNow_granted(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x18ff330);
/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
static AdcP_Config_adc_config_t AdcP_Config_getConfiguration(
/* # 48 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x18fc648);
/* # 189 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
static error_t AdcP_SingleChannel_getData(
/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x191ee60);
/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
static error_t AdcP_SingleChannel_configureSingle(
/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x191ee60, 
/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
const msp430adc12_channel_config_t * config);
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t AdcP_ResourceRead_release(
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x18ffe18);
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t AdcP_ResourceRead_request(
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
uint8_t arg_0x18ffe18);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t AdcP_readDone_postTask(void );
/* # 136 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
enum AdcP___nesc_unnamed4285 {
/* #line 136 */
  AdcP_readDone = 2U
};
/* #line 136 */
typedef int AdcP___nesc_sillytask_readDone[AdcP_readDone];
/* #line 54 */
enum AdcP___nesc_unnamed4286 {
  AdcP_STATE_READ, 
  AdcP_STATE_READNOW, 
  AdcP_STATE_READNOW_INVALID_CONFIG
};


uint8_t AdcP_state;
uint8_t AdcP_owner;
uint16_t AdcP_value;

static error_t AdcP_configure(uint8_t client);









static inline error_t AdcP_Read_read(uint8_t client);




static void AdcP_ResourceRead_granted(uint8_t client);
/* #line 98 */
static void AdcP_SubResourceReadNow_granted(uint8_t nowClient);
/* #line 136 */
static inline void AdcP_readDone_runTask(void );





static error_t AdcP_SingleChannel_singleDataReady(uint8_t client, uint16_t data);
/* #line 161 */
static inline uint16_t *AdcP_SingleChannel_multipleDataReady(uint8_t client, 
uint16_t *buf, uint16_t numSamples);





static inline error_t AdcP_ResourceRead_default_request(uint8_t client);

static inline error_t AdcP_ResourceRead_default_release(uint8_t client);

static inline void AdcP_Read_default_readDone(uint8_t client, error_t result, uint16_t val);




static inline void AdcP_ResourceReadNow_default_granted(uint8_t nowClient);
static inline void AdcP_ReadNow_default_readDone(uint8_t client, error_t result, uint16_t val);

static inline error_t AdcP_SingleChannel_default_getData(uint8_t client);




const msp430adc12_channel_config_t AdcP_defaultConfig = { INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0 };
static inline const msp430adc12_channel_config_t *
AdcP_Config_default_getConfiguration(uint8_t client);



static inline error_t AdcP_SingleChannel_default_configureSingle(uint8_t client, 
const msp430adc12_channel_config_t *config);
/* # 105 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc" */
static void Msp430Adc12ImplP_MultiChannel_dataReady(
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
uint8_t arg_0x1972108, 
/* # 105 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc" */
uint16_t *buffer, uint16_t numSamples);
/* # 63 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
static adc12ctl0_t Msp430Adc12ImplP_HplAdc12_getCtl0(void );
/* #line 82 */
static adc12memctl_t Msp430Adc12ImplP_HplAdc12_getMCtl(uint8_t idx);
/* #line 106 */
static void Msp430Adc12ImplP_HplAdc12_resetIFGs(void );
/* #line 75 */
static void Msp430Adc12ImplP_HplAdc12_setMCtl(uint8_t idx, adc12memctl_t memControl);
/* #line 128 */
static void Msp430Adc12ImplP_HplAdc12_startConversion(void );
/* #line 51 */
static void Msp430Adc12ImplP_HplAdc12_setCtl0(adc12ctl0_t control0);
/* #line 89 */
static uint16_t Msp430Adc12ImplP_HplAdc12_getMem(uint8_t idx);





static void Msp430Adc12ImplP_HplAdc12_setIEFlags(uint16_t mask);
/* #line 123 */
static void Msp430Adc12ImplP_HplAdc12_stopConversion(void );
/* #line 57 */
static void Msp430Adc12ImplP_HplAdc12_setCtl1(adc12ctl1_t control1);
/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void Msp430Adc12ImplP_Port64_makeInput(void );
/* #line 85 */
static void Msp430Adc12ImplP_Port64_selectIOFunc(void );
/* #line 78 */
static void Msp430Adc12ImplP_Port64_selectModuleFunc(void );
/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void Msp430Adc12ImplP_CompareA1_setEvent(uint16_t time);
/* # 35 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static void Msp430Adc12ImplP_ControlA0_setControl(msp430_compare_control_t control);
/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void Msp430Adc12ImplP_Port62_makeInput(void );
/* #line 85 */
static void Msp430Adc12ImplP_Port62_selectIOFunc(void );
/* #line 78 */
static void Msp430Adc12ImplP_Port62_selectModuleFunc(void );
/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc" */
static void Msp430Adc12ImplP_Overflow_memOverflow(
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
uint8_t arg_0x19729f8);
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc" */
static void Msp430Adc12ImplP_Overflow_conversionTimeOverflow(
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
uint8_t arg_0x19729f8);
/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void Msp430Adc12ImplP_Port67_makeInput(void );
/* #line 85 */
static void Msp430Adc12ImplP_Port67_selectIOFunc(void );
/* #line 78 */
static void Msp430Adc12ImplP_Port67_selectModuleFunc(void );
/* #line 64 */
static void Msp430Adc12ImplP_Port60_makeInput(void );
/* #line 85 */
static void Msp430Adc12ImplP_Port60_selectIOFunc(void );
/* #line 78 */
static void Msp430Adc12ImplP_Port60_selectModuleFunc(void );
/* #line 64 */
static void Msp430Adc12ImplP_Port65_makeInput(void );
/* #line 85 */
static void Msp430Adc12ImplP_Port65_selectIOFunc(void );
/* #line 78 */
static void Msp430Adc12ImplP_Port65_selectModuleFunc(void );
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void Msp430Adc12ImplP_TimerA_clear(void );


static void Msp430Adc12ImplP_TimerA_setClockSource(uint16_t clockSource);
/* #line 43 */
static void Msp430Adc12ImplP_TimerA_disableEvents(void );
/* #line 39 */
static void Msp430Adc12ImplP_TimerA_setMode(int mode);





static void Msp430Adc12ImplP_TimerA_setInputDivider(uint16_t inputDivider);
/* # 88 "/Users/doina/tinyos-2.x/tos/interfaces/ArbiterInfo.nc" */
static uint8_t Msp430Adc12ImplP_ADCArbiterInfo_userId(void );
/* # 35 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static void Msp430Adc12ImplP_ControlA1_setControl(msp430_compare_control_t control);
/* # 227 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
static uint16_t * Msp430Adc12ImplP_SingleChannel_multipleDataReady(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
uint8_t arg_0x19743f0, 
/* # 227 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
uint16_t * buffer, uint16_t numSamples);
/* #line 206 */
static error_t Msp430Adc12ImplP_SingleChannel_singleDataReady(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
uint8_t arg_0x19743f0, 
/* # 206 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
uint16_t data);
/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void Msp430Adc12ImplP_Port63_makeInput(void );
/* #line 85 */
static void Msp430Adc12ImplP_Port63_selectIOFunc(void );
/* #line 78 */
static void Msp430Adc12ImplP_Port63_selectModuleFunc(void );
/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void Msp430Adc12ImplP_CompareA0_setEvent(uint16_t time);
/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void Msp430Adc12ImplP_Port61_makeInput(void );
/* #line 85 */
static void Msp430Adc12ImplP_Port61_selectIOFunc(void );
/* #line 78 */
static void Msp430Adc12ImplP_Port61_selectModuleFunc(void );
/* #line 64 */
static void Msp430Adc12ImplP_Port66_makeInput(void );
/* #line 85 */
static void Msp430Adc12ImplP_Port66_selectIOFunc(void );
/* #line 78 */
static void Msp430Adc12ImplP_Port66_selectModuleFunc(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
enum Msp430Adc12ImplP___nesc_unnamed4287 {
  Msp430Adc12ImplP_SINGLE_DATA = 1, 
  Msp430Adc12ImplP_SINGLE_DATA_REPEAT = 2, 
  Msp430Adc12ImplP_MULTIPLE_DATA = 4, 
  Msp430Adc12ImplP_MULTIPLE_DATA_REPEAT = 8, 
  Msp430Adc12ImplP_MULTI_CHANNEL = 16, 
  Msp430Adc12ImplP_CONVERSION_MODE_MASK = 0x1F, 

  Msp430Adc12ImplP_ADC_BUSY = 32, 
  Msp430Adc12ImplP_USE_TIMERA = 64, 
  Msp430Adc12ImplP_ADC_OVERFLOW = 128
};

uint8_t Msp430Adc12ImplP_state;

uint16_t Msp430Adc12ImplP_resultBufferLength;
uint16_t * Msp430Adc12ImplP_resultBufferStart;
uint16_t Msp430Adc12ImplP_resultBufferIndex;
uint8_t Msp430Adc12ImplP_numChannels;
uint8_t Msp430Adc12ImplP_clientID;

static inline error_t Msp430Adc12ImplP_Init_init(void );










static inline void Msp430Adc12ImplP_prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON);
/* #line 121 */
static inline void Msp430Adc12ImplP_startTimerA(void );
/* #line 142 */
static inline void Msp430Adc12ImplP_configureAdcPin(uint8_t inch);
/* #line 159 */
static void Msp430Adc12ImplP_resetAdcPin(uint8_t inch);
/* #line 176 */
static error_t Msp430Adc12ImplP_SingleChannel_configureSingle(uint8_t id, 
const msp430adc12_channel_config_t *config);
/* #line 271 */
static inline error_t Msp430Adc12ImplP_SingleChannel_configureMultiple(uint8_t id, 
const msp430adc12_channel_config_t *config, 
uint16_t *buf, uint16_t length, uint16_t jiffies);
/* #line 394 */
static error_t Msp430Adc12ImplP_SingleChannel_getData(uint8_t id);
/* #line 503 */
static void Msp430Adc12ImplP_stopConversion(void );
/* #line 540 */
static inline void Msp430Adc12ImplP_TimerA_overflow(void );
static inline void Msp430Adc12ImplP_CompareA0_fired(void );
static inline void Msp430Adc12ImplP_CompareA1_fired(void );

static inline void Msp430Adc12ImplP_HplAdc12_conversionDone(uint16_t iv);
/* #line 640 */
static inline error_t Msp430Adc12ImplP_SingleChannel_default_singleDataReady(uint8_t id, uint16_t data);




static inline uint16_t *Msp430Adc12ImplP_SingleChannel_default_multipleDataReady(uint8_t id, 
uint16_t *buf, uint16_t numSamples);




static inline void Msp430Adc12ImplP_MultiChannel_default_dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples);

static inline void Msp430Adc12ImplP_Overflow_default_memOverflow(uint8_t id);
static inline void Msp430Adc12ImplP_Overflow_default_conversionTimeOverflow(uint8_t id);
/* # 112 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
static void HplAdc12P_HplAdc12_conversionDone(uint16_t iv);
/* # 51 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc" */
static volatile uint16_t HplAdc12P_ADC12CTL0 __asm ("0x01A0");
static volatile uint16_t HplAdc12P_ADC12CTL1 __asm ("0x01A2");
static volatile uint16_t HplAdc12P_ADC12IFG __asm ("0x01A4");
static volatile uint16_t HplAdc12P_ADC12IE __asm ("0x01A6");
static volatile uint16_t HplAdc12P_ADC12IV __asm ("0x01A8");





static inline void HplAdc12P_HplAdc12_setCtl0(adc12ctl0_t control0);



static inline void HplAdc12P_HplAdc12_setCtl1(adc12ctl1_t control1);



static inline adc12ctl0_t HplAdc12P_HplAdc12_getCtl0(void );







static inline void HplAdc12P_HplAdc12_setMCtl(uint8_t i, adc12memctl_t memControl);





static adc12memctl_t HplAdc12P_HplAdc12_getMCtl(uint8_t i);







static inline uint16_t HplAdc12P_HplAdc12_getMem(uint8_t i);



static inline void HplAdc12P_HplAdc12_setIEFlags(uint16_t mask);


static inline void HplAdc12P_HplAdc12_resetIFGs(void );




static inline void HplAdc12P_HplAdc12_startConversion(void );




static void HplAdc12P_HplAdc12_stopConversion(void );
/* #line 121 */
static inline bool HplAdc12P_HplAdc12_isBusy(void );

void sig_ADC_VECTOR(void )   ;
/* # 39 "/Users/doina/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc" */
enum /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0___nesc_unnamed4288 {
  RoundRobinResourceQueueC_0_NO_ENTRY = 0xFF, 
  RoundRobinResourceQueueC_0_SIZE = 3U ? (3U - 1) / 8 + 1 : 0
};

uint8_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_resQ[/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_SIZE];
uint8_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_last = 0;

static inline void /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_clearEntry(uint8_t id);



static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_Init_init(void );




static inline bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_isEmpty(void );








static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_dequeue(void );
/* #line 87 */
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_enqueue(resource_client_id_t id);
/* # 43 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceRequested.nc" */
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceRequested_requested(
/* # 52 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
uint8_t arg_0x1a813f0);
/* # 55 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceConfigure.nc" */
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceConfigure_unconfigure(
/* # 56 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
uint8_t arg_0x1a7f030);
/* # 49 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceConfigure.nc" */
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceConfigure_configure(
/* # 56 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
uint8_t arg_0x1a7f030);
/* # 69 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceQueue.nc" */
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Queue_enqueue(resource_client_id_t id);
/* #line 43 */
static bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Queue_isEmpty(void );
/* #line 60 */
static resource_client_id_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Queue_dequeue(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_granted(
/* # 51 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
uint8_t arg_0x1a82930);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_grantedTask_postTask(void );
/* # 69 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0___nesc_unnamed4289 {
/* #line 69 */
  SimpleArbiterP_0_grantedTask = 3U
};
/* #line 69 */
typedef int /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0___nesc_sillytask_grantedTask[/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_grantedTask];
/* #line 62 */
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0___nesc_unnamed4290 {
/* #line 62 */
  SimpleArbiterP_0_RES_IDLE = 0, SimpleArbiterP_0_RES_GRANTING = 1, SimpleArbiterP_0_RES_BUSY = 2
};
/* #line 63 */
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0___nesc_unnamed4291 {
/* #line 63 */
  SimpleArbiterP_0_NO_RES = 0xFF
};
uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_RES_IDLE;
uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_NO_RES;
uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_reqResId;



static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_request(uint8_t id);
/* #line 97 */
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_release(uint8_t id);
/* #line 137 */
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ArbiterInfo_userId(void );
/* #line 155 */
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_grantedTask_runTask(void );









static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_default_granted(uint8_t id);

static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceRequested_default_requested(uint8_t id);



static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceConfigure_default_configure(uint8_t id);

static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceConfigure_default_unconfigure(uint8_t id);
/* # 63 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
static adc12ctl0_t Msp430RefVoltGeneratorP_HplAdc12_getCtl0(void );
/* #line 118 */
static bool Msp430RefVoltGeneratorP_HplAdc12_isBusy(void );
/* #line 51 */
static void Msp430RefVoltGeneratorP_HplAdc12_setCtl0(adc12ctl0_t control0);
/* # 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void Msp430RefVoltGeneratorP_SwitchOffTimer_startOneShot(uint32_t dt);




static void Msp430RefVoltGeneratorP_SwitchOffTimer_stop(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
static void Msp430RefVoltGeneratorP_RefVolt_2_5V_startDone(error_t error);
/* #line 117 */
static void Msp430RefVoltGeneratorP_RefVolt_2_5V_stopDone(error_t error);
/* #line 92 */
static void Msp430RefVoltGeneratorP_RefVolt_1_5V_startDone(error_t error);
/* #line 117 */
static void Msp430RefVoltGeneratorP_RefVolt_1_5V_stopDone(error_t error);
/* # 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void Msp430RefVoltGeneratorP_SwitchOnTimer_startOneShot(uint32_t dt);




static void Msp430RefVoltGeneratorP_SwitchOnTimer_stop(void );
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc" */
enum Msp430RefVoltGeneratorP___nesc_unnamed4292 {

  Msp430RefVoltGeneratorP_GENERATOR_OFF, 
  Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING, 
  Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING, 
  Msp430RefVoltGeneratorP_REFERENCE_1_5V_STABLE, 
  Msp430RefVoltGeneratorP_REFERENCE_2_5V_STABLE
};

uint8_t Msp430RefVoltGeneratorP_state;

static error_t Msp430RefVoltGeneratorP_switchOn(uint8_t level);
/* #line 78 */
static error_t Msp430RefVoltGeneratorP_switchOff(void );
/* #line 94 */
static inline error_t Msp430RefVoltGeneratorP_RefVolt_1_5V_start(void );
/* #line 127 */
static inline error_t Msp430RefVoltGeneratorP_RefVolt_1_5V_stop(void );
/* #line 157 */
static inline error_t Msp430RefVoltGeneratorP_RefVolt_2_5V_start(void );
/* #line 220 */
static inline void Msp430RefVoltGeneratorP_SwitchOnTimer_fired(void );
/* #line 244 */
static inline void Msp430RefVoltGeneratorP_SwitchOffTimer_fired(void );
/* #line 274 */
static inline void Msp430RefVoltGeneratorP_HplAdc12_conversionDone(uint16_t iv);
/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
static Msp430RefVoltArbiterImplP_Config_adc_config_t Msp430RefVoltArbiterImplP_Config_getConfiguration(
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
uint8_t arg_0x1af6638);
/* # 83 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
static error_t Msp430RefVoltArbiterImplP_RefVolt_2_5V_start(void );
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t Msp430RefVoltArbiterImplP_AdcResource_release(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
uint8_t arg_0x1af9ac8);
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t Msp430RefVoltArbiterImplP_AdcResource_request(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
uint8_t arg_0x1af9ac8);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void Msp430RefVoltArbiterImplP_ClientResource_granted(
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
uint8_t arg_0x1af9010);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t Msp430RefVoltArbiterImplP_switchOff_postTask(void );
/* # 83 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
static error_t Msp430RefVoltArbiterImplP_RefVolt_1_5V_start(void );
/* #line 109 */
static error_t Msp430RefVoltArbiterImplP_RefVolt_1_5V_stop(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
enum Msp430RefVoltArbiterImplP___nesc_unnamed4293 {
/* #line 51 */
  Msp430RefVoltArbiterImplP_switchOff = 4U
};
/* #line 51 */
typedef int Msp430RefVoltArbiterImplP___nesc_sillytask_switchOff[Msp430RefVoltArbiterImplP_switchOff];
/* #line 46 */
enum Msp430RefVoltArbiterImplP___nesc_unnamed4294 {
  Msp430RefVoltArbiterImplP_NO_OWNER = 0xFF
};
uint8_t Msp430RefVoltArbiterImplP_syncOwner = Msp430RefVoltArbiterImplP_NO_OWNER;



static inline error_t Msp430RefVoltArbiterImplP_ClientResource_request(uint8_t client);
/* #line 70 */
static void Msp430RefVoltArbiterImplP_AdcResource_granted(uint8_t client);
/* #line 98 */
static inline void Msp430RefVoltArbiterImplP_RefVolt_1_5V_startDone(error_t error);








static void Msp430RefVoltArbiterImplP_RefVolt_2_5V_startDone(error_t error);








static error_t Msp430RefVoltArbiterImplP_ClientResource_release(uint8_t client);
/* #line 136 */
static inline void Msp430RefVoltArbiterImplP_switchOff_runTask(void );










static inline void Msp430RefVoltArbiterImplP_RefVolt_1_5V_stopDone(error_t error);



static inline void Msp430RefVoltArbiterImplP_RefVolt_2_5V_stopDone(error_t error);








static inline void Msp430RefVoltArbiterImplP_ClientResource_default_granted(uint8_t client);
static inline error_t Msp430RefVoltArbiterImplP_AdcResource_default_request(uint8_t client);








static inline error_t Msp430RefVoltArbiterImplP_AdcResource_default_release(uint8_t client);
const msp430adc12_channel_config_t Msp430RefVoltArbiterImplP_defaultConfig = { INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0 };
static inline const msp430adc12_channel_config_t *
Msp430RefVoltArbiterImplP_Config_default_getConfiguration(uint8_t client);
/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_0_ConfUp_adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_0_ConfUp_getConfiguration(void );
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc" */
static inline const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_0_ConfSub_getConfiguration(void );
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t AdcStreamP_bufferDone_postTask(void );
/* #line 56 */
static error_t AdcStreamP_readStreamDone_postTask(void );
/* #line 56 */
static error_t AdcStreamP_readStreamFail_postTask(void );
/* # 98 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static AdcStreamP_Alarm_size_type AdcStreamP_Alarm_getNow(void );
/* #line 92 */
static void AdcStreamP_Alarm_startAt(AdcStreamP_Alarm_size_type t0, AdcStreamP_Alarm_size_type dt);
/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
static AdcStreamP_AdcConfigure_adc_config_t AdcStreamP_AdcConfigure_getConfiguration(
/* # 53 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
uint8_t arg_0x1b5cde0);
/* # 189 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
static error_t AdcStreamP_SingleChannel_getData(
/* # 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
uint8_t arg_0x1b5c010);
/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
static error_t AdcStreamP_SingleChannel_configureSingle(
/* # 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
uint8_t arg_0x1b5c010, 
/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
const msp430adc12_channel_config_t * config);
/* #line 138 */
static error_t AdcStreamP_SingleChannel_configureMultiple(
/* # 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
uint8_t arg_0x1b5c010, 
/* # 138 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
static void AdcStreamP_ReadStream_bufferDone(
/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
uint8_t arg_0x1b5f340, 
/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
error_t result, 
/* #line 86 */
AdcStreamP_ReadStream_val_t * buf, 



uint16_t count);
/* #line 102 */
static void AdcStreamP_ReadStream_readDone(
/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
uint8_t arg_0x1b5f340, 
/* # 102 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
error_t result, uint32_t usActualPeriod);
/* # 119 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
enum AdcStreamP___nesc_unnamed4295 {
/* #line 119 */
  AdcStreamP_readStreamDone = 5U
};
/* #line 119 */
typedef int AdcStreamP___nesc_sillytask_readStreamDone[AdcStreamP_readStreamDone];
/* #line 135 */
enum AdcStreamP___nesc_unnamed4296 {
/* #line 135 */
  AdcStreamP_readStreamFail = 6U
};
/* #line 135 */
typedef int AdcStreamP___nesc_sillytask_readStreamFail[AdcStreamP_readStreamFail];
/* #line 156 */
enum AdcStreamP___nesc_unnamed4297 {
/* #line 156 */
  AdcStreamP_bufferDone = 7U
};
/* #line 156 */
typedef int AdcStreamP___nesc_sillytask_bufferDone[AdcStreamP_bufferDone];
/* #line 58 */
enum AdcStreamP___nesc_unnamed4298 {
  AdcStreamP_NSTREAM = 1U
};




uint8_t AdcStreamP_client = AdcStreamP_NSTREAM;


struct AdcStreamP_list_entry_t {
  uint16_t count;
  struct AdcStreamP_list_entry_t * next;
};
struct AdcStreamP_list_entry_t *AdcStreamP_bufferQueue[AdcStreamP_NSTREAM];
struct AdcStreamP_list_entry_t * *AdcStreamP_bufferQueueEnd[AdcStreamP_NSTREAM];
uint16_t * AdcStreamP_lastBuffer;
/* #line 74 */
uint16_t AdcStreamP_lastCount;

uint16_t AdcStreamP_count;
uint16_t * AdcStreamP_buffer;
uint16_t * AdcStreamP_pos;
uint32_t AdcStreamP_now;
/* #line 79 */
uint32_t AdcStreamP_period;
bool AdcStreamP_periodModified;


static inline error_t AdcStreamP_Init_init(void );








static inline void AdcStreamP_sampleSingle(void );



static inline error_t AdcStreamP_postBuffer(uint8_t c, uint16_t *buf, uint16_t n);
/* #line 119 */
static inline void AdcStreamP_readStreamDone_runTask(void );
/* #line 135 */
static inline void AdcStreamP_readStreamFail_runTask(void );
/* #line 156 */
static inline void AdcStreamP_bufferDone_runTask(void );
/* #line 168 */
static inline void AdcStreamP_nextAlarm(void );




static inline void AdcStreamP_Alarm_fired(void );



static error_t AdcStreamP_nextBuffer(bool startNextAlarm);
/* #line 206 */
static void AdcStreamP_nextMultiple(uint8_t c);
/* #line 221 */
static error_t AdcStreamP_ReadStream_read(uint8_t c, uint32_t usPeriod);
/* #line 242 */
static error_t AdcStreamP_SingleChannel_singleDataReady(uint8_t streamClient, uint16_t data);
/* #line 281 */
static uint16_t *AdcStreamP_SingleChannel_multipleDataReady(uint8_t streamClient, 
uint16_t *buf, uint16_t length);
/* #line 304 */
const msp430adc12_channel_config_t AdcStreamP_defaultConfig = { 
.inch = SUPPLY_VOLTAGE_HALF_CHANNEL, 
.sref = REFERENCE_VREFplus_AVss, 
.ref2_5v = REFVOLT_LEVEL_1_5, 
.adc12ssel = SHT_SOURCE_ACLK, 
.adc12div = SHT_CLOCK_DIV_1, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };

static inline const msp430adc12_channel_config_t *AdcStreamP_AdcConfigure_default_getConfiguration(uint8_t c);



static inline error_t AdcStreamP_SingleChannel_default_configureMultiple(uint8_t c, 
const msp430adc12_channel_config_t *config, uint16_t b[], 
uint16_t numSamples, uint16_t jiffies);



static inline error_t AdcStreamP_SingleChannel_default_getData(uint8_t c);



static inline error_t AdcStreamP_SingleChannel_default_configureSingle(uint8_t c, 
const msp430adc12_channel_config_t *config);
/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_setEvent(uint16_t time);

static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_setEventFromNow(uint16_t delta);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Timer_get(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_fired(void );
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_enableEvents(void );
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_disableEvents(void );
/* #line 33 */
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_clearPendingInterrupt(void );
/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_fired(void );










static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_startAt(uint16_t t0, uint16_t dt);
/* #line 103 */
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Timer_overflow(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_fired(void );
/* #line 92 */
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_AlarmFrom_startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_AlarmFrom_size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_AlarmFrom_size_type dt);
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Counter_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Counter_get(void );
/* # 66 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_t0;
/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_dt;

enum /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1___nesc_unnamed4299 {

  TransformAlarmC_1_MAX_DELAY_LOG2 = 8 * sizeof(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_from_size_type ) - 1 - 5, 
  TransformAlarmC_1_MAX_DELAY = (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type )1 << /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_MAX_DELAY_LOG2
};

static inline /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_getNow(void );
/* #line 96 */
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_set_alarm(void );
/* #line 136 */
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type dt);
/* #line 151 */
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_AlarmFrom_fired(void );
/* #line 166 */
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Counter_overflow(void );
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Service_read(
/* # 26 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
uint8_t arg_0x1bc37c8, 
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
uint32_t usPeriod);










static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_bufferDone(
/* # 24 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
uint8_t arg_0x1bc7bb0, 
/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
error_t result, 
/* #line 86 */
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_val_t * buf, 



uint16_t count);
/* #line 102 */
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_readDone(
/* # 24 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
uint8_t arg_0x1bc7bb0, 
/* # 102 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
error_t result, uint32_t usActualPeriod);
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Resource_release(
/* # 27 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
uint8_t arg_0x1bbf278);



uint32_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_period[1U];
/* #line 48 */
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Service_bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_val_t *buf, uint16_t count);




static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Service_readDone(uint8_t client, error_t result, uint32_t actualPeriod);





static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Resource_granted(uint8_t client);







static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Resource_default_release(uint8_t client);
/* #line 79 */
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_default_bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_val_t *buf, uint16_t count);



static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_default_readDone(uint8_t client, error_t result, uint32_t actualPeriod);
/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_1_ConfUp_adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_1_ConfUp_getConfiguration(void );
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc" */
static inline const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_1_ConfSub_getConfiguration(void );
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/sensors/Msp430InternalVoltageP.nc" */
const msp430adc12_channel_config_t Msp430InternalVoltageP_config = { 
.inch = SUPPLY_VOLTAGE_HALF_CHANNEL, 
.sref = REFERENCE_VREFplus_AVss, 
.ref2_5v = REFVOLT_LEVEL_1_5, 
.adc12ssel = SHT_SOURCE_ACLK, 
.adc12div = SHT_CLOCK_DIV_1, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };


static inline const msp430adc12_channel_config_t *Msp430InternalVoltageP_AdcConfigure_getConfiguration(void );
/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_2_ConfUp_adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_2_ConfUp_getConfiguration(void );
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc" */
static inline const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_2_ConfSub_getConfiguration(void );
/* # 212 "/Users/doina/tinyos-2.x/tos/chips/msp430/msp430hardware.h" */
static inline  void __nesc_enable_interrupt(void )
{
_R2 |= 0x0008;}

/* # 185 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Timer_overflow(void )
{
}

/* #line 185 */
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Timer_overflow(void )
{
}

/* #line 185 */
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Timer_overflow(void )
{
}

/* # 540 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static inline void Msp430Adc12ImplP_TimerA_overflow(void )
/* #line 540 */
{
}

/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_overflow(void ){
/* #line 37 */
  Msp430Adc12ImplP_TimerA_overflow();
/* #line 37 */
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Timer_overflow();
/* #line 37 */
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Timer_overflow();
/* #line 37 */
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Timer_overflow();
/* #line 37 */
}
/* #line 37 */
/* # 126 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Overflow_fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Event_default_fired(uint8_t n)
{
}

/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Event_fired(uint8_t arg_0x15734f0){
/* #line 28 */
  switch (arg_0x15734f0) {
/* #line 28 */
    case 0:
/* #line 28 */
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Event_fired();
/* #line 28 */
      break;
/* #line 28 */
    case 1:
/* #line 28 */
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Event_fired();
/* #line 28 */
      break;
/* #line 28 */
    case 2:
/* #line 28 */
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Event_fired();
/* #line 28 */
      break;
/* #line 28 */
    case 5:
/* #line 28 */
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Overflow_fired();
/* #line 28 */
      break;
/* #line 28 */
    default:
/* #line 28 */
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Event_default_fired(arg_0x15734f0);
/* #line 28 */
      break;
/* #line 28 */
    }
/* #line 28 */
}
/* #line 28 */
/* # 115 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_VectorTimerX0_fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Event_fired(0);
}

/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
inline static void Msp430TimerCommonP_VectorTimerA0_fired(void ){
/* #line 28 */
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_VectorTimerX0_fired();
/* #line 28 */
}
/* #line 28 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_int2CC(uint16_t x)
/* #line 47 */
{
/* #line 47 */
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0___nesc_unnamed4300 {
/* #line 47 */
    uint16_t f;
/* #line 47 */
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_cc_t t;
  } 
/* #line 47 */
  c = { .f = x };

/* #line 47 */
  return c.t;
}

/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Control_getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_int2CC(* (volatile uint16_t * )354U);
}

/* #line 177 */
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_default_captured(uint16_t n)
{
}

/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_captured(uint16_t time){
/* #line 75 */
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_default_captured(time);
/* #line 75 */
}
/* #line 75 */
/* # 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

/* # 541 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static inline void Msp430Adc12ImplP_CompareA0_fired(void )
/* #line 541 */
{
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Compare_fired(void ){
/* #line 34 */
  Msp430Adc12ImplP_CompareA0_fired();
/* #line 34 */
}
/* #line 34 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_int2CC(uint16_t x)
/* #line 47 */
{
/* #line 47 */
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1___nesc_unnamed4301 {
/* #line 47 */
    uint16_t f;
/* #line 47 */
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t t;
  } 
/* #line 47 */
  c = { .f = x };

/* #line 47 */
  return c.t;
}

/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Control_getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_int2CC(* (volatile uint16_t * )356U);
}

/* #line 177 */
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_default_captured(uint16_t n)
{
}

/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_captured(uint16_t time){
/* #line 75 */
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_default_captured(time);
/* #line 75 */
}
/* #line 75 */
/* # 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

/* # 542 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static inline void Msp430Adc12ImplP_CompareA1_fired(void )
/* #line 542 */
{
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Compare_fired(void ){
/* #line 34 */
  Msp430Adc12ImplP_CompareA1_fired();
/* #line 34 */
}
/* #line 34 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_int2CC(uint16_t x)
/* #line 47 */
{
/* #line 47 */
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2___nesc_unnamed4302 {
/* #line 47 */
    uint16_t f;
/* #line 47 */
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_cc_t t;
  } 
/* #line 47 */
  c = { .f = x };

/* #line 47 */
  return c.t;
}

/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Control_getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_int2CC(* (volatile uint16_t * )358U);
}

/* #line 177 */
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Capture_default_captured(uint16_t n)
{
}

/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Capture_captured(uint16_t time){
/* #line 75 */
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Capture_default_captured(time);
/* #line 75 */
}
/* #line 75 */
/* # 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Capture_getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

/* #line 181 */
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Compare_default_fired(void )
{
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Compare_fired(void ){
/* #line 34 */
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Compare_default_fired();
/* #line 34 */
}
/* #line 34 */
/* # 120 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_VectorTimerX1_fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

/* #line 123 */
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Event_fired(n >> 1);
}

/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
inline static void Msp430TimerCommonP_VectorTimerA1_fired(void ){
/* #line 28 */
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_VectorTimerX1_fired();
/* #line 28 */
}
/* #line 28 */
/* # 115 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_VectorTimerX0_fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_fired(0);
}

/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
inline static void Msp430TimerCommonP_VectorTimerB0_fired(void ){
/* #line 28 */
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_VectorTimerX0_fired();
/* #line 28 */
}
/* #line 28 */
/* # 185 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Timer_overflow(void )
{
}

/* #line 185 */
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Timer_overflow(void )
{
}

/* #line 185 */
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Timer_overflow(void )
{
}

/* #line 185 */
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Timer_overflow(void )
{
}

/* #line 185 */
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Timer_overflow(void )
{
}

/* #line 185 */
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Timer_overflow(void )
{
}

/* #line 185 */
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Timer_overflow(void )
{
}

/* # 103 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Timer_overflow(void )
{
}

/* #line 103 */
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Timer_overflow(void )
{
}

/* # 47 "/Users/doina/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc" */
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_0_Counter_overflow(void )
{
}

/* # 166 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Counter_overflow(void )
{
}

/* #line 166 */
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Counter_overflow(void )
{
}

/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
inline static void /*CounterMilli32C.Transform*/TransformCounterC_0_Counter_overflow(void ){
/* #line 71 */
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Counter_overflow();
/* #line 71 */
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Counter_overflow();
/* #line 71 */
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_0_Counter_overflow();
/* #line 71 */
}
/* #line 71 */
/* # 122 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformCounterC.nc" */
static inline void /*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC_0_m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC_0_m_upper & /*CounterMilli32C.Transform*/TransformCounterC_0_OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC_0_Counter_overflow();
      }
  }
}

/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_overflow(void ){
/* #line 71 */
  /*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_overflow();
/* #line 71 */
}
/* #line 71 */
/* # 53 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc" */
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Msp430Timer_overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_overflow();
}

/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_overflow(void ){
/* #line 37 */
  /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Msp430Timer_overflow();
/* #line 37 */
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Timer_overflow();
/* #line 37 */
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Timer_overflow();
/* #line 37 */
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Timer_overflow();
/* #line 37 */
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Timer_overflow();
/* #line 37 */
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Timer_overflow();
/* #line 37 */
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Timer_overflow();
/* #line 37 */
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Timer_overflow();
/* #line 37 */
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Timer_overflow();
/* #line 37 */
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Timer_overflow();
/* #line 37 */
}
/* #line 37 */
/* # 126 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Overflow_fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_overflow();
}

/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_fired_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_fired);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 70 "/Users/doina/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc" */
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_fired(void )
{
/* #line 71 */
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_fired_postTask();
}

/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_fired(void ){
/* #line 67 */
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_fired();
/* #line 67 */
}
/* #line 67 */
/* # 151 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_set_alarm();
      }
  }
}

/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_fired(void ){
/* #line 67 */
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_fired();
/* #line 67 */
}
/* #line 67 */
/* # 124 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_disableEvents(void ){
/* #line 47 */
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_disableEvents();
/* #line 47 */
}
/* #line 47 */
/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_fired();
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Compare_fired(void ){
/* #line 34 */
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_fired();
/* #line 34 */
}
/* #line 34 */
/* # 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Capture_getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

/* #line 177 */
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Capture_default_captured(uint16_t n)
{
}

/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Capture_captured(uint16_t time){
/* #line 75 */
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Capture_default_captured(time);
/* #line 75 */
}
/* #line 75 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_int2CC(uint16_t x)
/* #line 47 */
{
/* #line 47 */
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3___nesc_unnamed4303 {
/* #line 47 */
    uint16_t f;
/* #line 47 */
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_cc_t t;
  } 
/* #line 47 */
  c = { .f = x };

/* #line 47 */
  return c.t;
}

/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_int2CC(* (volatile uint16_t * )386U);
}

/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Event_fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Capture_captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Capture_getEvent());
    }
  else {
/* #line 174 */
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Compare_fired();
    }
}

/* # 86 "/Users/doina/tinyos-2.x/tos/system/SchedulerBasicP.nc" */
static inline bool SchedulerBasicP_isWaiting(uint8_t id)
{
  return SchedulerBasicP_m_next[id] != SchedulerBasicP_NO_TASK || SchedulerBasicP_m_tail == id;
}

static inline bool SchedulerBasicP_pushTask(uint8_t id)
{
  if (!SchedulerBasicP_isWaiting(id)) 
    {
      if (SchedulerBasicP_m_head == SchedulerBasicP_NO_TASK) 
        {
          SchedulerBasicP_m_head = id;
          SchedulerBasicP_m_tail = id;
        }
      else 
        {
          SchedulerBasicP_m_next[SchedulerBasicP_m_tail] = id;
          SchedulerBasicP_m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Msp430Timer_get(void ){
/* #line 34 */
  unsigned int result;
/* #line 34 */

/* #line 34 */
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_get();
/* #line 34 */

/* #line 34 */
  return result;
/* #line 34 */
}
/* #line 34 */
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc" */
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Msp430Timer_get();
}

/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
inline static /*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_size_type /*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_get(void ){
/* #line 53 */
  unsigned int result;
/* #line 53 */

/* #line 53 */
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_get();
/* #line 53 */

/* #line 53 */
  return result;
/* #line 53 */
}
/* #line 53 */
/* # 70 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

/* # 35 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Msp430Timer_isOverflowPending(void ){
/* #line 35 */
  unsigned char result;
/* #line 35 */

/* #line 35 */
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_isOverflowPending();
/* #line 35 */

/* #line 35 */
  return result;
/* #line 35 */
}
/* #line 35 */
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc" */
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Msp430Timer_isOverflowPending();
}

/* # 60 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
inline static bool /*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_isOverflowPending(void ){
/* #line 60 */
  unsigned char result;
/* #line 60 */

/* #line 60 */
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_isOverflowPending();
/* #line 60 */

/* #line 60 */
  return result;
/* #line 60 */
}
/* #line 60 */
/* # 119 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_enableEvents(void ){
/* #line 46 */
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_enableEvents();
/* #line 46 */
}
/* #line 46 */
/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_clearPendingInterrupt(void ){
/* #line 33 */
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_clearPendingInterrupt();
/* #line 33 */
}
/* #line 33 */
/* # 144 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Compare_setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_setEvent(uint16_t time){
/* #line 30 */
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Compare_setEvent(time);
/* #line 30 */
}
/* #line 30 */
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Timer_get(void ){
/* #line 34 */
  unsigned int result;
/* #line 34 */

/* #line 34 */
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_get();
/* #line 34 */

/* #line 34 */
  return result;
/* #line 34 */
}
/* #line 34 */
/* # 154 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Compare_setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Timer_get() + x;
}

/* # 32 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_setEventFromNow(uint16_t delta){
/* #line 32 */
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Compare_setEventFromNow(delta);
/* #line 32 */
}
/* #line 32 */
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Timer_get(void ){
/* #line 34 */
  unsigned int result;
/* #line 34 */

/* #line 34 */
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_get();
/* #line 34 */

/* #line 34 */
  return result;
/* #line 34 */
}
/* #line 34 */
/* # 70 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Timer_get();
    uint16_t elapsed = now - t0;

/* #line 76 */
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

/* #line 83 */
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_setEventFromNow(2);
          }
        else {
/* #line 86 */
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_setEvent(now + remaining);
          }
      }
/* #line 88 */
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_enableEvents();
  }
}

/* # 92 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_size_type dt){
/* #line 92 */
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_startAt(t0, dt);
/* #line 92 */
}
/* #line 92 */
/* # 181 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Compare_default_fired(void )
{
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Compare_fired(void ){
/* #line 34 */
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Compare_default_fired();
/* #line 34 */
}
/* #line 34 */
/* # 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

/* #line 177 */
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_default_captured(uint16_t n)
{
}

/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_captured(uint16_t time){
/* #line 75 */
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_default_captured(time);
/* #line 75 */
}
/* #line 75 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_int2CC(uint16_t x)
/* #line 47 */
{
/* #line 47 */
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4___nesc_unnamed4304 {
/* #line 47 */
    uint16_t f;
/* #line 47 */
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_cc_t t;
  } 
/* #line 47 */
  c = { .f = x };

/* #line 47 */
  return c.t;
}

/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_int2CC(* (volatile uint16_t * )388U);
}

/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Event_fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_getEvent());
    }
  else {
/* #line 174 */
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Compare_fired();
    }
}

/* # 324 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
static inline error_t AdcStreamP_SingleChannel_default_getData(uint8_t c)
{
  return FAIL;
}

/* # 189 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
inline static error_t AdcStreamP_SingleChannel_getData(uint8_t arg_0x1b5c010){
/* #line 189 */
  unsigned char result;
/* #line 189 */

/* #line 189 */
  switch (arg_0x1b5c010) {
/* #line 189 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC_0_RSCLIENT:
/* #line 189 */
      result = Msp430Adc12ImplP_SingleChannel_getData(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1_ID);
/* #line 189 */
      break;
/* #line 189 */
    default:
/* #line 189 */
      result = AdcStreamP_SingleChannel_default_getData(arg_0x1b5c010);
/* #line 189 */
      break;
/* #line 189 */
    }
/* #line 189 */

/* #line 189 */
  return result;
/* #line 189 */
}
/* #line 189 */
/* # 92 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
static inline void AdcStreamP_sampleSingle(void )
/* #line 92 */
{
  AdcStreamP_SingleChannel_getData(AdcStreamP_client);
}

/* #line 173 */
static inline void AdcStreamP_Alarm_fired(void )
/* #line 173 */
{
  AdcStreamP_sampleSingle();
}

/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_fired(void ){
/* #line 67 */
  AdcStreamP_Alarm_fired();
/* #line 67 */
}
/* #line 67 */
/* # 151 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_AlarmFrom_fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_dt == 0) 
      {
        /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_fired();
      }
    else 
      {
        /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_set_alarm();
      }
  }
}

/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_fired(void ){
/* #line 67 */
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_AlarmFrom_fired();
/* #line 67 */
}
/* #line 67 */
/* # 124 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_disableEvents(void )
{
  * (volatile uint16_t * )390U &= ~0x0010;
}

/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_disableEvents(void ){
/* #line 47 */
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_disableEvents();
/* #line 47 */
}
/* #line 47 */
/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_fired(void )
{
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_disableEvents();
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_fired();
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_fired(void ){
/* #line 34 */
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_fired();
/* #line 34 */
}
/* #line 34 */
/* # 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Capture_getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

/* #line 177 */
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Capture_default_captured(uint16_t n)
{
}

/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Capture_captured(uint16_t time){
/* #line 75 */
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Capture_default_captured(time);
/* #line 75 */
}
/* #line 75 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_int2CC(uint16_t x)
/* #line 47 */
{
/* #line 47 */
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5___nesc_unnamed4305 {
/* #line 47 */
    uint16_t f;
/* #line 47 */
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t t;
  } 
/* #line 47 */
  c = { .f = x };

/* #line 47 */
  return c.t;
}

/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_int2CC(* (volatile uint16_t * )390U);
}

/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Event_fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Capture_captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Capture_getEvent());
    }
  else {
/* #line 174 */
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_fired();
    }
}

/* # 50 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP_47_IO_makeInput(void )
/* #line 50 */
{
  /* atomic removed: atomic calls only */
/* #line 50 */
  * (volatile uint8_t * )54U &= ~(0x01 << 7);
}

/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port67_makeInput(void ){
/* #line 64 */
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP_47_IO_makeInput();
/* #line 64 */
}
/* #line 64 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP_47_IO_selectModuleFunc(void )
/* #line 54 */
{
  /* atomic removed: atomic calls only */
/* #line 54 */
  * (volatile uint8_t * )55U |= 0x01 << 7;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port67_selectModuleFunc(void ){
/* #line 78 */
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP_47_IO_selectModuleFunc();
/* #line 78 */
}
/* #line 78 */
/* # 50 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP_46_IO_makeInput(void )
/* #line 50 */
{
  /* atomic removed: atomic calls only */
/* #line 50 */
  * (volatile uint8_t * )54U &= ~(0x01 << 6);
}

/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port66_makeInput(void ){
/* #line 64 */
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP_46_IO_makeInput();
/* #line 64 */
}
/* #line 64 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP_46_IO_selectModuleFunc(void )
/* #line 54 */
{
  /* atomic removed: atomic calls only */
/* #line 54 */
  * (volatile uint8_t * )55U |= 0x01 << 6;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port66_selectModuleFunc(void ){
/* #line 78 */
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP_46_IO_selectModuleFunc();
/* #line 78 */
}
/* #line 78 */
/* # 50 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP_45_IO_makeInput(void )
/* #line 50 */
{
  /* atomic removed: atomic calls only */
/* #line 50 */
  * (volatile uint8_t * )54U &= ~(0x01 << 5);
}

/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port65_makeInput(void ){
/* #line 64 */
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP_45_IO_makeInput();
/* #line 64 */
}
/* #line 64 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP_45_IO_selectModuleFunc(void )
/* #line 54 */
{
  /* atomic removed: atomic calls only */
/* #line 54 */
  * (volatile uint8_t * )55U |= 0x01 << 5;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port65_selectModuleFunc(void ){
/* #line 78 */
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP_45_IO_selectModuleFunc();
/* #line 78 */
}
/* #line 78 */
/* # 50 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP_44_IO_makeInput(void )
/* #line 50 */
{
  /* atomic removed: atomic calls only */
/* #line 50 */
  * (volatile uint8_t * )54U &= ~(0x01 << 4);
}

/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port64_makeInput(void ){
/* #line 64 */
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP_44_IO_makeInput();
/* #line 64 */
}
/* #line 64 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP_44_IO_selectModuleFunc(void )
/* #line 54 */
{
  /* atomic removed: atomic calls only */
/* #line 54 */
  * (volatile uint8_t * )55U |= 0x01 << 4;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port64_selectModuleFunc(void ){
/* #line 78 */
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP_44_IO_selectModuleFunc();
/* #line 78 */
}
/* #line 78 */
/* # 50 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP_43_IO_makeInput(void )
/* #line 50 */
{
  /* atomic removed: atomic calls only */
/* #line 50 */
  * (volatile uint8_t * )54U &= ~(0x01 << 3);
}

/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port63_makeInput(void ){
/* #line 64 */
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP_43_IO_makeInput();
/* #line 64 */
}
/* #line 64 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP_43_IO_selectModuleFunc(void )
/* #line 54 */
{
  /* atomic removed: atomic calls only */
/* #line 54 */
  * (volatile uint8_t * )55U |= 0x01 << 3;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port63_selectModuleFunc(void ){
/* #line 78 */
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP_43_IO_selectModuleFunc();
/* #line 78 */
}
/* #line 78 */
/* # 50 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP_42_IO_makeInput(void )
/* #line 50 */
{
  /* atomic removed: atomic calls only */
/* #line 50 */
  * (volatile uint8_t * )54U &= ~(0x01 << 2);
}

/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port62_makeInput(void ){
/* #line 64 */
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP_42_IO_makeInput();
/* #line 64 */
}
/* #line 64 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP_42_IO_selectModuleFunc(void )
/* #line 54 */
{
  /* atomic removed: atomic calls only */
/* #line 54 */
  * (volatile uint8_t * )55U |= 0x01 << 2;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port62_selectModuleFunc(void ){
/* #line 78 */
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP_42_IO_selectModuleFunc();
/* #line 78 */
}
/* #line 78 */
/* # 50 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP_41_IO_makeInput(void )
/* #line 50 */
{
  /* atomic removed: atomic calls only */
/* #line 50 */
  * (volatile uint8_t * )54U &= ~(0x01 << 1);
}

/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port61_makeInput(void ){
/* #line 64 */
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP_41_IO_makeInput();
/* #line 64 */
}
/* #line 64 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP_41_IO_selectModuleFunc(void )
/* #line 54 */
{
  /* atomic removed: atomic calls only */
/* #line 54 */
  * (volatile uint8_t * )55U |= 0x01 << 1;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port61_selectModuleFunc(void ){
/* #line 78 */
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP_41_IO_selectModuleFunc();
/* #line 78 */
}
/* #line 78 */
/* # 50 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP_40_IO_makeInput(void )
/* #line 50 */
{
  /* atomic removed: atomic calls only */
/* #line 50 */
  * (volatile uint8_t * )54U &= ~(0x01 << 0);
}

/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port60_makeInput(void ){
/* #line 64 */
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP_40_IO_makeInput();
/* #line 64 */
}
/* #line 64 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP_40_IO_selectModuleFunc(void )
/* #line 54 */
{
  /* atomic removed: atomic calls only */
/* #line 54 */
  * (volatile uint8_t * )55U |= 0x01 << 0;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port60_selectModuleFunc(void ){
/* #line 78 */
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP_40_IO_selectModuleFunc();
/* #line 78 */
}
/* #line 78 */
/* # 142 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static inline void Msp430Adc12ImplP_configureAdcPin(uint8_t inch)
{

  switch (inch) 
    {
      case 0: Msp430Adc12ImplP_Port60_selectModuleFunc();
/* #line 147 */
      Msp430Adc12ImplP_Port60_makeInput();
/* #line 147 */
      break;
      case 1: Msp430Adc12ImplP_Port61_selectModuleFunc();
/* #line 148 */
      Msp430Adc12ImplP_Port61_makeInput();
/* #line 148 */
      break;
      case 2: Msp430Adc12ImplP_Port62_selectModuleFunc();
/* #line 149 */
      Msp430Adc12ImplP_Port62_makeInput();
/* #line 149 */
      break;
      case 3: Msp430Adc12ImplP_Port63_selectModuleFunc();
/* #line 150 */
      Msp430Adc12ImplP_Port63_makeInput();
/* #line 150 */
      break;
      case 4: Msp430Adc12ImplP_Port64_selectModuleFunc();
/* #line 151 */
      Msp430Adc12ImplP_Port64_makeInput();
/* #line 151 */
      break;
      case 5: Msp430Adc12ImplP_Port65_selectModuleFunc();
/* #line 152 */
      Msp430Adc12ImplP_Port65_makeInput();
/* #line 152 */
      break;
      case 6: Msp430Adc12ImplP_Port66_selectModuleFunc();
/* #line 153 */
      Msp430Adc12ImplP_Port66_makeInput();
/* #line 153 */
      break;
      case 7: Msp430Adc12ImplP_Port67_selectModuleFunc();
/* #line 154 */
      Msp430Adc12ImplP_Port67_makeInput();
/* #line 154 */
      break;
    }
}

/* # 103 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc" */
static inline void HplAdc12P_HplAdc12_startConversion(void )
/* #line 103 */
{
  HplAdc12P_ADC12CTL0 |= 0x0010;
  HplAdc12P_ADC12CTL0 |= 0x0001 + 0x0002;
}

/* # 128 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
inline static void Msp430Adc12ImplP_HplAdc12_startConversion(void ){
/* #line 128 */
  HplAdc12P_HplAdc12_startConversion();
/* #line 128 */
}
/* #line 128 */
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static void Msp430Adc12ImplP_TimerA_setMode(int mode){
/* #line 39 */
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_setMode(mode);
/* #line 39 */
}
/* #line 39 */
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t x)
/* #line 46 */
{
/* #line 46 */
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1___nesc_unnamed4306 {
/* #line 46 */
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t f;
/* #line 46 */
    uint16_t t;
  } 
/* #line 46 */
  c = { .f = x };

/* #line 46 */
  return c.t;
}

/* #line 89 */
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Control_setControl(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t x)
{
  * (volatile uint16_t * )356U = /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_CC2int(x);
}

/* # 35 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void Msp430Adc12ImplP_ControlA1_setControl(msp430_compare_control_t control){
/* #line 35 */
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Control_setControl(control);
/* #line 35 */
}
/* #line 35 */
/* # 121 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static inline void Msp430Adc12ImplP_startTimerA(void )
{

  msp430_compare_control_t ccSetSHI = { 
  .ccifg = 0, .cov = 0, .out = 1, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };
  msp430_compare_control_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };
  msp430_compare_control_t ccRSOutmod = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 7, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };

  Msp430Adc12ImplP_ControlA1_setControl(ccResetSHI);
  Msp430Adc12ImplP_ControlA1_setControl(ccSetSHI);

  Msp430Adc12ImplP_ControlA1_setControl(ccRSOutmod);
  Msp430Adc12ImplP_TimerA_setMode(MSP430TIMER_UP_MODE);
}

/* # 119 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_enableEvents(void )
{
  * (volatile uint16_t * )390U |= 0x0010;
}

/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_enableEvents(void ){
/* #line 46 */
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_enableEvents();
/* #line 46 */
}
/* #line 46 */
/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_clearPendingInterrupt(void )
{
  * (volatile uint16_t * )390U &= ~0x0001;
}

/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_clearPendingInterrupt(void ){
/* #line 33 */
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_clearPendingInterrupt();
/* #line 33 */
}
/* #line 33 */
/* # 144 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_setEvent(uint16_t x)
{
  * (volatile uint16_t * )406U = x;
}

/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_setEvent(uint16_t time){
/* #line 30 */
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_setEvent(time);
/* #line 30 */
}
/* #line 30 */
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Timer_get(void ){
/* #line 34 */
  unsigned int result;
/* #line 34 */

/* #line 34 */
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_get();
/* #line 34 */

/* #line 34 */
  return result;
/* #line 34 */
}
/* #line 34 */
/* # 154 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )406U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Timer_get() + x;
}

/* # 32 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_setEventFromNow(uint16_t delta){
/* #line 32 */
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_setEventFromNow(delta);
/* #line 32 */
}
/* #line 32 */
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Timer_get(void ){
/* #line 34 */
  unsigned int result;
/* #line 34 */

/* #line 34 */
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_get();
/* #line 34 */

/* #line 34 */
  return result;
/* #line 34 */
}
/* #line 34 */
/* # 70 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Timer_get();
    uint16_t elapsed = now - t0;

/* #line 76 */
    if (elapsed >= dt) 
      {
        /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

/* #line 83 */
        if (remaining <= 2) {
          /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_setEventFromNow(2);
          }
        else {
/* #line 86 */
          /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_setEvent(now + remaining);
          }
      }
/* #line 88 */
    /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_clearPendingInterrupt();
    /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_enableEvents();
  }
}

/* # 92 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_AlarmFrom_startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_AlarmFrom_size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_AlarmFrom_size_type dt){
/* #line 92 */
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_startAt(t0, dt);
/* #line 92 */
}
/* #line 92 */
/* # 181 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Compare_default_fired(void )
{
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Compare_fired(void ){
/* #line 34 */
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Compare_default_fired();
/* #line 34 */
}
/* #line 34 */
/* # 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Capture_getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

/* #line 177 */
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Capture_default_captured(uint16_t n)
{
}

/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Capture_captured(uint16_t time){
/* #line 75 */
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Capture_default_captured(time);
/* #line 75 */
}
/* #line 75 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_int2CC(uint16_t x)
/* #line 47 */
{
/* #line 47 */
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6___nesc_unnamed4307 {
/* #line 47 */
    uint16_t f;
/* #line 47 */
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_cc_t t;
  } 
/* #line 47 */
  c = { .f = x };

/* #line 47 */
  return c.t;
}

/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Control_getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_int2CC(* (volatile uint16_t * )392U);
}

/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Event_fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Control_getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Capture_captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Capture_getEvent());
    }
  else {
/* #line 174 */
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Compare_fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Compare_default_fired(void )
{
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Compare_fired(void ){
/* #line 34 */
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Compare_default_fired();
/* #line 34 */
}
/* #line 34 */
/* # 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Capture_getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

/* #line 177 */
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Capture_default_captured(uint16_t n)
{
}

/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Capture_captured(uint16_t time){
/* #line 75 */
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Capture_default_captured(time);
/* #line 75 */
}
/* #line 75 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_int2CC(uint16_t x)
/* #line 47 */
{
/* #line 47 */
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7___nesc_unnamed4308 {
/* #line 47 */
    uint16_t f;
/* #line 47 */
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_cc_t t;
  } 
/* #line 47 */
  c = { .f = x };

/* #line 47 */
  return c.t;
}

/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Control_getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_int2CC(* (volatile uint16_t * )394U);
}

/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Event_fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Control_getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Capture_captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Capture_getEvent());
    }
  else {
/* #line 174 */
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Compare_fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Compare_default_fired(void )
{
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Compare_fired(void ){
/* #line 34 */
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Compare_default_fired();
/* #line 34 */
}
/* #line 34 */
/* # 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Capture_getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

/* #line 177 */
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Capture_default_captured(uint16_t n)
{
}

/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Capture_captured(uint16_t time){
/* #line 75 */
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Capture_default_captured(time);
/* #line 75 */
}
/* #line 75 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_int2CC(uint16_t x)
/* #line 47 */
{
/* #line 47 */
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8___nesc_unnamed4309 {
/* #line 47 */
    uint16_t f;
/* #line 47 */
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_cc_t t;
  } 
/* #line 47 */
  c = { .f = x };

/* #line 47 */
  return c.t;
}

/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Control_getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_int2CC(* (volatile uint16_t * )396U);
}

/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Event_fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Control_getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Capture_captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Capture_getEvent());
    }
  else {
/* #line 174 */
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Compare_fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Compare_default_fired(void )
{
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Compare_fired(void ){
/* #line 34 */
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Compare_default_fired();
/* #line 34 */
}
/* #line 34 */
/* # 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Capture_getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

/* #line 177 */
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Capture_default_captured(uint16_t n)
{
}

/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Capture_captured(uint16_t time){
/* #line 75 */
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Capture_default_captured(time);
/* #line 75 */
}
/* #line 75 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_int2CC(uint16_t x)
/* #line 47 */
{
/* #line 47 */
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9___nesc_unnamed4310 {
/* #line 47 */
    uint16_t f;
/* #line 47 */
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_cc_t t;
  } 
/* #line 47 */
  c = { .f = x };

/* #line 47 */
  return c.t;
}

/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Control_getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_int2CC(* (volatile uint16_t * )398U);
}

/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Event_fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Control_getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Capture_captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Capture_getEvent());
    }
  else {
/* #line 174 */
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Compare_fired();
    }
}

/* # 120 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_VectorTimerX1_fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

/* #line 123 */
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_fired(n >> 1);
}

/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
inline static void Msp430TimerCommonP_VectorTimerB1_fired(void ){
/* #line 28 */
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_VectorTimerX1_fired();
/* #line 28 */
}
/* #line 28 */
/* # 113 "/Users/doina/tinyos-2.x/tos/system/SchedulerBasicP.nc" */
static inline void SchedulerBasicP_Scheduler_init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP_m_next, SchedulerBasicP_NO_TASK, sizeof SchedulerBasicP_m_next);
    SchedulerBasicP_m_head = SchedulerBasicP_NO_TASK;
    SchedulerBasicP_m_tail = SchedulerBasicP_NO_TASK;
  }
}

/* # 46 "/Users/doina/tinyos-2.x/tos/interfaces/Scheduler.nc" */
inline static void RealMainP_Scheduler_init(void ){
/* #line 46 */
  SchedulerBasicP_Scheduler_init();
/* #line 46 */
}
/* #line 46 */
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_HplGeneralIO_set(void ){
/* #line 34 */
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_set();
/* #line 34 */
}
/* #line 34 */
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_set(void )
/* #line 37 */
{
/* #line 37 */
  /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_HplGeneralIO_set();
}

/* # 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led2_set(void ){
/* #line 29 */
  /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_set();
/* #line 29 */
}
/* #line 29 */
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_HplGeneralIO_set(void ){
/* #line 34 */
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_set();
/* #line 34 */
}
/* #line 34 */
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_set(void )
/* #line 37 */
{
/* #line 37 */
  /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_HplGeneralIO_set();
}

/* # 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led1_set(void ){
/* #line 29 */
  /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_set();
/* #line 29 */
}
/* #line 29 */
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_HplGeneralIO_set(void ){
/* #line 34 */
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_set();
/* #line 34 */
}
/* #line 34 */
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_set(void )
/* #line 37 */
{
/* #line 37 */
  /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_HplGeneralIO_set();
}

/* # 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led0_set(void ){
/* #line 29 */
  /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_set();
/* #line 29 */
}
/* #line 29 */
/* # 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_makeOutput(void )
/* #line 52 */
{
  /* atomic removed: atomic calls only */
/* #line 52 */
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

/* # 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_HplGeneralIO_makeOutput(void ){
/* #line 71 */
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_makeOutput();
/* #line 71 */
}
/* #line 71 */
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_makeOutput(void )
/* #line 43 */
{
/* #line 43 */
  /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_HplGeneralIO_makeOutput();
}

/* # 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led2_makeOutput(void ){
/* #line 35 */
  /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_makeOutput();
/* #line 35 */
}
/* #line 35 */
/* # 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_makeOutput(void )
/* #line 52 */
{
  /* atomic removed: atomic calls only */
/* #line 52 */
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

/* # 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_HplGeneralIO_makeOutput(void ){
/* #line 71 */
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_makeOutput();
/* #line 71 */
}
/* #line 71 */
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_makeOutput(void )
/* #line 43 */
{
/* #line 43 */
  /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_HplGeneralIO_makeOutput();
}

/* # 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led1_makeOutput(void ){
/* #line 35 */
  /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_makeOutput();
/* #line 35 */
}
/* #line 35 */
/* # 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_makeOutput(void )
/* #line 52 */
{
  /* atomic removed: atomic calls only */
/* #line 52 */
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

/* # 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_HplGeneralIO_makeOutput(void ){
/* #line 71 */
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_makeOutput();
/* #line 71 */
}
/* #line 71 */
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_makeOutput(void )
/* #line 43 */
{
/* #line 43 */
  /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_HplGeneralIO_makeOutput();
}

/* # 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led0_makeOutput(void ){
/* #line 35 */
  /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_makeOutput();
/* #line 35 */
}
/* #line 35 */
/* # 45 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc" */
static inline error_t LedsP_Init_init(void )
/* #line 45 */
{
  /* atomic removed: atomic calls only */
/* #line 46 */
  {
    ;
    LedsP_Led0_makeOutput();
    LedsP_Led1_makeOutput();
    LedsP_Led2_makeOutput();
    LedsP_Led0_set();
    LedsP_Led1_set();
    LedsP_Led2_set();
  }
  return SUCCESS;
}

/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
inline static error_t PlatformP_LedsInit_init(void ){
/* #line 51 */
  unsigned char result;
/* #line 51 */

/* #line 51 */
  result = LedsP_Init_init();
/* #line 51 */

/* #line 51 */
  return result;
/* #line 51 */
}
/* #line 51 */
/* # 33 "/Users/doina/tinyos-2.x/tos/platforms/telosb/hardware.h" */
static inline  void TOSH_SET_SIMO0_PIN()
/* #line 33 */
{
/* #line 33 */
  static volatile uint8_t r __asm ("0x0019");

/* #line 33 */
  r |= 1 << 1;
}

/* #line 34 */
static inline  void TOSH_SET_UCLK0_PIN()
/* #line 34 */
{
/* #line 34 */
  static volatile uint8_t r __asm ("0x0019");

/* #line 34 */
  r |= 1 << 3;
}

/* #line 85 */
static inline  void TOSH_SET_FLASH_CS_PIN()
/* #line 85 */
{
/* #line 85 */
  static volatile uint8_t r __asm ("0x001D");

/* #line 85 */
  r |= 1 << 4;
}

/* #line 34 */
static inline  void TOSH_CLR_UCLK0_PIN()
/* #line 34 */
{
/* #line 34 */
  static volatile uint8_t r __asm ("0x0019");

/* #line 34 */
  r &= ~(1 << 3);
}

/* #line 85 */
static inline  void TOSH_CLR_FLASH_CS_PIN()
/* #line 85 */
{
/* #line 85 */
  static volatile uint8_t r __asm ("0x001D");

/* #line 85 */
  r &= ~(1 << 4);
}

/* # 11 "/Users/doina/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc" */
static __inline void MotePlatformC_TOSH_wait(void )
/* #line 11 */
{
;;}

/* # 86 "/Users/doina/tinyos-2.x/tos/platforms/telosb/hardware.h" */
static inline  void TOSH_SET_FLASH_HOLD_PIN()
/* #line 86 */
{
/* #line 86 */
  static volatile uint8_t r __asm ("0x001D");

/* #line 86 */
  r |= 1 << 7;
}

/* #line 85 */
static inline  void TOSH_MAKE_FLASH_CS_OUTPUT()
/* #line 85 */
{
/* #line 85 */
  static volatile uint8_t r __asm ("0x001E");

/* #line 85 */
  r |= 1 << 4;
}

/* #line 86 */
static inline  void TOSH_MAKE_FLASH_HOLD_OUTPUT()
/* #line 86 */
{
/* #line 86 */
  static volatile uint8_t r __asm ("0x001E");

/* #line 86 */
  r |= 1 << 7;
}

/* #line 34 */
static inline  void TOSH_MAKE_UCLK0_OUTPUT()
/* #line 34 */
{
/* #line 34 */
  static volatile uint8_t r __asm ("0x001A");

/* #line 34 */
  r |= 1 << 3;
}

/* #line 33 */
static inline  void TOSH_MAKE_SIMO0_OUTPUT()
/* #line 33 */
{
/* #line 33 */
  static volatile uint8_t r __asm ("0x001A");

/* #line 33 */
  r |= 1 << 1;
}

/* # 27 "/Users/doina/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc" */
static inline void MotePlatformC_TOSH_FLASH_M25P_DP(void )
/* #line 27 */
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC_TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC_TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC_TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC_TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC_TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC_TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC_TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC_TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC_TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

/* #line 6 */
static __inline void MotePlatformC_uwait(uint16_t u)
/* #line 6 */
{
  uint16_t t0 = TA0R;

/* #line 8 */
  while (TA0R - t0 <= u) ;
}

/* #line 56 */
static inline error_t MotePlatformC_Init_init(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1OUT = 0x00;
    P1DIR = 0xe0;

    P2OUT = 0x30;
    P2DIR = 0x7b;

    P3OUT = 0x00;
    P3DIR = 0xf1;

    P4OUT = 0xdd;
    P4DIR = 0xfd;

    P5OUT = 0xff;
    P5DIR = 0xff;

    P6OUT = 0x00;
    P6DIR = 0xff;

    P1IE = 0;
    P2IE = 0;






/*    MotePlatformC_uwait(1024 * 10); */

    MotePlatformC_TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
inline static error_t PlatformP_MoteInit_init(void ){
/* #line 51 */
  unsigned char result;
/* #line 51 */

/* #line 51 */
  result = MotePlatformC_Init_init();
/* #line 51 */

/* #line 51 */
  return result;
/* #line 51 */
}
/* #line 51 */
/* # 148 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc" */
static inline void Msp430ClockP_startTimerB(void )
{

  Msp430ClockP_TBCTL = 0x0020 | (Msp430ClockP_TBCTL & ~(0x0020 | 0x0010));
}

/* #line 136 */
static inline void Msp430ClockP_startTimerA(void )
{

  Msp430ClockP_TA0CTL = 0x0020 | (Msp430ClockP_TA0CTL & ~(0x0020 | 0x0010));
}

/* #line 100 */
static inline void Msp430ClockP_Msp430ClockInit_defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP_TBCTL = 0x0100 | 0x0002;
}

/* #line 130 */
static inline void Msp430ClockP_Msp430ClockInit_default_initTimerB(void )
{
  Msp430ClockP_Msp430ClockInit_defaultInitTimerB();
}

/* # 32 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc" */
inline static void Msp430ClockP_Msp430ClockInit_initTimerB(void ){
/* #line 32 */
  Msp430ClockP_Msp430ClockInit_default_initTimerB();
/* #line 32 */
}
/* #line 32 */
/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc" */
static inline void Msp430ClockP_Msp430ClockInit_defaultInitTimerA(void )
{
  TA0R = 0;









  Msp430ClockP_TA0CTL = 0x0200 | 0x0002;
}

/* #line 125 */
static inline void Msp430ClockP_Msp430ClockInit_default_initTimerA(void )
{
  Msp430ClockP_Msp430ClockInit_defaultInitTimerA();
}

/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc" */
inline static void Msp430ClockP_Msp430ClockInit_initTimerA(void ){
/* #line 31 */
  Msp430ClockP_Msp430ClockInit_default_initTimerA();
/* #line 31 */
}
/* #line 31 */
/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc" */
static inline void Msp430ClockP_Msp430ClockInit_defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP_IE1 &= ~(1 << 1);
}

/* #line 120 */
static inline void Msp430ClockP_Msp430ClockInit_default_initClocks(void )
{
  Msp430ClockP_Msp430ClockInit_defaultInitClocks();
}

/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc" */
inline static void Msp430ClockP_Msp430ClockInit_initClocks(void ){
/* #line 30 */
  Msp430ClockP_Msp430ClockInit_default_initClocks();
/* #line 30 */
}
/* #line 30 */
/* # 166 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc" */
static inline uint16_t Msp430ClockP_test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP_set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + Msp430ClockP_ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TA0R;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP_busyCalibrateDco(void )
{

  int calib;
  int step;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP_test_calib_busywait_delta(calib | step) <= Msp430ClockP_TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP_set_dco_calib(calib);
}

/* #line 52 */
static inline void Msp430ClockP_Msp430ClockInit_defaultSetupDcoCalibrate(void )
{



  Msp430ClockP_TA0CTL = 0x0200 | 0x0020;
  Msp430ClockP_TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

/* #line 115 */
static inline void Msp430ClockP_Msp430ClockInit_default_setupDcoCalibrate(void )
{
  Msp430ClockP_Msp430ClockInit_defaultSetupDcoCalibrate();
}

/* # 29 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc" */
inline static void Msp430ClockP_Msp430ClockInit_setupDcoCalibrate(void ){
/* #line 29 */
  Msp430ClockP_Msp430ClockInit_default_setupDcoCalibrate();
/* #line 29 */
}
/* #line 29 */
/* # 214 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc" */
static inline error_t Msp430ClockP_Init_init(void )
{

  Msp430ClockP_TA0CTL = 0x0004;
  Msp430ClockP_TA0IV = 0;
  Msp430ClockP_TBCTL = 0x0004;
  Msp430ClockP_TBIV = 0;
  /* atomic removed: atomic calls only */

  {
/*    Msp430ClockP_Msp430ClockInit_setupDcoCalibrate(); */
/*    Msp430ClockP_busyCalibrateDco(); */
    Msp430ClockP_Msp430ClockInit_initClocks();
    Msp430ClockP_Msp430ClockInit_initTimerA();
    Msp430ClockP_Msp430ClockInit_initTimerB();
    Msp430ClockP_startTimerA();
    Msp430ClockP_startTimerB();
  }

  return SUCCESS;
}

/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
inline static error_t PlatformP_MoteClockInit_init(void ){
/* #line 51 */
  unsigned char result;
/* #line 51 */

/* #line 51 */
  result = Msp430ClockP_Init_init();
/* #line 51 */

/* #line 51 */
  return result;
/* #line 51 */
}
/* #line 51 */
/* # 10 "/Users/doina/tinyos-2.x/tos/platforms/telosa/PlatformP.nc" */
static inline error_t PlatformP_Init_init(void )
/* #line 10 */
{
  PlatformP_MoteClockInit_init();
  PlatformP_MoteInit_init();
  PlatformP_LedsInit_init();
  return SUCCESS;
}

/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
inline static error_t RealMainP_PlatformInit_init(void ){
/* #line 51 */
  unsigned char result;
/* #line 51 */

/* #line 51 */
  result = PlatformP_Init_init();
/* #line 51 */

/* #line 51 */
  return result;
/* #line 51 */
}
/* #line 51 */
/* # 33 "/Users/doina/tinyos-2.x/tos/platforms/telosb/hardware.h" */
static inline  void TOSH_CLR_SIMO0_PIN()
/* #line 33 */
{
/* #line 33 */
  static volatile uint8_t r __asm ("0x0019");

/* #line 33 */
  r &= ~(1 << 1);
}

/* # 54 "/Users/doina/tinyos-2.x/tos/interfaces/Scheduler.nc" */
inline static bool RealMainP_Scheduler_runNextTask(void ){
/* #line 54 */
  unsigned char result;
/* #line 54 */

/* #line 54 */
  result = SchedulerBasicP_Scheduler_runNextTask();
/* #line 54 */

/* #line 54 */
  return result;
/* #line 54 */
}
/* #line 54 */
/* # 79 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_default_bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_val_t *buf, uint16_t count)
{
}

/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
inline static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_bufferDone(uint8_t arg_0x1bc7bb0, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_val_t * buf, uint16_t count){
/* #line 89 */
    /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_default_bufferDone(arg_0x1bc7bb0, result, buf, count);
/* #line 89 */
}
/* #line 89 */
/* # 48 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Service_bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_val_t *buf, uint16_t count)
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_bufferDone(client, result, buf, count);
}

/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
inline static void AdcStreamP_ReadStream_bufferDone(uint8_t arg_0x1b5f340, error_t result, AdcStreamP_ReadStream_val_t * buf, uint16_t count){
/* #line 89 */
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Service_bufferDone(arg_0x1b5f340, result, buf, count);
/* #line 89 */
}
/* #line 89 */
/* # 156 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
static inline void AdcStreamP_bufferDone_runTask(void )
/* #line 156 */
{
  uint16_t *b;
/* #line 157 */
  uint16_t c;

/* #line 158 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      b = AdcStreamP_lastBuffer;
      c = AdcStreamP_lastCount;
      AdcStreamP_lastBuffer = (void *)0;
    }
/* #line 163 */
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP_ReadStream_bufferDone(AdcStreamP_client, SUCCESS, b, c);
}

/* # 83 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_default_readDone(uint8_t client, error_t result, uint32_t actualPeriod)
{
}

/* # 102 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
inline static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_readDone(uint8_t arg_0x1bc7bb0, error_t result, uint32_t usActualPeriod){
/* #line 102 */
    /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_default_readDone(arg_0x1bc7bb0, result, usActualPeriod);
/* #line 102 */
}
/* #line 102 */
/* # 67 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Resource_default_release(uint8_t client)
/* #line 67 */
{
/* #line 67 */
  return FAIL;
}

/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Resource_release(uint8_t arg_0x1bbf278){
/* #line 110 */
  unsigned char result;
/* #line 110 */

/* #line 110 */
  switch (arg_0x1bbf278) {
/* #line 110 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC_0_RSCLIENT:
/* #line 110 */
      result = Msp430RefVoltArbiterImplP_ClientResource_release(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1_ID);
/* #line 110 */
      break;
/* #line 110 */
    default:
/* #line 110 */
      result = /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Resource_default_release(arg_0x1bbf278);
/* #line 110 */
      break;
/* #line 110 */
    }
/* #line 110 */

/* #line 110 */
  return result;
/* #line 110 */
}
/* #line 110 */
/* # 53 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Service_readDone(uint8_t client, error_t result, uint32_t actualPeriod)
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Resource_release(client);
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_ReadStream_readDone(client, result, actualPeriod);
}

/* # 102 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
inline static void AdcStreamP_ReadStream_readDone(uint8_t arg_0x1b5f340, error_t result, uint32_t usActualPeriod){
/* #line 102 */
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Service_readDone(arg_0x1b5f340, result, usActualPeriod);
/* #line 102 */
}
/* #line 102 */
/* # 135 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
static inline void AdcStreamP_readStreamFail_runTask(void )
/* #line 135 */
{

  struct AdcStreamP_list_entry_t *entry;
  uint8_t c = AdcStreamP_client;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 140 */
    entry = AdcStreamP_bufferQueue[c];
/* #line 140 */
    __nesc_atomic_end(__nesc_atomic); }
  for (; entry; entry = entry->next) {
      uint16_t tmp_count  = entry->count;

/* #line 143 */
      AdcStreamP_ReadStream_bufferDone(c, FAIL, (uint16_t * )entry, entry->count);
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      AdcStreamP_bufferQueue[c] = (void *)0;
      AdcStreamP_bufferQueueEnd[c] = &AdcStreamP_bufferQueue[c];
    }
/* #line 150 */
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP_client = AdcStreamP_NSTREAM;
  AdcStreamP_ReadStream_readDone(c, FAIL, 0);
}

/* # 170 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
static inline error_t Msp430RefVoltArbiterImplP_AdcResource_default_release(uint8_t client)
/* #line 170 */
{
/* #line 170 */
  return FAIL;
}

/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t Msp430RefVoltArbiterImplP_AdcResource_release(uint8_t arg_0x1af9ac8){
/* #line 110 */
  unsigned char result;
/* #line 110 */

/* #line 110 */
  switch (arg_0x1af9ac8) {
/* #line 110 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0_ID:
/* #line 110 */
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_release(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0_ID);
/* #line 110 */
      break;
/* #line 110 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1_ID:
/* #line 110 */
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_release(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1_ID);
/* #line 110 */
      break;
/* #line 110 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_2_ID:
/* #line 110 */
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_release(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_2_ID);
/* #line 110 */
      break;
/* #line 110 */
    default:
/* #line 110 */
      result = Msp430RefVoltArbiterImplP_AdcResource_default_release(arg_0x1af9ac8);
/* #line 110 */
      break;
/* #line 110 */
    }
/* #line 110 */

/* #line 110 */
  return result;
/* #line 110 */
}
/* #line 110 */
/* # 56 "/Users/doina/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc" */
static inline bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_isEmpty(void )
/* #line 56 */
{
  int i;

  /* atomic removed: atomic calls only */
/* #line 58 */
  {
    for (i = 0; i < sizeof /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_resQ; i++) 
      if (/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_resQ[i] > 0) {
          unsigned char __nesc_temp = 
/* #line 60 */
          FALSE;

/* #line 60 */
          return __nesc_temp;
        }
/* #line 61 */
    {
      unsigned char __nesc_temp = 
/* #line 61 */
      TRUE;

/* #line 61 */
      return __nesc_temp;
    }
  }
}

/* # 43 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceQueue.nc" */
inline static bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Queue_isEmpty(void ){
/* #line 43 */
  unsigned char result;
/* #line 43 */

/* #line 43 */
  result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_isEmpty();
/* #line 43 */

/* #line 43 */
  return result;
/* #line 43 */
}
/* #line 43 */
/* # 47 "/Users/doina/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc" */
static inline void /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_clearEntry(uint8_t id)
/* #line 47 */
{
  /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_resQ[id / 8] &= ~(1 << id % 8);
}

/* #line 69 */
static inline resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_dequeue(void )
/* #line 69 */
{
  int i;

  /* atomic removed: atomic calls only */
/* #line 71 */
  {
    for (i = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_last + 1; ; i++) {
        if (i == 3U) {
          i = 0;
          }
/* #line 75 */
        if (/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_isEnqueued(i)) {
            /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_clearEntry(i);
            /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_last = i;
            {
              unsigned char __nesc_temp = 
/* #line 78 */
              i;

/* #line 78 */
              return __nesc_temp;
            }
          }
/* #line 80 */
        if (i == /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_last) {
          break;
          }
      }
/* #line 83 */
    {
      unsigned char __nesc_temp = 
/* #line 83 */
      /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_NO_ENTRY;

/* #line 83 */
      return __nesc_temp;
    }
  }
}

/* # 60 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceQueue.nc" */
inline static resource_client_id_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Queue_dequeue(void ){
/* #line 60 */
  unsigned char result;
/* #line 60 */

/* #line 60 */
  result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_dequeue();
/* #line 60 */

/* #line 60 */
  return result;
/* #line 60 */
}
/* #line 60 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_grantedTask_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_grantedTask);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 173 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceConfigure_default_unconfigure(uint8_t id)
/* #line 173 */
{
}

/* # 55 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceConfigure.nc" */
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceConfigure_unconfigure(uint8_t arg_0x1a7f030){
/* #line 55 */
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceConfigure_default_unconfigure(arg_0x1a7f030);
/* #line 55 */
}
/* #line 55 */
/* # 119 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
static inline void AdcStreamP_readStreamDone_runTask(void )
/* #line 119 */
{
  uint8_t c = AdcStreamP_client;
  uint32_t actualPeriod = AdcStreamP_period;

/* #line 122 */
  if (AdcStreamP_periodModified) {
    actualPeriod = AdcStreamP_period - AdcStreamP_period % 1000;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      AdcStreamP_bufferQueue[c] = (void *)0;
      AdcStreamP_bufferQueueEnd[c] = &AdcStreamP_bufferQueue[c];
    }
/* #line 129 */
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP_client = AdcStreamP_NSTREAM;
  AdcStreamP_ReadStream_readDone(c, SUCCESS, actualPeriod);
}

/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t Msp430RefVoltArbiterImplP_switchOff_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(Msp430RefVoltArbiterImplP_switchOff);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Counter_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Counter_get(void ){
/* #line 53 */
  unsigned long result;
/* #line 53 */

/* #line 53 */
  result = /*CounterMilli32C.Transform*/TransformCounterC_0_Counter_get();
/* #line 53 */

/* #line 53 */
  return result;
/* #line 53 */
}
/* #line 53 */
/* # 75 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Counter_get();
}

/* # 98 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_getNow(void ){
/* #line 98 */
  unsigned long result;
/* #line 98 */

/* #line 98 */
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_getNow();
/* #line 98 */

/* #line 98 */
  return result;
/* #line 98 */
}
/* #line 98 */
/* # 85 "/Users/doina/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc" */
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_getNow(void )
{
/* #line 86 */
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_getNow();
}

/* # 125 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_getNow(void ){
/* #line 125 */
  unsigned long result;
/* #line 125 */

/* #line 125 */
  result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_getNow();
/* #line 125 */

/* #line 125 */
  return result;
/* #line 125 */
}
/* #line 125 */
/* # 148 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_getNow(), dt, TRUE);
}

/* # 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static void Msp430RefVoltGeneratorP_SwitchOffTimer_startOneShot(uint32_t dt){
/* #line 62 */
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startOneShot(2U, dt);
/* #line 62 */
}
/* #line 62 */
/* # 151 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
static inline void Msp430RefVoltArbiterImplP_RefVolt_2_5V_stopDone(error_t error)
{
}

/* # 117 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
inline static void Msp430RefVoltGeneratorP_RefVolt_2_5V_stopDone(error_t error){
/* #line 117 */
  Msp430RefVoltArbiterImplP_RefVolt_2_5V_stopDone(error);
/* #line 117 */
}
/* #line 117 */
/* # 147 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
static inline void Msp430RefVoltArbiterImplP_RefVolt_1_5V_stopDone(error_t error)
{
}

/* # 117 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
inline static void Msp430RefVoltGeneratorP_RefVolt_1_5V_stopDone(error_t error){
/* #line 117 */
  Msp430RefVoltArbiterImplP_RefVolt_1_5V_stopDone(error);
/* #line 117 */
}
/* #line 117 */
/* # 153 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_m_timers[num].isrunning = FALSE;
}

/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static void Msp430RefVoltGeneratorP_SwitchOnTimer_stop(void ){
/* #line 67 */
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_stop(1U);
/* #line 67 */
}
/* #line 67 */
/* # 127 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc" */
static inline error_t Msp430RefVoltGeneratorP_RefVolt_1_5V_stop(void )
{
  switch (Msp430RefVoltGeneratorP_state) 
    {
      case Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING: 

        case Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING: 
          if (Msp430RefVoltGeneratorP_switchOff() == SUCCESS) {
              Msp430RefVoltGeneratorP_SwitchOnTimer_stop();
              Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_GENERATOR_OFF;
              if (Msp430RefVoltGeneratorP_state == Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING) {
                Msp430RefVoltGeneratorP_RefVolt_1_5V_stopDone(SUCCESS);
                }
              else {
/* #line 140 */
                Msp430RefVoltGeneratorP_RefVolt_2_5V_stopDone(SUCCESS);
                }
/* #line 141 */
              return SUCCESS;
            }
          else {
/* #line 143 */
            return FAIL;
            }
/* #line 144 */
      case Msp430RefVoltGeneratorP_REFERENCE_1_5V_STABLE: 

        case Msp430RefVoltGeneratorP_REFERENCE_2_5V_STABLE: 
          Msp430RefVoltGeneratorP_SwitchOffTimer_startOneShot(20);
      return SUCCESS;
      case Msp430RefVoltGeneratorP_GENERATOR_OFF: 

        default: 

          return FAIL;
    }
}

/* # 109 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
inline static error_t Msp430RefVoltArbiterImplP_RefVolt_1_5V_stop(void ){
/* #line 109 */
  unsigned char result;
/* #line 109 */

/* #line 109 */
  result = Msp430RefVoltGeneratorP_RefVolt_1_5V_stop();
/* #line 109 */

/* #line 109 */
  return result;
/* #line 109 */
}
/* #line 109 */
/* # 136 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
static inline void Msp430RefVoltArbiterImplP_switchOff_runTask(void )
{

  if (Msp430RefVoltArbiterImplP_syncOwner != Msp430RefVoltArbiterImplP_NO_OWNER) {
      if (Msp430RefVoltArbiterImplP_RefVolt_1_5V_stop() == SUCCESS) {
          Msp430RefVoltArbiterImplP_syncOwner = Msp430RefVoltArbiterImplP_NO_OWNER;
        }
      else {
/* #line 143 */
        Msp430RefVoltArbiterImplP_switchOff_postTask();
        }
    }
}

/* # 121 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc" */
static inline bool HplAdc12P_HplAdc12_isBusy(void )
/* #line 121 */
{
/* #line 121 */
  return HplAdc12P_ADC12CTL1 & 0x0001;
}

/* # 118 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
inline static bool Msp430RefVoltGeneratorP_HplAdc12_isBusy(void ){
/* #line 118 */
  unsigned char result;
/* #line 118 */

/* #line 118 */
  result = HplAdc12P_HplAdc12_isBusy();
/* #line 118 */

/* #line 118 */
  return result;
/* #line 118 */
}
/* #line 118 */
/* # 69 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc" */
static inline adc12ctl0_t HplAdc12P_HplAdc12_getCtl0(void )
/* #line 69 */
{
  return * (adc12ctl0_t *)&HplAdc12P_ADC12CTL0;
}

/* # 63 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
inline static adc12ctl0_t Msp430RefVoltGeneratorP_HplAdc12_getCtl0(void ){
/* #line 63 */
  struct __nesc_unnamed4254 result;
/* #line 63 */

/* #line 63 */
  result = HplAdc12P_HplAdc12_getCtl0();
/* #line 63 */

/* #line 63 */
  return result;
/* #line 63 */
}
/* #line 63 */
/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc" */
static inline void HplAdc12P_HplAdc12_setCtl0(adc12ctl0_t control0)
/* #line 61 */
{
  HplAdc12P_ADC12CTL0 = * (uint16_t *)&control0;
}

/* # 51 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
inline static void Msp430RefVoltGeneratorP_HplAdc12_setCtl0(adc12ctl0_t control0){
/* #line 51 */
  HplAdc12P_HplAdc12_setCtl0(control0);
/* #line 51 */
}
/* #line 51 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 165 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_default_granted(uint8_t id)
/* #line 165 */
{
}

/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_granted(uint8_t arg_0x1a82930){
/* #line 92 */
  switch (arg_0x1a82930) {
/* #line 92 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0_ID:
/* #line 92 */
      Msp430RefVoltArbiterImplP_AdcResource_granted(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0_ID);
/* #line 92 */
      break;
/* #line 92 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1_ID:
/* #line 92 */
      Msp430RefVoltArbiterImplP_AdcResource_granted(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1_ID);
/* #line 92 */
      break;
/* #line 92 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_2_ID:
/* #line 92 */
      Msp430RefVoltArbiterImplP_AdcResource_granted(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_2_ID);
/* #line 92 */
      break;
/* #line 92 */
    default:
/* #line 92 */
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_default_granted(arg_0x1a82930);
/* #line 92 */
      break;
/* #line 92 */
    }
/* #line 92 */
}
/* #line 92 */
/* # 171 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceConfigure_default_configure(uint8_t id)
/* #line 171 */
{
}

/* # 49 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceConfigure.nc" */
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceConfigure_configure(uint8_t arg_0x1a7f030){
/* #line 49 */
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceConfigure_default_configure(arg_0x1a7f030);
/* #line 49 */
}
/* #line 49 */
/* # 155 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_grantedTask_runTask(void )
/* #line 155 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 156 */
    {
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_reqResId;
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_RES_BUSY;
    }
/* #line 159 */
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceConfigure_configure(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_resId);
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_granted(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_resId);
}

/* # 50 "/Users/doina/tinyos-2.x/tos/chips/msp430/sensors/Msp430InternalVoltageP.nc" */
static inline const msp430adc12_channel_config_t *Msp430InternalVoltageP_AdcConfigure_getConfiguration(void )
{
  return &Msp430InternalVoltageP_config;
}

/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
inline static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_0_ConfUp_adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_0_ConfUp_getConfiguration(void ){
/* #line 58 */
  struct __nesc_unnamed4267 const *result;
/* #line 58 */

/* #line 58 */
  result = Msp430InternalVoltageP_AdcConfigure_getConfiguration();
/* #line 58 */

/* #line 58 */
  return result;
/* #line 58 */
}
/* #line 58 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc" */
static inline const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_0_ConfSub_getConfiguration(void )
{
  return /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_0_ConfUp_getConfiguration();
}

/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
inline static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_1_ConfUp_adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_1_ConfUp_getConfiguration(void ){
/* #line 58 */
  struct __nesc_unnamed4267 const *result;
/* #line 58 */

/* #line 58 */
  result = Msp430InternalVoltageP_AdcConfigure_getConfiguration();
/* #line 58 */

/* #line 58 */
  return result;
/* #line 58 */
}
/* #line 58 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc" */
static inline const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_1_ConfSub_getConfiguration(void )
{
  return /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_1_ConfUp_getConfiguration();
}

/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
inline static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_2_ConfUp_adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_2_ConfUp_getConfiguration(void ){
/* #line 58 */
  struct __nesc_unnamed4267 const *result;
/* #line 58 */

/* #line 58 */
  result = Msp430InternalVoltageP_AdcConfigure_getConfiguration();
/* #line 58 */

/* #line 58 */
  return result;
/* #line 58 */
}
/* #line 58 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc" */
static inline const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_2_ConfSub_getConfiguration(void )
{
  return /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_2_ConfUp_getConfiguration();
}

/* # 172 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
static inline const msp430adc12_channel_config_t *
Msp430RefVoltArbiterImplP_Config_default_getConfiguration(uint8_t client)
{
  return &Msp430RefVoltArbiterImplP_defaultConfig;
}

/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
inline static Msp430RefVoltArbiterImplP_Config_adc_config_t Msp430RefVoltArbiterImplP_Config_getConfiguration(uint8_t arg_0x1af6638){
/* #line 58 */
  struct __nesc_unnamed4267 const *result;
/* #line 58 */

/* #line 58 */
  switch (arg_0x1af6638) {
/* #line 58 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0_ID:
/* #line 58 */
      result = /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_0_ConfSub_getConfiguration();
/* #line 58 */
      break;
/* #line 58 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1_ID:
/* #line 58 */
      result = /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_1_ConfSub_getConfiguration();
/* #line 58 */
      break;
/* #line 58 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_2_ID:
/* #line 58 */
      result = /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC_2_ConfSub_getConfiguration();
/* #line 58 */
      break;
/* #line 58 */
    default:
/* #line 58 */
      result = Msp430RefVoltArbiterImplP_Config_default_getConfiguration(arg_0x1af6638);
/* #line 58 */
      break;
/* #line 58 */
    }
/* #line 58 */

/* #line 58 */
  return result;
/* #line 58 */
}
/* #line 58 */
/* # 167 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceRequested_default_requested(uint8_t id)
/* #line 167 */
{
}

/* # 43 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceRequested.nc" */
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceRequested_requested(uint8_t arg_0x1a813f0){
/* #line 43 */
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceRequested_default_requested(arg_0x1a813f0);
/* #line 43 */
}
/* #line 43 */
/* # 87 "/Users/doina/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc" */
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_enqueue(resource_client_id_t id)
/* #line 87 */
{
  /* atomic removed: atomic calls only */
/* #line 88 */
  {
    if (!/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_isEnqueued(id)) {
        /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_resQ[id / 8] |= 1 << id % 8;
        {
          unsigned char __nesc_temp = 
/* #line 91 */
          SUCCESS;

/* #line 91 */
          return __nesc_temp;
        }
      }
/* #line 93 */
    {
      unsigned char __nesc_temp = 
/* #line 93 */
      EBUSY;

/* #line 93 */
      return __nesc_temp;
    }
  }
}

/* # 69 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceQueue.nc" */
inline static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Queue_enqueue(resource_client_id_t id){
/* #line 69 */
  unsigned char result;
/* #line 69 */

/* #line 69 */
  result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_enqueue(id);
/* #line 69 */

/* #line 69 */
  return result;
/* #line 69 */
}
/* #line 69 */
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/ReadStream.nc" */
inline static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Service_read(uint8_t arg_0x1bc37c8, uint32_t usPeriod){
/* #line 78 */
  unsigned char result;
/* #line 78 */

/* #line 78 */
  result = AdcStreamP_ReadStream_read(arg_0x1bc37c8, usPeriod);
/* #line 78 */

/* #line 78 */
  return result;
/* #line 78 */
}
/* #line 78 */
/* # 59 "/Users/doina/tinyos-2.x/tos/system/ArbitratedReadStreamC.nc" */
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Resource_granted(uint8_t client)
/* #line 59 */
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Service_read(client, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_period[client]);
}

/* # 160 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
static inline void Msp430RefVoltArbiterImplP_ClientResource_default_granted(uint8_t client)
/* #line 160 */
{
}

/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static void Msp430RefVoltArbiterImplP_ClientResource_granted(uint8_t arg_0x1af9010){
/* #line 92 */
  switch (arg_0x1af9010) {
/* #line 92 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0_ID:
/* #line 92 */
      AdcP_ResourceRead_granted(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC_0_CLIENT);
/* #line 92 */
      break;
/* #line 92 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1_ID:
/* #line 92 */
      /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC_0_Resource_granted(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC_0_RSCLIENT);
/* #line 92 */
      break;
/* #line 92 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_2_ID:
/* #line 92 */
      AdcP_SubResourceReadNow_granted(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC_0_CLIENT);
/* #line 92 */
      break;
/* #line 92 */
    default:
/* #line 92 */
      Msp430RefVoltArbiterImplP_ClientResource_default_granted(arg_0x1af9010);
/* #line 92 */
      break;
/* #line 92 */
    }
/* #line 92 */
}
/* #line 92 */
/* # 98 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
static inline void Msp430RefVoltArbiterImplP_RefVolt_1_5V_startDone(error_t error)
{
  if (Msp430RefVoltArbiterImplP_syncOwner != Msp430RefVoltArbiterImplP_NO_OWNER) {


      Msp430RefVoltArbiterImplP_ClientResource_granted(Msp430RefVoltArbiterImplP_syncOwner);
    }
}

/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
inline static void Msp430RefVoltGeneratorP_RefVolt_1_5V_startDone(error_t error){
/* #line 92 */
  Msp430RefVoltArbiterImplP_RefVolt_1_5V_startDone(error);
/* #line 92 */
}
/* #line 92 */
/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static void Msp430RefVoltGeneratorP_SwitchOffTimer_stop(void ){
/* #line 67 */
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_stop(2U);
/* #line 67 */
}
/* #line 67 */
/* # 94 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc" */
static inline error_t Msp430RefVoltGeneratorP_RefVolt_1_5V_start(void )
{
  switch (Msp430RefVoltGeneratorP_state) 
    {
      case Msp430RefVoltGeneratorP_REFERENCE_1_5V_STABLE: 
        Msp430RefVoltGeneratorP_SwitchOffTimer_stop();
      Msp430RefVoltGeneratorP_RefVolt_1_5V_startDone(SUCCESS);
      return SUCCESS;
      case Msp430RefVoltGeneratorP_GENERATOR_OFF: 
        if (Msp430RefVoltGeneratorP_switchOn(Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING) == SUCCESS) {
            Msp430RefVoltGeneratorP_SwitchOnTimer_startOneShot(17);
            Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING;
            return SUCCESS;
          }
        else {
/* #line 108 */
          return FAIL;
          }
/* #line 109 */
      case Msp430RefVoltGeneratorP_REFERENCE_2_5V_STABLE: 
        if (Msp430RefVoltGeneratorP_switchOn(Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING) == SUCCESS) {
            Msp430RefVoltGeneratorP_SwitchOffTimer_stop();
            Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_REFERENCE_1_5V_STABLE;
            Msp430RefVoltGeneratorP_RefVolt_1_5V_startDone(SUCCESS);
            return SUCCESS;
          }
        else {
/* #line 116 */
          return FAIL;
          }
/* #line 117 */
      case Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING: 

        case Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING: 

          default: 

            return FAIL;
    }
}

/* # 83 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
inline static error_t Msp430RefVoltArbiterImplP_RefVolt_1_5V_start(void ){
/* #line 83 */
  unsigned char result;
/* #line 83 */

/* #line 83 */
  result = Msp430RefVoltGeneratorP_RefVolt_1_5V_start();
/* #line 83 */

/* #line 83 */
  return result;
/* #line 83 */
}
/* #line 83 */
/* # 186 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
static inline const msp430adc12_channel_config_t *
AdcP_Config_default_getConfiguration(uint8_t client)
{
  return &AdcP_defaultConfig;
}

/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
inline static AdcP_Config_adc_config_t AdcP_Config_getConfiguration(uint8_t arg_0x18fc648){
/* #line 58 */
  struct __nesc_unnamed4267 const *result;
/* #line 58 */

/* #line 58 */
  switch (arg_0x18fc648) {
/* #line 58 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC_0_CLIENT:
/* #line 58 */
      result = Msp430InternalVoltageP_AdcConfigure_getConfiguration();
/* #line 58 */
      break;
/* #line 58 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC_0_CLIENT:
/* #line 58 */
      result = Msp430InternalVoltageP_AdcConfigure_getConfiguration();
/* #line 58 */
      break;
/* #line 58 */
    default:
/* #line 58 */
      result = AdcP_Config_default_getConfiguration(arg_0x18fc648);
/* #line 58 */
      break;
/* #line 58 */
    }
/* #line 58 */

/* #line 58 */
  return result;
/* #line 58 */
}
/* #line 58 */
/* # 191 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
static inline error_t AdcP_SingleChannel_default_configureSingle(uint8_t client, 
const msp430adc12_channel_config_t *config)
/* #line 192 */
{
/* #line 192 */
  return FAIL;
}

/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
inline static error_t AdcP_SingleChannel_configureSingle(uint8_t arg_0x191ee60, const msp430adc12_channel_config_t * config){
/* #line 84 */
  unsigned char result;
/* #line 84 */

/* #line 84 */
  switch (arg_0x191ee60) {
/* #line 84 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC_0_CLIENT:
/* #line 84 */
      result = Msp430Adc12ImplP_SingleChannel_configureSingle(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0_ID, config);
/* #line 84 */
      break;
/* #line 84 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC_0_CLIENT:
/* #line 84 */
      result = Msp430Adc12ImplP_SingleChannel_configureSingle(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_2_ID, config);
/* #line 84 */
      break;
/* #line 84 */
    default:
/* #line 84 */
      result = AdcP_SingleChannel_default_configureSingle(arg_0x191ee60, config);
/* #line 84 */
      break;
/* #line 84 */
    }
/* #line 84 */

/* #line 84 */
  return result;
/* #line 84 */
}
/* #line 84 */
/* # 177 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
static inline void AdcP_ResourceReadNow_default_granted(uint8_t nowClient)
/* #line 177 */
{
}

/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static void AdcP_ResourceReadNow_granted(uint8_t arg_0x18ff330){
/* #line 92 */
    AdcP_ResourceReadNow_default_granted(arg_0x18ff330);
/* #line 92 */
}
/* #line 92 */
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
inline static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Counter_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Counter_get(void ){
/* #line 53 */
  unsigned long result;
/* #line 53 */

/* #line 53 */
  result = /*CounterMilli32C.Transform*/TransformCounterC_0_Counter_get();
/* #line 53 */

/* #line 53 */
  return result;
/* #line 53 */
}
/* #line 53 */
/* # 75 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_getNow(void )
{
  return /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Counter_get();
}

/* # 98 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static AdcStreamP_Alarm_size_type AdcStreamP_Alarm_getNow(void ){
/* #line 98 */
  unsigned long result;
/* #line 98 */

/* #line 98 */
  result = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_getNow();
/* #line 98 */

/* #line 98 */
  return result;
/* #line 98 */
}
/* #line 98 */
/* # 328 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
static inline error_t AdcStreamP_SingleChannel_default_configureSingle(uint8_t c, 
const msp430adc12_channel_config_t *config)
/* #line 329 */
{
/* #line 329 */
  return FAIL;
}

/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
inline static error_t AdcStreamP_SingleChannel_configureSingle(uint8_t arg_0x1b5c010, const msp430adc12_channel_config_t * config){
/* #line 84 */
  unsigned char result;
/* #line 84 */

/* #line 84 */
  switch (arg_0x1b5c010) {
/* #line 84 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC_0_RSCLIENT:
/* #line 84 */
      result = Msp430Adc12ImplP_SingleChannel_configureSingle(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1_ID, config);
/* #line 84 */
      break;
/* #line 84 */
    default:
/* #line 84 */
      result = AdcStreamP_SingleChannel_default_configureSingle(arg_0x1b5c010, config);
/* #line 84 */
      break;
/* #line 84 */
    }
/* #line 84 */

/* #line 84 */
  return result;
/* #line 84 */
}
/* #line 84 */
/* # 314 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
static inline const msp430adc12_channel_config_t *AdcStreamP_AdcConfigure_default_getConfiguration(uint8_t c)
{
  return &AdcStreamP_defaultConfig;
}

/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/AdcConfigure.nc" */
inline static AdcStreamP_AdcConfigure_adc_config_t AdcStreamP_AdcConfigure_getConfiguration(uint8_t arg_0x1b5cde0){
/* #line 58 */
  struct __nesc_unnamed4267 const *result;
/* #line 58 */

/* #line 58 */
  switch (arg_0x1b5cde0) {
/* #line 58 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC_0_RSCLIENT:
/* #line 58 */
      result = Msp430InternalVoltageP_AdcConfigure_getConfiguration();
/* #line 58 */
      break;
/* #line 58 */
    default:
/* #line 58 */
      result = AdcStreamP_AdcConfigure_default_getConfiguration(arg_0x1b5cde0);
/* #line 58 */
      break;
/* #line 58 */
    }
/* #line 58 */

/* #line 58 */
  return result;
/* #line 58 */
}
/* #line 58 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t AdcStreamP_readStreamDone_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(AdcStreamP_readStreamDone);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 136 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type dt)
{
  /* atomic removed: atomic calls only */
  {
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_t0 = t0;
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_dt = dt;
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_set_alarm();
  }
}

/* # 92 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void AdcStreamP_Alarm_startAt(AdcStreamP_Alarm_size_type t0, AdcStreamP_Alarm_size_type dt){
/* #line 92 */
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Alarm_startAt(t0, dt);
/* #line 92 */
}
/* #line 92 */
/* # 168 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
static inline void AdcStreamP_nextAlarm(void )
/* #line 168 */
{
  AdcStreamP_Alarm_startAt(AdcStreamP_now, AdcStreamP_period);
  AdcStreamP_now += AdcStreamP_period;
}

/* # 144 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Compare_setEvent(uint16_t x)
{
  * (volatile uint16_t * )372U = x;
}

/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void Msp430Adc12ImplP_CompareA1_setEvent(uint16_t time){
/* #line 30 */
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Compare_setEvent(time);
/* #line 30 */
}
/* #line 30 */
/* # 144 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Compare_setEvent(uint16_t x)
{
  * (volatile uint16_t * )370U = x;
}

/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void Msp430Adc12ImplP_CompareA0_setEvent(uint16_t time){
/* #line 30 */
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Compare_setEvent(time);
/* #line 30 */
}
/* #line 30 */
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_cc_t x)
/* #line 46 */
{
/* #line 46 */
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0___nesc_unnamed4311 {
/* #line 46 */
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_cc_t f;
/* #line 46 */
    uint16_t t;
  } 
/* #line 46 */
  c = { .f = x };

/* #line 46 */
  return c.t;
}

/* #line 89 */
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Control_setControl(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_cc_t x)
{
  * (volatile uint16_t * )354U = /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_CC2int(x);
}

/* # 35 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void Msp430Adc12ImplP_ControlA0_setControl(msp430_compare_control_t control){
/* #line 35 */
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Control_setControl(control);
/* #line 35 */
}
/* #line 35 */
/* # 110 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_setInputDivider(uint16_t inputDivider)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(0x0040 | 0x0080)) | ((inputDivider << 6) & (0x0040 | 0x0080));
}

/* # 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static void Msp430Adc12ImplP_TimerA_setInputDivider(uint16_t inputDivider){
/* #line 45 */
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_setInputDivider(inputDivider);
/* #line 45 */
}
/* #line 45 */
/* # 105 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_setClockSource(uint16_t clockSource)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(256U | 512U)) | ((clockSource << 8) & (256U | 512U));
}

/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static void Msp430Adc12ImplP_TimerA_setClockSource(uint16_t clockSource){
/* #line 44 */
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_setClockSource(clockSource);
/* #line 44 */
}
/* #line 44 */
/* # 100 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_disableEvents(void )
{
  * (volatile uint16_t * )352U &= ~2U;
}

/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static void Msp430Adc12ImplP_TimerA_disableEvents(void ){
/* #line 43 */
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_disableEvents();
/* #line 43 */
}
/* #line 43 */
/* # 90 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_clear(void )
{
  * (volatile uint16_t * )352U |= 4U;
}

/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static void Msp430Adc12ImplP_TimerA_clear(void ){
/* #line 41 */
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_clear();
/* #line 41 */
}
/* #line 41 */
/* # 103 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static inline void Msp430Adc12ImplP_prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON)
{

  msp430_compare_control_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };

  Msp430Adc12ImplP_TimerA_setMode(MSP430TIMER_STOP_MODE);
  Msp430Adc12ImplP_TimerA_clear();
  Msp430Adc12ImplP_TimerA_disableEvents();
  Msp430Adc12ImplP_TimerA_setClockSource(csSAMPCON);
  Msp430Adc12ImplP_TimerA_setInputDivider(cdSAMPCON);
  Msp430Adc12ImplP_ControlA0_setControl(ccResetSHI);
  Msp430Adc12ImplP_CompareA0_setEvent(interval - 1);
  Msp430Adc12ImplP_CompareA1_setEvent((interval - 1) / 2);
}

/* # 95 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc" */
static inline void HplAdc12P_HplAdc12_setIEFlags(uint16_t mask)
/* #line 95 */
{
/* #line 95 */
  HplAdc12P_ADC12IE = mask;
}

/* # 95 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
inline static void Msp430Adc12ImplP_HplAdc12_setIEFlags(uint16_t mask){
/* #line 95 */
  HplAdc12P_HplAdc12_setIEFlags(mask);
/* #line 95 */
}
/* #line 95 */
/* # 77 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc" */
static inline void HplAdc12P_HplAdc12_setMCtl(uint8_t i, adc12memctl_t memControl)
/* #line 77 */
{
  uint8_t *memCtlPtr = (uint8_t *)(char *)0x0080;

/* #line 79 */
  memCtlPtr += i;
  *memCtlPtr = * (uint8_t *)&memControl;
}

/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
inline static void Msp430Adc12ImplP_HplAdc12_setMCtl(uint8_t idx, adc12memctl_t memControl){
/* #line 75 */
  HplAdc12P_HplAdc12_setMCtl(idx, memControl);
/* #line 75 */
}
/* #line 75 */
/* # 65 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc" */
static inline void HplAdc12P_HplAdc12_setCtl1(adc12ctl1_t control1)
/* #line 65 */
{
  HplAdc12P_ADC12CTL1 = * (uint16_t *)&control1;
}

/* # 57 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
inline static void Msp430Adc12ImplP_HplAdc12_setCtl1(adc12ctl1_t control1){
/* #line 57 */
  HplAdc12P_HplAdc12_setCtl1(control1);
/* #line 57 */
}
/* #line 57 */
/* #line 51 */
inline static void Msp430Adc12ImplP_HplAdc12_setCtl0(adc12ctl0_t control0){
/* #line 51 */
  HplAdc12P_HplAdc12_setCtl0(control0);
/* #line 51 */
}
/* #line 51 */
/* #line 63 */
inline static adc12ctl0_t Msp430Adc12ImplP_HplAdc12_getCtl0(void ){
/* #line 63 */
  struct __nesc_unnamed4254 result;
/* #line 63 */

/* #line 63 */
  result = HplAdc12P_HplAdc12_getCtl0();
/* #line 63 */

/* #line 63 */
  return result;
/* #line 63 */
}
/* #line 63 */
/* # 88 "/Users/doina/tinyos-2.x/tos/interfaces/ArbiterInfo.nc" */
inline static uint8_t Msp430Adc12ImplP_ADCArbiterInfo_userId(void ){
/* #line 88 */
  unsigned char result;
/* #line 88 */

/* #line 88 */
  result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ArbiterInfo_userId();
/* #line 88 */

/* #line 88 */
  return result;
/* #line 88 */
}
/* #line 88 */
/* # 271 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static inline error_t Msp430Adc12ImplP_SingleChannel_configureMultiple(uint8_t id, 
const msp430adc12_channel_config_t *config, 
uint16_t *buf, uint16_t length, uint16_t jiffies)
{
  error_t result = ERESERVE;





  if ((((!config || !buf) || !length) || jiffies == 1) || jiffies == 2) {
    return EINVAL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 284 */
    {
      if (Msp430Adc12ImplP_state & Msp430Adc12ImplP_ADC_BUSY) 
        {
          unsigned char __nesc_temp = 
/* #line 286 */
          EBUSY;

          {
/* #line 286 */
            __nesc_atomic_end(__nesc_atomic); 
/* #line 286 */
            return __nesc_temp;
          }
        }
/* #line 287 */
      if (Msp430Adc12ImplP_ADCArbiterInfo_userId() == id) {
          adc12ctl1_t ctl1 = { 
          .adc12busy = 0, 
          .conseq = length > 16 ? 3 : 1, 
          .adc12ssel = config->adc12ssel, 
          .adc12div = config->adc12div, 
          .issh = 0, 
          .shp = 1, 
          .shs = jiffies == 0 ? 0 : 1, 
          .cstartadd = 0 };

          adc12memctl_t memctl = { 
          .inch = config->inch, 
          .sref = config->sref, 
          .eos = 0 };

          uint16_t i;
/* #line 303 */
          uint16_t mask = 1;
          adc12ctl0_t ctl0 = Msp430Adc12ImplP_HplAdc12_getCtl0();

/* #line 305 */
          ctl0.msc = jiffies == 0 ? 1 : 0;
          ctl0.sht0 = config->sht;
          ctl0.sht1 = config->sht;

          Msp430Adc12ImplP_state = Msp430Adc12ImplP_MULTIPLE_DATA;
          Msp430Adc12ImplP_resultBufferStart = (void *)0;
          Msp430Adc12ImplP_resultBufferLength = length;
          Msp430Adc12ImplP_resultBufferStart = buf;
          Msp430Adc12ImplP_resultBufferIndex = 0;
          Msp430Adc12ImplP_HplAdc12_setCtl0(ctl0);
          Msp430Adc12ImplP_HplAdc12_setCtl1(ctl1);
          for (i = 0; i < length - 1 && i < 15; i++) 
            Msp430Adc12ImplP_HplAdc12_setMCtl(i, memctl);
          memctl.eos = 1;
          Msp430Adc12ImplP_HplAdc12_setMCtl(i, memctl);
          Msp430Adc12ImplP_HplAdc12_setIEFlags(mask << i);

          if (jiffies) {
              Msp430Adc12ImplP_state |= Msp430Adc12ImplP_USE_TIMERA;
              Msp430Adc12ImplP_prepareTimerA(jiffies, config->sampcon_ssel, config->sampcon_id);
            }
          result = SUCCESS;
        }
    }
/* #line 328 */
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

/* # 318 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
static inline error_t AdcStreamP_SingleChannel_default_configureMultiple(uint8_t c, 
const msp430adc12_channel_config_t *config, uint16_t b[], 
uint16_t numSamples, uint16_t jiffies)
{
  return FAIL;
}

/* # 138 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
inline static error_t AdcStreamP_SingleChannel_configureMultiple(uint8_t arg_0x1b5c010, const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies){
/* #line 138 */
  unsigned char result;
/* #line 138 */

/* #line 138 */
  switch (arg_0x1b5c010) {
/* #line 138 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC_0_RSCLIENT:
/* #line 138 */
      result = Msp430Adc12ImplP_SingleChannel_configureMultiple(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1_ID, config, buffer, numSamples, jiffies);
/* #line 138 */
      break;
/* #line 138 */
    default:
/* #line 138 */
      result = AdcStreamP_SingleChannel_default_configureMultiple(arg_0x1b5c010, config, buffer, numSamples, jiffies);
/* #line 138 */
      break;
/* #line 138 */
    }
/* #line 138 */

/* #line 138 */
  return result;
/* #line 138 */
}
/* #line 138 */
/* # 96 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
static inline error_t AdcStreamP_postBuffer(uint8_t c, uint16_t *buf, uint16_t n)
{
  if (n < sizeof(struct AdcStreamP_list_entry_t )) {
    return ESIZE;
    }
/* #line 100 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      struct AdcStreamP_list_entry_t * newEntry = (struct AdcStreamP_list_entry_t * )buf;

      if (!AdcStreamP_bufferQueueEnd[c]) 
        {
          unsigned char __nesc_temp = 
/* #line 105 */
          FAIL;

          {
/* #line 105 */
            __nesc_atomic_end(__nesc_atomic); 
/* #line 105 */
            return __nesc_temp;
          }
        }
/* #line 107 */
      newEntry->count = n;
      newEntry->next = (void *)0;
      *AdcStreamP_bufferQueueEnd[c] = newEntry;
      AdcStreamP_bufferQueueEnd[c] = & newEntry->next;
    }
/* #line 111 */
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t AdcStreamP_readStreamFail_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(AdcStreamP_readStreamFail);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 180 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
static inline error_t AdcP_SingleChannel_default_getData(uint8_t client)
{
  return EINVAL;
}

/* # 189 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
inline static error_t AdcP_SingleChannel_getData(uint8_t arg_0x191ee60){
/* #line 189 */
  unsigned char result;
/* #line 189 */

/* #line 189 */
  switch (arg_0x191ee60) {
/* #line 189 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC_0_CLIENT:
/* #line 189 */
      result = Msp430Adc12ImplP_SingleChannel_getData(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0_ID);
/* #line 189 */
      break;
/* #line 189 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC_0_CLIENT:
/* #line 189 */
      result = Msp430Adc12ImplP_SingleChannel_getData(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_2_ID);
/* #line 189 */
      break;
/* #line 189 */
    default:
/* #line 189 */
      result = AdcP_SingleChannel_default_getData(arg_0x191ee60);
/* #line 189 */
      break;
/* #line 189 */
    }
/* #line 189 */

/* #line 189 */
  return result;
/* #line 189 */
}
/* #line 189 */
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_clr(void )
/* #line 46 */
{
/* #line 46 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 46 */
    * (volatile uint8_t * )49U &= ~(0x01 << 6);
/* #line 46 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_HplGeneralIO_clr(void ){
/* #line 39 */
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_clr();
/* #line 39 */
}
/* #line 39 */
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_clr(void )
/* #line 38 */
{
/* #line 38 */
  /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_HplGeneralIO_clr();
}

/* # 30 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led2_clr(void ){
/* #line 30 */
  /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_clr();
/* #line 30 */
}
/* #line 30 */
/* # 93 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc" */
static inline void LedsP_Leds_led2On(void )
/* #line 93 */
{
  LedsP_Led2_clr();
  ;
/* #line 95 */
  ;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
inline static void SenseC_Leds_led2On(void ){
/* #line 78 */
  LedsP_Leds_led2On();
/* #line 78 */
}
/* #line 78 */
/* # 98 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc" */
static inline void LedsP_Leds_led2Off(void )
/* #line 98 */
{
  LedsP_Led2_set();
  ;
/* #line 100 */
  ;
}

/* # 83 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
inline static void SenseC_Leds_led2Off(void ){
/* #line 83 */
  LedsP_Leds_led2Off();
/* #line 83 */
}
/* #line 83 */
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_clr(void )
/* #line 46 */
{
/* #line 46 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 46 */
    * (volatile uint8_t * )49U &= ~(0x01 << 5);
/* #line 46 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_HplGeneralIO_clr(void ){
/* #line 39 */
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_clr();
/* #line 39 */
}
/* #line 39 */
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_clr(void )
/* #line 38 */
{
/* #line 38 */
  /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_HplGeneralIO_clr();
}

/* # 30 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led1_clr(void ){
/* #line 30 */
  /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_clr();
/* #line 30 */
}
/* #line 30 */
/* # 78 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc" */
static inline void LedsP_Leds_led1On(void )
/* #line 78 */
{
  LedsP_Led1_clr();
  ;
/* #line 80 */
  ;
}

/* # 61 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
inline static void SenseC_Leds_led1On(void ){
/* #line 61 */
  LedsP_Leds_led1On();
/* #line 61 */
}
/* #line 61 */
/* # 83 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc" */
static inline void LedsP_Leds_led1Off(void )
/* #line 83 */
{
  LedsP_Led1_set();
  ;
/* #line 85 */
  ;
}

/* # 66 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
inline static void SenseC_Leds_led1Off(void ){
/* #line 66 */
  LedsP_Leds_led1Off();
/* #line 66 */
}
/* #line 66 */
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_clr(void )
/* #line 46 */
{
/* #line 46 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 46 */
    * (volatile uint8_t * )49U &= ~(0x01 << 4);
/* #line 46 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_HplGeneralIO_clr(void ){
/* #line 39 */
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_clr();
/* #line 39 */
}
/* #line 39 */
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_clr(void )
/* #line 38 */
{
/* #line 38 */
  /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_HplGeneralIO_clr();
}

/* # 30 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led0_clr(void ){
/* #line 30 */
  /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_clr();
/* #line 30 */
}
/* #line 30 */
/* # 63 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc" */
static inline void LedsP_Leds_led0On(void )
/* #line 63 */
{
  LedsP_Led0_clr();
  ;
/* #line 65 */
  ;
}

/* # 45 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
inline static void SenseC_Leds_led0On(void ){
/* #line 45 */
  LedsP_Leds_led0On();
/* #line 45 */
}
/* #line 45 */
/* # 68 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc" */
static inline void LedsP_Leds_led0Off(void )
/* #line 68 */
{
  LedsP_Led0_set();
  ;
/* #line 70 */
  ;
}

/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
inline static void SenseC_Leds_led0Off(void ){
/* #line 50 */
  LedsP_Leds_led0Off();
/* #line 50 */
}
/* #line 50 */
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
inline static void Msp430RefVoltGeneratorP_RefVolt_2_5V_startDone(error_t error){
/* #line 92 */
  Msp430RefVoltArbiterImplP_RefVolt_2_5V_startDone(error);
/* #line 92 */
}
/* #line 92 */
/* # 157 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc" */
static inline error_t Msp430RefVoltGeneratorP_RefVolt_2_5V_start(void )
{
  switch (Msp430RefVoltGeneratorP_state) 
    {
      case Msp430RefVoltGeneratorP_REFERENCE_2_5V_STABLE: 
        Msp430RefVoltGeneratorP_SwitchOffTimer_stop();
      Msp430RefVoltGeneratorP_RefVolt_2_5V_startDone(SUCCESS);
      return SUCCESS;
      case Msp430RefVoltGeneratorP_GENERATOR_OFF: 
        if (Msp430RefVoltGeneratorP_switchOn(Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING) == SUCCESS) {
            Msp430RefVoltGeneratorP_SwitchOnTimer_startOneShot(17);
            Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING;
            return SUCCESS;
          }
        else {
/* #line 171 */
          return FAIL;
          }
/* #line 172 */
      case Msp430RefVoltGeneratorP_REFERENCE_1_5V_STABLE: 
        if (Msp430RefVoltGeneratorP_switchOn(Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING) == SUCCESS) {
            Msp430RefVoltGeneratorP_SwitchOffTimer_stop();
            Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_REFERENCE_2_5V_STABLE;
            Msp430RefVoltGeneratorP_RefVolt_2_5V_startDone(SUCCESS);
            return SUCCESS;
          }
        else {
/* #line 179 */
          return FAIL;
          }
/* #line 180 */
      case Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING: 

        case Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING: 

          default: 

            return FAIL;
    }
}

/* # 83 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
inline static error_t Msp430RefVoltArbiterImplP_RefVolt_2_5V_start(void ){
/* #line 83 */
  unsigned char result;
/* #line 83 */

/* #line 83 */
  result = Msp430RefVoltGeneratorP_RefVolt_2_5V_start();
/* #line 83 */

/* #line 83 */
  return result;
/* #line 83 */
}
/* #line 83 */
/* # 172 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
static inline void AdcP_Read_default_readDone(uint8_t client, error_t result, uint16_t val)
/* #line 172 */
{
}

/* # 63 "/Users/doina/tinyos-2.x/tos/interfaces/Read.nc" */
inline static void AdcP_Read_readDone(uint8_t arg_0x1903338, error_t result, AdcP_Read_val_t val){
/* #line 63 */
  switch (arg_0x1903338) {
/* #line 63 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC_0_CLIENT:
/* #line 63 */
      SenseC_Read_readDone(result, val);
/* #line 63 */
      break;
/* #line 63 */
    default:
/* #line 63 */
      AdcP_Read_default_readDone(arg_0x1903338, result, val);
/* #line 63 */
      break;
/* #line 63 */
    }
/* #line 63 */
}
/* #line 63 */
/* # 170 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
static inline error_t AdcP_ResourceRead_default_release(uint8_t client)
/* #line 170 */
{
/* #line 170 */
  return FAIL;
}

/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t AdcP_ResourceRead_release(uint8_t arg_0x18ffe18){
/* #line 110 */
  unsigned char result;
/* #line 110 */

/* #line 110 */
  switch (arg_0x18ffe18) {
/* #line 110 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC_0_CLIENT:
/* #line 110 */
      result = Msp430RefVoltArbiterImplP_ClientResource_release(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0_ID);
/* #line 110 */
      break;
/* #line 110 */
    default:
/* #line 110 */
      result = AdcP_ResourceRead_default_release(arg_0x18ffe18);
/* #line 110 */
      break;
/* #line 110 */
    }
/* #line 110 */

/* #line 110 */
  return result;
/* #line 110 */
}
/* #line 110 */
/* # 136 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
static inline void AdcP_readDone_runTask(void )
{
  AdcP_ResourceRead_release(AdcP_owner);
  AdcP_Read_readDone(AdcP_owner, SUCCESS, AdcP_value);
}

/* # 92 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_size_type dt){
/* #line 92 */
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_startAt(t0, dt);
/* #line 92 */
}
/* #line 92 */
/* # 47 "/Users/doina/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc" */
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_startAt(t0, dt);
}

/* #line 82 */
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_startOneShotAt(uint32_t t0, uint32_t dt)
{
/* #line 83 */
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_start(t0, dt, TRUE);
}

/* # 118 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_startOneShotAt(uint32_t t0, uint32_t dt){
/* #line 118 */
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_startOneShotAt(t0, dt);
/* #line 118 */
}
/* #line 118 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_disableEvents();
}

/* # 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_stop(void ){
/* #line 62 */
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Alarm_stop();
/* #line 62 */
}
/* #line 62 */
/* # 91 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_stop();
}

/* # 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_stop(void ){
/* #line 62 */
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_stop();
/* #line 62 */
}
/* #line 62 */
/* # 60 "/Users/doina/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc" */
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_stop(void )
{
/* #line 61 */
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_stop();
}

/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_stop(void ){
/* #line 67 */
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_stop();
/* #line 67 */
}
/* #line 67 */
/* # 89 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer_runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_fireTimers(now);
        }
      else {
/* #line 124 */
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_startOneShotAt(now, min_remaining);
        }
    }
}

/* # 161 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
static inline error_t Msp430RefVoltArbiterImplP_AdcResource_default_request(uint8_t client)
{
  return FAIL;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t Msp430RefVoltArbiterImplP_AdcResource_request(uint8_t arg_0x1af9ac8){
/* #line 78 */
  unsigned char result;
/* #line 78 */

/* #line 78 */
  switch (arg_0x1af9ac8) {
/* #line 78 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0_ID:
/* #line 78 */
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_request(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0_ID);
/* #line 78 */
      break;
/* #line 78 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1_ID:
/* #line 78 */
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_request(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1_ID);
/* #line 78 */
      break;
/* #line 78 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_2_ID:
/* #line 78 */
      result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_request(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_2_ID);
/* #line 78 */
      break;
/* #line 78 */
    default:
/* #line 78 */
      result = Msp430RefVoltArbiterImplP_AdcResource_default_request(arg_0x1af9ac8);
/* #line 78 */
      break;
/* #line 78 */
    }
/* #line 78 */

/* #line 78 */
  return result;
/* #line 78 */
}
/* #line 78 */
/* # 53 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
static inline error_t Msp430RefVoltArbiterImplP_ClientResource_request(uint8_t client)
{
  return Msp430RefVoltArbiterImplP_AdcResource_request(client);
}

/* # 168 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
static inline error_t AdcP_ResourceRead_default_request(uint8_t client)
/* #line 168 */
{
/* #line 168 */
  return FAIL;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t AdcP_ResourceRead_request(uint8_t arg_0x18ffe18){
/* #line 78 */
  unsigned char result;
/* #line 78 */

/* #line 78 */
  switch (arg_0x18ffe18) {
/* #line 78 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC_0_CLIENT:
/* #line 78 */
      result = Msp430RefVoltArbiterImplP_ClientResource_request(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0_ID);
/* #line 78 */
      break;
/* #line 78 */
    default:
/* #line 78 */
      result = AdcP_ResourceRead_default_request(arg_0x18ffe18);
/* #line 78 */
      break;
/* #line 78 */
    }
/* #line 78 */

/* #line 78 */
  return result;
/* #line 78 */
}
/* #line 78 */
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
static inline error_t AdcP_Read_read(uint8_t client)
{
  return AdcP_ResourceRead_request(client);
}

/* # 55 "/Users/doina/tinyos-2.x/tos/interfaces/Read.nc" */
inline static error_t SenseC_Read_read(void ){
/* #line 55 */
  unsigned char result;
/* #line 55 */

/* #line 55 */
  result = AdcP_Read_read(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC_0_CLIENT);
/* #line 55 */

/* #line 55 */
  return result;
/* #line 55 */
}
/* #line 55 */
/* # 65 "SenseC.nc" */
static inline void SenseC_Timer_fired(void )
{
  SenseC_Read_read();
}

/* # 220 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc" */
static inline void Msp430RefVoltGeneratorP_SwitchOnTimer_fired(void )
{
  switch (Msp430RefVoltGeneratorP_state) 
    {
      case Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING: 
        Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_REFERENCE_1_5V_STABLE;
      Msp430RefVoltGeneratorP_RefVolt_1_5V_startDone(SUCCESS);
      break;
      case Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING: 
        Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_REFERENCE_2_5V_STABLE;
      Msp430RefVoltGeneratorP_RefVolt_2_5V_startDone(SUCCESS);
      break;
      case Msp430RefVoltGeneratorP_REFERENCE_1_5V_STABLE: 

        case Msp430RefVoltGeneratorP_GENERATOR_OFF: 

          case Msp430RefVoltGeneratorP_REFERENCE_2_5V_STABLE: 

            default: 

              return;
    }
}

static inline void Msp430RefVoltGeneratorP_SwitchOffTimer_fired(void )
{
  switch (Msp430RefVoltGeneratorP_state) 
    {
      case Msp430RefVoltGeneratorP_REFERENCE_1_5V_STABLE: 
        if (Msp430RefVoltGeneratorP_switchOff() == SUCCESS) {
            Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_GENERATOR_OFF;
            Msp430RefVoltGeneratorP_RefVolt_1_5V_stopDone(SUCCESS);
          }
        else {
/* #line 253 */
          Msp430RefVoltGeneratorP_SwitchOffTimer_startOneShot(20);
          }
/* #line 254 */
      break;
      case Msp430RefVoltGeneratorP_REFERENCE_2_5V_STABLE: 
        if (Msp430RefVoltGeneratorP_switchOff() == SUCCESS) {
            Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_GENERATOR_OFF;
            Msp430RefVoltGeneratorP_RefVolt_2_5V_stopDone(SUCCESS);
          }
        else {
/* #line 260 */
          Msp430RefVoltGeneratorP_SwitchOffTimer_startOneShot(20);
          }
/* #line 261 */
      break;
      case Msp430RefVoltGeneratorP_GENERATOR_OFF: 

        case Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING: 

          case Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING: 

            default: 

              return;
    }
}

/* # 193 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_default_fired(uint8_t num)
{
}

/* # 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_fired(uint8_t arg_0x18923c8){
/* #line 72 */
  switch (arg_0x18923c8) {
/* #line 72 */
    case 0U:
/* #line 72 */
      SenseC_Timer_fired();
/* #line 72 */
      break;
/* #line 72 */
    case 1U:
/* #line 72 */
      Msp430RefVoltGeneratorP_SwitchOnTimer_fired();
/* #line 72 */
      break;
/* #line 72 */
    case 2U:
/* #line 72 */
      Msp430RefVoltGeneratorP_SwitchOffTimer_fired();
/* #line 72 */
      break;
/* #line 72 */
    default:
/* #line 72 */
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_default_fired(arg_0x18923c8);
/* #line 72 */
      break;
/* #line 72 */
    }
/* #line 72 */
}
/* #line 72 */
/* # 128 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_getNow());
}

/* # 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_fired(void ){
/* #line 72 */
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_fired();
/* #line 72 */
}
/* #line 72 */
/* # 80 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 82 */
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type __nesc_temp = 
/* #line 82 */
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_dt;

      {
/* #line 82 */
        __nesc_atomic_end(__nesc_atomic); 
/* #line 82 */
        return __nesc_temp;
      }
    }
/* #line 84 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 105 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_getAlarm(void ){
/* #line 105 */
  unsigned long result;
/* #line 105 */

/* #line 105 */
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_getAlarm();
/* #line 105 */

/* #line 105 */
  return result;
/* #line 105 */
}
/* #line 105 */
/* # 63 "/Users/doina/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc" */
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_fired_runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_m_dt, FALSE);
    }
/* #line 67 */
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_fired();
}

/* # 58 "/Users/doina/tinyos-2.x/tos/types/TinyError.h" */
static inline error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_cc_t x)
/* #line 46 */
{
/* #line 46 */
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3___nesc_unnamed4312 {
/* #line 46 */
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_cc_t f;
/* #line 46 */
    uint16_t t;
  } 
/* #line 46 */
  c = { .f = x };

/* #line 46 */
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_CC2int(x);
}

/* #line 94 */
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_compareControl();
}

/* # 36 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_setControlAsCompare(void ){
/* #line 36 */
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_setControlAsCompare();
/* #line 36 */
}
/* #line 36 */
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Init_init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_setControlAsCompare();
  return SUCCESS;
}

/* # 123 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
inline static void Msp430Adc12ImplP_HplAdc12_stopConversion(void ){
/* #line 123 */
  HplAdc12P_HplAdc12_stopConversion();
/* #line 123 */
}
/* #line 123 */
/* # 92 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static inline error_t Msp430Adc12ImplP_Init_init(void )
{
  adc12ctl0_t ctl0;

/* #line 95 */
  Msp430Adc12ImplP_HplAdc12_stopConversion();
  ctl0 = Msp430Adc12ImplP_HplAdc12_getCtl0();
  ctl0.adc12tovie = 1;
  ctl0.adc12ovie = 1;
  Msp430Adc12ImplP_HplAdc12_setCtl0(ctl0);
  return SUCCESS;
}

/* # 51 "/Users/doina/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc" */
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_Init_init(void )
/* #line 51 */
{
  memset(/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_resQ, 0, sizeof /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_resQ);
  return SUCCESS;
}

/* # 83 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
static inline error_t AdcStreamP_Init_init(void )
/* #line 83 */
{
  uint8_t i;

  for (i = 0; i != AdcStreamP_NSTREAM; i++) 
    AdcStreamP_bufferQueueEnd[i] = &AdcStreamP_bufferQueue[i];

  return SUCCESS;
}

/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
inline static error_t RealMainP_SoftwareInit_init(void ){
/* #line 51 */
  unsigned char result;
/* #line 51 */

/* #line 51 */
  result = AdcStreamP_Init_init();
/* #line 51 */
  result = ecombine(result, /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_Init_init());
/* #line 51 */
  result = ecombine(result, Msp430Adc12ImplP_Init_init());
/* #line 51 */
  result = ecombine(result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Init_init());
/* #line 51 */

/* #line 51 */
  return result;
/* #line 51 */
}
/* #line 51 */
/* # 143 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_getNow(), dt, FALSE);
}

/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static void SenseC_Timer_startPeriodic(uint32_t dt){
/* #line 53 */
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startPeriodic(0U, dt);
/* #line 53 */
}
/* #line 53 */
/* # 61 "SenseC.nc" */
static inline void SenseC_Boot_booted(void )
/* #line 61 */
{
  SenseC_Timer_startPeriodic(100);
}

/* # 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc" */
inline static void RealMainP_Boot_booted(void ){
/* #line 49 */
  SenseC_Boot_booted();
/* #line 49 */
}
/* #line 49 */
/* # 206 "/Users/doina/tinyos-2.x/tos/chips/msp430/msp430hardware.h" */
static inline  void __nesc_disable_interrupt(void )
{
_R2 &= 0xfff7;
;}

/* # 126 "/Users/doina/tinyos-2.x/tos/chips/msp430/McuSleepC.nc" */
static inline mcu_power_t McuSleepC_McuPowerOverride_default_lowestState(void )
/* #line 126 */
{
  return MSP430_POWER_LPM4;
}

/* # 54 "/Users/doina/tinyos-2.x/tos/interfaces/McuPowerOverride.nc" */
inline static mcu_power_t McuSleepC_McuPowerOverride_lowestState(void ){
/* #line 54 */
  unsigned char result;
/* #line 54 */

/* #line 54 */
  result = McuSleepC_McuPowerOverride_default_lowestState();
/* #line 54 */

/* #line 54 */
  return result;
/* #line 54 */
}
/* #line 54 */
/* # 66 "/Users/doina/tinyos-2.x/tos/chips/msp430/McuSleepC.nc" */
static inline mcu_power_t McuSleepC_getPowerState(void )
/* #line 66 */
{
  mcu_power_t pState = MSP430_POWER_LPM3;









  if ((((((
/* #line 69 */
  TA0CCTL0 & 0x0010 || 
  TA0CCTL1 & 0x0010) || 
  TA0CCTL2 & 0x0010) && (
  TA0CTL & (3 << 8)) == 2 << 8) || (
  ME1 & ((1 << 7) | (1 << 6)) && U0TCTL & 0x20)) || (
  ME2 & ((1 << 5) | (1 << 4)) && U1TCTL & 0x20))


   || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
  I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20)) {


    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x0010) {
      if (ADC12CTL1 & (2 << 3)) {

          if (ADC12CTL1 & (1 << 3)) {
            pState = MSP430_POWER_LPM1;
            }
          else {
/* #line 91 */
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
/* #line 92 */
        if (ADC12CTL1 & 0x0400 && (TA0CTL & (3 << 8)) == 2 << 8) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

/* # 194 "/Users/doina/tinyos-2.x/tos/chips/msp430/msp430hardware.h" */
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
/* #line 194 */
{
  return m1 < m2 ? m1 : m2;
}

/* # 104 "/Users/doina/tinyos-2.x/tos/chips/msp430/McuSleepC.nc" */
static inline void McuSleepC_computePowerState(void )
/* #line 104 */
{
  McuSleepC_powerState = mcombine(McuSleepC_getPowerState(), 
  McuSleepC_McuPowerOverride_lowestState());
}

static inline void McuSleepC_McuSleep_sleep(void )
/* #line 109 */
{
  uint16_t temp;

/* #line 111 */
  if (McuSleepC_dirty) {
      McuSleepC_computePowerState();
    }

  temp = McuSleepC_msp430PowerBits[McuSleepC_powerState] | 0x0008;
_R2 |= temp;

;
  __nesc_disable_interrupt();
}

/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/McuSleep.nc" */
inline static void SchedulerBasicP_McuSleep_sleep(void ){
/* #line 59 */
  McuSleepC_McuSleep_sleep();
/* #line 59 */
}
/* #line 59 */
/* # 67 "/Users/doina/tinyos-2.x/tos/system/SchedulerBasicP.nc" */
static __inline uint8_t SchedulerBasicP_popTask(void )
{
  if (SchedulerBasicP_m_head != SchedulerBasicP_NO_TASK) 
    {
      uint8_t id = SchedulerBasicP_m_head;

/* #line 72 */
      SchedulerBasicP_m_head = SchedulerBasicP_m_next[SchedulerBasicP_m_head];
      if (SchedulerBasicP_m_head == SchedulerBasicP_NO_TASK) 
        {
          SchedulerBasicP_m_tail = SchedulerBasicP_NO_TASK;
        }
      SchedulerBasicP_m_next[id] = SchedulerBasicP_NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP_NO_TASK;
    }
}

/* #line 138 */
static inline void SchedulerBasicP_Scheduler_taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP_popTask()) == SchedulerBasicP_NO_TASK) 
            {
              SchedulerBasicP_McuSleep_sleep();
            }
        }
/* #line 150 */
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP_TaskBasic_runTask(nextTask);
    }
}

/* # 61 "/Users/doina/tinyos-2.x/tos/interfaces/Scheduler.nc" */
inline static void RealMainP_Scheduler_taskLoop(void ){
/* #line 61 */
  SchedulerBasicP_Scheduler_taskLoop();
/* #line 61 */
}
/* #line 61 */
/* # 161 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
static inline uint16_t *AdcP_SingleChannel_multipleDataReady(uint8_t client, 
uint16_t *buf, uint16_t numSamples)
{

  return 0;
}

/* # 645 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static inline uint16_t *Msp430Adc12ImplP_SingleChannel_default_multipleDataReady(uint8_t id, 
uint16_t *buf, uint16_t numSamples)
{
  return 0;
}

/* # 227 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
inline static uint16_t * Msp430Adc12ImplP_SingleChannel_multipleDataReady(uint8_t arg_0x19743f0, uint16_t * buffer, uint16_t numSamples){
/* #line 227 */
  unsigned int *result;
/* #line 227 */

/* #line 227 */
  switch (arg_0x19743f0) {
/* #line 227 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0_ID:
/* #line 227 */
      result = AdcP_SingleChannel_multipleDataReady(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC_0_CLIENT, buffer, numSamples);
/* #line 227 */
      break;
/* #line 227 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1_ID:
/* #line 227 */
      result = AdcStreamP_SingleChannel_multipleDataReady(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC_0_RSCLIENT, buffer, numSamples);
/* #line 227 */
      break;
/* #line 227 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_2_ID:
/* #line 227 */
      result = AdcP_SingleChannel_multipleDataReady(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC_0_CLIENT, buffer, numSamples);
/* #line 227 */
      break;
/* #line 227 */
    default:
/* #line 227 */
      result = Msp430Adc12ImplP_SingleChannel_default_multipleDataReady(arg_0x19743f0, buffer, numSamples);
/* #line 227 */
      break;
/* #line 227 */
    }
/* #line 227 */

/* #line 227 */
  return result;
/* #line 227 */
}
/* #line 227 */
/* # 91 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc" */
static inline uint16_t HplAdc12P_HplAdc12_getMem(uint8_t i)
/* #line 91 */
{
  return *((uint16_t *)(int *)0x0140 + i);
}

/* # 89 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
inline static uint16_t Msp430Adc12ImplP_HplAdc12_getMem(uint8_t idx){
/* #line 89 */
  unsigned int result;
/* #line 89 */

/* #line 89 */
  result = HplAdc12P_HplAdc12_getMem(idx);
/* #line 89 */

/* #line 89 */
  return result;
/* #line 89 */
}
/* #line 89 */
/* #line 82 */
inline static adc12memctl_t Msp430Adc12ImplP_HplAdc12_getMCtl(uint8_t idx){
/* #line 82 */
  struct __nesc_unnamed4268 result;
/* #line 82 */

/* #line 82 */
  result = HplAdc12P_HplAdc12_getMCtl(idx);
/* #line 82 */

/* #line 82 */
  return result;
/* #line 82 */
}
/* #line 82 */
/* # 651 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static inline void Msp430Adc12ImplP_MultiChannel_default_dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples)
/* #line 651 */
{
}

/* # 105 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc" */
inline static void Msp430Adc12ImplP_MultiChannel_dataReady(uint8_t arg_0x1972108, uint16_t *buffer, uint16_t numSamples){
/* #line 105 */
    Msp430Adc12ImplP_MultiChannel_default_dataReady(arg_0x1972108, buffer, numSamples);
/* #line 105 */
}
/* #line 105 */
/* # 640 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static inline error_t Msp430Adc12ImplP_SingleChannel_default_singleDataReady(uint8_t id, uint16_t data)
{
  return FAIL;
}

/* # 206 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc" */
inline static error_t Msp430Adc12ImplP_SingleChannel_singleDataReady(uint8_t arg_0x19743f0, uint16_t data){
/* #line 206 */
  unsigned char result;
/* #line 206 */

/* #line 206 */
  switch (arg_0x19743f0) {
/* #line 206 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_0_ID:
/* #line 206 */
      result = AdcP_SingleChannel_singleDataReady(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC_0_CLIENT, data);
/* #line 206 */
      break;
/* #line 206 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_1_ID:
/* #line 206 */
      result = AdcStreamP_SingleChannel_singleDataReady(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC_0_RSCLIENT, data);
/* #line 206 */
      break;
/* #line 206 */
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC_2_ID:
/* #line 206 */
      result = AdcP_SingleChannel_singleDataReady(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC_0_CLIENT, data);
/* #line 206 */
      break;
/* #line 206 */
    default:
/* #line 206 */
      result = Msp430Adc12ImplP_SingleChannel_default_singleDataReady(arg_0x19743f0, data);
/* #line 206 */
      break;
/* #line 206 */
    }
/* #line 206 */

/* #line 206 */
  return result;
/* #line 206 */
}
/* #line 206 */
/* # 654 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static inline void Msp430Adc12ImplP_Overflow_default_conversionTimeOverflow(uint8_t id)
/* #line 654 */
{
}

/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc" */
inline static void Msp430Adc12ImplP_Overflow_conversionTimeOverflow(uint8_t arg_0x19729f8){
/* #line 54 */
    Msp430Adc12ImplP_Overflow_default_conversionTimeOverflow(arg_0x19729f8);
/* #line 54 */
}
/* #line 54 */
/* # 653 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static inline void Msp430Adc12ImplP_Overflow_default_memOverflow(uint8_t id)
/* #line 653 */
{
}

/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc" */
inline static void Msp430Adc12ImplP_Overflow_memOverflow(uint8_t arg_0x19729f8){
/* #line 49 */
    Msp430Adc12ImplP_Overflow_default_memOverflow(arg_0x19729f8);
/* #line 49 */
}
/* #line 49 */
/* # 544 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static inline void Msp430Adc12ImplP_HplAdc12_conversionDone(uint16_t iv)
{
  bool overflow = FALSE;
  uint16_t *resultBuffer;

  if (iv <= 4) {
      if (iv == 2) {
        Msp430Adc12ImplP_Overflow_memOverflow(Msp430Adc12ImplP_clientID);
        }
      else {
/* #line 553 */
        Msp430Adc12ImplP_Overflow_conversionTimeOverflow(Msp430Adc12ImplP_clientID);
        }
      if (! Msp430Adc12ImplP_HplAdc12_getCtl0().msc) {
        overflow = TRUE;
        }
    }
/* #line 558 */
  switch (Msp430Adc12ImplP_state & Msp430Adc12ImplP_CONVERSION_MODE_MASK) 
    {
      case Msp430Adc12ImplP_SINGLE_DATA: 
        Msp430Adc12ImplP_stopConversion();
      Msp430Adc12ImplP_SingleChannel_singleDataReady(Msp430Adc12ImplP_clientID, Msp430Adc12ImplP_HplAdc12_getMem(0));
      break;
      case Msp430Adc12ImplP_SINGLE_DATA_REPEAT: 
        {
          error_t repeatContinue;

/* #line 567 */
          repeatContinue = Msp430Adc12ImplP_SingleChannel_singleDataReady(Msp430Adc12ImplP_clientID, 
          Msp430Adc12ImplP_HplAdc12_getMem(0));
          if (repeatContinue != SUCCESS) {
            Msp430Adc12ImplP_stopConversion();
            }
/* #line 571 */
          break;
        }

      case Msp430Adc12ImplP_MULTI_CHANNEL: 
        {
          uint16_t i = 0;
/* #line 576 */
          uint16_t k;

/* #line 577 */
          resultBuffer = Msp430Adc12ImplP_resultBufferStart + Msp430Adc12ImplP_resultBufferIndex;
          do {
              * resultBuffer++ = Msp430Adc12ImplP_HplAdc12_getMem(i);
            }
          while (
/* #line 580 */
          ++i < Msp430Adc12ImplP_numChannels);
          Msp430Adc12ImplP_resultBufferIndex += Msp430Adc12ImplP_numChannels;
          if (overflow || Msp430Adc12ImplP_resultBufferLength == Msp430Adc12ImplP_resultBufferIndex) {
              Msp430Adc12ImplP_stopConversion();
              resultBuffer -= Msp430Adc12ImplP_resultBufferIndex;
              k = Msp430Adc12ImplP_resultBufferIndex - Msp430Adc12ImplP_numChannels;
              Msp430Adc12ImplP_resultBufferIndex = 0;
              Msp430Adc12ImplP_MultiChannel_dataReady(Msp430Adc12ImplP_clientID, resultBuffer, 
              overflow ? k : Msp430Adc12ImplP_resultBufferLength);
            }
        }
      break;
      case Msp430Adc12ImplP_MULTIPLE_DATA: 
        {
          uint16_t i = 0;
/* #line 594 */
          uint16_t length;
/* #line 594 */
          uint16_t k;

/* #line 595 */
          resultBuffer = Msp430Adc12ImplP_resultBufferStart + Msp430Adc12ImplP_resultBufferIndex;
          if (Msp430Adc12ImplP_resultBufferLength - Msp430Adc12ImplP_resultBufferIndex > 16) {
            length = 16;
            }
          else {
/* #line 599 */
            length = Msp430Adc12ImplP_resultBufferLength - Msp430Adc12ImplP_resultBufferIndex;
            }
/* #line 600 */
          do {
              * resultBuffer++ = Msp430Adc12ImplP_HplAdc12_getMem(i);
            }
          while (
/* #line 602 */
          ++i < length);
          Msp430Adc12ImplP_resultBufferIndex += length;
          if (overflow || Msp430Adc12ImplP_resultBufferLength == Msp430Adc12ImplP_resultBufferIndex) {
              Msp430Adc12ImplP_stopConversion();
              resultBuffer -= Msp430Adc12ImplP_resultBufferIndex;
              k = Msp430Adc12ImplP_resultBufferIndex - length;
              Msp430Adc12ImplP_resultBufferIndex = 0;
              Msp430Adc12ImplP_SingleChannel_multipleDataReady(Msp430Adc12ImplP_clientID, resultBuffer, 
              overflow ? k : Msp430Adc12ImplP_resultBufferLength);
            }
          else {
/* #line 611 */
            if (Msp430Adc12ImplP_resultBufferLength - Msp430Adc12ImplP_resultBufferIndex > 15) {
              return;
              }
            else 
/* #line 613 */
              {

                adc12memctl_t memctl = Msp430Adc12ImplP_HplAdc12_getMCtl(0);

/* #line 616 */
                memctl.eos = 1;
                Msp430Adc12ImplP_HplAdc12_setMCtl(Msp430Adc12ImplP_resultBufferLength - Msp430Adc12ImplP_resultBufferIndex, memctl);
              }
            }
        }
/* #line 620 */
      break;
      case Msp430Adc12ImplP_MULTIPLE_DATA_REPEAT: 
        {
          uint8_t i = 0;

/* #line 624 */
          resultBuffer = Msp430Adc12ImplP_resultBufferStart;
          do {
              * resultBuffer++ = Msp430Adc12ImplP_HplAdc12_getMem(i);
            }
          while (
/* #line 627 */
          ++i < Msp430Adc12ImplP_resultBufferLength);

          Msp430Adc12ImplP_resultBufferStart = Msp430Adc12ImplP_SingleChannel_multipleDataReady(Msp430Adc12ImplP_clientID, 
          resultBuffer - Msp430Adc12ImplP_resultBufferLength, 
          overflow ? 0 : Msp430Adc12ImplP_resultBufferLength);
          if (!Msp430Adc12ImplP_resultBufferStart) {
            Msp430Adc12ImplP_stopConversion();
            }
/* #line 634 */
          break;
        }
    }
}

/* # 274 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc" */
static inline void Msp430RefVoltGeneratorP_HplAdc12_conversionDone(uint16_t iv)
/* #line 274 */
{
}

/* # 112 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
inline static void HplAdc12P_HplAdc12_conversionDone(uint16_t iv){
/* #line 112 */
  Msp430RefVoltGeneratorP_HplAdc12_conversionDone(iv);
/* #line 112 */
  Msp430Adc12ImplP_HplAdc12_conversionDone(iv);
/* #line 112 */
}
/* #line 112 */
/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP_40_IO_selectIOFunc(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  * (volatile uint8_t * )55U &= ~(0x01 << 0);
}

/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port60_selectIOFunc(void ){
/* #line 85 */
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP_40_IO_selectIOFunc();
/* #line 85 */
}
/* #line 85 */
/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP_41_IO_selectIOFunc(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  * (volatile uint8_t * )55U &= ~(0x01 << 1);
}

/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port61_selectIOFunc(void ){
/* #line 85 */
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP_41_IO_selectIOFunc();
/* #line 85 */
}
/* #line 85 */
/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP_42_IO_selectIOFunc(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  * (volatile uint8_t * )55U &= ~(0x01 << 2);
}

/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port62_selectIOFunc(void ){
/* #line 85 */
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP_42_IO_selectIOFunc();
/* #line 85 */
}
/* #line 85 */
/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP_43_IO_selectIOFunc(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  * (volatile uint8_t * )55U &= ~(0x01 << 3);
}

/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port63_selectIOFunc(void ){
/* #line 85 */
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP_43_IO_selectIOFunc();
/* #line 85 */
}
/* #line 85 */
/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP_44_IO_selectIOFunc(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  * (volatile uint8_t * )55U &= ~(0x01 << 4);
}

/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port64_selectIOFunc(void ){
/* #line 85 */
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP_44_IO_selectIOFunc();
/* #line 85 */
}
/* #line 85 */
/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP_45_IO_selectIOFunc(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  * (volatile uint8_t * )55U &= ~(0x01 << 5);
}

/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port65_selectIOFunc(void ){
/* #line 85 */
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP_45_IO_selectIOFunc();
/* #line 85 */
}
/* #line 85 */
/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP_46_IO_selectIOFunc(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  * (volatile uint8_t * )55U &= ~(0x01 << 6);
}

/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port66_selectIOFunc(void ){
/* #line 85 */
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP_46_IO_selectIOFunc();
/* #line 85 */
}
/* #line 85 */
/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP_47_IO_selectIOFunc(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  * (volatile uint8_t * )55U &= ~(0x01 << 7);
}

/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void Msp430Adc12ImplP_Port67_selectIOFunc(void ){
/* #line 85 */
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP_47_IO_selectIOFunc();
/* #line 85 */
}
/* #line 85 */
/* # 98 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc" */
static inline void HplAdc12P_HplAdc12_resetIFGs(void )
/* #line 98 */
{
  HplAdc12P_ADC12IV = 0;
  HplAdc12P_ADC12IFG = 0;
}

/* # 106 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12.nc" */
inline static void Msp430Adc12ImplP_HplAdc12_resetIFGs(void ){
/* #line 106 */
  HplAdc12P_HplAdc12_resetIFGs();
/* #line 106 */
}
/* #line 106 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t AdcP_readDone_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(AdcP_readDone);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 178 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
static inline void AdcP_ReadNow_default_readDone(uint8_t client, error_t result, uint16_t val)
/* #line 178 */
{
}

/* # 66 "/Users/doina/tinyos-2.x/tos/interfaces/ReadNow.nc" */
inline static void AdcP_ReadNow_readDone(uint8_t arg_0x1901350, error_t result, AdcP_ReadNow_val_t val){
/* #line 66 */
    AdcP_ReadNow_default_readDone(arg_0x1901350, result, val);
/* #line 66 */
}
/* #line 66 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t AdcStreamP_bufferDone_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(AdcStreamP_bufferDone);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 226 "/Users/doina/tinyos-2.x/tos/chips/msp430/msp430hardware.h" */
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (({
/* #line 228 */
    uint16_t __x;

/* #line 228 */
(uint16_t )__x = _R2;__x;
  }
  )
/* #line 228 */
   & 0x0008) != 0;

/* #line 229 */
  __nesc_disable_interrupt();
;
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
;
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

/* # 11 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc" */
  void sig_TIMERA0_VECTOR(void )
/* #line 11 */
{
/* #line 11 */
  Msp430TimerCommonP_VectorTimerA0_fired();
}

/* # 169 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Event_fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Control_getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_getEvent());
    }
  else {
/* #line 174 */
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Compare_fired();
    }
}

/* #line 169 */
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Event_fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Control_getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_getEvent());
    }
  else {
/* #line 174 */
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Compare_fired();
    }
}

/* #line 169 */
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Event_fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Control_getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Capture_captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Capture_getEvent());
    }
  else {
/* #line 174 */
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_Compare_fired();
    }
}

/* # 12 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc" */
  void sig_TIMERA1_VECTOR(void )
/* #line 12 */
{
/* #line 12 */
  Msp430TimerCommonP_VectorTimerA1_fired();
}

/* #line 13 */
  void sig_TIMERB0_VECTOR(void )
/* #line 13 */
{
/* #line 13 */
  Msp430TimerCommonP_VectorTimerB0_fired();
}

/* # 135 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_default_fired(uint8_t n)
{
}

/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_fired(uint8_t arg_0x15734f0){
/* #line 28 */
  switch (arg_0x15734f0) {
/* #line 28 */
    case 0:
/* #line 28 */
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Event_fired();
/* #line 28 */
      break;
/* #line 28 */
    case 1:
/* #line 28 */
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Event_fired();
/* #line 28 */
      break;
/* #line 28 */
    case 2:
/* #line 28 */
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Event_fired();
/* #line 28 */
      break;
/* #line 28 */
    case 3:
/* #line 28 */
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6_Event_fired();
/* #line 28 */
      break;
/* #line 28 */
    case 4:
/* #line 28 */
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7_Event_fired();
/* #line 28 */
      break;
/* #line 28 */
    case 5:
/* #line 28 */
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8_Event_fired();
/* #line 28 */
      break;
/* #line 28 */
    case 6:
/* #line 28 */
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9_Event_fired();
/* #line 28 */
      break;
/* #line 28 */
    case 7:
/* #line 28 */
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Overflow_fired();
/* #line 28 */
      break;
/* #line 28 */
    default:
/* #line 28 */
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_default_fired(arg_0x15734f0);
/* #line 28 */
      break;
/* #line 28 */
    }
/* #line 28 */
}
/* #line 28 */
/* # 159 "/Users/doina/tinyos-2.x/tos/system/SchedulerBasicP.nc" */
static error_t SchedulerBasicP_TaskBasic_postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 161 */
    {
/* #line 161 */
      {
        unsigned char __nesc_temp = 
/* #line 161 */
        SchedulerBasicP_pushTask(id) ? SUCCESS : EBUSY;

        {
/* #line 161 */
          __nesc_atomic_end(__nesc_atomic); 
/* #line 161 */
          return __nesc_temp;
        }
      }
    }
/* #line 164 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 96 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Counter_get();
/* #line 98 */
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type expires;
/* #line 98 */
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
/* #line 121 */
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_AlarmFrom_startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_from_size_type )remaining << 5);
}

/* # 69 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformCounterC.nc" */
static /*CounterMilli32C.Transform*/TransformCounterC_0_to_size_type /*CounterMilli32C.Transform*/TransformCounterC_0_Counter_get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC_0_to_size_type rv = 0;

/* #line 72 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC_0_upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC_0_m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC_0_from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_get();

/* #line 76 */
      if (/*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC_0_CounterFrom_get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC_0_to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC_0_to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC_0_LOW_SHIFT_RIGHT;

/* #line 90 */
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC_0_HIGH_SHIFT_LEFT) | low_to;
      }
    }
/* #line 92 */
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

/* # 51 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
/* #line 58 */
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )400U;

/* #line 61 */
        do {
/* #line 61 */
            t0 = t1;
/* #line 61 */
            t1 = * (volatile uint16_t * )400U;
          }
        while (
/* #line 61 */
        t0 != t1);
        {
          unsigned int __nesc_temp = 
/* #line 62 */
          t1;

/* #line 62 */
          return __nesc_temp;
        }
      }
    }
  else 
/* #line 65 */
    {
      return * (volatile uint16_t * )400U;
    }
}

/* # 394 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static error_t Msp430Adc12ImplP_SingleChannel_getData(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 396 */
    {
      if (Msp430Adc12ImplP_ADCArbiterInfo_userId() == id) {
          if (Msp430Adc12ImplP_state & Msp430Adc12ImplP_MULTIPLE_DATA_REPEAT && !Msp430Adc12ImplP_resultBufferStart) 
            {
              unsigned char __nesc_temp = 
/* #line 399 */
              EINVAL;

              {
/* #line 399 */
                __nesc_atomic_end(__nesc_atomic); 
/* #line 399 */
                return __nesc_temp;
              }
            }
/* #line 400 */
          if (Msp430Adc12ImplP_state & Msp430Adc12ImplP_ADC_BUSY) 
            {
              unsigned char __nesc_temp = 
/* #line 401 */
              EBUSY;

              {
/* #line 401 */
                __nesc_atomic_end(__nesc_atomic); 
/* #line 401 */
                return __nesc_temp;
              }
            }
/* #line 402 */
          Msp430Adc12ImplP_state |= Msp430Adc12ImplP_ADC_BUSY;
          Msp430Adc12ImplP_clientID = id;
          Msp430Adc12ImplP_configureAdcPin(Msp430Adc12ImplP_HplAdc12_getMCtl(0).inch);
          Msp430Adc12ImplP_HplAdc12_startConversion();
          if (Msp430Adc12ImplP_state & Msp430Adc12ImplP_USE_TIMERA) {
            Msp430Adc12ImplP_startTimerA();
            }
/* #line 408 */
          {
            unsigned char __nesc_temp = 
/* #line 408 */
            SUCCESS;

            {
/* #line 408 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 408 */
              return __nesc_temp;
            }
          }
        }
    }
/* #line 412 */
    __nesc_atomic_end(__nesc_atomic); }
/* #line 411 */
  return FAIL;
}

/* # 137 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ArbiterInfo_userId(void )
/* #line 137 */
{
  /* atomic removed: atomic calls only */
/* #line 138 */
  {
    if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_state != /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_RES_BUSY) 
      {
        unsigned char __nesc_temp = 
/* #line 140 */
        /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_NO_RES;

/* #line 140 */
        return __nesc_temp;
      }
/* #line 141 */
    {
      unsigned char __nesc_temp = 
/* #line 141 */
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_resId;

/* #line 141 */
      return __nesc_temp;
    }
  }
}

/* # 83 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc" */
static adc12memctl_t HplAdc12P_HplAdc12_getMCtl(uint8_t i)
/* #line 83 */
{
  adc12memctl_t x = { .inch = 0, .sref = 0, .eos = 0 };
  uint8_t *memCtlPtr = (uint8_t *)(char *)0x0080;

/* #line 86 */
  memCtlPtr += i;
  x = * (adc12memctl_t *)memCtlPtr;
  return x;
}

/* # 80 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_setMode(int mode)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(0x0020 | 0x0010)) | ((mode << 4) & (0x0020 | 0x0010));
}

/* # 96 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_set_alarm(void )
{
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type now = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_Counter_get();
/* #line 98 */
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type expires;
/* #line 98 */
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type remaining;




  expires = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_t0 + /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_dt;


  remaining = (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_to_size_type )(expires - now);


  if (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_t0 <= now) 
    {
      if (expires >= /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
/* #line 121 */
  if (remaining > /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_MAX_DELAY) 
    {
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_t0 = now + /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_MAX_DELAY;
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_dt = remaining - /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_MAX_DELAY;
      remaining = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_MAX_DELAY;
    }
  else 
    {
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_t0 += /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_dt;
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_m_dt = 0;
    }
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_AlarmFrom_startAt((/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_from_size_type )now << 5, 
  (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC_1_from_size_type )remaining << 5);
}

/* # 14 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc" */
  void sig_TIMERB1_VECTOR(void )
/* #line 14 */
{
/* #line 14 */
  Msp430TimerCommonP_VectorTimerB1_fired();
}

/* # 52 "/Users/doina/tinyos-2.x/tos/system/RealMainP.nc" */
  int main(void )
/* #line 52 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
/* #line 60 */
      ;

      RealMainP_Scheduler_init();





      RealMainP_PlatformInit_init();
      while (RealMainP_Scheduler_runNextTask()) ;





      RealMainP_SoftwareInit_init();
      while (RealMainP_Scheduler_runNextTask()) ;
    }
/* #line 77 */
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP_Boot_booted();


  RealMainP_Scheduler_taskLoop();




  return -1;
}

/* # 160 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc" */
static void Msp430ClockP_set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

/* # 16 "/Users/doina/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc" */
static void MotePlatformC_TOSH_FLASH_M25P_DP_bit(bool set)
/* #line 16 */
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
/* #line 20 */
    TOSH_CLR_SIMO0_PIN();
    }
/* #line 21 */
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

/* # 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_set(void )
/* #line 45 */
{
/* #line 45 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 45 */
    * (volatile uint8_t * )49U |= 0x01 << 4;
/* #line 45 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* #line 45 */
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_set(void )
/* #line 45 */
{
/* #line 45 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 45 */
    * (volatile uint8_t * )49U |= 0x01 << 5;
/* #line 45 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* #line 45 */
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_set(void )
/* #line 45 */
{
/* #line 45 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 45 */
    * (volatile uint8_t * )49U |= 0x01 << 6;
/* #line 45 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 123 "/Users/doina/tinyos-2.x/tos/system/SchedulerBasicP.nc" */
static bool SchedulerBasicP_Scheduler_runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
/* #line 127 */
  {
    nextTask = SchedulerBasicP_popTask();
    if (nextTask == SchedulerBasicP_NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
/* #line 131 */
          FALSE;

/* #line 131 */
          return __nesc_temp;
        }
      }
  }
/* #line 134 */
  SchedulerBasicP_TaskBasic_runTask(nextTask);
  return TRUE;
}

/* #line 164 */
static void SchedulerBasicP_TaskBasic_default_runTask(uint8_t id)
{
}

/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void SchedulerBasicP_TaskBasic_runTask(uint8_t arg_0x147bb38){
/* #line 64 */
  switch (arg_0x147bb38) {
/* #line 64 */
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_fired:
/* #line 64 */
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_fired_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer:
/* #line 64 */
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case AdcP_readDone:
/* #line 64 */
      AdcP_readDone_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_grantedTask:
/* #line 64 */
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_grantedTask_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case Msp430RefVoltArbiterImplP_switchOff:
/* #line 64 */
      Msp430RefVoltArbiterImplP_switchOff_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case AdcStreamP_readStreamDone:
/* #line 64 */
      AdcStreamP_readStreamDone_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case AdcStreamP_readStreamFail:
/* #line 64 */
      AdcStreamP_readStreamFail_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case AdcStreamP_bufferDone:
/* #line 64 */
      AdcStreamP_bufferDone_runTask();
/* #line 64 */
      break;
/* #line 64 */
    default:
/* #line 64 */
      SchedulerBasicP_TaskBasic_default_runTask(arg_0x147bb38);
/* #line 64 */
      break;
/* #line 64 */
    }
/* #line 64 */
}
/* #line 64 */
/* # 116 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
static error_t Msp430RefVoltArbiterImplP_ClientResource_release(uint8_t client)
{
  error_t error;

/* #line 119 */
  if (Msp430RefVoltArbiterImplP_syncOwner == client) {
    Msp430RefVoltArbiterImplP_switchOff_postTask();
    }
/* #line 121 */
  error = Msp430RefVoltArbiterImplP_AdcResource_release(client);
/* #line 133 */
  return error;
}

/* # 97 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_release(uint8_t id)
/* #line 97 */
{
  bool released = FALSE;

/* #line 99 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 99 */
    {
      if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_state == /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_RES_BUSY && /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_resId == id) {
          if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Queue_isEmpty() == FALSE) {
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_NO_RES;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_reqResId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Queue_dequeue();
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_RES_GRANTING;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_grantedTask_postTask();
            }
          else {
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_NO_RES;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_RES_IDLE;
            }
          released = TRUE;
        }
    }
/* #line 113 */
    __nesc_atomic_end(__nesc_atomic); }
  if (released == TRUE) {
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceConfigure_unconfigure(id);
      return SUCCESS;
    }
  return FAIL;
}

/* # 65 "/Users/doina/tinyos-2.x/tos/system/RoundRobinResourceQueueC.nc" */
static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_RoundRobinQueue_isEnqueued(resource_client_id_t id)
/* #line 65 */
{
  return /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC_0_resQ[id / 8] & (1 << id % 8);
}

/* # 78 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc" */
static error_t Msp430RefVoltGeneratorP_switchOff(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 80 */
    {
      if (Msp430RefVoltGeneratorP_HplAdc12_isBusy()) 
        {
          unsigned char __nesc_temp = 
/* #line 82 */
          FAIL;

          {
/* #line 82 */
            __nesc_atomic_end(__nesc_atomic); 
/* #line 82 */
            return __nesc_temp;
          }
        }
      else 
/* #line 83 */
        {
          adc12ctl0_t ctl0 = Msp430RefVoltGeneratorP_HplAdc12_getCtl0();

/* #line 85 */
          ctl0.enc = 0;
          Msp430RefVoltGeneratorP_HplAdc12_setCtl0(ctl0);
          ctl0.refon = 0;
          Msp430RefVoltGeneratorP_HplAdc12_setCtl0(ctl0);
          {
            unsigned char __nesc_temp = 
/* #line 89 */
            SUCCESS;

            {
/* #line 89 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 89 */
              return __nesc_temp;
            }
          }
        }
    }
/* #line 93 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 133 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_m_timers[num];

/* #line 136 */
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer_postTask();
}

/* # 70 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
static void Msp430RefVoltArbiterImplP_AdcResource_granted(uint8_t client)
{
  const msp430adc12_channel_config_t *settings = Msp430RefVoltArbiterImplP_Config_getConfiguration(client);

/* #line 73 */
  if (settings->sref == REFERENCE_VREFplus_AVss || 
  settings->sref == REFERENCE_VREFplus_VREFnegterm) {
      error_t started;

/* #line 76 */
      if (Msp430RefVoltArbiterImplP_syncOwner != Msp430RefVoltArbiterImplP_NO_OWNER) {



          Msp430RefVoltArbiterImplP_AdcResource_release(client);
          Msp430RefVoltArbiterImplP_AdcResource_request(client);
          return;
        }
      Msp430RefVoltArbiterImplP_syncOwner = client;
      if (settings->ref2_5v == REFVOLT_LEVEL_1_5) {
        started = Msp430RefVoltArbiterImplP_RefVolt_1_5V_start();
        }
      else {
/* #line 88 */
        started = Msp430RefVoltArbiterImplP_RefVolt_2_5V_start();
        }
/* #line 89 */
      if (started != SUCCESS) {
          Msp430RefVoltArbiterImplP_syncOwner = Msp430RefVoltArbiterImplP_NO_OWNER;
          Msp430RefVoltArbiterImplP_AdcResource_release(client);
          Msp430RefVoltArbiterImplP_AdcResource_request(client);
        }
    }
  else {
/* #line 95 */
    Msp430RefVoltArbiterImplP_ClientResource_granted(client);
    }
}

/* # 71 "/Users/doina/tinyos-2.x/tos/system/SimpleArbiterP.nc" */
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Resource_request(uint8_t id)
/* #line 71 */
{
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_ResourceRequested_requested(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 73 */
    {
      if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_state == /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_RES_IDLE) {
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_RES_GRANTING;
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_reqResId = id;
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_grantedTask_postTask();
          {
            unsigned char __nesc_temp = 
/* #line 78 */
            SUCCESS;

            {
/* #line 78 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 78 */
              return __nesc_temp;
            }
          }
        }
/* #line 80 */
      {
        unsigned char __nesc_temp = 
/* #line 80 */
        /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP_0_Queue_enqueue(id);

        {
/* #line 80 */
          __nesc_atomic_end(__nesc_atomic); 
/* #line 80 */
          return __nesc_temp;
        }
      }
    }
/* #line 83 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 98 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
static void AdcP_SubResourceReadNow_granted(uint8_t nowClient)
{
  if (AdcP_configure(nowClient) == SUCCESS) {
    AdcP_state = AdcP_STATE_READNOW;
    }
  else {
/* #line 103 */
    AdcP_state = AdcP_STATE_READNOW_INVALID_CONFIG;
    }
/* #line 104 */
  AdcP_ResourceReadNow_granted(nowClient);
}

/* #line 65 */
static error_t AdcP_configure(uint8_t client)
{
  error_t result = EINVAL;
  const msp430adc12_channel_config_t * config;

/* #line 69 */
  config = AdcP_Config_getConfiguration(client);
  if (config->inch != INPUT_CHANNEL_NONE) {
    result = AdcP_SingleChannel_configureSingle(client, config);
    }
/* #line 72 */
  return result;
}

/* # 176 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static error_t Msp430Adc12ImplP_SingleChannel_configureSingle(uint8_t id, 
const msp430adc12_channel_config_t *config)
{
  error_t result = ERESERVE;

  if (!config) {
    return EINVAL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 184 */
    {
      if (Msp430Adc12ImplP_state & Msp430Adc12ImplP_ADC_BUSY) 
        {
          unsigned char __nesc_temp = 
/* #line 186 */
          EBUSY;

          {
/* #line 186 */
            __nesc_atomic_end(__nesc_atomic); 
/* #line 186 */
            return __nesc_temp;
          }
        }
/* #line 187 */
      if (Msp430Adc12ImplP_ADCArbiterInfo_userId() == id) {
          adc12ctl1_t ctl1 = { 
          .adc12busy = 0, 
          .conseq = 0, 
          .adc12ssel = config->adc12ssel, 
          .adc12div = config->adc12div, 
          .issh = 0, 
          .shp = 1, 
          .shs = 0, 
          .cstartadd = 0 };

          adc12memctl_t memctl = { 
          .inch = config->inch, 
          .sref = config->sref, 
          .eos = 1 };

          adc12ctl0_t ctl0 = Msp430Adc12ImplP_HplAdc12_getCtl0();

/* #line 204 */
          ctl0.msc = 1;
          ctl0.sht0 = config->sht;
          ctl0.sht1 = config->sht;

          Msp430Adc12ImplP_state = Msp430Adc12ImplP_SINGLE_DATA;
          Msp430Adc12ImplP_HplAdc12_setCtl0(ctl0);
          Msp430Adc12ImplP_HplAdc12_setCtl1(ctl1);
          Msp430Adc12ImplP_HplAdc12_setMCtl(0, memctl);
          Msp430Adc12ImplP_HplAdc12_setIEFlags(0x01);
          result = SUCCESS;
        }
    }
/* #line 215 */
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

/* # 221 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
static error_t AdcStreamP_ReadStream_read(uint8_t c, uint32_t usPeriod)
{
  if (usPeriod & 0xFFFF0000) {

      AdcStreamP_period = usPeriod / 1000;
      AdcStreamP_periodModified = TRUE;
      AdcStreamP_client = c;
      AdcStreamP_now = AdcStreamP_Alarm_getNow();
      AdcStreamP_SingleChannel_configureSingle(c, AdcStreamP_AdcConfigure_getConfiguration(c));
      if (AdcStreamP_nextBuffer(FALSE) == SUCCESS) {
        AdcStreamP_sampleSingle();
        }
    }
  else 
/* #line 232 */
    {
      AdcStreamP_period = usPeriod;
      AdcStreamP_periodModified = FALSE;
      AdcStreamP_client = c;
      AdcStreamP_nextMultiple(c);
    }
  return SUCCESS;
}

/* #line 177 */
static error_t AdcStreamP_nextBuffer(bool startNextAlarm)
/* #line 177 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      struct AdcStreamP_list_entry_t *entry = AdcStreamP_bufferQueue[AdcStreamP_client];

      if (!entry) 
        {

          AdcStreamP_bufferQueueEnd[AdcStreamP_client] = (void *)0;
          AdcStreamP_readStreamDone_postTask();
          {
            unsigned char __nesc_temp = 
/* #line 187 */
            FAIL;

            {
/* #line 187 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 187 */
              return __nesc_temp;
            }
          }
        }
      else 
/* #line 190 */
        {
          uint16_t tmp_count;

/* #line 192 */
          AdcStreamP_bufferQueue[AdcStreamP_client] = entry->next;
          if (!AdcStreamP_bufferQueue[AdcStreamP_client]) {
            AdcStreamP_bufferQueueEnd[AdcStreamP_client] = &AdcStreamP_bufferQueue[AdcStreamP_client];
            }
/* #line 195 */
          AdcStreamP_pos = AdcStreamP_buffer = (void *)0;
          AdcStreamP_count = entry->count;
          tmp_count = AdcStreamP_count;
          AdcStreamP_pos = AdcStreamP_buffer = (uint16_t * )entry;
          if (startNextAlarm) {
            AdcStreamP_nextAlarm();
            }
/* #line 201 */
          {
            unsigned char __nesc_temp = 
/* #line 201 */
            SUCCESS;

            {
/* #line 201 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 201 */
              return __nesc_temp;
            }
          }
        }
    }
/* #line 205 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* #line 206 */
static void AdcStreamP_nextMultiple(uint8_t c)
{
  if (AdcStreamP_nextBuffer(FALSE) == SUCCESS) {
      msp430adc12_channel_config_t config = *AdcStreamP_AdcConfigure_getConfiguration(c);

/* #line 210 */
      config.sampcon_ssel = SAMPCON_SOURCE_SMCLK;
      config.sampcon_id = SAMPCON_CLOCK_DIV_1;
      if (AdcStreamP_SingleChannel_configureMultiple(c, &config, AdcStreamP_pos, AdcStreamP_count, AdcStreamP_period) == SUCCESS) {
        AdcStreamP_SingleChannel_getData(c);
        }
      else 
/* #line 214 */
        {
          AdcStreamP_postBuffer(c, AdcStreamP_pos, AdcStreamP_count);
          AdcStreamP_readStreamFail_postTask();
        }
    }
}

/* # 80 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
static void AdcP_ResourceRead_granted(uint8_t client)
{

  error_t result = AdcP_configure(client);

/* #line 84 */
  if (result == SUCCESS) {
      AdcP_state = AdcP_STATE_READ;
      result = AdcP_SingleChannel_getData(client);
    }
  else 
/* #line 87 */
    {
      AdcP_ResourceRead_release(client);
      AdcP_Read_readDone(client, result, 0);
    }
}

/* # 70 "SenseC.nc" */
static void SenseC_Read_readDone(error_t result, uint16_t data)
{
  if (result == SUCCESS) {
      if (data & 0x0004) {
        SenseC_Leds_led2On();
        }
      else {
/* #line 76 */
        SenseC_Leds_led2Off();
        }
/* #line 77 */
      if (data & 0x0002) {
        SenseC_Leds_led1On();
        }
      else {
/* #line 80 */
        SenseC_Leds_led1Off();
        }
/* #line 81 */
      if (data & 0x0001) {
        SenseC_Leds_led0On();
        }
      else {
/* #line 84 */
        SenseC_Leds_led0Off();
        }
    }
}

/* # 58 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc" */
static error_t Msp430RefVoltGeneratorP_switchOn(uint8_t level)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 60 */
    {
      if (Msp430RefVoltGeneratorP_HplAdc12_isBusy()) 
        {
          unsigned char __nesc_temp = 
/* #line 62 */
          FAIL;

          {
/* #line 62 */
            __nesc_atomic_end(__nesc_atomic); 
/* #line 62 */
            return __nesc_temp;
          }
        }
      else 
/* #line 63 */
        {
          adc12ctl0_t ctl0 = Msp430RefVoltGeneratorP_HplAdc12_getCtl0();

/* #line 65 */
          ctl0.enc = 0;
          Msp430RefVoltGeneratorP_HplAdc12_setCtl0(ctl0);
          ctl0.refon = 1;
          if (level == Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING) {
            ctl0.r2_5v = 0;
            }
          else {
/* #line 71 */
            ctl0.r2_5v = 1;
            }
/* #line 72 */
          Msp430RefVoltGeneratorP_HplAdc12_setCtl0(ctl0);
          {
            unsigned char __nesc_temp = 
/* #line 73 */
            SUCCESS;

            {
/* #line 73 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 73 */
              return __nesc_temp;
            }
          }
        }
    }
/* #line 77 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void Msp430RefVoltGeneratorP_SwitchOnTimer_startOneShot(uint32_t dt){
/* #line 62 */
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startOneShot(1U, dt);
/* #line 62 */
}
/* #line 62 */
/* # 107 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc" */
static void Msp430RefVoltArbiterImplP_RefVolt_2_5V_startDone(error_t error)
{
  if (Msp430RefVoltArbiterImplP_syncOwner != Msp430RefVoltArbiterImplP_NO_OWNER) {


      Msp430RefVoltArbiterImplP_ClientResource_granted(Msp430RefVoltArbiterImplP_syncOwner);
    }
}

/* # 62 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
/* #line 79 */
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer_postTask();
}

/* # 136 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_Alarm_startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0_set_alarm();
    }
/* #line 143 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 108 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/HplAdc12P.nc" */
static void HplAdc12P_HplAdc12_stopConversion(void )
/* #line 108 */
{

  uint16_t ctl1 = HplAdc12P_ADC12CTL1;

/* #line 111 */
  HplAdc12P_ADC12CTL1 &= ~(0x0002 | 0x0004);
  HplAdc12P_ADC12CTL0 &= ~(0x0001 + 0x0002);
  HplAdc12P_ADC12CTL0 &= ~0x0010;
  HplAdc12P_ADC12CTL1 |= ctl1 & (0x0002 | 0x0004);
}







  void sig_ADC_VECTOR(void )
/* #line 123 */
{
  HplAdc12P_HplAdc12_conversionDone(HplAdc12P_ADC12IV);
}

/* # 503 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc" */
static void Msp430Adc12ImplP_stopConversion(void )
{
  uint8_t i;

  if (Msp430Adc12ImplP_state & Msp430Adc12ImplP_USE_TIMERA) {
    Msp430Adc12ImplP_TimerA_setMode(MSP430TIMER_STOP_MODE);
    }
  Msp430Adc12ImplP_resetAdcPin(Msp430Adc12ImplP_HplAdc12_getMCtl(0).inch);
  if (Msp430Adc12ImplP_state & Msp430Adc12ImplP_MULTI_CHANNEL) {
      for (i = 1; i < Msp430Adc12ImplP_numChannels; i++) 
        Msp430Adc12ImplP_resetAdcPin(Msp430Adc12ImplP_HplAdc12_getMCtl(i).inch);
    }
  /* atomic removed: atomic calls only */
/* #line 515 */
  {
    Msp430Adc12ImplP_HplAdc12_stopConversion();
    Msp430Adc12ImplP_HplAdc12_resetIFGs();
    Msp430Adc12ImplP_state &= ~Msp430Adc12ImplP_ADC_BUSY;
  }
}

/* #line 159 */
static void Msp430Adc12ImplP_resetAdcPin(uint8_t inch)
{

  switch (inch) 
    {
      case 0: Msp430Adc12ImplP_Port60_selectIOFunc();
/* #line 164 */
      break;
      case 1: Msp430Adc12ImplP_Port61_selectIOFunc();
/* #line 165 */
      break;
      case 2: Msp430Adc12ImplP_Port62_selectIOFunc();
/* #line 166 */
      break;
      case 3: Msp430Adc12ImplP_Port63_selectIOFunc();
/* #line 167 */
      break;
      case 4: Msp430Adc12ImplP_Port64_selectIOFunc();
/* #line 168 */
      break;
      case 5: Msp430Adc12ImplP_Port65_selectIOFunc();
/* #line 169 */
      break;
      case 6: Msp430Adc12ImplP_Port66_selectIOFunc();
/* #line 170 */
      break;
      case 7: Msp430Adc12ImplP_Port67_selectIOFunc();
/* #line 171 */
      break;
    }
}

/* # 142 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcP.nc" */
static error_t AdcP_SingleChannel_singleDataReady(uint8_t client, uint16_t data)
{
  switch (AdcP_state) 
    {
      case AdcP_STATE_READ: 
        AdcP_owner = client;
      AdcP_value = data;
      AdcP_readDone_postTask();
      break;
      case AdcP_STATE_READNOW: 
        AdcP_ReadNow_readDone(client, SUCCESS, data);
      break;
      default: 

        break;
    }
  return SUCCESS;
}

/* # 242 "/Users/doina/tinyos-2.x/tos/chips/msp430/adc12/AdcStreamP.nc" */
static error_t AdcStreamP_SingleChannel_singleDataReady(uint8_t streamClient, uint16_t data)
{
  if (AdcStreamP_client == AdcStreamP_NSTREAM) {
    return FAIL;
    }
  if (AdcStreamP_count == 0) 
    {
      AdcStreamP_now = AdcStreamP_Alarm_getNow();
      AdcStreamP_nextBuffer(TRUE);
    }
  else 
    {
      * AdcStreamP_pos++ = data;
      if (AdcStreamP_pos == AdcStreamP_buffer + AdcStreamP_count) 
        {
          /* atomic removed: atomic calls only */
          {
            if (AdcStreamP_lastBuffer) 
              {

                AdcStreamP_bufferQueueEnd[AdcStreamP_client] = (void *)0;
                AdcStreamP_readStreamFail_postTask();
                {
                  unsigned char __nesc_temp = 
/* #line 264 */
                  FAIL;

/* #line 264 */
                  return __nesc_temp;
                }
              }
            else {
                AdcStreamP_lastCount = AdcStreamP_count;
                AdcStreamP_lastBuffer = AdcStreamP_buffer;
              }
          }
          AdcStreamP_bufferDone_postTask();
          AdcStreamP_nextBuffer(TRUE);
        }
      else {
        AdcStreamP_nextAlarm();
        }
    }
/* #line 278 */
  return FAIL;
}

static uint16_t *AdcStreamP_SingleChannel_multipleDataReady(uint8_t streamClient, 
uint16_t *buf, uint16_t length)
{
  /* atomic removed: atomic calls only */
  {
    if (AdcStreamP_lastBuffer) 
      {

        AdcStreamP_bufferQueueEnd[AdcStreamP_client] = (void *)0;
        AdcStreamP_readStreamFail_postTask();
        {
          unsigned int *__nesc_temp = 
/* #line 291 */
          0;

/* #line 291 */
          return __nesc_temp;
        }
      }
    else {
        AdcStreamP_lastBuffer = AdcStreamP_buffer;
        AdcStreamP_lastCount = AdcStreamP_pos - AdcStreamP_buffer;
      }
  }
  AdcStreamP_bufferDone_postTask();
  AdcStreamP_nextMultiple(streamClient);
  return 0;
}

