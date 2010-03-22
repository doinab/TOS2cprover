#define nx_struct struct
#define nx_union union
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
/* # 235 "/opt/local/lib/ncc/nesc_nx.h" */
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;





static __inline uint8_t __nesc_ntoh_leuint8(const void * source)  ;




static __inline uint8_t __nesc_hton_leuint8(void * target, uint8_t value)  ;





static __inline int8_t __nesc_ntoh_int8(const void * source)  ;
/* #line 257 */
static __inline int8_t __nesc_hton_int8(void * target, int8_t value)  ;






static __inline uint16_t __nesc_ntoh_uint16(const void * source)  ;




static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;






static __inline uint16_t __nesc_ntoh_leuint16(const void * source)  ;




static __inline uint16_t __nesc_hton_leuint16(void * target, uint16_t value)  ;
/* #line 294 */
static __inline uint32_t __nesc_ntoh_uint32(const void * source)  ;






static __inline uint32_t __nesc_hton_uint32(void * target, uint32_t value)  ;
/* #line 385 */
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
/* # 40 "/opt/local/msp430/include/string.h" 3 */
extern void *memcpy(void *arg_0x5fa670, const void *arg_0x5fa808, size_t arg_0x5fa9a0);

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
uint16_t TOS_NODE_ID = 1;






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

volatile unsigned char P1IFG __asm ("0x0023");

volatile unsigned char P1IES __asm ("0x0024");

volatile unsigned char P1IE __asm ("0x0025");

volatile unsigned char P1SEL __asm ("0x0026");










volatile unsigned char P2OUT __asm ("0x0029");

volatile unsigned char P2DIR __asm ("0x002A");

volatile unsigned char P2IFG __asm ("0x002B");



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
/* # 92 "/opt/local/msp430/include/msp430/usart.h" 3 */
volatile unsigned char U0CTL __asm ("0x0070");

volatile unsigned char U0TCTL __asm ("0x0071");



volatile unsigned char U0MCTL __asm ("0x0073");

volatile unsigned char U0BR0 __asm ("0x0074");

volatile unsigned char U0BR1 __asm ("0x0075");

volatile unsigned char U0RXBUF __asm ("0x0076");
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
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420.h" */
typedef uint8_t cc2420_status_t;
/* #line 101 */
/* #line 84 */
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;



  nxle_uint8_t network;



  nxle_uint8_t type;
} __attribute_packed 

cc2420_header_t;





/* #line 106 */
typedef nx_struct cc2420_footer_t {
} __attribute_packed cc2420_footer_t;
/* #line 132 */
/* #line 116 */
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute_packed 






cc2420_metadata_t;





/* #line 135 */
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute_packed cc2420_packet_t;
/* #line 169 */
enum __nesc_unnamed4267 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 28 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};

enum __nesc_unnamed4268 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
/* # 6 "/Users/doina/tinyos-2.x/tos/types/AM.h" */
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4269 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4270 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
/* # 72 "/Users/doina/tinyos-2.x/tos/lib/serial/Serial.h" */
typedef uint8_t uart_id_t;



enum __nesc_unnamed4271 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4272 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4273 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
/* #line 110 */
/* #line 98 */
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







/* #line 112 */
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute_packed serial_header_t;




/* #line 120 */
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute_packed serial_packet_t;



/* #line 125 */
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute_packed serial_metadata_t;
/* # 48 "/Users/doina/tinyos-2.x/tos/platforms/telosa/platform_message.h" */
/* #line 45 */
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



/* #line 50 */
typedef union TOSRadioFooter {
  cc2420_footer_t cc2420;
} message_footer_t;




/* #line 54 */
typedef union TOSRadioMetadata {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;
/* # 19 "/Users/doina/tinyos-2.x/tos/types/message.h" */
/* #line 14 */
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[28];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute_packed message_t;
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/cc2420/IEEE802154.h" */
enum ieee154_fcf_enums {
  IEEE154_FCF_FRAME_TYPE = 0, 
  IEEE154_FCF_SECURITY_ENABLED = 3, 
  IEEE154_FCF_FRAME_PENDING = 4, 
  IEEE154_FCF_ACK_REQ = 5, 
  IEEE154_FCF_INTRAPAN = 6, 
  IEEE154_FCF_DEST_ADDR_MODE = 10, 
  IEEE154_FCF_SRC_ADDR_MODE = 14
};

enum ieee154_fcf_type_enums {
  IEEE154_TYPE_BEACON = 0, 
  IEEE154_TYPE_DATA = 1, 
  IEEE154_TYPE_ACK = 2, 
  IEEE154_TYPE_MAC_CMD = 3
};

enum iee154_fcf_addr_mode_enums {
  IEEE154_ADDR_NONE = 0, 
  IEEE154_ADDR_SHORT = 2, 
  IEEE154_ADDR_EXT = 3
};
/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/msp430usart.h" */
/* #line 48 */
typedef enum __nesc_unnamed4274 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










/* #line 58 */
typedef struct __nesc_unnamed4275 {
  unsigned int swrst : 1;
  unsigned int mm : 1;
  unsigned int sync : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
} __attribute_packed  msp430_uctl_t;









/* #line 69 */
typedef struct __nesc_unnamed4276 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute_packed  msp430_utctl_t;










/* #line 79 */
typedef struct __nesc_unnamed4277 {
  unsigned int rxerr : 1;
  unsigned int rxwake : 1;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int brk : 1;
  unsigned int oe : 1;
  unsigned int pe : 1;
  unsigned int fe : 1;
} __attribute_packed  msp430_urctl_t;
/* #line 116 */
/* #line 99 */
typedef struct __nesc_unnamed4278 {
  unsigned int ubr : 16;

/*  unsigned int  : 1; */
  unsigned int mm : 1;
/*  unsigned int  : 1; */
  unsigned int listen : 1;
  unsigned int clen : 1;
/*  unsigned int  : 3; */

/*  unsigned int  : 1; */
  unsigned int stc : 1;
/*  unsigned int  : 2; */
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
/*  unsigned int  : 0; */
} msp430_spi_config_t;





/* #line 118 */
typedef struct __nesc_unnamed4279 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




/* #line 124 */
typedef union __nesc_unnamed4280 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;

msp430_spi_union_config_t msp430_spi_default_config = { 
{ 
.ubr = 0x0002, 
.ssel = 0x02, 
.clen = 1, 
.listen = 0, 
.mm = 1, 
.ckph = 1, 
.ckpl = 0, 
.stc = 1 } };
/* #line 169 */
/* #line 150 */
typedef enum __nesc_unnamed4281 {

  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, 
  UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, 

  UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, 
  UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, 
  UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, 
  UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, 
  UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, 
  UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, 
  UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, 
  UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, 
  UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, 
  UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, 
  UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55
} msp430_uart_rate_t;
/* #line 200 */
/* #line 171 */
typedef struct __nesc_unnamed4282 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

/*  unsigned int  : 1; */
  unsigned int mm : 1;
/*  unsigned int  : 1; */
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
/*  unsigned int  : 0; */

/*  unsigned int  : 3; */
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
/*  unsigned int  : 1; */

/*  unsigned int  : 2; */
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
/*  unsigned int  : 4; */
/*  unsigned int  : 0; */

  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;








/* #line 202 */
typedef struct __nesc_unnamed4283 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




/* #line 211 */
typedef union __nesc_unnamed4284 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;
/* #line 248 */
/* #line 240 */
typedef struct __nesc_unnamed4285 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute_packed  msp430_i2ctctl_t;
/* #line 276 */
/* #line 253 */
typedef struct __nesc_unnamed4286 {
/*  unsigned int  : 1; */
  unsigned int mst : 1;
/*  unsigned int  : 1; */
  unsigned int listen : 1;
  unsigned int xa : 1;
/*  unsigned int  : 1; */
  unsigned int txdmaen : 1;
  unsigned int rxdmaen : 1;

/*  unsigned int  : 4; */
  unsigned int i2cssel : 2;
  unsigned int i2crm : 1;
  unsigned int i2cword : 1;

  unsigned int i2cpsc : 8;

  unsigned int i2csclh : 8;

  unsigned int i2cscll : 8;

  unsigned int i2coa : 10;
/*  unsigned int  : 6; */
} msp430_i2c_config_t;








/* #line 278 */
typedef struct __nesc_unnamed4287 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




/* #line 287 */
typedef union __nesc_unnamed4288 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
/* # 33 "/Users/doina/tinyos-2.x/tos/types/Resource.h" */
typedef uint8_t resource_client_id_t;
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420TimeSyncMessage.h" */
typedef nx_uint32_t timesync_radio_t;
/* # 40 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngine.h" */
enum __nesc_unnamed4289 {
  AM_DISSEMINATION_MESSAGE = 0x60, 
  AM_DISSEMINATION_PROBE_MESSAGE = 0x61, 
  DISSEMINATION_SEQNO_UNKNOWN = 0
};





/* #line 46 */
typedef nx_struct dissemination_message {
  nx_uint16_t key;
  nx_uint32_t seqno;
  nx_uint8_t ( data)[0];
} __attribute_packed dissemination_message_t;



/* #line 52 */
typedef nx_struct dissemination_probe_message {
  nx_uint16_t key;
} __attribute_packed dissemination_probe_message_t;
typedef uint32_t TestDisseminationC_Update32_t;
typedef uint32_t TestDisseminationC_Value32_t;
typedef uint16_t TestDisseminationC_Value16_t;
typedef uint16_t TestDisseminationC_Update16_t;
typedef TMilli TestDisseminationC_Timer_precision_tag;
typedef T32khz CC2420ControlP_StartupTimer_precision_tag;
typedef uint32_t CC2420ControlP_StartupTimer_size_type;
typedef uint16_t CC2420ControlP_ReadRssi_val_t;
enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Timer*/Msp430Timer32khzC_0___nesc_unnamed4290 {
  Msp430Timer32khzC_0_ALARM_ID = 0U
};
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_frequency_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_frequency_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC_0_to_precision_tag;
typedef uint32_t /*Counter32khz32C.Transform*/TransformCounterC_0_to_size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC_0_from_precision_tag;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC_0_from_size_type;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC_0_upper_count_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC_0_from_precision_tag /*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC_0_from_size_type /*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_size_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC_0_to_precision_tag /*Counter32khz32C.Transform*/TransformCounterC_0_Counter_precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC_0_to_size_type /*Counter32khz32C.Transform*/TransformCounterC_0_Counter_size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_precision_tag;
typedef uint32_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_from_precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_from_size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_from_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_from_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Counter_precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Counter_size_type;
enum /*CC2420ControlC.Spi*/CC2420SpiC_0___nesc_unnamed4291 {
  CC2420SpiC_0_CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0___nesc_unnamed4292 {
  Msp430Spi0C_0_CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C_0___nesc_unnamed4293 {
  Msp430Usart0C_0_CLIENT_ID = 0U
};
enum /*CC2420ControlC.SyncSpiC*/CC2420SpiC_1___nesc_unnamed4294 {
  CC2420SpiC_1_CLIENT_ID = 1U
};
enum /*CC2420ControlC.RssiResource*/CC2420SpiC_2___nesc_unnamed4295 {
  CC2420SpiC_2_CLIENT_ID = 2U
};
typedef T32khz CC2420TransmitP_PacketTimeStamp_precision_tag;
typedef uint32_t CC2420TransmitP_PacketTimeStamp_size_type;
typedef T32khz CC2420TransmitP_BackoffTimer_precision_tag;
typedef uint32_t CC2420TransmitP_BackoffTimer_size_type;
enum /*CC2420TransmitC.Spi*/CC2420SpiC_3___nesc_unnamed4296 {
  CC2420SpiC_3_CLIENT_ID = 3U
};
typedef T32khz CC2420ReceiveP_PacketTimeStamp_precision_tag;
typedef uint32_t CC2420ReceiveP_PacketTimeStamp_size_type;
typedef T32khz CC2420PacketP_PacketTimeStamp32khz_precision_tag;
typedef uint32_t CC2420PacketP_PacketTimeStamp32khz_size_type;
typedef T32khz CC2420PacketP_LocalTime32khz_precision_tag;
typedef TMilli CC2420PacketP_LocalTimeMilli_precision_tag;
typedef TMilli CC2420PacketP_PacketTimeStampMilli_precision_tag;
typedef uint32_t CC2420PacketP_PacketTimeStampMilli_size_type;
typedef T32khz /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC_0_precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC_0_precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC_0_LocalTime_precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC_0_precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC_0_Counter_precision_tag;
typedef uint32_t /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC_0_Counter_size_type;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC_1___nesc_unnamed4297 {
  Msp430Timer32khzC_1_ALARM_ID = 1U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC_1_to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC_1_to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC_1_from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC_1_from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC_1_upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC_1_from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC_1_from_size_type /*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC_1_to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC_1_Counter_precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC_1_to_size_type /*CounterMilli32C.Transform*/TransformCounterC_1_Counter_size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Counter_precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Counter_size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_1_precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_1_precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_1_LocalTime_precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_1_precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_1_Counter_precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_1_Counter_size_type;
enum /*CC2420ReceiveC.Spi*/CC2420SpiC_4___nesc_unnamed4298 {
  CC2420SpiC_4_CLIENT_ID = 4U
};
typedef uint16_t RandomMlcgC_SeedInit_parameter;
enum AMQueueP___nesc_unnamed4299 {
  AMQueueP_NUM_CLIENTS = 2U
};
typedef uint32_t /*TestDisseminationAppC.Object32C*/DisseminatorC_0_t;
enum /*TestDisseminationAppC.Object32C*/DisseminatorC_0___nesc_unnamed4300 {
  DisseminatorC_0_TIMER_ID = 0U
};
typedef /*TestDisseminationAppC.Object32C*/DisseminatorC_0_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_t;
typedef /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationUpdate_t;
typedef /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_t;
typedef TMilli /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_precision_tag;
typedef uint16_t /*TestDisseminationAppC.Object16C*/DisseminatorC_1_t;
enum /*TestDisseminationAppC.Object16C*/DisseminatorC_1___nesc_unnamed4301 {
  DisseminatorC_1_TIMER_ID = 1U
};
typedef /*TestDisseminationAppC.Object16C*/DisseminatorC_1_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_t;
typedef /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationUpdate_t;
typedef /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_t;
/* # 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc" */
static void TestDisseminationC_Boot_booted(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
static void TestDisseminationC_RadioControl_startDone(error_t error);
/* #line 117 */
static void TestDisseminationC_RadioControl_stopDone(error_t error);
/* # 61 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationValue.nc" */
static void TestDisseminationC_Value32_changed(void );
/* #line 61 */
static void TestDisseminationC_Value16_changed(void );
/* # 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void TestDisseminationC_Timer_fired(void );
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
uint8_t arg_0x1593680);
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_VectorTimerX0_fired(void );
/* #line 28 */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Overflow_fired(void );
/* #line 28 */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_VectorTimerX1_fired(void );
/* #line 28 */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_default_fired(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
uint8_t arg_0x1593680);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Timer_isOverflowPending(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_getEvent(void );
/* #line 75 */
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_default_captured(uint16_t time);
/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Control_getControl(void );
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Event_fired(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Compare_default_fired(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Timer_overflow(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_getEvent(void );
/* #line 75 */
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_default_captured(uint16_t time);
/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Control_getControl(void );
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Event_fired(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Compare_default_fired(void );
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
/* #line 57 */
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_clearOverflow(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_setControlAsCapture(uint8_t cm);
/* #line 31 */
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_getControl(void );
/* #line 46 */
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_disableEvents(void );
/* #line 33 */
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_clearPendingInterrupt(void );
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
/* #line 36 */
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_setControlAsCompare(void );










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
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
static 
/* #line 63 */
message_t * 



CC2420ActiveMessageP_SubReceive_receive(
/* #line 60 */
message_t * msg, 
void * payload, 





uint8_t len);
/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static void CC2420ActiveMessageP_SubSend_sendDone(
/* #line 85 */
message_t * msg, 



error_t error);
/* # 53 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc" */
static void CC2420ActiveMessageP_CC2420Config_syncDone(error_t error);
/* # 95 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
static void CC2420ActiveMessageP_RadioBackoff_default_requestCca(
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16fd2e8, 
/* # 95 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
message_t * msg);
/* #line 81 */
static void CC2420ActiveMessageP_RadioBackoff_default_requestInitialBackoff(
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16fd2e8, 
/* # 81 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
message_t * msg);






static void CC2420ActiveMessageP_RadioBackoff_default_requestCongestionBackoff(
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16fd2e8, 
/* # 88 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
message_t * msg);
/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/SendNotifier.nc" */
static void CC2420ActiveMessageP_SendNotifier_default_aboutToSend(
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16fec58, 
/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/SendNotifier.nc" */
am_addr_t dest, 
/* #line 57 */
message_t * msg);
/* # 95 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
static void CC2420ActiveMessageP_SubBackoff_requestCca(message_t * msg);
/* #line 81 */
static void CC2420ActiveMessageP_SubBackoff_requestInitialBackoff(message_t * msg);






static void CC2420ActiveMessageP_SubBackoff_requestCongestionBackoff(message_t * msg);
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Packet.nc" */
static uint8_t CC2420ActiveMessageP_Packet_payloadLength(
/* #line 63 */
message_t * msg);
/* #line 115 */
static 
/* #line 112 */
void * 


CC2420ActiveMessageP_Packet_getPayload(
/* #line 110 */
message_t * msg, 




uint8_t len);
/* #line 95 */
static uint8_t CC2420ActiveMessageP_Packet_maxPayloadLength(void );
/* #line 83 */
static void CC2420ActiveMessageP_Packet_setPayloadLength(
/* #line 79 */
message_t * msg, 



uint8_t len);
/* # 69 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
static error_t CC2420ActiveMessageP_AMSend_send(
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16e11b8, 
/* # 69 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
am_addr_t addr, 
/* #line 60 */
message_t * msg, 








uint8_t len);
/* #line 124 */
static 
/* #line 122 */
void * 

CC2420ActiveMessageP_AMSend_getPayload(
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16e11b8, 
/* # 121 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
message_t * msg, 


uint8_t len);
/* #line 112 */
static uint8_t CC2420ActiveMessageP_AMSend_maxPayloadLength(
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16e11b8);
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
static 
/* #line 63 */
message_t * 



CC2420ActiveMessageP_Snoop_default_receive(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16df220, 
/* # 60 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
message_t * msg, 
void * payload, 





uint8_t len);
/* #line 67 */
static 
/* #line 63 */
message_t * 



CC2420ActiveMessageP_Receive_default_receive(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16e1b78, 
/* # 60 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
message_t * msg, 
void * payload, 





uint8_t len);
/* # 57 "/Users/doina/tinyos-2.x/tos/interfaces/AMPacket.nc" */
static am_addr_t CC2420ActiveMessageP_AMPacket_address(void );









static am_addr_t CC2420ActiveMessageP_AMPacket_destination(
/* #line 63 */
message_t * amsg);
/* #line 92 */
static void CC2420ActiveMessageP_AMPacket_setDestination(
/* #line 88 */
message_t * amsg, 



am_addr_t addr);
/* #line 136 */
static am_id_t CC2420ActiveMessageP_AMPacket_type(
/* #line 132 */
message_t * amsg);
/* #line 151 */
static void CC2420ActiveMessageP_AMPacket_setType(
/* #line 147 */
message_t * amsg, 



am_id_t t);
/* #line 125 */
static bool CC2420ActiveMessageP_AMPacket_isForMe(
/* #line 122 */
message_t * amsg);
/* # 83 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
static error_t CC2420CsmaP_SplitControl_start(void );
/* # 81 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
static void CC2420CsmaP_SubBackoff_requestInitialBackoff(message_t * msg);






static void CC2420CsmaP_SubBackoff_requestCongestionBackoff(message_t * msg);
/* # 73 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc" */
static void CC2420CsmaP_CC2420Transmit_sendDone(message_t * p_msg, error_t error);
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static error_t CC2420CsmaP_Send_send(
/* #line 56 */
message_t * msg, 







uint8_t len);
/* #line 114 */
static 
/* #line 112 */
void * 

CC2420CsmaP_Send_getPayload(
/* #line 111 */
message_t * msg, 


uint8_t len);
/* #line 101 */
static uint8_t CC2420CsmaP_Send_maxPayloadLength(void );
/* # 76 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc" */
static void CC2420CsmaP_CC2420Power_startOscillatorDone(void );
/* #line 56 */
static void CC2420CsmaP_CC2420Power_startVRegDone(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void CC2420CsmaP_Resource_granted(void );
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void CC2420CsmaP_sendDone_task_runTask(void );
/* #line 64 */
static void CC2420CsmaP_stopDone_task_runTask(void );
/* #line 64 */
static void CC2420CsmaP_startDone_task_runTask(void );
/* # 86 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc" */
static bool CC2420ControlP_CC2420Config_isAddressRecognitionEnabled(void );
/* #line 110 */
static bool CC2420ControlP_CC2420Config_isAutoAckEnabled(void );
/* #line 105 */
static bool CC2420ControlP_CC2420Config_isHwAutoAckDefault(void );
/* #line 64 */
static uint16_t CC2420ControlP_CC2420Config_getShortAddr(void );
/* #line 52 */
static error_t CC2420ControlP_CC2420Config_sync(void );
/* #line 70 */
static uint16_t CC2420ControlP_CC2420Config_getPanAddr(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void CC2420ControlP_StartupTimer_fired(void );
/* # 63 "/Users/doina/tinyos-2.x/tos/interfaces/Read.nc" */
static void CC2420ControlP_ReadRssi_default_readDone(error_t result, CC2420ControlP_ReadRssi_val_t val);
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void CC2420ControlP_syncDone_runTask(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t CC2420ControlP_Init_init(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void CC2420ControlP_SpiResource_granted(void );
/* #line 92 */
static void CC2420ControlP_SyncResource_granted(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc" */
static error_t CC2420ControlP_CC2420Power_startOscillator(void );
/* #line 90 */
static error_t CC2420ControlP_CC2420Power_rxOn(void );
/* #line 51 */
static error_t CC2420ControlP_CC2420Power_startVReg(void );
/* #line 63 */
static error_t CC2420ControlP_CC2420Power_stopVReg(void );
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void CC2420ControlP_sync_runTask(void );
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t CC2420ControlP_Resource_release(void );
/* #line 78 */
static error_t CC2420ControlP_Resource_request(void );
/* # 57 "/Users/doina/tinyos-2.x/tos/interfaces/GpioInterrupt.nc" */
static void CC2420ControlP_InterruptCCA_fired(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void CC2420ControlP_RssiResource_granted(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_fired(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Timer_overflow(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_size_type dt);
/* #line 62 */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_stop(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Init_init(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Msp430Timer_overflow(void );
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_isOverflowPending(void );










static void /*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_overflow(void );
/* #line 53 */
static /*Counter32khz32C.Transform*/TransformCounterC_0_Counter_size_type /*Counter32khz32C.Transform*/TransformCounterC_0_Counter_get(void );
/* # 98 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_getNow(void );
/* #line 92 */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_size_type dt);
/* #line 55 */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_size_type dt);






static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_stop(void );




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_fired(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Counter_overflow(void );
/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP_0_IO_get(void );
/* #line 52 */
static uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP_0_IO_getRaw(void );






static bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP_3_IO_get(void );
/* #line 52 */
static uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP_3_IO_getRaw(void );
/* #line 64 */
static void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP_4_IO_makeInput(void );
/* #line 59 */
static bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP_4_IO_get(void );
/* #line 52 */
static uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP_4_IO_getRaw(void );
/* #line 85 */
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP_17_IO_selectIOFunc(void );
/* #line 78 */
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP_17_IO_selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP_18_IO_selectIOFunc(void );
/* #line 78 */
static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP_18_IO_selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP_19_IO_selectIOFunc(void );
/* #line 78 */
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP_19_IO_selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP_20_IO_selectIOFunc(void );
/* #line 85 */
static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP_21_IO_selectIOFunc(void );
/* #line 64 */
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_makeInput(void );
/* #line 59 */
static bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_get(void );
/* #line 85 */
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_selectIOFunc(void );
/* #line 52 */
static uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_getRaw(void );
/* #line 78 */
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_selectModuleFunc(void );
/* #line 71 */
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP_26_IO_makeOutput(void );
/* #line 34 */
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP_26_IO_set(void );




static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP_26_IO_clr(void );
/* #line 71 */
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP_29_IO_makeOutput(void );
/* #line 34 */
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP_29_IO_set(void );




static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP_29_IO_clr(void );
/* #line 71 */
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP_30_IO_makeOutput(void );
/* #line 34 */
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP_30_IO_set(void );




static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP_30_IO_clr(void );




static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_toggle(void );
/* #line 71 */
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_makeOutput(void );
/* #line 34 */
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_set(void );









static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_toggle(void );
/* #line 71 */
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_makeOutput(void );
/* #line 34 */
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_set(void );









static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_toggle(void );
/* #line 71 */
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_makeOutput(void );
/* #line 34 */
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_set(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC_0_GeneralIO_makeInput(void );
/* #line 32 */
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC_0_GeneralIO_get(void );


static void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_makeOutput(void );
/* #line 29 */
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_set(void );
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_clr(void );

static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC_2_GeneralIO_get(void );
/* #line 32 */
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC_3_GeneralIO_get(void );


static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_GeneralIO_makeOutput(void );
/* #line 29 */
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_GeneralIO_set(void );
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_GeneralIO_clr(void );


static void /*HplCC2420PinsC.SFDM*/Msp430GpioC_5_GeneralIO_makeInput(void );
/* #line 32 */
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC_5_GeneralIO_get(void );


static void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_GeneralIO_makeOutput(void );
/* #line 29 */
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_GeneralIO_set(void );
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_GeneralIO_clr(void );
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430Capture_captured(uint16_t time);
/* # 43 "/Users/doina/tinyos-2.x/tos/interfaces/GpioCapture.nc" */
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_captureFallingEdge(void );
/* #line 55 */
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_disable(void );
/* #line 42 */
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_captureRisingEdge(void );
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
static void HplMsp430InterruptP_Port14_clear(void );
/* #line 36 */
static void HplMsp430InterruptP_Port14_disable(void );
/* #line 56 */
static void HplMsp430InterruptP_Port14_edge(bool low_to_high);
/* #line 31 */
static void HplMsp430InterruptP_Port14_enable(void );









static void HplMsp430InterruptP_Port26_clear(void );
/* #line 61 */
static void HplMsp430InterruptP_Port26_default_fired(void );
/* #line 41 */
static void HplMsp430InterruptP_Port17_clear(void );
/* #line 61 */
static void HplMsp430InterruptP_Port17_default_fired(void );
/* #line 41 */
static void HplMsp430InterruptP_Port21_clear(void );
/* #line 61 */
static void HplMsp430InterruptP_Port21_default_fired(void );
/* #line 41 */
static void HplMsp430InterruptP_Port12_clear(void );
/* #line 61 */
static void HplMsp430InterruptP_Port12_default_fired(void );
/* #line 41 */
static void HplMsp430InterruptP_Port24_clear(void );
/* #line 61 */
static void HplMsp430InterruptP_Port24_default_fired(void );
/* #line 41 */
static void HplMsp430InterruptP_Port15_clear(void );
/* #line 61 */
static void HplMsp430InterruptP_Port15_default_fired(void );
/* #line 41 */
static void HplMsp430InterruptP_Port27_clear(void );
/* #line 61 */
static void HplMsp430InterruptP_Port27_default_fired(void );
/* #line 41 */
static void HplMsp430InterruptP_Port10_clear(void );
/* #line 36 */
static void HplMsp430InterruptP_Port10_disable(void );
/* #line 56 */
static void HplMsp430InterruptP_Port10_edge(bool low_to_high);
/* #line 31 */
static void HplMsp430InterruptP_Port10_enable(void );









static void HplMsp430InterruptP_Port22_clear(void );
/* #line 61 */
static void HplMsp430InterruptP_Port22_default_fired(void );
/* #line 41 */
static void HplMsp430InterruptP_Port13_clear(void );
/* #line 61 */
static void HplMsp430InterruptP_Port13_default_fired(void );
/* #line 41 */
static void HplMsp430InterruptP_Port25_clear(void );
/* #line 61 */
static void HplMsp430InterruptP_Port25_default_fired(void );
/* #line 41 */
static void HplMsp430InterruptP_Port16_clear(void );
/* #line 61 */
static void HplMsp430InterruptP_Port16_default_fired(void );
/* #line 41 */
static void HplMsp430InterruptP_Port20_clear(void );
/* #line 61 */
static void HplMsp430InterruptP_Port20_default_fired(void );
/* #line 41 */
static void HplMsp430InterruptP_Port11_clear(void );
/* #line 61 */
static void HplMsp430InterruptP_Port11_default_fired(void );
/* #line 41 */
static void HplMsp430InterruptP_Port23_clear(void );
/* #line 61 */
static void HplMsp430InterruptP_Port23_default_fired(void );
/* #line 61 */
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_fired(void );
/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/GpioInterrupt.nc" */
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_Interrupt_disable(void );
/* #line 42 */
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_Interrupt_enableRisingEdge(void );
/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_fired(void );
/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/GpioInterrupt.nc" */
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_Interrupt_disable(void );
/* #line 43 */
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_Interrupt_enableFallingEdge(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/interfaces/SpiPacket.nc" */
static void CC2420SpiP_SpiPacket_sendDone(
/* #line 64 */
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
/* # 62 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
static error_t CC2420SpiP_Fifo_continueRead(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b79698, 
/* # 62 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
uint8_t * data, uint8_t length);
/* #line 91 */
static void CC2420SpiP_Fifo_default_writeDone(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b79698, 
/* # 91 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
uint8_t * data, uint8_t length, error_t error);
/* #line 82 */
static cc2420_status_t CC2420SpiP_Fifo_write(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b79698, 
/* # 82 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
uint8_t * data, uint8_t length);
/* #line 51 */
static cc2420_status_t CC2420SpiP_Fifo_beginRead(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b79698, 
/* # 51 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
uint8_t * data, uint8_t length);
/* #line 71 */
static void CC2420SpiP_Fifo_default_readDone(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b79698, 
/* # 71 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
uint8_t * data, uint8_t length, error_t error);
/* # 31 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc" */
static void CC2420SpiP_ChipSpiResource_abortRelease(void );







static error_t CC2420SpiP_ChipSpiResource_attemptRelease(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void CC2420SpiP_SpiResource_granted(void );
/* # 63 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc" */
static cc2420_status_t CC2420SpiP_Ram_write(
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint16_t arg_0x1b77118, 
/* # 63 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc" */
uint8_t offset, uint8_t * data, uint8_t length);
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc" */
static cc2420_status_t CC2420SpiP_Reg_read(
/* # 48 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b778c0, 
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc" */
uint16_t *data);







static cc2420_status_t CC2420SpiP_Reg_write(
/* # 48 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b778c0, 
/* # 55 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc" */
uint16_t data);
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t CC2420SpiP_Resource_release(
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b7abd0);
/* # 87 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t CC2420SpiP_Resource_immediateRequest(
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b7abd0);
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t CC2420SpiP_Resource_request(
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b7abd0);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void CC2420SpiP_Resource_default_granted(
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b7abd0);
/* # 118 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static bool CC2420SpiP_Resource_isOwner(
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b7abd0);
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void CC2420SpiP_grant_runTask(void );
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
static cc2420_status_t CC2420SpiP_Strobe_strobe(
/* # 49 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b76088);
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t StateImplP_Init_init(void );
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
static void StateImplP_State_toIdle(
/* # 67 "/Users/doina/tinyos-2.x/tos/system/StateImplP.nc" */
uint8_t arg_0x1bee428);
/* # 66 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
static bool StateImplP_State_isState(
/* # 67 "/Users/doina/tinyos-2.x/tos/system/StateImplP.nc" */
uint8_t arg_0x1bee428, 
/* # 66 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
uint8_t myState);
/* #line 61 */
static bool StateImplP_State_isIdle(
/* # 67 "/Users/doina/tinyos-2.x/tos/system/StateImplP.nc" */
uint8_t arg_0x1bee428);
/* # 45 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
static error_t StateImplP_State_requestState(
/* # 67 "/Users/doina/tinyos-2.x/tos/system/StateImplP.nc" */
uint8_t arg_0x1bee428, 
/* # 45 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
uint8_t reqState);





static void StateImplP_State_forceState(
/* # 67 "/Users/doina/tinyos-2.x/tos/system/StateImplP.nc" */
uint8_t arg_0x1bee428, 
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
uint8_t reqState);
/* # 55 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceConfigure.nc" */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_ResourceConfigure_unconfigure(
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c69c00);
/* # 49 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceConfigure.nc" */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_ResourceConfigure_configure(
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c69c00);
/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/SpiPacket.nc" */
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiPacket_send(
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c67740, 
/* # 48 "/Users/doina/tinyos-2.x/tos/interfaces/SpiPacket.nc" */
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
/* #line 71 */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiPacket_default_sendDone(
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c67740, 
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/SpiPacket.nc" */
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiConfigure.nc" */
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Msp430SpiConfigure_default_getConfig(
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c66a60);
/* # 34 "/Users/doina/tinyos-2.x/tos/interfaces/SpiByte.nc" */
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiByte_write(uint8_t tx);
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_release(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c66010);
/* # 87 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_immediateRequest(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c66010);
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_request(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c66010);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_granted(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c66010);
/* # 118 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_isOwner(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c66010);
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_release(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c691b8);
/* # 87 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_immediateRequest(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c691b8);
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_request(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c691b8);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_default_granted(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c691b8);
/* # 118 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_isOwner(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c691b8);
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc" */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartInterrupts_rxDone(uint8_t data);
/* #line 49 */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartInterrupts_txDone(void );
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone_task_runTask(void );
/* # 180 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc" */
static void HplMsp430Usart0P_Usart_enableRxIntr(void );
/* #line 197 */
static void HplMsp430Usart0P_Usart_clrRxIntr(void );
/* #line 97 */
static void HplMsp430Usart0P_Usart_resetUsart(bool reset);
/* #line 179 */
static void HplMsp430Usart0P_Usart_disableIntr(void );
/* #line 90 */
static void HplMsp430Usart0P_Usart_setUmctl(uint8_t umctl);
/* #line 177 */
static void HplMsp430Usart0P_Usart_disableRxIntr(void );
/* #line 207 */
static void HplMsp430Usart0P_Usart_clrIntr(void );
/* #line 80 */
static void HplMsp430Usart0P_Usart_setUbr(uint16_t ubr);
/* #line 224 */
static void HplMsp430Usart0P_Usart_tx(uint8_t data);
/* #line 128 */
static void HplMsp430Usart0P_Usart_disableUart(void );
/* #line 153 */
static void HplMsp430Usart0P_Usart_enableSpi(void );
/* #line 168 */
static void HplMsp430Usart0P_Usart_setModeSpi(msp430_spi_union_config_t *config);
/* #line 231 */
static uint8_t HplMsp430Usart0P_Usart_rx(void );
/* #line 192 */
static bool HplMsp430Usart0P_Usart_isRxIntrPending(void );
/* #line 158 */
static void HplMsp430Usart0P_Usart_disableSpi(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t LedsP_Init_init(void );
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
static void LedsP_Leds_led0Toggle(void );
/* #line 72 */
static void LedsP_Leds_led1Toggle(void );
/* #line 89 */
static void LedsP_Leds_led2Toggle(void );
/* # 31 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_GeneralIO_toggle(void );



static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_GeneralIO_makeOutput(void );
/* #line 29 */
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_GeneralIO_set(void );

static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_GeneralIO_toggle(void );



static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_GeneralIO_makeOutput(void );
/* #line 29 */
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_GeneralIO_set(void );

static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_GeneralIO_toggle(void );



static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_GeneralIO_makeOutput(void );
/* #line 29 */
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_GeneralIO_set(void );
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc" */
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_Interrupts_default_rxDone(
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc" */
uint8_t arg_0x1dac1d8, 
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc" */
uint8_t data);
/* #line 49 */
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_Interrupts_default_txDone(
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc" */
uint8_t arg_0x1dac1d8);
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc" */
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_RawI2CInterrupts_fired(void );
/* #line 39 */
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_I2CInterrupts_default_fired(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc" */
uint8_t arg_0x1daca58);
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc" */
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_RawInterrupts_rxDone(uint8_t data);
/* #line 49 */
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_RawInterrupts_txDone(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_Init_init(void );
/* # 69 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceQueue.nc" */
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_enqueue(resource_client_id_t id);
/* #line 43 */
static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_isEmpty(void );








static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_dequeue(void );
/* # 43 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceRequested.nc" */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceRequested_default_requested(
/* # 55 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
uint8_t arg_0x1db7c58);
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceRequested.nc" */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceRequested_default_immediateRequested(
/* # 55 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
uint8_t arg_0x1db7c58);
/* # 55 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceConfigure.nc" */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_default_unconfigure(
/* # 60 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
uint8_t arg_0x1db4030);
/* # 49 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceConfigure.nc" */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_default_configure(
/* # 60 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
uint8_t arg_0x1db4030);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc" */
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_release(void );
/* #line 73 */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_default_requested(void );
/* #line 46 */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_default_granted(void );
/* #line 81 */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_default_immediateRequested(void );
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_release(
/* # 54 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
uint8_t arg_0x1db7230);
/* # 87 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_immediateRequest(
/* # 54 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
uint8_t arg_0x1db7230);
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_request(
/* # 54 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
uint8_t arg_0x1db7230);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_default_granted(
/* # 54 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
uint8_t arg_0x1db7230);
/* # 118 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_isOwner(
/* # 54 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
uint8_t arg_0x1db7230);
/* # 80 "/Users/doina/tinyos-2.x/tos/interfaces/ArbiterInfo.nc" */
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ArbiterInfo_inUse(void );







static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ArbiterInfo_userId(void );
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_grantedTask_runTask(void );
/* # 7 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc" */
static void HplMsp430I2C0P_HplI2C_clearModeI2C(void );
/* #line 6 */
static bool HplMsp430I2C0P_HplI2C_isI2C(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/system/ActiveMessageAddressC.nc" */
static am_addr_t ActiveMessageAddressC_amAddress(void );
/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc" */
static am_addr_t ActiveMessageAddressC_ActiveMessageAddress_amAddress(void );




static am_group_t ActiveMessageAddressC_ActiveMessageAddress_amGroup(void );
/* # 66 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
static void CC2420TransmitP_RadioBackoff_setCongestionBackoff(uint16_t backoffTime);
/* #line 60 */
static void CC2420TransmitP_RadioBackoff_setInitialBackoff(uint16_t backoffTime);
/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/GpioCapture.nc" */
static void CC2420TransmitP_CaptureSFD_captured(uint16_t time);
/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void CC2420TransmitP_BackoffTimer_fired(void );
/* # 63 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc" */
static void CC2420TransmitP_CC2420Receive_receive(uint8_t type, message_t * message);
/* # 51 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc" */
static error_t CC2420TransmitP_Send_send(message_t * p_msg, bool useCca);
/* # 24 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc" */
static void CC2420TransmitP_ChipSpiResource_releasing(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t CC2420TransmitP_Init_init(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void CC2420TransmitP_SpiResource_granted(void );
/* # 74 "/Users/doina/tinyos-2.x/tos/interfaces/StdControl.nc" */
static error_t CC2420TransmitP_StdControl_start(void );









static error_t CC2420TransmitP_StdControl_stop(void );
/* # 91 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
static void CC2420TransmitP_TXFIFO_writeDone(uint8_t * data, uint8_t length, error_t error);
/* #line 71 */
static void CC2420TransmitP_TXFIFO_readDone(uint8_t * data, uint8_t length, error_t error);
/* # 53 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc" */
static void CC2420ReceiveP_CC2420Config_syncDone(error_t error);
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void CC2420ReceiveP_receiveDone_task_runTask(void );
/* # 55 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc" */
static void CC2420ReceiveP_CC2420Receive_sfd_dropped(void );
/* #line 49 */
static void CC2420ReceiveP_CC2420Receive_sfd(uint32_t time);
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t CC2420ReceiveP_Init_init(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void CC2420ReceiveP_SpiResource_granted(void );
/* # 91 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
static void CC2420ReceiveP_RXFIFO_writeDone(uint8_t * data, uint8_t length, error_t error);
/* #line 71 */
static void CC2420ReceiveP_RXFIFO_readDone(uint8_t * data, uint8_t length, error_t error);
/* # 57 "/Users/doina/tinyos-2.x/tos/interfaces/GpioInterrupt.nc" */
static void CC2420ReceiveP_InterruptFIFOP_fired(void );
/* # 74 "/Users/doina/tinyos-2.x/tos/interfaces/StdControl.nc" */
static error_t CC2420ReceiveP_StdControl_start(void );









static error_t CC2420ReceiveP_StdControl_stop(void );
/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc" */
static void CC2420PacketP_PacketTimeStamp32khz_clear(
/* #line 55 */
message_t * msg);
/* #line 67 */
static void CC2420PacketP_PacketTimeStamp32khz_set(
/* #line 62 */
message_t * msg, 




CC2420PacketP_PacketTimeStamp32khz_size_type value);
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
static cc2420_header_t * CC2420PacketP_CC2420PacketBody_getHeader(message_t * msg);




static cc2420_metadata_t * CC2420PacketP_CC2420PacketBody_getMetadata(message_t * msg);
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc" */
static uint8_t CC2420PacketP_PacketTimeSyncOffset_get(
/* #line 42 */
message_t * msg);
/* #line 39 */
static bool CC2420PacketP_PacketTimeSyncOffset_isSet(
/* #line 35 */
message_t * msg);
/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC_0_Counter_overflow(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_fired(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Timer_overflow(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_size_type dt);
/* #line 62 */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_stop(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Init_init(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static void /*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_overflow(void );
/* #line 53 */
static /*CounterMilli32C.Transform*/TransformCounterC_1_Counter_size_type /*CounterMilli32C.Transform*/TransformCounterC_1_Counter_get(void );
/* # 98 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_getNow(void );
/* #line 92 */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_size_type dt);
/* #line 105 */
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_getAlarm(void );
/* #line 62 */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_fired(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Counter_overflow(void );
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
/* #line 125 */
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_getNow(
/* # 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
uint8_t arg_0x20a3dd8);
/* # 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_default_fired(
/* # 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
uint8_t arg_0x20a3dd8);
/* # 140 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_getdt(
/* # 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
uint8_t arg_0x20a3dd8);
/* # 133 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_gett0(
/* # 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
uint8_t arg_0x20a3dd8);
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startPeriodic(
/* # 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
uint8_t arg_0x20a3dd8, 
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
uint32_t dt);








static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startOneShot(
/* # 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
uint8_t arg_0x20a3dd8, 
/* # 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_stop(
/* # 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
uint8_t arg_0x20a3dd8);
/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_1_Counter_overflow(void );
/* # 41 "/Users/doina/tinyos-2.x/tos/interfaces/Random.nc" */
static uint16_t RandomMlcgC_Random_rand16(void );
/* #line 35 */
static uint32_t RandomMlcgC_Random_rand32(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t RandomMlcgC_Init_init(void );
/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static void UniqueSendP_SubSend_sendDone(
/* #line 85 */
message_t * msg, 



error_t error);
/* #line 64 */
static error_t UniqueSendP_Send_send(
/* #line 56 */
message_t * msg, 







uint8_t len);
/* #line 114 */
static 
/* #line 112 */
void * 

UniqueSendP_Send_getPayload(
/* #line 111 */
message_t * msg, 


uint8_t len);
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t UniqueSendP_Init_init(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
static 
/* #line 63 */
message_t * 



UniqueReceiveP_SubReceive_receive(
/* #line 60 */
message_t * msg, 
void * payload, 





uint8_t len);
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t UniqueReceiveP_Init_init(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
static 
/* #line 63 */
message_t * 



UniqueReceiveP_DuplicateReceive_default_receive(
/* #line 60 */
message_t * msg, 
void * payload, 





uint8_t len);
/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static void CC2420TinyosNetworkP_SubSend_sendDone(
/* #line 85 */
message_t * msg, 



error_t error);
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
static 
/* #line 63 */
message_t * 



CC2420TinyosNetworkP_SubReceive_receive(
/* #line 60 */
message_t * msg, 
void * payload, 





uint8_t len);
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static error_t CC2420TinyosNetworkP_Send_send(
/* #line 56 */
message_t * msg, 







uint8_t len);
/* #line 114 */
static 
/* #line 112 */
void * 

CC2420TinyosNetworkP_Send_getPayload(
/* #line 111 */
message_t * msg, 


uint8_t len);
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
static 
/* #line 63 */
message_t * 



CC2420TinyosNetworkP_NonTinyosReceive_default_receive(
/* # 48 "/Users/doina/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc" */
uint8_t arg_0x218d9b8, 
/* # 60 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
message_t * msg, 
void * payload, 





uint8_t len);
/* #line 67 */
static 
/* #line 63 */
message_t * 



DisseminationEngineImplP_ProbeReceive_receive(
/* #line 60 */
message_t * msg, 
void * payload, 





uint8_t len);
/* # 99 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
static void DisseminationEngineImplP_ProbeAMSend_sendDone(
/* #line 92 */
message_t * msg, 






error_t error);
/* #line 99 */
static void DisseminationEngineImplP_AMSend_sendDone(
/* #line 92 */
message_t * msg, 






error_t error);
/* # 82 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
static void DisseminationEngineImplP_TrickleTimer_fired(
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4c10);
/* # 77 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
static void DisseminationEngineImplP_TrickleTimer_default_incrementCounter(
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4c10);
/* # 72 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
static void DisseminationEngineImplP_TrickleTimer_default_reset(
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4c10);
/* # 60 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
static error_t DisseminationEngineImplP_TrickleTimer_default_start(
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4c10);
/* # 48 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
static void DisseminationEngineImplP_DisseminationCache_default_storeData(
/* # 49 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4120, 
/* # 48 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
void * data, uint8_t size, uint32_t seqno);

static void DisseminationEngineImplP_DisseminationCache_newData(
/* # 49 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4120);
/* # 45 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
static error_t DisseminationEngineImplP_DisseminationCache_start(
/* # 49 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4120);
/* # 49 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
static uint32_t DisseminationEngineImplP_DisseminationCache_default_requestSeqno(
/* # 49 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4120);
/* # 47 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
static void *DisseminationEngineImplP_DisseminationCache_default_requestData(
/* # 49 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4120, 
/* # 47 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
uint8_t *size);
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
static 
/* #line 63 */
message_t * 



DisseminationEngineImplP_Receive_receive(
/* #line 60 */
message_t * msg, 
void * payload, 





uint8_t len);
/* # 74 "/Users/doina/tinyos-2.x/tos/interfaces/StdControl.nc" */
static error_t DisseminationEngineImplP_StdControl_start(void );
/* #line 74 */
static error_t DisseminationEngineImplP_DisseminatorControl_default_start(
/* # 51 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c2738);
/* # 69 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
static error_t /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_send(am_addr_t addr, 
/* #line 60 */
message_t * msg, 








uint8_t len);
/* #line 124 */
static 
/* #line 122 */
void * 

/*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_getPayload(
/* #line 121 */
message_t * msg, 


uint8_t len);
/* #line 112 */
static uint8_t /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_maxPayloadLength(void );
/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static void /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_Send_sendDone(
/* #line 85 */
message_t * msg, 



error_t error);
/* # 99 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMSend_sendDone(
/* # 40 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
am_id_t arg_0x2246108, 
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
message_t * msg, 






error_t error);
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_send(
/* # 38 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
uint8_t arg_0x22476f8, 
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
message_t * msg, 







uint8_t len);
/* #line 114 */
static 
/* #line 112 */
void * 

/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_getPayload(
/* # 38 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
uint8_t arg_0x22476f8, 
/* # 111 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
message_t * msg, 


uint8_t len);
/* #line 101 */
static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_maxPayloadLength(
/* # 38 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
uint8_t arg_0x22476f8);
/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_default_sendDone(
/* # 38 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
uint8_t arg_0x22476f8, 
/* # 85 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
message_t * msg, 



error_t error);
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_errorTask_runTask(void );
/* #line 64 */
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_CancelTask_runTask(void );
/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static void /*DisseminationEngineP.DisseminationProbeSendC.AMQueueEntryP*/AMQueueEntryP_1_Send_sendDone(
/* #line 85 */
message_t * msg, 



error_t error);
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_changedTask_runTask(void );
/* # 47 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
static void */*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_requestData(uint8_t *size);
static void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_storeData(void * data, uint8_t size, uint32_t seqno);
static uint32_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_requestSeqno(void );
/* # 52 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationUpdate.nc" */
static void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationUpdate_change(/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationUpdate_t * newVal);
/* # 47 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationValue.nc" */
static const /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_t */*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_get(void );








static void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_set(const /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_t *arg_0x14ad7a8);
/* # 74 "/Users/doina/tinyos-2.x/tos/interfaces/StdControl.nc" */
static error_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_StdControl_start(void );
/* # 82 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_default_fired(
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimerImplP.nc" */
uint8_t arg_0x22e4c20);
/* # 77 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_incrementCounter(
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimerImplP.nc" */
uint8_t arg_0x22e4c20);
/* # 72 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_reset(
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimerImplP.nc" */
uint8_t arg_0x22e4c20);
/* # 60 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
static error_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_start(
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimerImplP.nc" */
uint8_t arg_0x22e4c20);
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Init_init(void );
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_timerTask_runTask(void );
/* # 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_fired(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/interfaces/BitVector.nc" */
static void /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_clearAll(void );
/* #line 58 */
static void /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_clear(uint16_t bitnum);
/* #line 46 */
static bool /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_get(uint16_t bitnum);





static void /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_set(uint16_t bitnum);
/* #line 34 */
static void /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_clearAll(void );
/* #line 58 */
static void /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_clear(uint16_t bitnum);
/* #line 46 */
static bool /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_get(uint16_t bitnum);





static void /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_set(uint16_t bitnum);
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_changedTask_runTask(void );
/* # 47 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
static void */*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_requestData(uint8_t *size);
static void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_storeData(void * data, uint8_t size, uint32_t seqno);
static uint32_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_requestSeqno(void );
/* # 52 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationUpdate.nc" */
static void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationUpdate_change(/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationUpdate_t * newVal);
/* # 47 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationValue.nc" */
static const /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_t */*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_get(void );








static void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_set(const /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_t *arg_0x14ad7a8);
/* # 74 "/Users/doina/tinyos-2.x/tos/interfaces/StdControl.nc" */
static error_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_StdControl_start(void );
/* # 83 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
static error_t TestDisseminationC_RadioControl_start(void );
/* # 74 "/Users/doina/tinyos-2.x/tos/interfaces/StdControl.nc" */
static error_t TestDisseminationC_DisseminationControl_start(void );
/* # 52 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationUpdate.nc" */
static void TestDisseminationC_Update32_change(TestDisseminationC_Update32_t * newVal);
/* # 47 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationValue.nc" */
static const TestDisseminationC_Value32_t *TestDisseminationC_Value32_get(void );








static void TestDisseminationC_Value32_set(const TestDisseminationC_Value32_t *arg_0x14ad7a8);
/* #line 47 */
static const TestDisseminationC_Value16_t *TestDisseminationC_Value16_get(void );








static void TestDisseminationC_Value16_set(const TestDisseminationC_Value16_t *arg_0x14ad7a8);
/* # 52 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationUpdate.nc" */
static void TestDisseminationC_Update16_change(TestDisseminationC_Update16_t * newVal);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
static void TestDisseminationC_Leds_led0Toggle(void );
/* #line 72 */
static void TestDisseminationC_Leds_led1Toggle(void );
/* #line 89 */
static void TestDisseminationC_Leds_led2Toggle(void );
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void TestDisseminationC_Timer_startPeriodic(uint32_t dt);
/* # 68 "TestDisseminationC.nc" */
static inline void TestDisseminationC_Boot_booted(void );









static inline void TestDisseminationC_RadioControl_startDone(error_t result);
/* #line 96 */
static inline void TestDisseminationC_RadioControl_stopDone(error_t result);

static inline void TestDisseminationC_Timer_fired(void );
/* #line 116 */
static void TestDisseminationC_Value32_changed(void );










static void TestDisseminationC_Value16_changed(void );
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

enum Msp430ClockP___nesc_unnamed4302 {

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
uint8_t arg_0x1593680);
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_overflow(void );
/* # 115 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_VectorTimerX0_fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_VectorTimerX1_fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Overflow_fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Event_default_fired(uint8_t n);
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_fired(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
uint8_t arg_0x1593680);
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


static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_int2CC(uint16_t x)  ;
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Control_getControl(void );
/* #line 139 */
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_getEvent(void );
/* #line 169 */
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Event_fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Capture_default_captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Compare_default_fired(void );



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Timer_overflow(void );
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_captured(uint16_t time);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Compare_fired(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_int2CC(uint16_t x)  ;
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Control_getControl(void );
/* #line 139 */
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_getEvent(void );
/* #line 169 */
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Event_fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Capture_default_captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Compare_default_fired(void );



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

static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_int2CC(uint16_t x)  ;
/* #line 61 */
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_captureControl(uint8_t l_cm);
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_clearPendingInterrupt(void );
/* #line 99 */
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_setControlAsCapture(uint8_t cm);
/* #line 119 */
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_disableEvents(void );
/* #line 139 */
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_getEvent(void );
/* #line 164 */
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_clearOverflow(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Event_fired(void );
/* #line 181 */
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

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_CC2int(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_compareControl(void );
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_setControlAsCompare(void );
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
enum SchedulerBasicP___nesc_unnamed4303 {

  SchedulerBasicP_NUM_TASKS = 16U, 
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
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static error_t CC2420ActiveMessageP_SubSend_send(
/* #line 56 */
message_t * msg, 







uint8_t len);
/* #line 114 */
static 
/* #line 112 */
void * 

CC2420ActiveMessageP_SubSend_getPayload(
/* #line 111 */
message_t * msg, 


uint8_t len);
/* # 70 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc" */
static uint16_t CC2420ActiveMessageP_CC2420Config_getPanAddr(void );
/* # 95 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
static void CC2420ActiveMessageP_RadioBackoff_requestCca(
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16fd2e8, 
/* # 95 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
message_t * msg);
/* #line 81 */
static void CC2420ActiveMessageP_RadioBackoff_requestInitialBackoff(
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16fd2e8, 
/* # 81 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
message_t * msg);






static void CC2420ActiveMessageP_RadioBackoff_requestCongestionBackoff(
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16fd2e8, 
/* # 88 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
message_t * msg);
/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/SendNotifier.nc" */
static void CC2420ActiveMessageP_SendNotifier_aboutToSend(
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16fec58, 
/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/SendNotifier.nc" */
am_addr_t dest, 
/* #line 57 */
message_t * msg);
/* # 99 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
static void CC2420ActiveMessageP_AMSend_sendDone(
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16e11b8, 
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
message_t * msg, 






error_t error);
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
static 
/* #line 63 */
message_t * 



CC2420ActiveMessageP_Snoop_receive(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16df220, 
/* # 60 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
message_t * msg, 
void * payload, 





uint8_t len);
/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc" */
static am_addr_t CC2420ActiveMessageP_ActiveMessageAddress_amAddress(void );
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
static cc2420_header_t * CC2420ActiveMessageP_CC2420PacketBody_getHeader(message_t * msg);




static cc2420_metadata_t * CC2420ActiveMessageP_CC2420PacketBody_getMetadata(message_t * msg);
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
static 
/* #line 63 */
message_t * 



CC2420ActiveMessageP_Receive_receive(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
am_id_t arg_0x16e1b78, 
/* # 60 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
message_t * msg, 
void * payload, 





uint8_t len);
/* # 61 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static error_t CC2420ActiveMessageP_AMSend_send(am_id_t id, am_addr_t addr, 
message_t *msg, 
uint8_t len);
/* #line 84 */
static inline uint8_t CC2420ActiveMessageP_AMSend_maxPayloadLength(am_id_t id);



static inline void *CC2420ActiveMessageP_AMSend_getPayload(am_id_t id, message_t *m, uint8_t len);




static inline am_addr_t CC2420ActiveMessageP_AMPacket_address(void );



static am_addr_t CC2420ActiveMessageP_AMPacket_destination(message_t *amsg);









static inline void CC2420ActiveMessageP_AMPacket_setDestination(message_t *amsg, am_addr_t addr);









static inline bool CC2420ActiveMessageP_AMPacket_isForMe(message_t *amsg);




static am_id_t CC2420ActiveMessageP_AMPacket_type(message_t *amsg);




static inline void CC2420ActiveMessageP_AMPacket_setType(message_t *amsg, am_id_t type);
/* #line 152 */
static inline uint8_t CC2420ActiveMessageP_Packet_payloadLength(message_t *msg);



static inline void CC2420ActiveMessageP_Packet_setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t CC2420ActiveMessageP_Packet_maxPayloadLength(void );



static inline void *CC2420ActiveMessageP_Packet_getPayload(message_t *msg, uint8_t len);





static inline void CC2420ActiveMessageP_SubSend_sendDone(message_t *msg, error_t result);





static inline message_t *CC2420ActiveMessageP_SubReceive_receive(message_t *msg, void *payload, uint8_t len);
/* #line 196 */
static inline void CC2420ActiveMessageP_CC2420Config_syncDone(error_t error);





static inline void CC2420ActiveMessageP_SubBackoff_requestInitialBackoff(message_t *msg);




static inline void CC2420ActiveMessageP_SubBackoff_requestCongestionBackoff(message_t *msg);



static inline void CC2420ActiveMessageP_SubBackoff_requestCca(message_t *msg);
/* #line 242 */
static inline message_t *CC2420ActiveMessageP_Receive_default_receive(am_id_t id, message_t *msg, void *payload, uint8_t len);



static inline message_t *CC2420ActiveMessageP_Snoop_default_receive(am_id_t id, message_t *msg, void *payload, uint8_t len);






static inline void CC2420ActiveMessageP_SendNotifier_default_aboutToSend(am_id_t amId, am_addr_t addr, message_t *msg);

static inline void CC2420ActiveMessageP_RadioBackoff_default_requestInitialBackoff(am_id_t id, 
message_t *msg);


static inline void CC2420ActiveMessageP_RadioBackoff_default_requestCongestionBackoff(am_id_t id, 
message_t *msg);


static inline void CC2420ActiveMessageP_RadioBackoff_default_requestCca(am_id_t id, 
message_t *msg);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
static void CC2420CsmaP_SplitControl_startDone(error_t error);
/* #line 117 */
static void CC2420CsmaP_SplitControl_stopDone(error_t error);
/* # 95 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
static void CC2420CsmaP_RadioBackoff_requestCca(message_t * msg);
/* #line 81 */
static void CC2420CsmaP_RadioBackoff_requestInitialBackoff(message_t * msg);






static void CC2420CsmaP_RadioBackoff_requestCongestionBackoff(message_t * msg);
/* #line 66 */
static void CC2420CsmaP_SubBackoff_setCongestionBackoff(uint16_t backoffTime);
/* #line 60 */
static void CC2420CsmaP_SubBackoff_setInitialBackoff(uint16_t backoffTime);
/* # 51 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc" */
static error_t CC2420CsmaP_CC2420Transmit_send(message_t * p_msg, bool useCca);
/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static void CC2420CsmaP_Send_sendDone(
/* #line 85 */
message_t * msg, 



error_t error);
/* # 41 "/Users/doina/tinyos-2.x/tos/interfaces/Random.nc" */
static uint16_t CC2420CsmaP_Random_rand16(void );
/* # 74 "/Users/doina/tinyos-2.x/tos/interfaces/StdControl.nc" */
static error_t CC2420CsmaP_SubControl_start(void );









static error_t CC2420CsmaP_SubControl_stop(void );
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
static cc2420_header_t * CC2420CsmaP_CC2420PacketBody_getHeader(message_t * msg);




static cc2420_metadata_t * CC2420CsmaP_CC2420PacketBody_getMetadata(message_t * msg);
/* # 71 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc" */
static error_t CC2420CsmaP_CC2420Power_startOscillator(void );
/* #line 90 */
static error_t CC2420CsmaP_CC2420Power_rxOn(void );
/* #line 51 */
static error_t CC2420CsmaP_CC2420Power_startVReg(void );
/* #line 63 */
static error_t CC2420CsmaP_CC2420Power_stopVReg(void );
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t CC2420CsmaP_Resource_release(void );
/* #line 78 */
static error_t CC2420CsmaP_Resource_request(void );
/* # 66 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
static bool CC2420CsmaP_SplitControlState_isState(uint8_t myState);
/* #line 45 */
static error_t CC2420CsmaP_SplitControlState_requestState(uint8_t reqState);





static void CC2420CsmaP_SplitControlState_forceState(uint8_t reqState);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t CC2420CsmaP_sendDone_task_postTask(void );
/* #line 56 */
static error_t CC2420CsmaP_stopDone_task_postTask(void );
/* #line 56 */
static error_t CC2420CsmaP_startDone_task_postTask(void );
/* # 74 "/Users/doina/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc" */
enum CC2420CsmaP___nesc_unnamed4304 {
/* #line 74 */
  CC2420CsmaP_startDone_task = 0U
};
/* #line 74 */
typedef int CC2420CsmaP___nesc_sillytask_startDone_task[CC2420CsmaP_startDone_task];
enum CC2420CsmaP___nesc_unnamed4305 {
/* #line 75 */
  CC2420CsmaP_stopDone_task = 1U
};
/* #line 75 */
typedef int CC2420CsmaP___nesc_sillytask_stopDone_task[CC2420CsmaP_stopDone_task];
enum CC2420CsmaP___nesc_unnamed4306 {
/* #line 76 */
  CC2420CsmaP_sendDone_task = 2U
};
/* #line 76 */
typedef int CC2420CsmaP___nesc_sillytask_sendDone_task[CC2420CsmaP_sendDone_task];
/* #line 58 */
enum CC2420CsmaP___nesc_unnamed4307 {
  CC2420CsmaP_S_STOPPED, 
  CC2420CsmaP_S_STARTING, 
  CC2420CsmaP_S_STARTED, 
  CC2420CsmaP_S_STOPPING, 
  CC2420CsmaP_S_TRANSMITTING
};

message_t * CC2420CsmaP_m_msg;

error_t CC2420CsmaP_sendErr = SUCCESS;


bool CC2420CsmaP_ccaOn;






static inline void CC2420CsmaP_shutdown(void );


static error_t CC2420CsmaP_SplitControl_start(void );
/* #line 122 */
static inline error_t CC2420CsmaP_Send_send(message_t *p_msg, uint8_t len);
/* #line 157 */
static void *CC2420CsmaP_Send_getPayload(message_t *m, uint8_t len);








static inline uint8_t CC2420CsmaP_Send_maxPayloadLength(void );
/* #line 198 */
static inline void CC2420CsmaP_CC2420Transmit_sendDone(message_t *p_msg, error_t err);




static inline void CC2420CsmaP_CC2420Power_startVRegDone(void );



static inline void CC2420CsmaP_Resource_granted(void );



static inline void CC2420CsmaP_CC2420Power_startOscillatorDone(void );




static inline void CC2420CsmaP_SubBackoff_requestInitialBackoff(message_t *msg);






static inline void CC2420CsmaP_SubBackoff_requestCongestionBackoff(message_t *msg);
/* #line 237 */
static inline void CC2420CsmaP_sendDone_task_runTask(void );
/* #line 250 */
static inline void CC2420CsmaP_startDone_task_runTask(void );







static inline void CC2420CsmaP_stopDone_task_runTask(void );









static inline void CC2420CsmaP_shutdown(void );
/* # 53 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc" */
static void CC2420ControlP_CC2420Config_syncDone(error_t error);
/* # 55 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc" */
static cc2420_status_t CC2420ControlP_RXCTRL1_write(uint16_t data);
/* # 55 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void CC2420ControlP_StartupTimer_start(CC2420ControlP_StartupTimer_size_type dt);
/* # 55 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc" */
static cc2420_status_t CC2420ControlP_MDMCTRL0_write(uint16_t data);
/* # 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
static void CC2420ControlP_RSTN_makeOutput(void );
/* #line 29 */
static void CC2420ControlP_RSTN_set(void );
static void CC2420ControlP_RSTN_clr(void );
/* # 63 "/Users/doina/tinyos-2.x/tos/interfaces/Read.nc" */
static void CC2420ControlP_ReadRssi_readDone(error_t result, CC2420ControlP_ReadRssi_val_t val);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t CC2420ControlP_syncDone_postTask(void );
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc" */
static cc2420_status_t CC2420ControlP_RSSI_read(uint16_t *data);







static cc2420_status_t CC2420ControlP_IOCFG0_write(uint16_t data);
/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc" */
static am_addr_t CC2420ControlP_ActiveMessageAddress_amAddress(void );




static am_group_t CC2420ControlP_ActiveMessageAddress_amGroup(void );
/* # 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
static void CC2420ControlP_CSN_makeOutput(void );
/* #line 29 */
static void CC2420ControlP_CSN_set(void );
static void CC2420ControlP_CSN_clr(void );




static void CC2420ControlP_VREN_makeOutput(void );
/* #line 29 */
static void CC2420ControlP_VREN_set(void );
static void CC2420ControlP_VREN_clr(void );
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
static cc2420_status_t CC2420ControlP_SXOSCON_strobe(void );
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t CC2420ControlP_SpiResource_release(void );
/* #line 78 */
static error_t CC2420ControlP_SpiResource_request(void );
/* #line 110 */
static error_t CC2420ControlP_SyncResource_release(void );
/* #line 78 */
static error_t CC2420ControlP_SyncResource_request(void );
/* # 76 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc" */
static void CC2420ControlP_CC2420Power_startOscillatorDone(void );
/* #line 56 */
static void CC2420ControlP_CC2420Power_startVRegDone(void );
/* # 55 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc" */
static cc2420_status_t CC2420ControlP_IOCFG1_write(uint16_t data);
/* #line 55 */
static cc2420_status_t CC2420ControlP_FSCTRL_write(uint16_t data);
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
static cc2420_status_t CC2420ControlP_SRXON_strobe(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void CC2420ControlP_Resource_granted(void );
/* # 63 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc" */
static cc2420_status_t CC2420ControlP_PANID_write(uint8_t offset, uint8_t * data, uint8_t length);
/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/GpioInterrupt.nc" */
static error_t CC2420ControlP_InterruptCCA_disable(void );
/* #line 42 */
static error_t CC2420ControlP_InterruptCCA_enableRisingEdge(void );
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t CC2420ControlP_RssiResource_release(void );
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
static cc2420_status_t CC2420ControlP_SRFOFF_strobe(void );
/* # 117 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
enum CC2420ControlP___nesc_unnamed4308 {
/* #line 117 */
  CC2420ControlP_sync = 3U
};
/* #line 117 */
typedef int CC2420ControlP___nesc_sillytask_sync[CC2420ControlP_sync];
enum CC2420ControlP___nesc_unnamed4309 {
/* #line 118 */
  CC2420ControlP_syncDone = 4U
};
/* #line 118 */
typedef int CC2420ControlP___nesc_sillytask_syncDone[CC2420ControlP_syncDone];
/* #line 85 */
/* #line 79 */
typedef enum CC2420ControlP___nesc_unnamed4310 {
  CC2420ControlP_S_VREG_STOPPED, 
  CC2420ControlP_S_VREG_STARTING, 
  CC2420ControlP_S_VREG_STARTED, 
  CC2420ControlP_S_XOSC_STARTING, 
  CC2420ControlP_S_XOSC_STARTED
} CC2420ControlP_cc2420_control_state_t;

uint8_t CC2420ControlP_m_channel;

uint8_t CC2420ControlP_m_tx_power;

uint16_t CC2420ControlP_m_pan;

uint16_t CC2420ControlP_m_short_addr;

bool CC2420ControlP_m_sync_busy;


bool CC2420ControlP_autoAckEnabled;


bool CC2420ControlP_hwAutoAckDefault;


bool CC2420ControlP_addressRecognition;


bool CC2420ControlP_hwAddressRecognition;

CC2420ControlP_cc2420_control_state_t CC2420ControlP_m_state = CC2420ControlP_S_VREG_STOPPED;



static void CC2420ControlP_writeFsctrl(void );
static void CC2420ControlP_writeMdmctrl0(void );
static void CC2420ControlP_writeId(void );





static inline error_t CC2420ControlP_Init_init(void );
/* #line 171 */
static inline error_t CC2420ControlP_Resource_request(void );







static inline error_t CC2420ControlP_Resource_release(void );







static inline error_t CC2420ControlP_CC2420Power_startVReg(void );
/* #line 199 */
static inline error_t CC2420ControlP_CC2420Power_stopVReg(void );







static inline error_t CC2420ControlP_CC2420Power_startOscillator(void );
/* #line 249 */
static inline error_t CC2420ControlP_CC2420Power_rxOn(void );
/* #line 279 */
static uint16_t CC2420ControlP_CC2420Config_getShortAddr(void );







static inline uint16_t CC2420ControlP_CC2420Config_getPanAddr(void );
/* #line 300 */
static inline error_t CC2420ControlP_CC2420Config_sync(void );
/* #line 332 */
static inline bool CC2420ControlP_CC2420Config_isAddressRecognitionEnabled(void );
/* #line 359 */
static inline bool CC2420ControlP_CC2420Config_isHwAutoAckDefault(void );






static inline bool CC2420ControlP_CC2420Config_isAutoAckEnabled(void );









static inline void CC2420ControlP_SyncResource_granted(void );
/* #line 390 */
static inline void CC2420ControlP_SpiResource_granted(void );




static inline void CC2420ControlP_RssiResource_granted(void );
/* #line 408 */
static inline void CC2420ControlP_StartupTimer_fired(void );









static inline void CC2420ControlP_InterruptCCA_fired(void );
/* #line 442 */
static inline void CC2420ControlP_sync_runTask(void );



static inline void CC2420ControlP_syncDone_runTask(void );









static void CC2420ControlP_writeFsctrl(void );
/* #line 473 */
static void CC2420ControlP_writeMdmctrl0(void );
/* #line 492 */
static void CC2420ControlP_writeId(void );
/* #line 509 */
static inline void CC2420ControlP_ReadRssi_default_readDone(error_t error, uint16_t data);
/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_setEvent(uint16_t time);

static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_setEventFromNow(uint16_t delta);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Timer_get(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_fired(void );
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_enableEvents(void );
/* #line 36 */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_setControlAsCompare(void );










static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_disableEvents(void );
/* #line 33 */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_clearPendingInterrupt(void );
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Init_init(void );
/* #line 54 */
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_stop(void );




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_fired(void );










static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_startAt(uint16_t t0, uint16_t dt);
/* #line 103 */
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Timer_overflow(void );
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
static /*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_size_type /*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_get(void );






static bool /*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_isOverflowPending(void );










static void /*Counter32khz32C.Transform*/TransformCounterC_0_Counter_overflow(void );
/* # 56 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformCounterC.nc" */
/*Counter32khz32C.Transform*/TransformCounterC_0_upper_count_type /*Counter32khz32C.Transform*/TransformCounterC_0_m_upper;

enum /*Counter32khz32C.Transform*/TransformCounterC_0___nesc_unnamed4311 {

  TransformCounterC_0_LOW_SHIFT_RIGHT = 0, 
  TransformCounterC_0_HIGH_SHIFT_LEFT = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC_0_from_size_type ) - /*Counter32khz32C.Transform*/TransformCounterC_0_LOW_SHIFT_RIGHT, 
  TransformCounterC_0_NUM_UPPER_BITS = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC_0_to_size_type ) - 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC_0_from_size_type ) + 0, 



  TransformCounterC_0_OVERFLOW_MASK = /*Counter32khz32C.Transform*/TransformCounterC_0_NUM_UPPER_BITS ? ((/*Counter32khz32C.Transform*/TransformCounterC_0_upper_count_type )2 << (/*Counter32khz32C.Transform*/TransformCounterC_0_NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*Counter32khz32C.Transform*/TransformCounterC_0_to_size_type /*Counter32khz32C.Transform*/TransformCounterC_0_Counter_get(void );
/* #line 122 */
static inline void /*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_overflow(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_fired(void );
/* #line 92 */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_size_type dt);
/* #line 62 */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_stop(void );
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Counter_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Counter_get(void );
/* # 66 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_t0;
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_dt;

enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0___nesc_unnamed4312 {

  TransformAlarmC_0_MAX_DELAY_LOG2 = 8 * sizeof(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_from_size_type ) - 1 - 0, 
  TransformAlarmC_0_MAX_DELAY = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type )1 << /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_MAX_DELAY_LOG2
};

static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_getNow(void );
/* #line 91 */
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_stop(void );




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_set_alarm(void );
/* #line 136 */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type dt);









static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type dt);




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_fired(void );
/* #line 166 */
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Counter_overflow(void );
/* # 48 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP_0_IO_getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP_0_IO_get(void );
/* #line 48 */
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP_3_IO_getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP_3_IO_get(void );
/* #line 48 */
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP_4_IO_getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP_4_IO_get(void );
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP_4_IO_makeInput(void );



static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP_17_IO_selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP_17_IO_selectIOFunc(void );
/* #line 54 */
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP_18_IO_selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP_18_IO_selectIOFunc(void );
/* #line 54 */
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP_19_IO_selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP_19_IO_selectIOFunc(void );
/* #line 56 */
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP_20_IO_selectIOFunc(void );
/* #line 56 */
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP_21_IO_selectIOFunc(void );
/* #line 48 */
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_get(void );
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_makeInput(void );



static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_selectIOFunc(void );
/* #line 45 */
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP_26_IO_set(void );
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP_26_IO_clr(void );





static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP_26_IO_makeOutput(void );
/* #line 45 */
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP_29_IO_set(void );
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP_29_IO_clr(void );





static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP_29_IO_makeOutput(void );
/* #line 45 */
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP_30_IO_set(void );
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP_30_IO_clr(void );





static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP_30_IO_makeOutput(void );
/* #line 45 */
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_set(void );

static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_toggle(void );




static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_makeOutput(void );
/* #line 45 */
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_set(void );

static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_toggle(void );




static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_makeOutput(void );
/* #line 45 */
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_set(void );

static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_toggle(void );




static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_makeOutput(void );
/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC_0_HplGeneralIO_makeInput(void );
/* #line 59 */
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC_0_HplGeneralIO_get(void );
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC_0_GeneralIO_get(void );
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC_0_GeneralIO_makeInput(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_HplGeneralIO_makeOutput(void );
/* #line 34 */
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_HplGeneralIO_set(void );




static void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_HplGeneralIO_clr(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_set(void );
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_clr(void );




static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_makeOutput(void );
/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC_2_HplGeneralIO_get(void );
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC_2_GeneralIO_get(void );
/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC_3_HplGeneralIO_get(void );
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC_3_GeneralIO_get(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_HplGeneralIO_makeOutput(void );
/* #line 34 */
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_HplGeneralIO_set(void );




static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_HplGeneralIO_clr(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_GeneralIO_set(void );
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_GeneralIO_clr(void );




static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_GeneralIO_makeOutput(void );
/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*HplCC2420PinsC.SFDM*/Msp430GpioC_5_HplGeneralIO_makeInput(void );
/* #line 59 */
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC_5_HplGeneralIO_get(void );
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC_5_GeneralIO_get(void );
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC_5_GeneralIO_makeInput(void );
/* # 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_HplGeneralIO_makeOutput(void );
/* #line 34 */
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_HplGeneralIO_set(void );




static void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_HplGeneralIO_clr(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_GeneralIO_set(void );
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_GeneralIO_clr(void );




static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_GeneralIO_makeOutput(void );
/* # 57 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430Capture_clearOverflow(void );
/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/GpioCapture.nc" */
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_captured(uint16_t time);
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430TimerControl_setControlAsCapture(uint8_t cm);

static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430TimerControl_enableEvents(void );
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430TimerControl_disableEvents(void );
/* #line 33 */
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430TimerControl_clearPendingInterrupt(void );
/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_GeneralIO_selectIOFunc(void );
/* #line 78 */
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_GeneralIO_selectModuleFunc(void );
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc" */
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_enableCapture(uint8_t mode);
/* #line 50 */
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_captureRisingEdge(void );



static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_captureFallingEdge(void );



static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_disable(void );






static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430Capture_captured(uint16_t time);
/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
static void HplMsp430InterruptP_Port14_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port26_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port17_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port21_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port12_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port24_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port15_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port27_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port10_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port22_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port13_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port25_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port16_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port20_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port11_fired(void );
/* #line 61 */
static void HplMsp430InterruptP_Port23_fired(void );
/* # 53 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
void sig_PORT1_VECTOR(void )   ;
/* #line 68 */
static inline void HplMsp430InterruptP_Port11_default_fired(void );
static inline void HplMsp430InterruptP_Port12_default_fired(void );
static inline void HplMsp430InterruptP_Port13_default_fired(void );

static inline void HplMsp430InterruptP_Port15_default_fired(void );
static inline void HplMsp430InterruptP_Port16_default_fired(void );
static inline void HplMsp430InterruptP_Port17_default_fired(void );
static inline void HplMsp430InterruptP_Port10_enable(void );



static inline void HplMsp430InterruptP_Port14_enable(void );



static inline void HplMsp430InterruptP_Port10_disable(void );



static inline void HplMsp430InterruptP_Port14_disable(void );



static inline void HplMsp430InterruptP_Port10_clear(void );
static inline void HplMsp430InterruptP_Port11_clear(void );
static inline void HplMsp430InterruptP_Port12_clear(void );
static inline void HplMsp430InterruptP_Port13_clear(void );
static inline void HplMsp430InterruptP_Port14_clear(void );
static inline void HplMsp430InterruptP_Port15_clear(void );
static inline void HplMsp430InterruptP_Port16_clear(void );
static inline void HplMsp430InterruptP_Port17_clear(void );








static inline void HplMsp430InterruptP_Port10_edge(bool l2h);
/* #line 131 */
static inline void HplMsp430InterruptP_Port14_edge(bool l2h);
/* #line 158 */
void sig_PORT2_VECTOR(void )   ;
/* #line 171 */
static inline void HplMsp430InterruptP_Port20_default_fired(void );
static inline void HplMsp430InterruptP_Port21_default_fired(void );
static inline void HplMsp430InterruptP_Port22_default_fired(void );
static inline void HplMsp430InterruptP_Port23_default_fired(void );
static inline void HplMsp430InterruptP_Port24_default_fired(void );
static inline void HplMsp430InterruptP_Port25_default_fired(void );
static inline void HplMsp430InterruptP_Port26_default_fired(void );
static inline void HplMsp430InterruptP_Port27_default_fired(void );
/* #line 195 */
static inline void HplMsp430InterruptP_Port20_clear(void );
static inline void HplMsp430InterruptP_Port21_clear(void );
static inline void HplMsp430InterruptP_Port22_clear(void );
static inline void HplMsp430InterruptP_Port23_clear(void );
static inline void HplMsp430InterruptP_Port24_clear(void );
static inline void HplMsp430InterruptP_Port25_clear(void );
static inline void HplMsp430InterruptP_Port26_clear(void );
static inline void HplMsp430InterruptP_Port27_clear(void );
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_clear(void );
/* #line 36 */
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_disable(void );
/* #line 56 */
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_edge(bool low_to_high);
/* #line 31 */
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_enable(void );
/* # 57 "/Users/doina/tinyos-2.x/tos/interfaces/GpioInterrupt.nc" */
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_Interrupt_fired(void );
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc" */
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_enable(bool rising);








static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_Interrupt_enableRisingEdge(void );







static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_Interrupt_disable(void );







static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_fired(void );
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_clear(void );
/* #line 36 */
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_disable(void );
/* #line 56 */
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_edge(bool low_to_high);
/* #line 31 */
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_enable(void );
/* # 57 "/Users/doina/tinyos-2.x/tos/interfaces/GpioInterrupt.nc" */
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_Interrupt_fired(void );
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc" */
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_enable(bool rising);
/* #line 54 */
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_Interrupt_enableFallingEdge(void );



static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_Interrupt_disable(void );







static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_fired(void );
/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/SpiPacket.nc" */
static error_t CC2420SpiP_SpiPacket_send(
/* #line 48 */
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
/* # 91 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
static void CC2420SpiP_Fifo_writeDone(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b79698, 
/* # 91 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
uint8_t * data, uint8_t length, error_t error);
/* #line 71 */
static void CC2420SpiP_Fifo_readDone(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b79698, 
/* # 71 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
uint8_t * data, uint8_t length, error_t error);
/* # 24 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc" */
static void CC2420SpiP_ChipSpiResource_releasing(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/interfaces/SpiByte.nc" */
static uint8_t CC2420SpiP_SpiByte_write(uint8_t tx);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
static void CC2420SpiP_WorkingState_toIdle(void );




static bool CC2420SpiP_WorkingState_isIdle(void );
/* #line 45 */
static error_t CC2420SpiP_WorkingState_requestState(uint8_t reqState);
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t CC2420SpiP_SpiResource_release(void );
/* #line 87 */
static error_t CC2420SpiP_SpiResource_immediateRequest(void );
/* #line 78 */
static error_t CC2420SpiP_SpiResource_request(void );
/* #line 118 */
static bool CC2420SpiP_SpiResource_isOwner(void );
/* #line 92 */
static void CC2420SpiP_Resource_granted(
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
uint8_t arg_0x1b7abd0);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t CC2420SpiP_grant_postTask(void );
/* # 88 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
enum CC2420SpiP___nesc_unnamed4313 {
/* #line 88 */
  CC2420SpiP_grant = 5U
};
/* #line 88 */
typedef int CC2420SpiP___nesc_sillytask_grant[CC2420SpiP_grant];
/* #line 63 */
enum CC2420SpiP___nesc_unnamed4314 {
  CC2420SpiP_RESOURCE_COUNT = 5U, 
  CC2420SpiP_NO_HOLDER = 0xFF
};


enum CC2420SpiP___nesc_unnamed4315 {
  CC2420SpiP_S_IDLE, 
  CC2420SpiP_S_BUSY
};


uint16_t CC2420SpiP_m_addr;


uint8_t CC2420SpiP_m_requests = 0;


uint8_t CC2420SpiP_m_holder = CC2420SpiP_NO_HOLDER;


bool CC2420SpiP_release;


static error_t CC2420SpiP_attemptRelease(void );







static inline void CC2420SpiP_ChipSpiResource_abortRelease(void );






static inline error_t CC2420SpiP_ChipSpiResource_attemptRelease(void );




static error_t CC2420SpiP_Resource_request(uint8_t id);
/* #line 126 */
static error_t CC2420SpiP_Resource_immediateRequest(uint8_t id);
/* #line 149 */
static error_t CC2420SpiP_Resource_release(uint8_t id);
/* #line 178 */
static inline uint8_t CC2420SpiP_Resource_isOwner(uint8_t id);





static inline void CC2420SpiP_SpiResource_granted(void );




static cc2420_status_t CC2420SpiP_Fifo_beginRead(uint8_t addr, uint8_t *data, 
uint8_t len);
/* #line 209 */
static inline error_t CC2420SpiP_Fifo_continueRead(uint8_t addr, uint8_t *data, 
uint8_t len);



static inline cc2420_status_t CC2420SpiP_Fifo_write(uint8_t addr, uint8_t *data, 
uint8_t len);
/* #line 260 */
static cc2420_status_t CC2420SpiP_Ram_write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len);
/* #line 287 */
static inline cc2420_status_t CC2420SpiP_Reg_read(uint8_t addr, uint16_t *data);
/* #line 305 */
static cc2420_status_t CC2420SpiP_Reg_write(uint8_t addr, uint16_t data);
/* #line 318 */
static cc2420_status_t CC2420SpiP_Strobe_strobe(uint8_t addr);










static void CC2420SpiP_SpiPacket_sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error);








static error_t CC2420SpiP_attemptRelease(void );
/* #line 358 */
static inline void CC2420SpiP_grant_runTask(void );








static inline void CC2420SpiP_Resource_default_granted(uint8_t id);


static inline void CC2420SpiP_Fifo_default_readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error);


static inline void CC2420SpiP_Fifo_default_writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error);
/* # 74 "/Users/doina/tinyos-2.x/tos/system/StateImplP.nc" */
uint8_t StateImplP_state[4U];

enum StateImplP___nesc_unnamed4316 {
  StateImplP_S_IDLE = 0
};


static inline error_t StateImplP_Init_init(void );
/* #line 96 */
static error_t StateImplP_State_requestState(uint8_t id, uint8_t reqState);
/* #line 111 */
static inline void StateImplP_State_forceState(uint8_t id, uint8_t reqState);






static inline void StateImplP_State_toIdle(uint8_t id);







static inline bool StateImplP_State_isIdle(uint8_t id);






static bool StateImplP_State_isState(uint8_t id, uint8_t myState);
/* # 71 "/Users/doina/tinyos-2.x/tos/interfaces/SpiPacket.nc" */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiPacket_sendDone(
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c67740, 
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/SpiPacket.nc" */
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiConfigure.nc" */
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Msp430SpiConfigure_getConfig(
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c66a60);
/* # 180 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc" */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_enableRxIntr(void );
/* #line 197 */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_clrRxIntr(void );
/* #line 97 */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_resetUsart(bool reset);
/* #line 177 */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_disableRxIntr(void );
/* #line 224 */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_tx(uint8_t data);
/* #line 168 */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_setModeSpi(msp430_spi_union_config_t *config);
/* #line 231 */
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_rx(void );
/* #line 192 */
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_isRxIntrPending(void );
/* #line 158 */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_disableSpi(void );
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_release(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c66010);
/* # 87 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_immediateRequest(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c66010);
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_request(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c66010);
/* # 118 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_isOwner(
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c66010);
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_granted(
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
uint8_t arg_0x1c691b8);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone_task_postTask(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0___nesc_unnamed4317 {
/* #line 67 */
  Msp430SpiNoDmaP_0_signalDone_task = 6U
};
/* #line 67 */
typedef int /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0___nesc_sillytask_signalDone_task[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone_task];
/* #line 56 */
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0___nesc_unnamed4318 {
  Msp430SpiNoDmaP_0_SPI_ATOMIC_SIZE = 2
};

uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_len;
uint8_t * /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_tx_buf;
uint8_t * /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_rx_buf;
uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_pos;
uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_client;

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone(void );


static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_immediateRequest(uint8_t id);



static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_request(uint8_t id);



static inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_isOwner(uint8_t id);



static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_release(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_ResourceConfigure_configure(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_ResourceConfigure_unconfigure(uint8_t id);





static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_granted(uint8_t id);



static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiByte_write(uint8_t tx);
/* #line 111 */
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_isOwner(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_request(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_immediateRequest(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_release(uint8_t id);
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Msp430SpiConfigure_default_getConfig(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_default_granted(uint8_t id);

static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_continueOp(void );
/* #line 144 */
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiPacket_send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len);
/* #line 166 */
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone_task_runTask(void );



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartInterrupts_rxDone(uint8_t data);
/* #line 183 */
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone(void );




static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartInterrupts_txDone(void );

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiPacket_default_sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void HplMsp430Usart0P_UCLK_selectIOFunc(void );
/* #line 78 */
static void HplMsp430Usart0P_UCLK_selectModuleFunc(void );
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc" */
static void HplMsp430Usart0P_Interrupts_rxDone(uint8_t data);
/* #line 49 */
static void HplMsp430Usart0P_Interrupts_txDone(void );
/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void HplMsp430Usart0P_URXD_selectIOFunc(void );
/* #line 85 */
static void HplMsp430Usart0P_UTXD_selectIOFunc(void );
/* # 7 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc" */
static void HplMsp430Usart0P_HplI2C_clearModeI2C(void );
/* #line 6 */
static bool HplMsp430Usart0P_HplI2C_isI2C(void );
/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void HplMsp430Usart0P_SOMI_selectIOFunc(void );
/* #line 78 */
static void HplMsp430Usart0P_SOMI_selectModuleFunc(void );
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc" */
static void HplMsp430Usart0P_I2CInterrupts_fired(void );
/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void HplMsp430Usart0P_SIMO_selectIOFunc(void );
/* #line 78 */
static void HplMsp430Usart0P_SIMO_selectModuleFunc(void );
/* # 89 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc" */
static volatile uint8_t HplMsp430Usart0P_IE1 __asm ("0x0000");
static volatile uint8_t HplMsp430Usart0P_ME1 __asm ("0x0004");
static volatile uint8_t HplMsp430Usart0P_IFG1 __asm ("0x0002");
static volatile uint8_t HplMsp430Usart0P_U0TCTL __asm ("0x0071");

static volatile uint8_t HplMsp430Usart0P_U0TXBUF __asm ("0x0077");

void sig_UART0RX_VECTOR(void )   ;




void sig_UART0TX_VECTOR(void )   ;
/* #line 132 */
static inline void HplMsp430Usart0P_Usart_setUbr(uint16_t control);










static inline void HplMsp430Usart0P_Usart_setUmctl(uint8_t control);







static inline void HplMsp430Usart0P_Usart_resetUsart(bool reset);
/* #line 207 */
static inline void HplMsp430Usart0P_Usart_disableUart(void );
/* #line 238 */
static inline void HplMsp430Usart0P_Usart_enableSpi(void );








static void HplMsp430Usart0P_Usart_disableSpi(void );








static inline void HplMsp430Usart0P_configSpi(msp430_spi_union_config_t *config);








static void HplMsp430Usart0P_Usart_setModeSpi(msp430_spi_union_config_t *config);
/* #line 330 */
static inline bool HplMsp430Usart0P_Usart_isRxIntrPending(void );










static inline void HplMsp430Usart0P_Usart_clrRxIntr(void );



static inline void HplMsp430Usart0P_Usart_clrIntr(void );



static inline void HplMsp430Usart0P_Usart_disableRxIntr(void );







static inline void HplMsp430Usart0P_Usart_disableIntr(void );



static inline void HplMsp430Usart0P_Usart_enableRxIntr(void );
/* #line 382 */
static inline void HplMsp430Usart0P_Usart_tx(uint8_t data);



static uint8_t HplMsp430Usart0P_Usart_rx(void );
/* # 31 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
static void LedsP_Led0_toggle(void );



static void LedsP_Led0_makeOutput(void );
/* #line 29 */
static void LedsP_Led0_set(void );

static void LedsP_Led1_toggle(void );



static void LedsP_Led1_makeOutput(void );
/* #line 29 */
static void LedsP_Led1_set(void );

static void LedsP_Led2_toggle(void );



static void LedsP_Led2_makeOutput(void );
/* #line 29 */
static void LedsP_Led2_set(void );
/* # 45 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc" */
static inline error_t LedsP_Init_init(void );
/* #line 73 */
static inline void LedsP_Leds_led0Toggle(void );
/* #line 88 */
static inline void LedsP_Leds_led1Toggle(void );
/* #line 103 */
static inline void LedsP_Leds_led2Toggle(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_HplGeneralIO_toggle(void );
/* #line 71 */
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_HplGeneralIO_makeOutput(void );
/* #line 34 */
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_HplGeneralIO_set(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_GeneralIO_set(void );

static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_GeneralIO_toggle(void );



static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_GeneralIO_makeOutput(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_HplGeneralIO_toggle(void );
/* #line 71 */
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_HplGeneralIO_makeOutput(void );
/* #line 34 */
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_HplGeneralIO_set(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_GeneralIO_set(void );

static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_GeneralIO_toggle(void );



static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_GeneralIO_makeOutput(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_HplGeneralIO_toggle(void );
/* #line 71 */
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_HplGeneralIO_makeOutput(void );
/* #line 34 */
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_HplGeneralIO_set(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_GeneralIO_set(void );

static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_GeneralIO_toggle(void );



static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_GeneralIO_makeOutput(void );
/* # 80 "/Users/doina/tinyos-2.x/tos/interfaces/ArbiterInfo.nc" */
static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_ArbiterInfo_inUse(void );







static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_ArbiterInfo_userId(void );
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc" */
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_Interrupts_rxDone(
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc" */
uint8_t arg_0x1dac1d8, 
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc" */
uint8_t data);
/* #line 49 */
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_Interrupts_txDone(
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc" */
uint8_t arg_0x1dac1d8);
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc" */
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_I2CInterrupts_fired(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc" */
uint8_t arg_0x1daca58);








static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_RawInterrupts_txDone(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_RawInterrupts_rxDone(uint8_t data);




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_RawI2CInterrupts_fired(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_Interrupts_default_txDone(uint8_t id);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_Interrupts_default_rxDone(uint8_t id, uint8_t data);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_I2CInterrupts_default_fired(uint8_t id);
/* # 39 "/Users/doina/tinyos-2.x/tos/system/FcfsResourceQueueC.nc" */
enum /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0___nesc_unnamed4319 {
/* #line 39 */
  FcfsResourceQueueC_0_NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_resQ[1U];
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_NO_ENTRY;
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_NO_ENTRY;

static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_Init_init(void );




static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_isEmpty(void );



static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_dequeue(void );
/* #line 72 */
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_enqueue(resource_client_id_t id);
/* # 43 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceRequested.nc" */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceRequested_requested(
/* # 55 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
uint8_t arg_0x1db7c58);
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceRequested.nc" */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceRequested_immediateRequested(
/* # 55 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
uint8_t arg_0x1db7c58);
/* # 55 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceConfigure.nc" */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_unconfigure(
/* # 60 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
uint8_t arg_0x1db4030);
/* # 49 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceConfigure.nc" */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_configure(
/* # 60 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
uint8_t arg_0x1db4030);
/* # 69 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceQueue.nc" */
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Queue_enqueue(resource_client_id_t id);
/* #line 43 */
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Queue_isEmpty(void );
/* #line 60 */
static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Queue_dequeue(void );
/* # 73 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc" */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_requested(void );
/* #line 46 */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_granted(void );
/* #line 81 */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_immediateRequested(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_granted(
/* # 54 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
uint8_t arg_0x1db7230);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_grantedTask_postTask(void );
/* # 75 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0___nesc_unnamed4320 {
/* #line 75 */
  ArbiterP_0_grantedTask = 7U
};
/* #line 75 */
typedef int /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0___nesc_sillytask_grantedTask[/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_grantedTask];
/* #line 67 */
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0___nesc_unnamed4321 {
/* #line 67 */
  ArbiterP_0_RES_CONTROLLED, ArbiterP_0_RES_GRANTING, ArbiterP_0_RES_IMM_GRANTING, ArbiterP_0_RES_BUSY
};
/* #line 68 */
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0___nesc_unnamed4322 {
/* #line 68 */
  ArbiterP_0_default_owner_id = 1U
};
/* #line 69 */
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0___nesc_unnamed4323 {
/* #line 69 */
  ArbiterP_0_NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_CONTROLLED;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_default_owner_id;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_reqResId;



static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_request(uint8_t id);
/* #line 90 */
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_immediateRequest(uint8_t id);
/* #line 108 */
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_release(uint8_t id);
/* #line 130 */
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_release(void );
/* #line 150 */
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ArbiterInfo_inUse(void );
/* #line 163 */
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ArbiterInfo_userId(void );










static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_isOwner(uint8_t id);
/* #line 187 */
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_grantedTask_runTask(void );
/* #line 199 */
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_default_granted(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceRequested_default_requested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceRequested_default_immediateRequested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_default_granted(void );

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_default_requested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_default_immediateRequested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_default_configure(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_default_unconfigure(uint8_t id);
/* # 97 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc" */
static void HplMsp430I2C0P_HplUsart_resetUsart(bool reset);
/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C0P.nc" */
static volatile uint8_t HplMsp430I2C0P_U0CTL __asm ("0x0070");





static inline bool HplMsp430I2C0P_HplI2C_isI2C(void );



static inline void HplMsp430I2C0P_HplI2C_clearModeI2C(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/system/ActiveMessageAddressC.nc" */
am_addr_t ActiveMessageAddressC_addr = TOS_AM_ADDRESS;


am_group_t ActiveMessageAddressC_group = TOS_AM_GROUP;






static inline am_addr_t ActiveMessageAddressC_ActiveMessageAddress_amAddress(void );
/* #line 82 */
static inline am_group_t ActiveMessageAddressC_ActiveMessageAddress_amGroup(void );
/* #line 95 */
static am_addr_t ActiveMessageAddressC_amAddress(void );
/* # 81 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
static void CC2420TransmitP_RadioBackoff_requestInitialBackoff(message_t * msg);






static void CC2420TransmitP_RadioBackoff_requestCongestionBackoff(message_t * msg);
/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc" */
static void CC2420TransmitP_PacketTimeStamp_clear(
/* #line 55 */
message_t * msg);
/* #line 67 */
static void CC2420TransmitP_PacketTimeStamp_set(
/* #line 62 */
message_t * msg, 




CC2420TransmitP_PacketTimeStamp_size_type value);
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
static cc2420_status_t CC2420TransmitP_STXONCCA_strobe(void );
/* # 43 "/Users/doina/tinyos-2.x/tos/interfaces/GpioCapture.nc" */
static error_t CC2420TransmitP_CaptureSFD_captureFallingEdge(void );
/* #line 55 */
static void CC2420TransmitP_CaptureSFD_disable(void );
/* #line 42 */
static error_t CC2420TransmitP_CaptureSFD_captureRisingEdge(void );
/* # 98 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static CC2420TransmitP_BackoffTimer_size_type CC2420TransmitP_BackoffTimer_getNow(void );
/* #line 55 */
static void CC2420TransmitP_BackoffTimer_start(CC2420TransmitP_BackoffTimer_size_type dt);






static void CC2420TransmitP_BackoffTimer_stop(void );
/* # 63 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc" */
static cc2420_status_t CC2420TransmitP_TXFIFO_RAM_write(uint8_t offset, uint8_t * data, uint8_t length);
/* # 55 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc" */
static cc2420_status_t CC2420TransmitP_TXCTRL_write(uint16_t data);
/* # 55 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc" */
static void CC2420TransmitP_CC2420Receive_sfd_dropped(void );
/* #line 49 */
static void CC2420TransmitP_CC2420Receive_sfd(uint32_t time);
/* # 73 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc" */
static void CC2420TransmitP_Send_sendDone(message_t * p_msg, error_t error);
/* # 31 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc" */
static void CC2420TransmitP_ChipSpiResource_abortRelease(void );







static error_t CC2420TransmitP_ChipSpiResource_attemptRelease(void );
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
static cc2420_status_t CC2420TransmitP_SFLUSHTX_strobe(void );
/* # 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
static void CC2420TransmitP_CSN_makeOutput(void );
/* #line 29 */
static void CC2420TransmitP_CSN_set(void );
static void CC2420TransmitP_CSN_clr(void );
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
static cc2420_header_t * CC2420TransmitP_CC2420PacketBody_getHeader(message_t * msg);




static cc2420_metadata_t * CC2420TransmitP_CC2420PacketBody_getMetadata(message_t * msg);
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc" */
static uint8_t CC2420TransmitP_PacketTimeSyncOffset_get(
/* #line 42 */
message_t * msg);
/* #line 39 */
static bool CC2420TransmitP_PacketTimeSyncOffset_isSet(
/* #line 35 */
message_t * msg);
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t CC2420TransmitP_SpiResource_release(void );
/* #line 87 */
static error_t CC2420TransmitP_SpiResource_immediateRequest(void );
/* #line 78 */
static error_t CC2420TransmitP_SpiResource_request(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
static void CC2420TransmitP_CCA_makeInput(void );
/* #line 32 */
static bool CC2420TransmitP_CCA_get(void );
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
static cc2420_status_t CC2420TransmitP_SNOP_strobe(void );
/* # 33 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
static void CC2420TransmitP_SFD_makeInput(void );
/* #line 32 */
static bool CC2420TransmitP_SFD_get(void );
/* # 82 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
static cc2420_status_t CC2420TransmitP_TXFIFO_write(uint8_t * data, uint8_t length);
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
static cc2420_status_t CC2420TransmitP_STXON_strobe(void );
/* # 90 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
/* #line 80 */
typedef enum CC2420TransmitP___nesc_unnamed4324 {
  CC2420TransmitP_S_STOPPED, 
  CC2420TransmitP_S_STARTED, 
  CC2420TransmitP_S_LOAD, 
  CC2420TransmitP_S_SAMPLE_CCA, 
  CC2420TransmitP_S_BEGIN_TRANSMIT, 
  CC2420TransmitP_S_SFD, 
  CC2420TransmitP_S_EFD, 
  CC2420TransmitP_S_ACK_WAIT, 
  CC2420TransmitP_S_CANCEL
} CC2420TransmitP_cc2420_transmit_state_t;





enum CC2420TransmitP___nesc_unnamed4325 {
  CC2420TransmitP_CC2420_ABORT_PERIOD = 320
};

message_t * CC2420TransmitP_m_msg;

bool CC2420TransmitP_m_cca;

uint8_t CC2420TransmitP_m_tx_power;

CC2420TransmitP_cc2420_transmit_state_t CC2420TransmitP_m_state = CC2420TransmitP_S_STOPPED;

bool CC2420TransmitP_m_receiving = FALSE;

uint16_t CC2420TransmitP_m_prev_time;


bool CC2420TransmitP_sfdHigh;


bool CC2420TransmitP_abortSpiRelease;


int8_t CC2420TransmitP_totalCcaChecks;


uint16_t CC2420TransmitP_myInitialBackoff;


uint16_t CC2420TransmitP_myCongestionBackoff;



static inline error_t CC2420TransmitP_send(message_t * p_msg, bool cca);

static void CC2420TransmitP_loadTXFIFO(void );
static void CC2420TransmitP_attemptSend(void );
static void CC2420TransmitP_congestionBackoff(void );
static error_t CC2420TransmitP_acquireSpiResource(void );
static inline error_t CC2420TransmitP_releaseSpiResource(void );
static void CC2420TransmitP_signalDone(error_t err);



static inline error_t CC2420TransmitP_Init_init(void );







static inline error_t CC2420TransmitP_StdControl_start(void );










static inline error_t CC2420TransmitP_StdControl_stop(void );
/* #line 172 */
static inline error_t CC2420TransmitP_Send_send(message_t * p_msg, bool useCca);
/* #line 223 */
static inline void CC2420TransmitP_RadioBackoff_setInitialBackoff(uint16_t backoffTime);







static inline void CC2420TransmitP_RadioBackoff_setCongestionBackoff(uint16_t backoffTime);







static __inline uint32_t CC2420TransmitP_getTime32(uint16_t time);
/* #line 258 */
static inline void CC2420TransmitP_CaptureSFD_captured(uint16_t time);
/* #line 353 */
static inline void CC2420TransmitP_ChipSpiResource_releasing(void );
/* #line 365 */
static inline void CC2420TransmitP_CC2420Receive_receive(uint8_t type, message_t *ack_msg);
/* #line 393 */
static inline void CC2420TransmitP_SpiResource_granted(void );
/* #line 431 */
static inline void CC2420TransmitP_TXFIFO_writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);
/* #line 463 */
static inline void CC2420TransmitP_TXFIFO_readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);










static inline void CC2420TransmitP_BackoffTimer_fired(void );
/* #line 524 */
static inline error_t CC2420TransmitP_send(message_t * p_msg, bool cca);
/* #line 592 */
static void CC2420TransmitP_attemptSend(void );
/* #line 634 */
static void CC2420TransmitP_congestionBackoff(void );






static error_t CC2420TransmitP_acquireSpiResource(void );







static inline error_t CC2420TransmitP_releaseSpiResource(void );
/* #line 671 */
static void CC2420TransmitP_loadTXFIFO(void );
/* #line 696 */
static void CC2420TransmitP_signalDone(error_t err);
/* # 32 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
static bool CC2420ReceiveP_FIFO_get(void );
/* # 86 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc" */
static bool CC2420ReceiveP_CC2420Config_isAddressRecognitionEnabled(void );
/* #line 110 */
static bool CC2420ReceiveP_CC2420Config_isAutoAckEnabled(void );
/* #line 105 */
static bool CC2420ReceiveP_CC2420Config_isHwAutoAckDefault(void );
/* #line 64 */
static uint16_t CC2420ReceiveP_CC2420Config_getShortAddr(void );
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t CC2420ReceiveP_receiveDone_task_postTask(void );
/* # 32 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
static bool CC2420ReceiveP_FIFOP_get(void );
/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc" */
static void CC2420ReceiveP_PacketTimeStamp_clear(
/* #line 55 */
message_t * msg);
/* #line 67 */
static void CC2420ReceiveP_PacketTimeStamp_set(
/* #line 62 */
message_t * msg, 




CC2420ReceiveP_PacketTimeStamp_size_type value);
/* # 63 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc" */
static void CC2420ReceiveP_CC2420Receive_receive(uint8_t type, message_t * message);
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
static cc2420_status_t CC2420ReceiveP_SACK_strobe(void );
/* # 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
static void CC2420ReceiveP_CSN_set(void );
static void CC2420ReceiveP_CSN_clr(void );
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
static cc2420_header_t * CC2420ReceiveP_CC2420PacketBody_getHeader(message_t * msg);




static cc2420_metadata_t * CC2420ReceiveP_CC2420PacketBody_getMetadata(message_t * msg);
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
static 
/* #line 63 */
message_t * 



CC2420ReceiveP_Receive_receive(
/* #line 60 */
message_t * msg, 
void * payload, 





uint8_t len);
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
static error_t CC2420ReceiveP_SpiResource_release(void );
/* #line 87 */
static error_t CC2420ReceiveP_SpiResource_immediateRequest(void );
/* #line 78 */
static error_t CC2420ReceiveP_SpiResource_request(void );
/* #line 118 */
static bool CC2420ReceiveP_SpiResource_isOwner(void );
/* # 62 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
static error_t CC2420ReceiveP_RXFIFO_continueRead(uint8_t * data, uint8_t length);
/* #line 51 */
static cc2420_status_t CC2420ReceiveP_RXFIFO_beginRead(uint8_t * data, uint8_t length);
/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/GpioInterrupt.nc" */
static error_t CC2420ReceiveP_InterruptFIFOP_disable(void );
/* #line 43 */
static error_t CC2420ReceiveP_InterruptFIFOP_enableFallingEdge(void );
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
static cc2420_status_t CC2420ReceiveP_SFLUSHRX_strobe(void );
/* # 115 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
enum CC2420ReceiveP___nesc_unnamed4326 {
/* #line 115 */
  CC2420ReceiveP_receiveDone_task = 8U
};
/* #line 115 */
typedef int CC2420ReceiveP___nesc_sillytask_receiveDone_task[CC2420ReceiveP_receiveDone_task];
/* #line 76 */
/* #line 70 */
typedef enum CC2420ReceiveP___nesc_unnamed4327 {
  CC2420ReceiveP_S_STOPPED, 
  CC2420ReceiveP_S_STARTED, 
  CC2420ReceiveP_S_RX_LENGTH, 
  CC2420ReceiveP_S_RX_FCF, 
  CC2420ReceiveP_S_RX_PAYLOAD
} CC2420ReceiveP_cc2420_receive_state_t;

enum CC2420ReceiveP___nesc_unnamed4328 {
  CC2420ReceiveP_RXFIFO_SIZE = 128, 
  CC2420ReceiveP_TIMESTAMP_QUEUE_SIZE = 8, 
  CC2420ReceiveP_SACK_HEADER_LENGTH = 7
};

uint32_t CC2420ReceiveP_m_timestamp_queue[CC2420ReceiveP_TIMESTAMP_QUEUE_SIZE];

uint8_t CC2420ReceiveP_m_timestamp_head;

uint8_t CC2420ReceiveP_m_timestamp_size;


uint8_t CC2420ReceiveP_m_missed_packets;


bool CC2420ReceiveP_receivingPacket;


uint8_t CC2420ReceiveP_rxFrameLength;

uint8_t CC2420ReceiveP_m_bytes_left;

message_t * CC2420ReceiveP_m_p_rx_buf;

message_t CC2420ReceiveP_m_rx_buf;

CC2420ReceiveP_cc2420_receive_state_t CC2420ReceiveP_m_state;


static void CC2420ReceiveP_reset_state(void );
static void CC2420ReceiveP_beginReceive(void );
static void CC2420ReceiveP_receive(void );
static void CC2420ReceiveP_waitForNextPacket(void );
static void CC2420ReceiveP_flush(void );
static inline bool CC2420ReceiveP_passesAddressCheck(message_t * msg);




static inline error_t CC2420ReceiveP_Init_init(void );





static inline error_t CC2420ReceiveP_StdControl_start(void );
/* #line 138 */
static inline error_t CC2420ReceiveP_StdControl_stop(void );
/* #line 153 */
static inline void CC2420ReceiveP_CC2420Receive_sfd(uint32_t time);








static inline void CC2420ReceiveP_CC2420Receive_sfd_dropped(void );
/* #line 179 */
static inline void CC2420ReceiveP_InterruptFIFOP_fired(void );










static inline void CC2420ReceiveP_SpiResource_granted(void );








static inline void CC2420ReceiveP_RXFIFO_readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error);
/* #line 331 */
static inline void CC2420ReceiveP_RXFIFO_writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);







static inline void CC2420ReceiveP_receiveDone_task_runTask(void );
/* #line 360 */
static inline void CC2420ReceiveP_CC2420Config_syncDone(error_t error);






static void CC2420ReceiveP_beginReceive(void );
/* #line 385 */
static void CC2420ReceiveP_flush(void );
/* #line 402 */
static void CC2420ReceiveP_receive(void );









static void CC2420ReceiveP_waitForNextPacket(void );
/* #line 450 */
static void CC2420ReceiveP_reset_state(void );










static inline bool CC2420ReceiveP_passesAddressCheck(message_t *msg);
/* # 99 "/Users/doina/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc" */
static inline cc2420_header_t * CC2420PacketP_CC2420PacketBody_getHeader(message_t * msg);



static inline cc2420_metadata_t *CC2420PacketP_CC2420PacketBody_getMetadata(message_t *msg);
/* #line 122 */
static void CC2420PacketP_PacketTimeStamp32khz_clear(message_t *msg);





static inline void CC2420PacketP_PacketTimeStamp32khz_set(message_t *msg, uint32_t value);
/* #line 161 */
static inline bool CC2420PacketP_PacketTimeSyncOffset_isSet(message_t *msg);








static inline uint8_t CC2420PacketP_PacketTimeSyncOffset_get(message_t *msg);
/* # 47 "/Users/doina/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc" */
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC_0_Counter_overflow(void );
/* # 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_setEventFromNow(uint16_t delta);
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Timer_get(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_fired(void );
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_enableEvents(void );
/* #line 36 */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_disableEvents(void );
/* #line 33 */
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_clearPendingInterrupt(void );
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Init_init(void );
/* #line 54 */
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_startAt(uint16_t t0, uint16_t dt);
/* #line 103 */
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Timer_overflow(void );
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static /*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_size_type /*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC_1_Counter_overflow(void );
/* # 56 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformCounterC.nc" */
/*CounterMilli32C.Transform*/TransformCounterC_1_upper_count_type /*CounterMilli32C.Transform*/TransformCounterC_1_m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC_1___nesc_unnamed4329 {

  TransformCounterC_1_LOW_SHIFT_RIGHT = 5, 
  TransformCounterC_1_HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC_1_from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC_1_LOW_SHIFT_RIGHT, 
  TransformCounterC_1_NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC_1_to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC_1_from_size_type ) + 5, 



  TransformCounterC_1_OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC_1_NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC_1_upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC_1_NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC_1_to_size_type /*CounterMilli32C.Transform*/TransformCounterC_1_Counter_get(void );
/* #line 122 */
static inline void /*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_overflow(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_fired(void );
/* #line 92 */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_size_type dt);
/* #line 62 */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_stop(void );
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Counter_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Counter_get(void );
/* # 66 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1___nesc_unnamed4330 {

  TransformAlarmC_1_MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_from_size_type ) - 1 - 5, 
  TransformAlarmC_1_MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_set_alarm(void );
/* #line 136 */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type dt);
/* #line 151 */
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_fired(void );
/* #line 166 */
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Counter_overflow(void );
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
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0___nesc_unnamed4331 {
/* #line 63 */
  AlarmToTimerC_0_fired = 9U
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
uint8_t arg_0x20a3dd8);
/* #line 60 */
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0___nesc_unnamed4332 {
/* #line 60 */
  VirtualizeTimerC_0_updateFromTimer = 10U
};
/* #line 60 */
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0___nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer];
/* #line 42 */
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0___nesc_unnamed4333 {

  VirtualizeTimerC_0_NUM_TIMERS = 3U, 
  VirtualizeTimerC_0_END_OF_LIST = 255
};








/* #line 48 */
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0___nesc_unnamed4334 {

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









static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startPeriodic(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_stop(uint8_t num);
/* #line 178 */
static inline uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_getNow(uint8_t num);




static inline uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_gett0(uint8_t num);




static inline uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_getdt(uint8_t num);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_default_fired(uint8_t num);
/* # 47 "/Users/doina/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc" */
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_1_Counter_overflow(void );
/* # 41 "/Users/doina/tinyos-2.x/tos/system/RandomMlcgC.nc" */
uint32_t RandomMlcgC_seed;


static inline error_t RandomMlcgC_Init_init(void );
/* #line 58 */
static uint32_t RandomMlcgC_Random_rand32(void );
/* #line 78 */
static inline uint16_t RandomMlcgC_Random_rand16(void );
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static error_t UniqueSendP_SubSend_send(
/* #line 56 */
message_t * msg, 







uint8_t len);
/* #line 114 */
static 
/* #line 112 */
void * 

UniqueSendP_SubSend_getPayload(
/* #line 111 */
message_t * msg, 


uint8_t len);
/* #line 89 */
static void UniqueSendP_Send_sendDone(
/* #line 85 */
message_t * msg, 



error_t error);
/* # 41 "/Users/doina/tinyos-2.x/tos/interfaces/Random.nc" */
static uint16_t UniqueSendP_Random_rand16(void );
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
static cc2420_header_t * UniqueSendP_CC2420PacketBody_getHeader(message_t * msg);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
static void UniqueSendP_State_toIdle(void );
/* #line 45 */
static error_t UniqueSendP_State_requestState(uint8_t reqState);
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc" */
uint8_t UniqueSendP_localSendId;

enum UniqueSendP___nesc_unnamed4335 {
  UniqueSendP_S_IDLE, 
  UniqueSendP_S_SENDING
};


static inline error_t UniqueSendP_Init_init(void );
/* #line 75 */
static inline error_t UniqueSendP_Send_send(message_t *msg, uint8_t len);
/* #line 99 */
static inline void *UniqueSendP_Send_getPayload(message_t *msg, uint8_t len);




static inline void UniqueSendP_SubSend_sendDone(message_t *msg, error_t error);
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
static 
/* #line 63 */
message_t * 



UniqueReceiveP_Receive_receive(
/* #line 60 */
message_t * msg, 
void * payload, 





uint8_t len);
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
static cc2420_header_t * UniqueReceiveP_CC2420PacketBody_getHeader(message_t * msg);
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
static 
/* #line 63 */
message_t * 



UniqueReceiveP_DuplicateReceive_receive(
/* #line 60 */
message_t * msg, 
void * payload, 





uint8_t len);
/* # 59 "/Users/doina/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc" */
/* #line 56 */
struct UniqueReceiveP___nesc_unnamed4336 {
  am_addr_t source;
  uint8_t dsn;
} UniqueReceiveP_receivedMessages[4];

uint8_t UniqueReceiveP_writeIndex = 0;


uint8_t UniqueReceiveP_recycleSourceElement;

enum UniqueReceiveP___nesc_unnamed4337 {
  UniqueReceiveP_INVALID_ELEMENT = 0xFF
};


static inline error_t UniqueReceiveP_Init_init(void );









static inline bool UniqueReceiveP_hasSeen(uint16_t msgSource, uint8_t msgDsn);
static inline void UniqueReceiveP_insert(uint16_t msgSource, uint8_t msgDsn);


static inline message_t *UniqueReceiveP_SubReceive_receive(message_t *msg, void *payload, 
uint8_t len);
/* #line 111 */
static inline bool UniqueReceiveP_hasSeen(uint16_t msgSource, uint8_t msgDsn);
/* #line 137 */
static inline void UniqueReceiveP_insert(uint16_t msgSource, uint8_t msgDsn);
/* #line 158 */
static inline message_t *UniqueReceiveP_DuplicateReceive_default_receive(message_t *msg, void *payload, uint8_t len);
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static error_t CC2420TinyosNetworkP_SubSend_send(
/* #line 56 */
message_t * msg, 







uint8_t len);
/* #line 114 */
static 
/* #line 112 */
void * 

CC2420TinyosNetworkP_SubSend_getPayload(
/* #line 111 */
message_t * msg, 


uint8_t len);
/* #line 89 */
static void CC2420TinyosNetworkP_Send_sendDone(
/* #line 85 */
message_t * msg, 



error_t error);
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
static 
/* #line 63 */
message_t * 



CC2420TinyosNetworkP_NonTinyosReceive_receive(
/* # 48 "/Users/doina/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc" */
uint8_t arg_0x218d9b8, 
/* # 60 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
message_t * msg, 
void * payload, 





uint8_t len);
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
static cc2420_header_t * CC2420TinyosNetworkP_CC2420PacketBody_getHeader(message_t * msg);
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
static 
/* #line 63 */
message_t * 



CC2420TinyosNetworkP_Receive_receive(
/* #line 60 */
message_t * msg, 
void * payload, 





uint8_t len);
/* # 61 "/Users/doina/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc" */
static inline error_t CC2420TinyosNetworkP_Send_send(message_t *msg, uint8_t len);
/* #line 74 */
static inline void *CC2420TinyosNetworkP_Send_getPayload(message_t *msg, uint8_t len);




static inline void CC2420TinyosNetworkP_SubSend_sendDone(message_t *msg, error_t error);




static inline message_t *CC2420TinyosNetworkP_SubReceive_receive(message_t *msg, void *payload, uint8_t len);









static inline message_t *CC2420TinyosNetworkP_NonTinyosReceive_default_receive(uint8_t networkId, message_t *msg, void *payload, uint8_t len);
/* # 69 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
static error_t DisseminationEngineImplP_AMSend_send(am_addr_t addr, 
/* #line 60 */
message_t * msg, 








uint8_t len);
/* #line 124 */
static 
/* #line 122 */
void * 

DisseminationEngineImplP_AMSend_getPayload(
/* #line 121 */
message_t * msg, 


uint8_t len);
/* #line 112 */
static uint8_t DisseminationEngineImplP_AMSend_maxPayloadLength(void );
/* # 77 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
static void DisseminationEngineImplP_TrickleTimer_incrementCounter(
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4c10);
/* # 72 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
static void DisseminationEngineImplP_TrickleTimer_reset(
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4c10);
/* # 60 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
static error_t DisseminationEngineImplP_TrickleTimer_start(
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4c10);
/* # 48 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
static void DisseminationEngineImplP_DisseminationCache_storeData(
/* # 49 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4120, 
/* # 48 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
void * data, uint8_t size, uint32_t seqno);
static uint32_t DisseminationEngineImplP_DisseminationCache_requestSeqno(
/* # 49 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4120);
/* # 47 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
static void *DisseminationEngineImplP_DisseminationCache_requestData(
/* # 49 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c4120, 
/* # 47 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
uint8_t *size);
/* # 74 "/Users/doina/tinyos-2.x/tos/interfaces/StdControl.nc" */
static error_t DisseminationEngineImplP_DisseminatorControl_start(
/* # 51 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
uint16_t arg_0x21c2738);
/* #line 64 */
enum DisseminationEngineImplP___nesc_unnamed4338 {
/* #line 64 */
  DisseminationEngineImplP_NUM_DISSEMINATORS = 2U
};
message_t DisseminationEngineImplP_m_buf;
bool DisseminationEngineImplP_m_running;
bool DisseminationEngineImplP_m_bufBusy;


static void DisseminationEngineImplP_sendObject(uint16_t key);

static inline error_t DisseminationEngineImplP_StdControl_start(void );
/* #line 91 */
static error_t DisseminationEngineImplP_DisseminationCache_start(uint16_t key);










static void DisseminationEngineImplP_DisseminationCache_newData(uint16_t key);







static inline void DisseminationEngineImplP_TrickleTimer_fired(uint16_t key);
/* #line 130 */
static void DisseminationEngineImplP_sendObject(uint16_t key);
/* #line 155 */
static inline void DisseminationEngineImplP_ProbeAMSend_sendDone(message_t *msg, error_t error);



static inline void DisseminationEngineImplP_AMSend_sendDone(message_t *msg, error_t error);



static inline message_t *DisseminationEngineImplP_Receive_receive(message_t *msg, 
void *payload, 
uint8_t len);
/* #line 219 */
static inline message_t *DisseminationEngineImplP_ProbeReceive_receive(message_t *msg, 
void *payload, 
uint8_t len);
/* #line 236 */
static inline void *
DisseminationEngineImplP_DisseminationCache_default_requestData(uint16_t key, uint8_t *size);


static inline 
/* #line 239 */
void 
DisseminationEngineImplP_DisseminationCache_default_storeData(uint16_t key, void *data, 
uint8_t size, 
uint32_t seqno);


static inline 
/* #line 244 */
uint32_t 
DisseminationEngineImplP_DisseminationCache_default_requestSeqno(uint16_t key);

static inline error_t DisseminationEngineImplP_TrickleTimer_default_start(uint16_t key);



static inline void DisseminationEngineImplP_TrickleTimer_default_reset(uint16_t key);

static inline void DisseminationEngineImplP_TrickleTimer_default_incrementCounter(uint16_t key);

static inline error_t DisseminationEngineImplP_DisseminatorControl_default_start(uint16_t id);
/* # 99 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
static void /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_sendDone(
/* #line 92 */
message_t * msg, 






error_t error);
/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static error_t /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_Send_send(
/* #line 56 */
message_t * msg, 







uint8_t len);
/* #line 114 */
static 
/* #line 112 */
void * 

/*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_Send_getPayload(
/* #line 111 */
message_t * msg, 


uint8_t len);
/* #line 101 */
static uint8_t /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_Send_maxPayloadLength(void );
/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/AMPacket.nc" */
static void /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMPacket_setDestination(
/* #line 88 */
message_t * amsg, 



am_addr_t addr);
/* #line 151 */
static void /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMPacket_setType(
/* #line 147 */
message_t * amsg, 



am_id_t t);
/* # 45 "/Users/doina/tinyos-2.x/tos/system/AMQueueEntryP.nc" */
static inline error_t /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_Send_sendDone(message_t *m, error_t err);



static inline uint8_t /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_maxPayloadLength(void );



static inline void */*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_getPayload(message_t *m, uint8_t len);
/* # 69 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMSend_send(
/* # 40 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
am_id_t arg_0x2246108, 
/* # 69 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
am_addr_t addr, 
/* #line 60 */
message_t * msg, 








uint8_t len);
/* #line 124 */
static 
/* #line 122 */
void * 

/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMSend_getPayload(
/* # 40 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
am_id_t arg_0x2246108, 
/* # 121 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
message_t * msg, 


uint8_t len);
/* #line 112 */
static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMSend_maxPayloadLength(
/* # 40 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
am_id_t arg_0x2246108);
/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_sendDone(
/* # 38 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
uint8_t arg_0x22476f8, 
/* # 85 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
message_t * msg, 



error_t error);
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Packet.nc" */
static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Packet_payloadLength(
/* #line 63 */
message_t * msg);
/* #line 83 */
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Packet_setPayloadLength(
/* #line 79 */
message_t * msg, 



uint8_t len);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_errorTask_postTask(void );
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/AMPacket.nc" */
static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMPacket_destination(
/* #line 63 */
message_t * amsg);
/* #line 136 */
static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMPacket_type(
/* #line 132 */
message_t * amsg);
/* # 118 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP_0___nesc_unnamed4339 {
/* #line 118 */
  AMQueueImplP_0_CancelTask = 11U
};
/* #line 118 */
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP_0___nesc_sillytask_CancelTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_CancelTask];
/* #line 161 */
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP_0___nesc_unnamed4340 {
/* #line 161 */
  AMQueueImplP_0_errorTask = 12U
};
/* #line 161 */
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP_0___nesc_sillytask_errorTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_errorTask];
/* #line 49 */
/* #line 47 */
typedef struct /*AMQueueP.AMQueueImplP*/AMQueueImplP_0___nesc_unnamed4341 {
  message_t * msg;
} /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_queue_entry_t;

uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current = 2;
/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_queue_entry_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_queue[2];
uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_cancelMask[2 / 8 + 1];

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_tryToSend(void );

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_nextPacket(void );
/* #line 82 */
static inline error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_send(uint8_t clientId, message_t *msg, 
uint8_t len);
/* #line 118 */
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_CancelTask_runTask(void );
/* #line 155 */
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_sendDone(uint8_t last, message_t * msg, error_t err);





static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_errorTask_runTask(void );




static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_tryToSend(void );
/* #line 181 */
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMSend_sendDone(am_id_t id, message_t *msg, error_t err);
/* #line 199 */
static inline uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_maxPayloadLength(uint8_t id);



static inline void */*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_getPayload(uint8_t id, message_t *m, uint8_t len);



static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_default_sendDone(uint8_t id, message_t *msg, error_t err);
/* # 99 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
static void /*DisseminationEngineP.DisseminationProbeSendC.AMQueueEntryP*/AMQueueEntryP_1_AMSend_sendDone(
/* #line 92 */
message_t * msg, 






error_t error);
/* # 57 "/Users/doina/tinyos-2.x/tos/system/AMQueueEntryP.nc" */
static inline void /*DisseminationEngineP.DisseminationProbeSendC.AMQueueEntryP*/AMQueueEntryP_1_Send_sendDone(message_t *m, error_t err);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_changedTask_postTask(void );
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
static void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_newData(void );
/* #line 45 */
static error_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_start(void );
/* # 61 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationValue.nc" */
static void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_changed(void );
/* # 62 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminatorP.nc" */
enum /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0___nesc_unnamed4342 {
/* #line 62 */
  DisseminatorP_0_changedTask = 13U
};
/* #line 62 */
typedef int /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0___nesc_sillytask_changedTask[/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_changedTask];
/* #line 55 */
/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_valueCache;
bool /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_m_running;



uint32_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_seqno = DISSEMINATION_SEQNO_UNKNOWN;

static inline void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_changedTask_runTask(void );



static inline error_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_StdControl_start(void );
/* #line 78 */
static inline const /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_t */*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_get(void );



static inline void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_set(const /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_t *val);





static inline void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationUpdate_change(/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_t *newVal);
/* #line 101 */
static inline void */*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_requestData(uint8_t *size);




static void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_storeData(void *data, uint8_t size, 
uint32_t newSeqno);









static inline uint32_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_requestSeqno(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/interfaces/BitVector.nc" */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Pending_clearAll(void );
/* #line 58 */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Pending_clear(uint16_t bitnum);
/* #line 46 */
static bool /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Pending_get(uint16_t bitnum);





static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Pending_set(uint16_t bitnum);
/* #line 34 */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Changed_clearAll(void );
/* #line 58 */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Changed_clear(uint16_t bitnum);
/* #line 46 */
static bool /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Changed_get(uint16_t bitnum);





static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Changed_set(uint16_t bitnum);
/* # 41 "/Users/doina/tinyos-2.x/tos/interfaces/Random.nc" */
static uint16_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Random_rand16(void );
/* # 82 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_fired(
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimerImplP.nc" */
uint8_t arg_0x22e4c20);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_timerTask_postTask(void );
/* # 125 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static uint32_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_getNow(void );
/* #line 140 */
static uint32_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_getdt(void );
/* #line 133 */
static uint32_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_gett0(void );
/* #line 62 */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_startOneShot(uint32_t dt);




static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_stop(void );
/* # 146 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimerImplP.nc" */
enum /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0___nesc_unnamed4343 {
/* #line 146 */
  TrickleTimerImplP_0_timerTask = 14U
};
/* #line 146 */
typedef int /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0___nesc_sillytask_timerTask[/*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_timerTask];
/* #line 67 */
/* #line 62 */
typedef struct /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0___nesc_unnamed4344 {
  uint16_t period;
  uint32_t time;
  uint32_t remainder;
  uint8_t count;
} /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickle_t;

/*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickle_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[2U];

static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_adjustTimer(void );
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_generateTime(uint8_t id);

static inline error_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Init_init(void );
/* #line 92 */
static error_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_start(uint8_t id);
/* #line 122 */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_reset(uint8_t id);
/* #line 142 */
static inline void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_incrementCounter(uint8_t id);



static inline void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_timerTask_runTask(void );
/* #line 168 */
static inline void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_fired(void );
/* #line 203 */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_adjustTimer(void );
/* #line 246 */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_generateTime(uint8_t id);
/* #line 270 */
static inline void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_default_fired(uint8_t id);
/* # 40 "/Users/doina/tinyos-2.x/tos/system/BitVectorC.nc" */
typedef uint8_t /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_int_type;

enum /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0___nesc_unnamed4345 {

  BitVectorC_0_ELEMENT_SIZE = 8 * sizeof(/*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_int_type ), 
  BitVectorC_0_ARRAY_SIZE = (2U + /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_ELEMENT_SIZE - 1) / /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_ELEMENT_SIZE
};

/*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_int_type /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_m_bits[/*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_ARRAY_SIZE];

static inline uint16_t /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_getIndex(uint16_t bitnum);




static inline uint16_t /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_getMask(uint16_t bitnum);










static inline void /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_clearAll(void );









static inline bool /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_get(uint16_t bitnum);




static inline void /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_set(uint16_t bitnum);




static inline void /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_clear(uint16_t bitnum);
/* #line 40 */
typedef uint8_t /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_int_type;

enum /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1___nesc_unnamed4346 {

  BitVectorC_1_ELEMENT_SIZE = 8 * sizeof(/*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_int_type ), 
  BitVectorC_1_ARRAY_SIZE = (2U + /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_ELEMENT_SIZE - 1) / /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_ELEMENT_SIZE
};

/*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_int_type /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_m_bits[/*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_ARRAY_SIZE];

static inline uint16_t /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_getIndex(uint16_t bitnum);




static inline uint16_t /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_getMask(uint16_t bitnum);










static inline void /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_clearAll(void );









static inline bool /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_get(uint16_t bitnum);




static inline void /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_set(uint16_t bitnum);




static inline void /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_clear(uint16_t bitnum);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
static error_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_changedTask_postTask(void );
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
static void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_newData(void );
/* #line 45 */
static error_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_start(void );
/* # 61 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationValue.nc" */
static void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_changed(void );
/* # 62 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminatorP.nc" */
enum /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1___nesc_unnamed4347 {
/* #line 62 */
  DisseminatorP_1_changedTask = 15U
};
/* #line 62 */
typedef int /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1___nesc_sillytask_changedTask[/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_changedTask];
/* #line 55 */
/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_valueCache;
bool /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_m_running;



uint32_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_seqno = DISSEMINATION_SEQNO_UNKNOWN;

static inline void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_changedTask_runTask(void );



static inline error_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_StdControl_start(void );
/* #line 78 */
static inline const /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_t */*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_get(void );



static inline void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_set(const /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_t *val);





static inline void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationUpdate_change(/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_t *newVal);
/* #line 101 */
static inline void */*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_requestData(uint8_t *size);




static void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_storeData(void *data, uint8_t size, 
uint32_t newSeqno);









static inline uint32_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_requestSeqno(void );
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

/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Timer_overflow(void ){
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
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Event_fired(uint8_t arg_0x1593680){
/* #line 28 */
  switch (arg_0x1593680) {
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
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Event_default_fired(arg_0x1593680);
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
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0___nesc_unnamed4348 {
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

/* #line 181 */
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Compare_default_fired(void )
{
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Compare_fired(void ){
/* #line 34 */
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0_Compare_default_fired();
/* #line 34 */
}
/* #line 34 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_int2CC(uint16_t x)
/* #line 47 */
{
/* #line 47 */
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1___nesc_unnamed4349 {
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

/* #line 181 */
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Compare_default_fired(void )
{
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Compare_fired(void ){
/* #line 34 */
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1_Compare_default_fired();
/* #line 34 */
}
/* #line 34 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2_int2CC(uint16_t x)
/* #line 47 */
{
/* #line 47 */
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2___nesc_unnamed4350 {
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
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Timer_overflow(void )
{
}

/* #line 103 */
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Timer_overflow(void )
{
}

/* # 166 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Counter_overflow(void )
{
}

/* # 47 "/Users/doina/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc" */
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC_0_Counter_overflow(void )
{
}

/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
inline static void /*Counter32khz32C.Transform*/TransformCounterC_0_Counter_overflow(void ){
/* #line 71 */
  /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC_0_Counter_overflow();
/* #line 71 */
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Counter_overflow();
/* #line 71 */
}
/* #line 71 */
/* # 122 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformCounterC.nc" */
static inline void /*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*Counter32khz32C.Transform*/TransformCounterC_0_m_upper++;
    if ((/*Counter32khz32C.Transform*/TransformCounterC_0_m_upper & /*Counter32khz32C.Transform*/TransformCounterC_0_OVERFLOW_MASK) == 0) {
      /*Counter32khz32C.Transform*/TransformCounterC_0_Counter_overflow();
      }
  }
}

/* # 47 "/Users/doina/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc" */
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_1_Counter_overflow(void )
{
}

/* # 166 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Counter_overflow(void )
{
}

/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
inline static void /*CounterMilli32C.Transform*/TransformCounterC_1_Counter_overflow(void ){
/* #line 71 */
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Counter_overflow();
/* #line 71 */
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_1_Counter_overflow();
/* #line 71 */
}
/* #line 71 */
/* # 122 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformCounterC.nc" */
static inline void /*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC_1_m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC_1_m_upper & /*CounterMilli32C.Transform*/TransformCounterC_1_OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC_1_Counter_overflow();
      }
  }
}

/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC_0_Counter_overflow(void ){
/* #line 71 */
  /*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_overflow();
/* #line 71 */
  /*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_overflow();
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
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Timer_overflow();
/* #line 37 */
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Timer_overflow();
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

/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t CC2420ControlP_SpiResource_request(void ){
/* #line 78 */
  unsigned char result;
/* #line 78 */

/* #line 78 */
  result = CC2420SpiP_Resource_request(/*CC2420ControlC.Spi*/CC2420SpiC_0_CLIENT_ID);
/* #line 78 */

/* #line 78 */
  return result;
/* #line 78 */
}
/* #line 78 */
/* # 171 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline error_t CC2420ControlP_Resource_request(void )
/* #line 171 */
{
  return CC2420ControlP_SpiResource_request();
}

/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t CC2420CsmaP_Resource_request(void ){
/* #line 78 */
  unsigned char result;
/* #line 78 */

/* #line 78 */
  result = CC2420ControlP_Resource_request();
/* #line 78 */

/* #line 78 */
  return result;
/* #line 78 */
}
/* #line 78 */
/* # 203 "/Users/doina/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc" */
static inline void CC2420CsmaP_CC2420Power_startVRegDone(void )
/* #line 203 */
{
  CC2420CsmaP_Resource_request();
}

/* # 56 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc" */
inline static void CC2420ControlP_CC2420Power_startVRegDone(void ){
/* #line 56 */
  CC2420CsmaP_CC2420Power_startVRegDone();
/* #line 56 */
}
/* #line 56 */
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_HplGeneralIO_set(void ){
/* #line 34 */
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP_30_IO_set();
/* #line 34 */
}
/* #line 34 */
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_GeneralIO_set(void )
/* #line 37 */
{
/* #line 37 */
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_HplGeneralIO_set();
}

/* # 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420ControlP_RSTN_set(void ){
/* #line 29 */
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_GeneralIO_set();
/* #line 29 */
}
/* #line 29 */
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_HplGeneralIO_clr(void ){
/* #line 39 */
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP_30_IO_clr();
/* #line 39 */
}
/* #line 39 */
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_GeneralIO_clr(void )
/* #line 38 */
{
/* #line 38 */
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_HplGeneralIO_clr();
}

/* # 30 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420ControlP_RSTN_clr(void ){
/* #line 30 */
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_GeneralIO_clr();
/* #line 30 */
}
/* #line 30 */
/* # 408 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline void CC2420ControlP_StartupTimer_fired(void )
/* #line 408 */
{
  if (CC2420ControlP_m_state == CC2420ControlP_S_VREG_STARTING) {
      CC2420ControlP_m_state = CC2420ControlP_S_VREG_STARTED;
      CC2420ControlP_RSTN_clr();
      CC2420ControlP_RSTN_set();
      CC2420ControlP_CC2420Power_startVRegDone();
    }
}

/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t CC2420TransmitP_SpiResource_release(void ){
/* #line 110 */
  unsigned char result;
/* #line 110 */

/* #line 110 */
  result = CC2420SpiP_Resource_release(/*CC2420TransmitC.Spi*/CC2420SpiC_3_CLIENT_ID);
/* #line 110 */

/* #line 110 */
  return result;
/* #line 110 */
}
/* #line 110 */
/* # 649 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static inline error_t CC2420TransmitP_releaseSpiResource(void )
/* #line 649 */
{
  CC2420TransmitP_SpiResource_release();
  return SUCCESS;
}

/* # 50 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc" */
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_captureRisingEdge(void )
/* #line 50 */
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_enableCapture(MSP430TIMER_CM_RISING);
}

/* # 42 "/Users/doina/tinyos-2.x/tos/interfaces/GpioCapture.nc" */
inline static error_t CC2420TransmitP_CaptureSFD_captureRisingEdge(void ){
/* #line 42 */
  unsigned char result;
/* #line 42 */

/* #line 42 */
  result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_captureRisingEdge();
/* #line 42 */

/* #line 42 */
  return result;
/* #line 42 */
}
/* #line 42 */
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
inline static cc2420_status_t CC2420TransmitP_SFLUSHTX_strobe(void ){
/* #line 45 */
  unsigned char result;
/* #line 45 */

/* #line 45 */
  result = CC2420SpiP_Strobe_strobe(CC2420_SFLUSHTX);
/* #line 45 */

/* #line 45 */
  return result;
/* #line 45 */
}
/* #line 45 */
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
inline static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Counter_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Counter_get(void ){
/* #line 53 */
  unsigned long result;
/* #line 53 */

/* #line 53 */
  result = /*Counter32khz32C.Transform*/TransformCounterC_0_Counter_get();
/* #line 53 */

/* #line 53 */
  return result;
/* #line 53 */
}
/* #line 53 */
/* # 75 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_getNow(void )
{
  return /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Counter_get();
}

/* #line 146 */
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type dt)
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_getNow(), dt);
}

/* # 55 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void CC2420TransmitP_BackoffTimer_start(CC2420TransmitP_BackoffTimer_size_type dt){
/* #line 55 */
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_start(dt);
/* #line 55 */
}
/* #line 55 */
/* # 48 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP_4_IO_getRaw(void )
/* #line 48 */
{
/* #line 48 */
  return * (volatile uint8_t * )32U & (0x01 << 4);
}

/* #line 49 */
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP_4_IO_get(void )
/* #line 49 */
{
/* #line 49 */
  return /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP_4_IO_getRaw() != 0;
}

/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC_0_HplGeneralIO_get(void ){
/* #line 59 */
  unsigned char result;
/* #line 59 */

/* #line 59 */
  result = /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP_4_IO_get();
/* #line 59 */

/* #line 59 */
  return result;
/* #line 59 */
}
/* #line 59 */
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC_0_GeneralIO_get(void )
/* #line 40 */
{
/* #line 40 */
  return /*HplCC2420PinsC.CCAM*/Msp430GpioC_0_HplGeneralIO_get();
}

/* # 32 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static bool CC2420TransmitP_CCA_get(void ){
/* #line 32 */
  unsigned char result;
/* #line 32 */

/* #line 32 */
  result = /*HplCC2420PinsC.CCAM*/Msp430GpioC_0_GeneralIO_get();
/* #line 32 */

/* #line 32 */
  return result;
/* #line 32 */
}
/* #line 32 */
/* # 475 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static inline void CC2420TransmitP_BackoffTimer_fired(void )
/* #line 475 */
{
  /* atomic removed: atomic calls only */
/* #line 476 */
  {
    switch (CC2420TransmitP_m_state) {

        case CC2420TransmitP_S_SAMPLE_CCA: 


          if (CC2420TransmitP_CCA_get()) {
              CC2420TransmitP_m_state = CC2420TransmitP_S_BEGIN_TRANSMIT;
              CC2420TransmitP_BackoffTimer_start(CC2420_TIME_ACK_TURNAROUND);
            }
          else {
              CC2420TransmitP_congestionBackoff();
            }
        break;

        case CC2420TransmitP_S_BEGIN_TRANSMIT: 
          case CC2420TransmitP_S_CANCEL: 
            if (CC2420TransmitP_acquireSpiResource() == SUCCESS) {
                CC2420TransmitP_attemptSend();
              }
        break;

        case CC2420TransmitP_S_ACK_WAIT: 
          CC2420TransmitP_signalDone(SUCCESS);
        break;

        case CC2420TransmitP_S_SFD: 


          CC2420TransmitP_SFLUSHTX_strobe();
        CC2420TransmitP_CaptureSFD_captureRisingEdge();
        CC2420TransmitP_releaseSpiResource();
        CC2420TransmitP_signalDone(ERETRY);
        break;

        default: 
          break;
      }
  }
}

/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_fired(void ){
/* #line 67 */
  CC2420TransmitP_BackoffTimer_fired();
/* #line 67 */
  CC2420ControlP_StartupTimer_fired();
/* #line 67 */
}
/* #line 67 */
/* # 151 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_dt == 0) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_fired();
      }
    else 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_set_alarm();
      }
  }
}

/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_fired(void ){
/* #line 67 */
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_fired();
/* #line 67 */
}
/* #line 67 */
/* # 124 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_disableEvents(void ){
/* #line 47 */
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_disableEvents();
/* #line 47 */
}
/* #line 47 */
/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_fired(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_fired();
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Compare_fired(void ){
/* #line 34 */
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_fired();
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
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3___nesc_unnamed4351 {
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
inline static /*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_size_type /*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_get(void ){
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
inline static bool /*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_isOverflowPending(void ){
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
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_enableEvents(void ){
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
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_clearPendingInterrupt(void ){
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
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_setEvent(uint16_t time){
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
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_setEventFromNow(uint16_t delta){
/* #line 32 */
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Compare_setEventFromNow(delta);
/* #line 32 */
}
/* #line 32 */
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Timer_get(void ){
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
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Timer_get();
    uint16_t elapsed = now - t0;

/* #line 76 */
    if (elapsed >= dt) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

/* #line 83 */
        if (remaining <= 2) {
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_setEventFromNow(2);
          }
        else {
/* #line 86 */
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430Compare_setEvent(now + remaining);
          }
      }
/* #line 88 */
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_clearPendingInterrupt();
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_enableEvents();
  }
}

/* # 92 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_size_type dt){
/* #line 92 */
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_startAt(t0, dt);
/* #line 92 */
}
/* #line 92 */
/* # 246 "/opt/local/lib/ncc/nesc_nx.h" */
static __inline  uint8_t __nesc_ntoh_leuint8(const void * source)
/* #line 246 */
{
  const uint8_t *base = source;

/* #line 248 */
  return base[0];
}

/* # 259 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline void CC2420ActiveMessageP_RadioBackoff_default_requestCongestionBackoff(am_id_t id, 
message_t *msg)
/* #line 260 */
{
}

/* # 88 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
inline static void CC2420ActiveMessageP_RadioBackoff_requestCongestionBackoff(am_id_t arg_0x16fd2e8, message_t * msg){
/* #line 88 */
    CC2420ActiveMessageP_RadioBackoff_default_requestCongestionBackoff(arg_0x16fd2e8, msg);
/* #line 88 */
}
/* #line 88 */
/* # 207 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline void CC2420ActiveMessageP_SubBackoff_requestCongestionBackoff(message_t *msg)
/* #line 207 */
{
  CC2420ActiveMessageP_RadioBackoff_requestCongestionBackoff(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.data), msg);
}

/* # 88 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
inline static void CC2420CsmaP_RadioBackoff_requestCongestionBackoff(message_t * msg){
/* #line 88 */
  CC2420ActiveMessageP_SubBackoff_requestCongestionBackoff(msg);
/* #line 88 */
}
/* #line 88 */
/* # 78 "/Users/doina/tinyos-2.x/tos/system/RandomMlcgC.nc" */
static inline uint16_t RandomMlcgC_Random_rand16(void )
/* #line 78 */
{
  return (uint16_t )RandomMlcgC_Random_rand32();
}

/* # 41 "/Users/doina/tinyos-2.x/tos/interfaces/Random.nc" */
inline static uint16_t CC2420CsmaP_Random_rand16(void ){
/* #line 41 */
  unsigned int result;
/* #line 41 */

/* #line 41 */
  result = RandomMlcgC_Random_rand16();
/* #line 41 */

/* #line 41 */
  return result;
/* #line 41 */
}
/* #line 41 */
/* # 231 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static inline void CC2420TransmitP_RadioBackoff_setCongestionBackoff(uint16_t backoffTime)
/* #line 231 */
{
  CC2420TransmitP_myCongestionBackoff = backoffTime + 1;
}

/* # 66 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
inline static void CC2420CsmaP_SubBackoff_setCongestionBackoff(uint16_t backoffTime){
/* #line 66 */
  CC2420TransmitP_RadioBackoff_setCongestionBackoff(backoffTime);
/* #line 66 */
}
/* #line 66 */
/* # 223 "/Users/doina/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc" */
static inline void CC2420CsmaP_SubBackoff_requestCongestionBackoff(message_t *msg)
/* #line 223 */
{
  CC2420CsmaP_SubBackoff_setCongestionBackoff(CC2420CsmaP_Random_rand16()
   % (0x7 * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP_RadioBackoff_requestCongestionBackoff(msg);
}

/* # 88 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
inline static void CC2420TransmitP_RadioBackoff_requestCongestionBackoff(message_t * msg){
/* #line 88 */
  CC2420CsmaP_SubBackoff_requestCongestionBackoff(msg);
/* #line 88 */
}
/* #line 88 */
/* # 87 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t CC2420TransmitP_SpiResource_immediateRequest(void ){
/* #line 87 */
  unsigned char result;
/* #line 87 */

/* #line 87 */
  result = CC2420SpiP_Resource_immediateRequest(/*CC2420TransmitC.Spi*/CC2420SpiC_3_CLIENT_ID);
/* #line 87 */

/* #line 87 */
  return result;
/* #line 87 */
}
/* #line 87 */
/* # 45 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
inline static error_t CC2420SpiP_WorkingState_requestState(uint8_t reqState){
/* #line 45 */
  unsigned char result;
/* #line 45 */

/* #line 45 */
  result = StateImplP_State_requestState(0U, reqState);
/* #line 45 */

/* #line 45 */
  return result;
/* #line 45 */
}
/* #line 45 */
/* # 111 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_isOwner(uint8_t id)
/* #line 111 */
{
/* #line 111 */
  return FAIL;
}

/* # 118 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_isOwner(uint8_t arg_0x1c66010){
/* #line 118 */
  unsigned char result;
/* #line 118 */

/* #line 118 */
  switch (arg_0x1c66010) {
/* #line 118 */
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0_CLIENT_ID:
/* #line 118 */
      result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C_0_CLIENT_ID);
/* #line 118 */
      break;
/* #line 118 */
    default:
/* #line 118 */
      result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_isOwner(arg_0x1c66010);
/* #line 118 */
      break;
/* #line 118 */
    }
/* #line 118 */

/* #line 118 */
  return result;
/* #line 118 */
}
/* #line 118 */
/* # 77 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_isOwner(uint8_t id)
/* #line 77 */
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_isOwner(id);
}

/* # 118 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static bool CC2420SpiP_SpiResource_isOwner(void ){
/* #line 118 */
  unsigned char result;
/* #line 118 */

/* #line 118 */
  result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0_CLIENT_ID);
/* #line 118 */

/* #line 118 */
  return result;
/* #line 118 */
}
/* #line 118 */
/* # 115 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Msp430SpiConfigure_default_getConfig(uint8_t id)
/* #line 115 */
{
  return &msp430_spi_default_config;
}

/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiConfigure.nc" */
inline static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Msp430SpiConfigure_getConfig(uint8_t arg_0x1c66a60){
/* #line 39 */
  union __nesc_unnamed4280 *result;
/* #line 39 */

/* #line 39 */
    result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Msp430SpiConfigure_default_getConfig(arg_0x1c66a60);
/* #line 39 */

/* #line 39 */
  return result;
/* #line 39 */
}
/* #line 39 */
/* # 168 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc" */
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_setModeSpi(msp430_spi_union_config_t *config){
/* #line 168 */
  HplMsp430Usart0P_Usart_setModeSpi(config);
/* #line 168 */
}
/* #line 168 */
/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_ResourceConfigure_configure(uint8_t id)
/* #line 85 */
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_setModeSpi(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Msp430SpiConfigure_getConfig(id));
}

/* # 213 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_default_configure(uint8_t id)
/* #line 213 */
{
}

/* # 49 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceConfigure.nc" */
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_configure(uint8_t arg_0x1db4030){
/* #line 49 */
  switch (arg_0x1db4030) {
/* #line 49 */
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C_0_CLIENT_ID:
/* #line 49 */
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_ResourceConfigure_configure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0_CLIENT_ID);
/* #line 49 */
      break;
/* #line 49 */
    default:
/* #line 49 */
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_default_configure(arg_0x1db4030);
/* #line 49 */
      break;
/* #line 49 */
    }
/* #line 49 */
}
/* #line 49 */
/* # 210 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_default_immediateRequested(void )
/* #line 210 */
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_release();
}

/* # 81 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc" */
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_immediateRequested(void ){
/* #line 81 */
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_default_immediateRequested();
/* #line 81 */
}
/* #line 81 */
/* # 203 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceRequested_default_immediateRequested(uint8_t id)
/* #line 203 */
{
}

/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceRequested.nc" */
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceRequested_immediateRequested(uint8_t arg_0x1db7c58){
/* #line 51 */
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceRequested_default_immediateRequested(arg_0x1db7c58);
/* #line 51 */
}
/* #line 51 */
/* # 90 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_immediateRequest(uint8_t id)
/* #line 90 */
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceRequested_immediateRequested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId);
  /* atomic removed: atomic calls only */
/* #line 92 */
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_CONTROLLED) {
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_IMM_GRANTING;
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
/* #line 97 */
        FAIL;

/* #line 97 */
        return __nesc_temp;
      }
  }
/* #line 99 */
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_immediateRequested();
  if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId == id) {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId);
      return SUCCESS;
    }
  /* atomic removed: atomic calls only */
/* #line 104 */
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_CONTROLLED;
  return FAIL;
}

/* # 113 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_immediateRequest(uint8_t id)
/* #line 113 */
{
/* #line 113 */
  return FAIL;
}

/* # 87 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_immediateRequest(uint8_t arg_0x1c66010){
/* #line 87 */
  unsigned char result;
/* #line 87 */

/* #line 87 */
  switch (arg_0x1c66010) {
/* #line 87 */
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0_CLIENT_ID:
/* #line 87 */
      result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C_0_CLIENT_ID);
/* #line 87 */
      break;
/* #line 87 */
    default:
/* #line 87 */
      result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_immediateRequest(arg_0x1c66010);
/* #line 87 */
      break;
/* #line 87 */
    }
/* #line 87 */

/* #line 87 */
  return result;
/* #line 87 */
}
/* #line 87 */
/* # 69 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_immediateRequest(uint8_t id)
/* #line 69 */
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_immediateRequest(id);
}

/* # 87 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t CC2420SpiP_SpiResource_immediateRequest(void ){
/* #line 87 */
  unsigned char result;
/* #line 87 */

/* #line 87 */
  result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0_CLIENT_ID);
/* #line 87 */

/* #line 87 */
  return result;
/* #line 87 */
}
/* #line 87 */
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

/* # 151 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc" */
static inline void HplMsp430Usart0P_Usart_resetUsart(bool reset)
/* #line 151 */
{
  if (reset) {
      U0CTL = 0x01;
    }
  else {
      U0CTL &= ~0x01;
    }
}

/* # 97 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc" */
inline static void HplMsp430I2C0P_HplUsart_resetUsart(bool reset){
/* #line 97 */
  HplMsp430Usart0P_Usart_resetUsart(reset);
/* #line 97 */
}
/* #line 97 */
/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C0P.nc" */
static inline void HplMsp430I2C0P_HplI2C_clearModeI2C(void )
/* #line 59 */
{
  /* atomic removed: atomic calls only */
/* #line 60 */
  {
    HplMsp430I2C0P_U0CTL &= ~((0x20 | 0x04) | 0x01);
    HplMsp430I2C0P_HplUsart_resetUsart(TRUE);
  }
}

/* # 7 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc" */
inline static void HplMsp430Usart0P_HplI2C_clearModeI2C(void ){
/* #line 7 */
  HplMsp430I2C0P_HplI2C_clearModeI2C();
/* #line 7 */
}
/* #line 7 */
/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP_21_IO_selectIOFunc(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  * (volatile uint8_t * )27U &= ~(0x01 << 5);
}

/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void HplMsp430Usart0P_URXD_selectIOFunc(void ){
/* #line 85 */
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP_21_IO_selectIOFunc();
/* #line 85 */
}
/* #line 85 */
/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP_20_IO_selectIOFunc(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  * (volatile uint8_t * )27U &= ~(0x01 << 4);
}

/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void HplMsp430Usart0P_UTXD_selectIOFunc(void ){
/* #line 85 */
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP_20_IO_selectIOFunc();
/* #line 85 */
}
/* #line 85 */
/* # 207 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc" */
static inline void HplMsp430Usart0P_Usart_disableUart(void )
/* #line 207 */
{
  /* atomic removed: atomic calls only */
/* #line 208 */
  {
    HplMsp430Usart0P_ME1 &= ~((1 << 7) | (1 << 6));
    HplMsp430Usart0P_UTXD_selectIOFunc();
    HplMsp430Usart0P_URXD_selectIOFunc();
  }
}

/* #line 143 */
static inline void HplMsp430Usart0P_Usart_setUmctl(uint8_t control)
/* #line 143 */
{
  U0MCTL = control;
}

/* #line 132 */
static inline void HplMsp430Usart0P_Usart_setUbr(uint16_t control)
/* #line 132 */
{
  /* atomic removed: atomic calls only */
/* #line 133 */
  {
    U0BR0 = control & 0x00FF;
    U0BR1 = (control >> 8) & 0x00FF;
  }
}

/* #line 256 */
static inline void HplMsp430Usart0P_configSpi(msp430_spi_union_config_t *config)
/* #line 256 */
{

  U0CTL = (config->spiRegisters.uctl | 0x04) | 0x01;
  HplMsp430Usart0P_U0TCTL = config->spiRegisters.utctl;

  HplMsp430Usart0P_Usart_setUbr(config->spiRegisters.ubr);
  HplMsp430Usart0P_Usart_setUmctl(0x00);
}

/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP_19_IO_selectModuleFunc(void )
/* #line 54 */
{
  /* atomic removed: atomic calls only */
/* #line 54 */
  * (volatile uint8_t * )27U |= 0x01 << 3;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void HplMsp430Usart0P_UCLK_selectModuleFunc(void ){
/* #line 78 */
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP_19_IO_selectModuleFunc();
/* #line 78 */
}
/* #line 78 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP_18_IO_selectModuleFunc(void )
/* #line 54 */
{
  /* atomic removed: atomic calls only */
/* #line 54 */
  * (volatile uint8_t * )27U |= 0x01 << 2;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void HplMsp430Usart0P_SOMI_selectModuleFunc(void ){
/* #line 78 */
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP_18_IO_selectModuleFunc();
/* #line 78 */
}
/* #line 78 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP_17_IO_selectModuleFunc(void )
/* #line 54 */
{
  /* atomic removed: atomic calls only */
/* #line 54 */
  * (volatile uint8_t * )27U |= 0x01 << 1;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void HplMsp430Usart0P_SIMO_selectModuleFunc(void ){
/* #line 78 */
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP_17_IO_selectModuleFunc();
/* #line 78 */
}
/* #line 78 */
/* # 238 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc" */
static inline void HplMsp430Usart0P_Usart_enableSpi(void )
/* #line 238 */
{
  /* atomic removed: atomic calls only */
/* #line 239 */
  {
    HplMsp430Usart0P_SIMO_selectModuleFunc();
    HplMsp430Usart0P_SOMI_selectModuleFunc();
    HplMsp430Usart0P_UCLK_selectModuleFunc();
  }
  HplMsp430Usart0P_ME1 |= 1 << 6;
}

/* #line 345 */
static inline void HplMsp430Usart0P_Usart_clrIntr(void )
/* #line 345 */
{
  HplMsp430Usart0P_IFG1 &= ~((1 << 7) | (1 << 6));
}









static inline void HplMsp430Usart0P_Usart_disableIntr(void )
/* #line 357 */
{
  HplMsp430Usart0P_IE1 &= ~((1 << 7) | (1 << 6));
}

/* # 118 "/Users/doina/tinyos-2.x/tos/system/StateImplP.nc" */
static inline void StateImplP_State_toIdle(uint8_t id)
/* #line 118 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 119 */
    StateImplP_state[id] = StateImplP_S_IDLE;
/* #line 119 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
inline static void CC2420SpiP_WorkingState_toIdle(void ){
/* #line 56 */
  StateImplP_State_toIdle(0U);
/* #line 56 */
}
/* #line 56 */
/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t CC2420TransmitP_SpiResource_request(void ){
/* #line 78 */
  unsigned char result;
/* #line 78 */

/* #line 78 */
  result = CC2420SpiP_Resource_request(/*CC2420TransmitC.Spi*/CC2420SpiC_3_CLIENT_ID);
/* #line 78 */

/* #line 78 */
  return result;
/* #line 78 */
}
/* #line 78 */
/* # 207 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_default_requested(void )
/* #line 207 */
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_release();
}

/* # 73 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc" */
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_requested(void ){
/* #line 73 */
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_default_requested();
/* #line 73 */
}
/* #line 73 */
/* # 54 "/Users/doina/tinyos-2.x/tos/system/FcfsResourceQueueC.nc" */
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_isEnqueued(resource_client_id_t id)
/* #line 54 */
{
  return /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_resQ[id] != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_NO_ENTRY || /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_qTail == id;
}

/* #line 72 */
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_enqueue(resource_client_id_t id)
/* #line 72 */
{
  /* atomic removed: atomic calls only */
/* #line 73 */
  {
    if (!/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_isEnqueued(id)) {
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_qHead = id;
          }
        else {
/* #line 78 */
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_qTail] = id;
          }
/* #line 79 */
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_qTail = id;
        {
          unsigned char __nesc_temp = 
/* #line 80 */
          SUCCESS;

/* #line 80 */
          return __nesc_temp;
        }
      }
/* #line 82 */
    {
      unsigned char __nesc_temp = 
/* #line 82 */
      EBUSY;

/* #line 82 */
      return __nesc_temp;
    }
  }
}

/* # 69 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceQueue.nc" */
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Queue_enqueue(resource_client_id_t id){
/* #line 69 */
  unsigned char result;
/* #line 69 */

/* #line 69 */
  result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_enqueue(id);
/* #line 69 */

/* #line 69 */
  return result;
/* #line 69 */
}
/* #line 69 */
/* # 201 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceRequested_default_requested(uint8_t id)
/* #line 201 */
{
}

/* # 43 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceRequested.nc" */
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceRequested_requested(uint8_t arg_0x1db7c58){
/* #line 43 */
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceRequested_default_requested(arg_0x1db7c58);
/* #line 43 */
}
/* #line 43 */
/* # 77 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_request(uint8_t id)
/* #line 77 */
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceRequested_requested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId);
  /* atomic removed: atomic calls only */
/* #line 79 */
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_CONTROLLED) {
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_GRANTING;
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
/* #line 84 */
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Queue_enqueue(id);

/* #line 84 */
        return __nesc_temp;
      }
  }
/* #line 86 */
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_requested();
  return SUCCESS;
}

/* # 112 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_request(uint8_t id)
/* #line 112 */
{
/* #line 112 */
  return FAIL;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_request(uint8_t arg_0x1c66010){
/* #line 78 */
  unsigned char result;
/* #line 78 */

/* #line 78 */
  switch (arg_0x1c66010) {
/* #line 78 */
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0_CLIENT_ID:
/* #line 78 */
      result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C_0_CLIENT_ID);
/* #line 78 */
      break;
/* #line 78 */
    default:
/* #line 78 */
      result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_request(arg_0x1c66010);
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
/* # 73 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_request(uint8_t id)
/* #line 73 */
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_request(id);
}

/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t CC2420SpiP_SpiResource_request(void ){
/* #line 78 */
  unsigned char result;
/* #line 78 */

/* #line 78 */
  result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0_CLIENT_ID);
/* #line 78 */

/* #line 78 */
  return result;
/* #line 78 */
}
/* #line 78 */
/* # 382 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc" */
static inline void HplMsp430Usart0P_Usart_tx(uint8_t data)
/* #line 382 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 383 */
    HplMsp430Usart0P_U0TXBUF = data;
/* #line 383 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 224 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc" */
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_tx(uint8_t data){
/* #line 224 */
  HplMsp430Usart0P_Usart_tx(data);
/* #line 224 */
}
/* #line 224 */
/* # 330 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc" */
static inline bool HplMsp430Usart0P_Usart_isRxIntrPending(void )
/* #line 330 */
{
  if (HplMsp430Usart0P_IFG1 & (1 << 6)) {
      return TRUE;
    }
  return FALSE;
}

/* # 192 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc" */
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_isRxIntrPending(void ){
/* #line 192 */
  unsigned char result;
/* #line 192 */

/* #line 192 */
  result = HplMsp430Usart0P_Usart_isRxIntrPending();
/* #line 192 */

/* #line 192 */
  return result;
/* #line 192 */
}
/* #line 192 */
/* # 341 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc" */
static inline void HplMsp430Usart0P_Usart_clrRxIntr(void )
/* #line 341 */
{
  HplMsp430Usart0P_IFG1 &= ~(1 << 6);
}

/* # 197 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc" */
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_clrRxIntr(void ){
/* #line 197 */
  HplMsp430Usart0P_Usart_clrRxIntr();
/* #line 197 */
}
/* #line 197 */
/* #line 231 */
inline static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_rx(void ){
/* #line 231 */
  unsigned char result;
/* #line 231 */

/* #line 231 */
  result = HplMsp430Usart0P_Usart_rx();
/* #line 231 */

/* #line 231 */
  return result;
/* #line 231 */
}
/* #line 231 */
/* # 95 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static inline void CC2420SpiP_ChipSpiResource_abortRelease(void )
/* #line 95 */
{
  /* atomic removed: atomic calls only */
/* #line 96 */
  CC2420SpiP_release = FALSE;
}

/* # 31 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc" */
inline static void CC2420TransmitP_ChipSpiResource_abortRelease(void ){
/* #line 31 */
  CC2420SpiP_ChipSpiResource_abortRelease();
/* #line 31 */
}
/* #line 31 */
/* # 353 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static inline void CC2420TransmitP_ChipSpiResource_releasing(void )
/* #line 353 */
{
  if (CC2420TransmitP_abortSpiRelease) {
      CC2420TransmitP_ChipSpiResource_abortRelease();
    }
}

/* # 24 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc" */
inline static void CC2420SpiP_ChipSpiResource_releasing(void ){
/* #line 24 */
  CC2420TransmitP_ChipSpiResource_releasing();
/* #line 24 */
}
/* #line 24 */
/* # 205 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_default_granted(void )
/* #line 205 */
{
}

/* # 46 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc" */
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_granted(void ){
/* #line 46 */
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_default_granted();
/* #line 46 */
}
/* #line 46 */
/* # 97 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc" */
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_resetUsart(bool reset){
/* #line 97 */
  HplMsp430Usart0P_Usart_resetUsart(reset);
/* #line 97 */
}
/* #line 97 */
/* #line 158 */
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_disableSpi(void ){
/* #line 158 */
  HplMsp430Usart0P_Usart_disableSpi();
/* #line 158 */
}
/* #line 158 */
/* # 89 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_ResourceConfigure_unconfigure(uint8_t id)
/* #line 89 */
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_resetUsart(TRUE);
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_disableSpi();
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_resetUsart(FALSE);
}

/* # 215 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_default_unconfigure(uint8_t id)
/* #line 215 */
{
}

/* # 55 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceConfigure.nc" */
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_unconfigure(uint8_t arg_0x1db4030){
/* #line 55 */
  switch (arg_0x1db4030) {
/* #line 55 */
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C_0_CLIENT_ID:
/* #line 55 */
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_ResourceConfigure_unconfigure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0_CLIENT_ID);
/* #line 55 */
      break;
/* #line 55 */
    default:
/* #line 55 */
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_default_unconfigure(arg_0x1db4030);
/* #line 55 */
      break;
/* #line 55 */
    }
/* #line 55 */
}
/* #line 55 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_grantedTask_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_grantedTask);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 58 "/Users/doina/tinyos-2.x/tos/system/FcfsResourceQueueC.nc" */
static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_dequeue(void )
/* #line 58 */
{
  /* atomic removed: atomic calls only */
/* #line 59 */
  {
    if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_qHead != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_qHead;

/* #line 62 */
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_qHead];
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_NO_ENTRY;
          }
/* #line 65 */
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_resQ[id] = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_NO_ENTRY;
        {
          unsigned char __nesc_temp = 
/* #line 66 */
          id;

/* #line 66 */
          return __nesc_temp;
        }
      }
/* #line 68 */
    {
      unsigned char __nesc_temp = 
/* #line 68 */
      /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_NO_ENTRY;

/* #line 68 */
      return __nesc_temp;
    }
  }
}

/* # 60 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceQueue.nc" */
inline static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Queue_dequeue(void ){
/* #line 60 */
  unsigned char result;
/* #line 60 */

/* #line 60 */
  result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_dequeue();
/* #line 60 */

/* #line 60 */
  return result;
/* #line 60 */
}
/* #line 60 */
/* # 50 "/Users/doina/tinyos-2.x/tos/system/FcfsResourceQueueC.nc" */
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_isEmpty(void )
/* #line 50 */
{
  return /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_NO_ENTRY;
}

/* # 43 "/Users/doina/tinyos-2.x/tos/interfaces/ResourceQueue.nc" */
inline static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Queue_isEmpty(void ){
/* #line 43 */
  unsigned char result;
/* #line 43 */

/* #line 43 */
  result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_FcfsQueue_isEmpty();
/* #line 43 */

/* #line 43 */
  return result;
/* #line 43 */
}
/* #line 43 */
/* # 108 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_release(uint8_t id)
/* #line 108 */
{
  /* atomic removed: atomic calls only */
/* #line 109 */
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_BUSY && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId == id) {
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Queue_isEmpty() == FALSE) {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_reqResId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Queue_dequeue();
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_NO_RES;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_GRANTING;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_grantedTask_postTask();
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_unconfigure(id);
          }
        else {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_default_owner_id;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_CONTROLLED;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_unconfigure(id);
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_granted();
          }
        {
          unsigned char __nesc_temp = 
/* #line 124 */
          SUCCESS;

/* #line 124 */
          return __nesc_temp;
        }
      }
  }
/* #line 127 */
  return FAIL;
}

/* # 114 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_release(uint8_t id)
/* #line 114 */
{
/* #line 114 */
  return FAIL;
}

/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_release(uint8_t arg_0x1c66010){
/* #line 110 */
  unsigned char result;
/* #line 110 */

/* #line 110 */
  switch (arg_0x1c66010) {
/* #line 110 */
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0_CLIENT_ID:
/* #line 110 */
      result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C_0_CLIENT_ID);
/* #line 110 */
      break;
/* #line 110 */
    default:
/* #line 110 */
      result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_default_release(arg_0x1c66010);
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
/* # 81 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_release(uint8_t id)
/* #line 81 */
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_release(id);
}

/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t CC2420SpiP_SpiResource_release(void ){
/* #line 110 */
  unsigned char result;
/* #line 110 */

/* #line 110 */
  result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0_CLIENT_ID);
/* #line 110 */

/* #line 110 */
  return result;
/* #line 110 */
}
/* #line 110 */
/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP_17_IO_selectIOFunc(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  * (volatile uint8_t * )27U &= ~(0x01 << 1);
}

/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void HplMsp430Usart0P_SIMO_selectIOFunc(void ){
/* #line 85 */
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP_17_IO_selectIOFunc();
/* #line 85 */
}
/* #line 85 */
/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP_18_IO_selectIOFunc(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  * (volatile uint8_t * )27U &= ~(0x01 << 2);
}

/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void HplMsp430Usart0P_SOMI_selectIOFunc(void ){
/* #line 85 */
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP_18_IO_selectIOFunc();
/* #line 85 */
}
/* #line 85 */
/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP_19_IO_selectIOFunc(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  * (volatile uint8_t * )27U &= ~(0x01 << 3);
}

/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void HplMsp430Usart0P_UCLK_selectIOFunc(void ){
/* #line 85 */
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP_19_IO_selectIOFunc();
/* #line 85 */
}
/* #line 85 */
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
inline static cc2420_status_t CC2420TransmitP_STXONCCA_strobe(void ){
/* #line 45 */
  unsigned char result;
/* #line 45 */

/* #line 45 */
  result = CC2420SpiP_Strobe_strobe(CC2420_STXONCCA);
/* #line 45 */

/* #line 45 */
  return result;
/* #line 45 */
}
/* #line 45 */
inline static cc2420_status_t CC2420TransmitP_STXON_strobe(void ){
/* #line 45 */
  unsigned char result;
/* #line 45 */

/* #line 45 */
  result = CC2420SpiP_Strobe_strobe(CC2420_STXON);
/* #line 45 */

/* #line 45 */
  return result;
/* #line 45 */
}
/* #line 45 */
inline static cc2420_status_t CC2420TransmitP_SNOP_strobe(void ){
/* #line 45 */
  unsigned char result;
/* #line 45 */

/* #line 45 */
  result = CC2420SpiP_Strobe_strobe(CC2420_SNOP);
/* #line 45 */

/* #line 45 */
  return result;
/* #line 45 */
}
/* #line 45 */
/* # 102 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static inline error_t CC2420SpiP_ChipSpiResource_attemptRelease(void )
/* #line 102 */
{
  return CC2420SpiP_attemptRelease();
}

/* # 39 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc" */
inline static error_t CC2420TransmitP_ChipSpiResource_attemptRelease(void ){
/* #line 39 */
  unsigned char result;
/* #line 39 */

/* #line 39 */
  result = CC2420SpiP_ChipSpiResource_attemptRelease();
/* #line 39 */

/* #line 39 */
  return result;
/* #line 39 */
}
/* #line 39 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_selectModuleFunc(void )
/* #line 54 */
{
  /* atomic removed: atomic calls only */
/* #line 54 */
  * (volatile uint8_t * )31U |= 0x01 << 1;
}

/* # 78 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_GeneralIO_selectModuleFunc(void ){
/* #line 78 */
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_selectModuleFunc();
/* #line 78 */
}
/* #line 78 */
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_cc_t x)
/* #line 46 */
{
/* #line 46 */
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4___nesc_unnamed4352 {
/* #line 46 */
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_cc_t f;
/* #line 46 */
    uint16_t t;
  } 
/* #line 46 */
  c = { .f = x };

/* #line 46 */
  return c.t;
}

/* #line 61 */
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_captureControl(uint8_t l_cm)
{
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_cc_t x = { 
  .cm = l_cm & 0x03, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 1, 
  .scs = 1, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_CC2int(x);
}

/* #line 99 */
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_setControlAsCapture(uint8_t cm)
{
  * (volatile uint16_t * )388U = /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_captureControl(cm);
}

/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430TimerControl_setControlAsCapture(uint8_t cm){
/* #line 44 */
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_setControlAsCapture(cm);
/* #line 44 */
}
/* #line 44 */
/* # 119 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_enableEvents(void )
{
  * (volatile uint16_t * )388U |= 0x0010;
}

/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430TimerControl_enableEvents(void ){
/* #line 46 */
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_enableEvents();
/* #line 46 */
}
/* #line 46 */
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

/* # 276 "/opt/local/lib/ncc/nesc_nx.h" */
static __inline  uint16_t __nesc_ntoh_leuint16(const void * source)
/* #line 276 */
{
  const uint8_t *base = source;

/* #line 278 */
  return ((uint16_t )base[1] << 8) | base[0];
}

/* #line 301 */
static __inline  uint32_t __nesc_hton_uint32(void * target, uint32_t value)
/* #line 301 */
{
  uint8_t *base = target;

/* #line 303 */
  base[3] = value;
  base[2] = value >> 8;
  base[1] = value >> 16;
  base[0] = value >> 24;
  return value;
}

/* #line 294 */
static __inline  uint32_t __nesc_ntoh_uint32(const void * source)
/* #line 294 */
{
  const uint8_t *base = source;

/* #line 296 */
  return ((((uint32_t )base[0] << 24) | (
  (uint32_t )base[1] << 16)) | (
  (uint32_t )base[2] << 8)) | base[3];
}

/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc" */
inline static void CC2420TransmitP_PacketTimeStamp_clear(message_t * msg){
/* #line 59 */
  CC2420PacketP_PacketTimeStamp32khz_clear(msg);
/* #line 59 */
}
/* #line 59 */
/* # 162 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
static inline void CC2420ReceiveP_CC2420Receive_sfd_dropped(void )
/* #line 162 */
{
  if (CC2420ReceiveP_m_timestamp_size) {
      CC2420ReceiveP_m_timestamp_size--;
    }
}

/* # 55 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc" */
inline static void CC2420TransmitP_CC2420Receive_sfd_dropped(void ){
/* #line 55 */
  CC2420ReceiveP_CC2420Receive_sfd_dropped();
/* #line 55 */
}
/* #line 55 */
/* # 48 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_getRaw(void )
/* #line 48 */
{
/* #line 48 */
  return * (volatile uint8_t * )28U & (0x01 << 1);
}

/* #line 49 */
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_get(void )
/* #line 49 */
{
/* #line 49 */
  return /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_getRaw() != 0;
}

/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC_5_HplGeneralIO_get(void ){
/* #line 59 */
  unsigned char result;
/* #line 59 */

/* #line 59 */
  result = /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_get();
/* #line 59 */

/* #line 59 */
  return result;
/* #line 59 */
}
/* #line 59 */
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC_5_GeneralIO_get(void )
/* #line 40 */
{
/* #line 40 */
  return /*HplCC2420PinsC.SFDM*/Msp430GpioC_5_HplGeneralIO_get();
}

/* # 32 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static bool CC2420TransmitP_SFD_get(void ){
/* #line 32 */
  unsigned char result;
/* #line 32 */

/* #line 32 */
  result = /*HplCC2420PinsC.SFDM*/Msp430GpioC_5_GeneralIO_get();
/* #line 32 */

/* #line 32 */
  return result;
/* #line 32 */
}
/* #line 32 */
/* # 153 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
static inline void CC2420ReceiveP_CC2420Receive_sfd(uint32_t time)
/* #line 153 */
{
  if (CC2420ReceiveP_m_timestamp_size < CC2420ReceiveP_TIMESTAMP_QUEUE_SIZE) {
      uint8_t tail = (CC2420ReceiveP_m_timestamp_head + CC2420ReceiveP_m_timestamp_size) % 
      CC2420ReceiveP_TIMESTAMP_QUEUE_SIZE;

/* #line 157 */
      CC2420ReceiveP_m_timestamp_queue[tail] = time;
      CC2420ReceiveP_m_timestamp_size++;
    }
}

/* # 49 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc" */
inline static void CC2420TransmitP_CC2420Receive_sfd(uint32_t time){
/* #line 49 */
  CC2420ReceiveP_CC2420Receive_sfd(time);
/* #line 49 */
}
/* #line 49 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc" */
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_captureFallingEdge(void )
/* #line 54 */
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_enableCapture(MSP430TIMER_CM_FALLING);
}

/* # 43 "/Users/doina/tinyos-2.x/tos/interfaces/GpioCapture.nc" */
inline static error_t CC2420TransmitP_CaptureSFD_captureFallingEdge(void ){
/* #line 43 */
  unsigned char result;
/* #line 43 */

/* #line 43 */
  result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_captureFallingEdge();
/* #line 43 */

/* #line 43 */
  return result;
/* #line 43 */
}
/* #line 43 */
/* # 99 "/Users/doina/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc" */
static inline cc2420_header_t * CC2420PacketP_CC2420PacketBody_getHeader(message_t * msg)
/* #line 99 */
{
  return (cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
}

/* # 42 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
inline static cc2420_header_t * CC2420TransmitP_CC2420PacketBody_getHeader(message_t * msg){
/* #line 42 */
  nx_struct cc2420_header_t *result;
/* #line 42 */

/* #line 42 */
  result = CC2420PacketP_CC2420PacketBody_getHeader(msg);
/* #line 42 */

/* #line 42 */
  return result;
/* #line 42 */
}
/* #line 42 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_stop(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_disableEvents();
}

/* # 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_stop(void ){
/* #line 62 */
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Alarm_stop();
/* #line 62 */
}
/* #line 62 */
/* # 91 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_stop(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_stop();
}

/* # 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void CC2420TransmitP_BackoffTimer_stop(void ){
/* #line 62 */
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_stop();
/* #line 62 */
}
/* #line 62 */
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_HplGeneralIO_set(void ){
/* #line 34 */
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP_26_IO_set();
/* #line 34 */
}
/* #line 34 */
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_set(void )
/* #line 37 */
{
/* #line 37 */
  /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_HplGeneralIO_set();
}

/* # 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420TransmitP_CSN_set(void ){
/* #line 29 */
  /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_set();
/* #line 29 */
}
/* #line 29 */
/* # 63 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc" */
inline static cc2420_status_t CC2420TransmitP_TXFIFO_RAM_write(uint8_t offset, uint8_t * data, uint8_t length){
/* #line 63 */
  unsigned char result;
/* #line 63 */

/* #line 63 */
  result = CC2420SpiP_Ram_write(CC2420_RAM_TXFIFO, offset, data, length);
/* #line 63 */

/* #line 63 */
  return result;
/* #line 63 */
}
/* #line 63 */
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_HplGeneralIO_clr(void ){
/* #line 39 */
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP_26_IO_clr();
/* #line 39 */
}
/* #line 39 */
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_clr(void )
/* #line 38 */
{
/* #line 38 */
  /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_HplGeneralIO_clr();
}

/* # 30 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420TransmitP_CSN_clr(void ){
/* #line 30 */
  /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_clr();
/* #line 30 */
}
/* #line 30 */
/* # 170 "/Users/doina/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc" */
static inline uint8_t CC2420PacketP_PacketTimeSyncOffset_get(message_t *msg)
{
  return __nesc_ntoh_leuint8(CC2420PacketP_CC2420PacketBody_getHeader(msg)->length.data)
   + (sizeof(cc2420_header_t ) - MAC_HEADER_SIZE)
   - MAC_FOOTER_SIZE
   - sizeof(timesync_radio_t );
}

/* # 47 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc" */
inline static uint8_t CC2420TransmitP_PacketTimeSyncOffset_get(message_t * msg){
/* #line 47 */
  unsigned char result;
/* #line 47 */

/* #line 47 */
  result = CC2420PacketP_PacketTimeSyncOffset_get(msg);
/* #line 47 */

/* #line 47 */
  return result;
/* #line 47 */
}
/* #line 47 */
/* # 235 "/opt/local/lib/ncc/nesc_nx.h" */
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
/* #line 235 */
{
  const uint8_t *base = source;

/* #line 237 */
  return base[0];
}

/* #line 257 */
static __inline  int8_t __nesc_ntoh_int8(const void * source)
/* #line 257 */
{
/* #line 257 */
  return __nesc_ntoh_uint8(source);
}

/* # 103 "/Users/doina/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc" */
static inline cc2420_metadata_t *CC2420PacketP_CC2420PacketBody_getMetadata(message_t *msg)
/* #line 103 */
{
  return (cc2420_metadata_t *)msg->metadata;
}

/* #line 161 */
static inline bool CC2420PacketP_PacketTimeSyncOffset_isSet(message_t *msg)
{
  return __nesc_ntoh_int8(CC2420PacketP_CC2420PacketBody_getMetadata(msg)->timesync.data);
}

/* # 39 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc" */
inline static bool CC2420TransmitP_PacketTimeSyncOffset_isSet(message_t * msg){
/* #line 39 */
  unsigned char result;
/* #line 39 */

/* #line 39 */
  result = CC2420PacketP_PacketTimeSyncOffset_isSet(msg);
/* #line 39 */

/* #line 39 */
  return result;
/* #line 39 */
}
/* #line 39 */
/* # 128 "/Users/doina/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc" */
static inline void CC2420PacketP_PacketTimeStamp32khz_set(message_t *msg, uint32_t value)
{
  __nesc_hton_uint32(CC2420PacketP_CC2420PacketBody_getMetadata(msg)->timestamp.data, value);
}

/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc" */
inline static void CC2420TransmitP_PacketTimeStamp_set(message_t * msg, CC2420TransmitP_PacketTimeStamp_size_type value){
/* #line 67 */
  CC2420PacketP_PacketTimeStamp32khz_set(msg, value);
/* #line 67 */
}
/* #line 67 */
/* # 98 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static CC2420TransmitP_BackoffTimer_size_type CC2420TransmitP_BackoffTimer_getNow(void ){
/* #line 98 */
  unsigned long result;
/* #line 98 */

/* #line 98 */
  result = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_getNow();
/* #line 98 */

/* #line 98 */
  return result;
/* #line 98 */
}
/* #line 98 */
/* # 239 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static __inline uint32_t CC2420TransmitP_getTime32(uint16_t time)
{
  uint32_t recent_time = CC2420TransmitP_BackoffTimer_getNow();

/* #line 242 */
  return recent_time + (int16_t )(time - recent_time);
}

/* #line 258 */
static inline void CC2420TransmitP_CaptureSFD_captured(uint16_t time)
/* #line 258 */
{
  unsigned char *__nesc_temp44;
/* #line 259 */
  uint32_t time32;
  uint8_t sfd_state = 0;

  /* atomic removed: atomic calls only */
/* #line 261 */
  {
    time32 = CC2420TransmitP_getTime32(time);
    switch (CC2420TransmitP_m_state) {

        case CC2420TransmitP_S_SFD: 
          CC2420TransmitP_m_state = CC2420TransmitP_S_EFD;
        CC2420TransmitP_sfdHigh = TRUE;


        CC2420TransmitP_m_receiving = FALSE;
        CC2420TransmitP_CaptureSFD_captureFallingEdge();
        CC2420TransmitP_PacketTimeStamp_set(CC2420TransmitP_m_msg, time32);
        if (CC2420TransmitP_PacketTimeSyncOffset_isSet(CC2420TransmitP_m_msg)) {
            uint8_t absOffset = sizeof(message_header_t ) - sizeof(cc2420_header_t ) + CC2420TransmitP_PacketTimeSyncOffset_get(CC2420TransmitP_m_msg);
            timesync_radio_t *timesync = (timesync_radio_t *)((nx_uint8_t *)CC2420TransmitP_m_msg + absOffset);

            (__nesc_temp44 = (*timesync).data, __nesc_hton_uint32(__nesc_temp44, __nesc_ntoh_uint32(__nesc_temp44) - time32));
            CC2420TransmitP_CSN_clr();
            CC2420TransmitP_TXFIFO_RAM_write(absOffset, (uint8_t *)timesync, sizeof(timesync_radio_t ));
            CC2420TransmitP_CSN_set();
          }

        if (__nesc_ntoh_leuint16(CC2420TransmitP_CC2420PacketBody_getHeader(CC2420TransmitP_m_msg)->fcf.data) & (1 << IEEE154_FCF_ACK_REQ)) {

            CC2420TransmitP_abortSpiRelease = TRUE;
          }
        CC2420TransmitP_releaseSpiResource();
        CC2420TransmitP_BackoffTimer_stop();

        if (CC2420TransmitP_SFD_get()) {
            break;
          }


        case CC2420TransmitP_S_EFD: 
          CC2420TransmitP_sfdHigh = FALSE;
        CC2420TransmitP_CaptureSFD_captureRisingEdge();

        if (__nesc_ntoh_leuint16(CC2420TransmitP_CC2420PacketBody_getHeader(CC2420TransmitP_m_msg)->fcf.data) & (1 << IEEE154_FCF_ACK_REQ)) {
            CC2420TransmitP_m_state = CC2420TransmitP_S_ACK_WAIT;
            CC2420TransmitP_BackoffTimer_start(CC2420_ACK_WAIT_DELAY);
          }
        else 
/* #line 302 */
          {
            CC2420TransmitP_signalDone(SUCCESS);
          }

        if (!CC2420TransmitP_SFD_get()) {
            break;
          }


        default: 

          if (!CC2420TransmitP_m_receiving && CC2420TransmitP_sfdHigh == FALSE) {
              CC2420TransmitP_sfdHigh = TRUE;
              CC2420TransmitP_CaptureSFD_captureFallingEdge();

              sfd_state = CC2420TransmitP_SFD_get();
              CC2420TransmitP_CC2420Receive_sfd(time32);
              CC2420TransmitP_m_receiving = TRUE;
              CC2420TransmitP_m_prev_time = time;
              if (CC2420TransmitP_SFD_get()) {

                  return;
                }
            }



        if (CC2420TransmitP_sfdHigh == TRUE) {
            CC2420TransmitP_sfdHigh = FALSE;
            CC2420TransmitP_CaptureSFD_captureRisingEdge();
            CC2420TransmitP_m_receiving = FALSE;








            if (sfd_state == 0 && time - CC2420TransmitP_m_prev_time < 10) {
                CC2420TransmitP_CC2420Receive_sfd_dropped();
                if (CC2420TransmitP_m_msg) {
                  CC2420TransmitP_PacketTimeStamp_clear(CC2420TransmitP_m_msg);
                  }
              }
/* #line 346 */
            break;
          }
      }
  }
}

/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/GpioCapture.nc" */
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_captured(uint16_t time){
/* #line 50 */
  CC2420TransmitP_CaptureSFD_captured(time);
/* #line 50 */
}
/* #line 50 */
/* # 164 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_clearOverflow(void )
{
  * (volatile uint16_t * )388U &= ~0x0002;
}

/* # 57 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430Capture_clearOverflow(void ){
/* #line 57 */
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_clearOverflow();
/* #line 57 */
}
/* #line 57 */
/* # 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_clearPendingInterrupt(void )
{
  * (volatile uint16_t * )388U &= ~0x0001;
}

/* # 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430TimerControl_clearPendingInterrupt(void ){
/* #line 33 */
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_clearPendingInterrupt();
/* #line 33 */
}
/* #line 33 */
/* # 65 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc" */
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430Capture_captured(uint16_t time)
/* #line 65 */
{
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430TimerControl_clearPendingInterrupt();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430Capture_clearOverflow();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_captured(time);
}

/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc" */
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Capture_captured(uint16_t time){
/* #line 75 */
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430Capture_captured(time);
/* #line 75 */
}
/* #line 75 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_int2CC(uint16_t x)
/* #line 47 */
{
/* #line 47 */
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4___nesc_unnamed4353 {
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
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_fired(void ){
/* #line 67 */
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_fired();
/* #line 67 */
}
/* #line 67 */
/* # 151 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_set_alarm();
      }
  }
}

/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_fired(void ){
/* #line 67 */
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_fired();
/* #line 67 */
}
/* #line 67 */
/* # 124 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_disableEvents(void )
{
  * (volatile uint16_t * )390U &= ~0x0010;
}

/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_disableEvents(void ){
/* #line 47 */
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_disableEvents();
/* #line 47 */
}
/* #line 47 */
/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_fired();
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_fired(void ){
/* #line 34 */
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_fired();
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
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5___nesc_unnamed4354 {
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

/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
inline static /*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_size_type /*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_get(void ){
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







inline static bool /*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_isOverflowPending(void ){
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
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_enableEvents(void )
{
  * (volatile uint16_t * )390U |= 0x0010;
}

/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_enableEvents(void ){
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
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_clearPendingInterrupt(void ){
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
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_setEvent(uint16_t time){
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
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_setEventFromNow(uint16_t delta){
/* #line 32 */
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_setEventFromNow(delta);
/* #line 32 */
}
/* #line 32 */
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc" */
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Timer_get(void ){
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
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Timer_get();
    uint16_t elapsed = now - t0;

/* #line 76 */
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

/* #line 83 */
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_setEventFromNow(2);
          }
        else {
/* #line 86 */
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430Compare_setEvent(now + remaining);
          }
      }
/* #line 88 */
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_enableEvents();
  }
}

/* # 92 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_size_type dt){
/* #line 92 */
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_startAt(t0, dt);
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
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6___nesc_unnamed4355 {
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
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7___nesc_unnamed4356 {
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
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8___nesc_unnamed4357 {
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
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9___nesc_unnamed4358 {
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
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_set(void )
/* #line 45 */
{
  /* atomic removed: atomic calls only */
/* #line 45 */
  * (volatile uint8_t * )49U |= 0x01 << 6;
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_HplGeneralIO_set(void ){
/* #line 34 */
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_set();
/* #line 34 */
}
/* #line 34 */
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_GeneralIO_set(void )
/* #line 37 */
{
/* #line 37 */
  /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_HplGeneralIO_set();
}

/* # 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led2_set(void ){
/* #line 29 */
  /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_GeneralIO_set();
/* #line 29 */
}
/* #line 29 */
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_set(void )
/* #line 45 */
{
  /* atomic removed: atomic calls only */
/* #line 45 */
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_HplGeneralIO_set(void ){
/* #line 34 */
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_set();
/* #line 34 */
}
/* #line 34 */
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_GeneralIO_set(void )
/* #line 37 */
{
/* #line 37 */
  /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_HplGeneralIO_set();
}

/* # 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led1_set(void ){
/* #line 29 */
  /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_GeneralIO_set();
/* #line 29 */
}
/* #line 29 */
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_set(void )
/* #line 45 */
{
  /* atomic removed: atomic calls only */
/* #line 45 */
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_HplGeneralIO_set(void ){
/* #line 34 */
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_set();
/* #line 34 */
}
/* #line 34 */
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_GeneralIO_set(void )
/* #line 37 */
{
/* #line 37 */
  /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_HplGeneralIO_set();
}

/* # 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led0_set(void ){
/* #line 29 */
  /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_GeneralIO_set();
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
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_HplGeneralIO_makeOutput(void ){
/* #line 71 */
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_makeOutput();
/* #line 71 */
}
/* #line 71 */
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_GeneralIO_makeOutput(void )
/* #line 43 */
{
/* #line 43 */
  /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_HplGeneralIO_makeOutput();
}

/* # 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led2_makeOutput(void ){
/* #line 35 */
  /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_GeneralIO_makeOutput();
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
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_HplGeneralIO_makeOutput(void ){
/* #line 71 */
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_makeOutput();
/* #line 71 */
}
/* #line 71 */
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_GeneralIO_makeOutput(void )
/* #line 43 */
{
/* #line 43 */
  /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_HplGeneralIO_makeOutput();
}

/* # 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led1_makeOutput(void ){
/* #line 35 */
  /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_GeneralIO_makeOutput();
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
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_HplGeneralIO_makeOutput(void ){
/* #line 71 */
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_makeOutput();
/* #line 71 */
}
/* #line 71 */
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_GeneralIO_makeOutput(void )
/* #line 43 */
{
/* #line 43 */
  /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_HplGeneralIO_makeOutput();
}

/* # 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led0_makeOutput(void ){
/* #line 35 */
  /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_GeneralIO_makeOutput();
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






    MotePlatformC_uwait(1024 * 10);

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
    Msp430ClockP_Msp430ClockInit_setupDcoCalibrate();
    Msp430ClockP_busyCalibrateDco();
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
/* # 61 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationValue.nc" */
inline static void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_changed(void ){
/* #line 61 */
  TestDisseminationC_Value16_changed();
/* #line 61 */
}
/* #line 61 */
/* # 62 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminatorP.nc" */
static inline void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_changedTask_runTask(void )
/* #line 62 */
{
  /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_changed();
}

/* #line 78 */
static inline const /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_t */*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_get(void )
/* #line 78 */
{
  return &/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_valueCache;
}

/* # 47 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationValue.nc" */
inline static const TestDisseminationC_Value16_t *TestDisseminationC_Value16_get(void ){
/* #line 47 */
  unsigned int const *result;
/* #line 47 */

/* #line 47 */
  result = /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_get();
/* #line 47 */

/* #line 47 */
  return result;
/* #line 47 */
}
/* #line 47 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_timerTask_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(/*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_timerTask);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 110 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static inline void DisseminationEngineImplP_TrickleTimer_fired(uint16_t key)
/* #line 110 */
{

  if (!DisseminationEngineImplP_m_running || DisseminationEngineImplP_m_bufBusy) {
/* #line 112 */
      return;
    }
  DisseminationEngineImplP_sendObject(key);
}

/* # 270 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimerImplP.nc" */
static inline void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_default_fired(uint8_t id)
/* #line 270 */
{
  return;
}

/* # 82 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
inline static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_fired(uint8_t arg_0x22e4c20){
/* #line 82 */
  switch (arg_0x22e4c20) {
/* #line 82 */
    case /*TestDisseminationAppC.Object32C*/DisseminatorC_0_TIMER_ID:
/* #line 82 */
      DisseminationEngineImplP_TrickleTimer_fired(4660U);
/* #line 82 */
      break;
/* #line 82 */
    case /*TestDisseminationAppC.Object16C*/DisseminatorC_1_TIMER_ID:
/* #line 82 */
      DisseminationEngineImplP_TrickleTimer_fired(9029U);
/* #line 82 */
      break;
/* #line 82 */
    default:
/* #line 82 */
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_default_fired(arg_0x22e4c20);
/* #line 82 */
      break;
/* #line 82 */
    }
/* #line 82 */
}
/* #line 82 */
/* # 55 "/Users/doina/tinyos-2.x/tos/system/BitVectorC.nc" */
static inline uint16_t /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_getMask(uint16_t bitnum)
{
  return 1 << bitnum % /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_ELEMENT_SIZE;
}

/* #line 50 */
static inline uint16_t /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_getIndex(uint16_t bitnum)
{
  return bitnum / /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_ELEMENT_SIZE;
}

/* #line 86 */
static inline void /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_clear(uint16_t bitnum)
{
  /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_m_bits[/*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_getIndex(bitnum)] &= ~/*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_getMask(bitnum);
}

/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/BitVector.nc" */
inline static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Pending_clear(uint16_t bitnum){
/* #line 58 */
  /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_clear(bitnum);
/* #line 58 */
}
/* #line 58 */
/* # 76 "/Users/doina/tinyos-2.x/tos/system/BitVectorC.nc" */
static inline bool /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_get(uint16_t bitnum)
{
  return /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_m_bits[/*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_getIndex(bitnum)] & /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_getMask(bitnum) ? TRUE : FALSE;
}

/* # 46 "/Users/doina/tinyos-2.x/tos/interfaces/BitVector.nc" */
inline static bool /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Pending_get(uint16_t bitnum){
/* #line 46 */
  unsigned char result;
/* #line 46 */

/* #line 46 */
  result = /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_get(bitnum);
/* #line 46 */

/* #line 46 */
  return result;
/* #line 46 */
}
/* #line 46 */
/* # 146 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimerImplP.nc" */
static inline void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_timerTask_runTask(void )
/* #line 146 */
{
  uint8_t i;

/* #line 148 */
  for (i = 0; i < 2U; i++) {
      bool fire = FALSE;

/* #line 150 */
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 150 */
        {
          if (/*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Pending_get(i)) {
              /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Pending_clear(i);
              fire = TRUE;
            }
        }
/* #line 155 */
        __nesc_atomic_end(__nesc_atomic); }
      if (fire) {
          ;
          /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_fired(i);
          /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_timerTask_postTask();
          return;
        }
    }
}

/* # 114 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
inline static void * CC2420TinyosNetworkP_SubSend_getPayload(message_t * msg, uint8_t len){
/* #line 114 */
  void *result;
/* #line 114 */

/* #line 114 */
  result = CC2420CsmaP_Send_getPayload(msg, len);
/* #line 114 */

/* #line 114 */
  return result;
/* #line 114 */
}
/* #line 114 */
/* # 74 "/Users/doina/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc" */
static inline void *CC2420TinyosNetworkP_Send_getPayload(message_t *msg, uint8_t len)
/* #line 74 */
{
  return CC2420TinyosNetworkP_SubSend_getPayload(msg, len);
}

/* # 114 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
inline static void * UniqueSendP_SubSend_getPayload(message_t * msg, uint8_t len){
/* #line 114 */
  void *result;
/* #line 114 */

/* #line 114 */
  result = CC2420TinyosNetworkP_Send_getPayload(msg, len);
/* #line 114 */

/* #line 114 */
  return result;
/* #line 114 */
}
/* #line 114 */
/* # 99 "/Users/doina/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc" */
static inline void *UniqueSendP_Send_getPayload(message_t *msg, uint8_t len)
/* #line 99 */
{
  return UniqueSendP_SubSend_getPayload(msg, len);
}

/* # 114 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
inline static void * CC2420ActiveMessageP_SubSend_getPayload(message_t * msg, uint8_t len){
/* #line 114 */
  void *result;
/* #line 114 */

/* #line 114 */
  result = UniqueSendP_Send_getPayload(msg, len);
/* #line 114 */

/* #line 114 */
  return result;
/* #line 114 */
}
/* #line 114 */
/* # 164 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline void *CC2420ActiveMessageP_Packet_getPayload(message_t *msg, uint8_t len)
/* #line 164 */
{
  return CC2420ActiveMessageP_SubSend_getPayload(msg, len);
}

/* #line 88 */
static inline void *CC2420ActiveMessageP_AMSend_getPayload(am_id_t id, message_t *m, uint8_t len)
/* #line 88 */
{
  return CC2420ActiveMessageP_Packet_getPayload(m, len);
}

/* # 124 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
inline static void * /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMSend_getPayload(am_id_t arg_0x2246108, message_t * msg, uint8_t len){
/* #line 124 */
  void *result;
/* #line 124 */

/* #line 124 */
  result = CC2420ActiveMessageP_AMSend_getPayload(arg_0x2246108, msg, len);
/* #line 124 */

/* #line 124 */
  return result;
/* #line 124 */
}
/* #line 124 */
/* # 203 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
static inline void */*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_getPayload(uint8_t id, message_t *m, uint8_t len)
/* #line 203 */
{
  return /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMSend_getPayload(0, m, len);
}

/* # 114 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
inline static void * /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_Send_getPayload(message_t * msg, uint8_t len){
/* #line 114 */
  void *result;
/* #line 114 */

/* #line 114 */
  result = /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_getPayload(0U, msg, len);
/* #line 114 */

/* #line 114 */
  return result;
/* #line 114 */
}
/* #line 114 */
/* # 65 "/Users/doina/tinyos-2.x/tos/system/AMQueueEntryP.nc" */
static inline void */*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_getPayload(message_t *m, uint8_t len)
/* #line 65 */
{
  return /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_Send_getPayload(m, len);
}

/* # 124 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
inline static void * DisseminationEngineImplP_AMSend_getPayload(message_t * msg, uint8_t len){
/* #line 124 */
  void *result;
/* #line 124 */

/* #line 124 */
  result = /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_getPayload(msg, len);
/* #line 124 */

/* #line 124 */
  return result;
/* #line 124 */
}
/* #line 124 */
/* # 166 "/Users/doina/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc" */
static inline uint8_t CC2420CsmaP_Send_maxPayloadLength(void )
/* #line 166 */
{
  return 28;
}

/* # 101 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminatorP.nc" */
static inline void */*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_requestData(uint8_t *size)
/* #line 101 */
{
  *size = sizeof(/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_t );
  return &/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_valueCache;
}

/* #line 101 */
static inline void */*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_requestData(uint8_t *size)
/* #line 101 */
{
  *size = sizeof(/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_t );
  return &/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_valueCache;
}

/* # 236 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static inline void *
DisseminationEngineImplP_DisseminationCache_default_requestData(uint16_t key, uint8_t *size)
/* #line 237 */
{
/* #line 237 */
  return (void *)0;
}

/* # 47 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
inline static void *DisseminationEngineImplP_DisseminationCache_requestData(uint16_t arg_0x21c4120, uint8_t *size){
/* #line 47 */
  void *result;
/* #line 47 */

/* #line 47 */
  switch (arg_0x21c4120) {
/* #line 47 */
    case 4660U:
/* #line 47 */
      result = /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_requestData(size);
/* #line 47 */
      break;
/* #line 47 */
    case 9029U:
/* #line 47 */
      result = /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_requestData(size);
/* #line 47 */
      break;
/* #line 47 */
    default:
/* #line 47 */
      result = DisseminationEngineImplP_DisseminationCache_default_requestData(arg_0x21c4120, size);
/* #line 47 */
      break;
/* #line 47 */
    }
/* #line 47 */

/* #line 47 */
  return result;
/* #line 47 */
}
/* #line 47 */
/* # 160 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline uint8_t CC2420ActiveMessageP_Packet_maxPayloadLength(void )
/* #line 160 */
{
  return 28;
}

/* #line 84 */
static inline uint8_t CC2420ActiveMessageP_AMSend_maxPayloadLength(am_id_t id)
/* #line 84 */
{
  return CC2420ActiveMessageP_Packet_maxPayloadLength();
}

/* # 112 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
inline static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMSend_maxPayloadLength(am_id_t arg_0x2246108){
/* #line 112 */
  unsigned char result;
/* #line 112 */

/* #line 112 */
  result = CC2420ActiveMessageP_AMSend_maxPayloadLength(arg_0x2246108);
/* #line 112 */

/* #line 112 */
  return result;
/* #line 112 */
}
/* #line 112 */
/* # 199 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
static inline uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_maxPayloadLength(uint8_t id)
/* #line 199 */
{
  return /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMSend_maxPayloadLength(0);
}

/* # 101 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
inline static uint8_t /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_Send_maxPayloadLength(void ){
/* #line 101 */
  unsigned char result;
/* #line 101 */

/* #line 101 */
  result = /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_maxPayloadLength(0U);
/* #line 101 */

/* #line 101 */
  return result;
/* #line 101 */
}
/* #line 101 */
/* # 61 "/Users/doina/tinyos-2.x/tos/system/AMQueueEntryP.nc" */
static inline uint8_t /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_maxPayloadLength(void )
/* #line 61 */
{
  return /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_Send_maxPayloadLength();
}

/* # 112 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
inline static uint8_t DisseminationEngineImplP_AMSend_maxPayloadLength(void ){
/* #line 112 */
  unsigned char result;
/* #line 112 */

/* #line 112 */
  result = /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_maxPayloadLength();
/* #line 112 */

/* #line 112 */
  return result;
/* #line 112 */
}
/* #line 112 */
/* #line 69 */
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMSend_send(am_id_t arg_0x2246108, am_addr_t addr, message_t * msg, uint8_t len){
/* #line 69 */
  unsigned char result;
/* #line 69 */

/* #line 69 */
  result = CC2420ActiveMessageP_AMSend_send(arg_0x2246108, addr, msg, len);
/* #line 69 */

/* #line 69 */
  return result;
/* #line 69 */
}
/* #line 69 */
/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/AMPacket.nc" */
inline static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMPacket_destination(message_t * amsg){
/* #line 67 */
  unsigned int result;
/* #line 67 */

/* #line 67 */
  result = CC2420ActiveMessageP_AMPacket_destination(amsg);
/* #line 67 */

/* #line 67 */
  return result;
/* #line 67 */
}
/* #line 67 */
/* #line 136 */
inline static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMPacket_type(message_t * amsg){
/* #line 136 */
  unsigned char result;
/* #line 136 */

/* #line 136 */
  result = CC2420ActiveMessageP_AMPacket_type(amsg);
/* #line 136 */

/* #line 136 */
  return result;
/* #line 136 */
}
/* #line 136 */
/* # 251 "/opt/local/lib/ncc/nesc_nx.h" */
static __inline  uint8_t __nesc_hton_leuint8(void * target, uint8_t value)
/* #line 251 */
{
  uint8_t *base = target;

/* #line 253 */
  base[0] = value;
  return value;
}

/* # 42 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
inline static cc2420_header_t * CC2420ActiveMessageP_CC2420PacketBody_getHeader(message_t * msg){
/* #line 42 */
  nx_struct cc2420_header_t *result;
/* #line 42 */

/* #line 42 */
  result = CC2420PacketP_CC2420PacketBody_getHeader(msg);
/* #line 42 */

/* #line 42 */
  return result;
/* #line 42 */
}
/* #line 42 */
/* # 156 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline void CC2420ActiveMessageP_Packet_setPayloadLength(message_t *msg, uint8_t len)
/* #line 156 */
{
  __nesc_hton_leuint8(CC2420ActiveMessageP_CC2420PacketBody_getHeader(msg)->length.data, len + CC2420_SIZE);
}

/* # 83 "/Users/doina/tinyos-2.x/tos/interfaces/Packet.nc" */
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Packet_setPayloadLength(message_t * msg, uint8_t len){
/* #line 83 */
  CC2420ActiveMessageP_Packet_setPayloadLength(msg, len);
/* #line 83 */
}
/* #line 83 */
/* # 82 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
static inline error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_send(uint8_t clientId, message_t *msg, 
uint8_t len)
/* #line 83 */
{
  if (clientId >= 2) {
      return FAIL;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_queue[clientId].msg = msg;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Packet_setPayloadLength(msg, len);

  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current >= 2) {
      error_t err;
      am_id_t amId = /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMPacket_type(msg);
      am_addr_t dest = /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMPacket_destination(msg);

      ;
      /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current = clientId;

      err = /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMSend_send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current = 2;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
}

/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
inline static error_t /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_Send_send(message_t * msg, uint8_t len){
/* #line 64 */
  unsigned char result;
/* #line 64 */

/* #line 64 */
  result = /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_send(0U, msg, len);
/* #line 64 */

/* #line 64 */
  return result;
/* #line 64 */
}
/* #line 64 */
/* # 127 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline void CC2420ActiveMessageP_AMPacket_setType(message_t *amsg, am_id_t type)
/* #line 127 */
{
  cc2420_header_t *header = CC2420ActiveMessageP_CC2420PacketBody_getHeader(amsg);

/* #line 129 */
  __nesc_hton_leuint8(header->type.data, type);
}

/* # 151 "/Users/doina/tinyos-2.x/tos/interfaces/AMPacket.nc" */
inline static void /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMPacket_setType(message_t * amsg, am_id_t t){
/* #line 151 */
  CC2420ActiveMessageP_AMPacket_setType(amsg, t);
/* #line 151 */
}
/* #line 151 */
/* # 281 "/opt/local/lib/ncc/nesc_nx.h" */
static __inline  uint16_t __nesc_hton_leuint16(void * target, uint16_t value)
/* #line 281 */
{
  uint8_t *base = target;

/* #line 283 */
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

/* # 107 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline void CC2420ActiveMessageP_AMPacket_setDestination(message_t *amsg, am_addr_t addr)
/* #line 107 */
{
  cc2420_header_t *header = CC2420ActiveMessageP_CC2420PacketBody_getHeader(amsg);

/* #line 109 */
  __nesc_hton_leuint16(header->dest.data, addr);
}

/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/AMPacket.nc" */
inline static void /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMPacket_setDestination(message_t * amsg, am_addr_t addr){
/* #line 92 */
  CC2420ActiveMessageP_AMPacket_setDestination(amsg, addr);
/* #line 92 */
}
/* #line 92 */
/* # 45 "/Users/doina/tinyos-2.x/tos/system/AMQueueEntryP.nc" */
static inline error_t /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
/* #line 47 */
{
  /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMPacket_setDestination(msg, dest);
  /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMPacket_setType(msg, 96);
  return /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_Send_send(msg, len);
}

/* # 69 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
inline static error_t DisseminationEngineImplP_AMSend_send(am_addr_t addr, message_t * msg, uint8_t len){
/* #line 69 */
  unsigned char result;
/* #line 69 */

/* #line 69 */
  result = /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_send(addr, msg, len);
/* #line 69 */

/* #line 69 */
  return result;
/* #line 69 */
}
/* #line 69 */
/* # 287 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline uint16_t CC2420ControlP_CC2420Config_getPanAddr(void )
/* #line 287 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 288 */
    {
      unsigned int __nesc_temp = 
/* #line 288 */
      CC2420ControlP_m_pan;

      {
/* #line 288 */
        __nesc_atomic_end(__nesc_atomic); 
/* #line 288 */
        return __nesc_temp;
      }
    }
/* #line 290 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 70 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc" */
inline static uint16_t CC2420ActiveMessageP_CC2420Config_getPanAddr(void ){
/* #line 70 */
  unsigned int result;
/* #line 70 */

/* #line 70 */
  result = CC2420ControlP_CC2420Config_getPanAddr();
/* #line 70 */

/* #line 70 */
  return result;
/* #line 70 */
}
/* #line 70 */
/* # 253 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline void CC2420ActiveMessageP_SendNotifier_default_aboutToSend(am_id_t amId, am_addr_t addr, message_t *msg)
/* #line 253 */
{
}

/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/SendNotifier.nc" */
inline static void CC2420ActiveMessageP_SendNotifier_aboutToSend(am_id_t arg_0x16fec58, am_addr_t dest, message_t * msg){
/* #line 59 */
    CC2420ActiveMessageP_SendNotifier_default_aboutToSend(arg_0x16fec58, dest, msg);
/* #line 59 */
}
/* #line 59 */
/* # 240 "/opt/local/lib/ncc/nesc_nx.h" */
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
/* #line 240 */
{
  uint8_t *base = target;

/* #line 242 */
  base[0] = value;
  return value;
}

/* #line 257 */
static __inline  int8_t __nesc_hton_int8(void * target, int8_t value)
/* #line 257 */
{
/* #line 257 */
  __nesc_hton_uint8(target, value);
/* #line 257 */
  return value;
}

/* # 524 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static inline error_t CC2420TransmitP_send(message_t * p_msg, bool cca)
/* #line 524 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 525 */
    {
      if (CC2420TransmitP_m_state == CC2420TransmitP_S_CANCEL) {
          {
            unsigned char __nesc_temp = 
/* #line 527 */
            ECANCEL;

            {
/* #line 527 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 527 */
              return __nesc_temp;
            }
          }
        }
/* #line 530 */
      if (CC2420TransmitP_m_state != CC2420TransmitP_S_STARTED) {
          {
            unsigned char __nesc_temp = 
/* #line 531 */
            FAIL;

            {
/* #line 531 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 531 */
              return __nesc_temp;
            }
          }
        }
/* #line 534 */
      CC2420TransmitP_m_state = CC2420TransmitP_S_LOAD;
      CC2420TransmitP_m_cca = cca;
      CC2420TransmitP_m_msg = p_msg;
      CC2420TransmitP_totalCcaChecks = 0;
    }
/* #line 538 */
    __nesc_atomic_end(__nesc_atomic); }

  if (CC2420TransmitP_acquireSpiResource() == SUCCESS) {
      CC2420TransmitP_loadTXFIFO();
    }

  return SUCCESS;
}

/* #line 172 */
static inline error_t CC2420TransmitP_Send_send(message_t * p_msg, bool useCca)
/* #line 172 */
{
  return CC2420TransmitP_send(p_msg, useCca);
}

/* # 51 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc" */
inline static error_t CC2420CsmaP_CC2420Transmit_send(message_t * p_msg, bool useCca){
/* #line 51 */
  unsigned char result;
/* #line 51 */

/* #line 51 */
  result = CC2420TransmitP_Send_send(p_msg, useCca);
/* #line 51 */

/* #line 51 */
  return result;
/* #line 51 */
}
/* #line 51 */
/* # 263 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline void CC2420ActiveMessageP_RadioBackoff_default_requestCca(am_id_t id, 
message_t *msg)
/* #line 264 */
{
}

/* # 95 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
inline static void CC2420ActiveMessageP_RadioBackoff_requestCca(am_id_t arg_0x16fd2e8, message_t * msg){
/* #line 95 */
    CC2420ActiveMessageP_RadioBackoff_default_requestCca(arg_0x16fd2e8, msg);
/* #line 95 */
}
/* #line 95 */
/* # 211 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline void CC2420ActiveMessageP_SubBackoff_requestCca(message_t *msg)
/* #line 211 */
{

  CC2420ActiveMessageP_RadioBackoff_requestCca(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.data), msg);
}

/* # 95 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
inline static void CC2420CsmaP_RadioBackoff_requestCca(message_t * msg){
/* #line 95 */
  CC2420ActiveMessageP_SubBackoff_requestCca(msg);
/* #line 95 */
}
/* #line 95 */
/* # 111 "/Users/doina/tinyos-2.x/tos/system/StateImplP.nc" */
static inline void StateImplP_State_forceState(uint8_t id, uint8_t reqState)
/* #line 111 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 112 */
    StateImplP_state[id] = reqState;
/* #line 112 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
inline static void CC2420CsmaP_SplitControlState_forceState(uint8_t reqState){
/* #line 51 */
  StateImplP_State_forceState(1U, reqState);
/* #line 51 */
}
/* #line 51 */
/* #line 66 */
inline static bool CC2420CsmaP_SplitControlState_isState(uint8_t myState){
/* #line 66 */
  unsigned char result;
/* #line 66 */

/* #line 66 */
  result = StateImplP_State_isState(1U, myState);
/* #line 66 */

/* #line 66 */
  return result;
/* #line 66 */
}
/* #line 66 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
inline static cc2420_metadata_t * CC2420CsmaP_CC2420PacketBody_getMetadata(message_t * msg){
/* #line 47 */
  nx_struct cc2420_metadata_t *result;
/* #line 47 */

/* #line 47 */
  result = CC2420PacketP_CC2420PacketBody_getMetadata(msg);
/* #line 47 */

/* #line 47 */
  return result;
/* #line 47 */
}
/* #line 47 */
/* #line 42 */
inline static cc2420_header_t * CC2420CsmaP_CC2420PacketBody_getHeader(message_t * msg){
/* #line 42 */
  nx_struct cc2420_header_t *result;
/* #line 42 */

/* #line 42 */
  result = CC2420PacketP_CC2420PacketBody_getHeader(msg);
/* #line 42 */

/* #line 42 */
  return result;
/* #line 42 */
}
/* #line 42 */
/* # 122 "/Users/doina/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc" */
static inline error_t CC2420CsmaP_Send_send(message_t *p_msg, uint8_t len)
/* #line 122 */
{
  unsigned char *__nesc_temp43;
  unsigned char *__nesc_temp42;
/* #line 124 */
  cc2420_header_t *header = CC2420CsmaP_CC2420PacketBody_getHeader(p_msg);
  cc2420_metadata_t *metadata = CC2420CsmaP_CC2420PacketBody_getMetadata(p_msg);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 127 */
    {
      if (!CC2420CsmaP_SplitControlState_isState(CC2420CsmaP_S_STARTED)) {
          {
            unsigned char __nesc_temp = 
/* #line 129 */
            FAIL;

            {
/* #line 129 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 129 */
              return __nesc_temp;
            }
          }
        }
/* #line 132 */
      CC2420CsmaP_SplitControlState_forceState(CC2420CsmaP_S_TRANSMITTING);
      CC2420CsmaP_m_msg = p_msg;
    }
/* #line 134 */
    __nesc_atomic_end(__nesc_atomic); }

  __nesc_hton_leuint8(header->length.data, len + CC2420_SIZE);
  (__nesc_temp42 = header->fcf.data, __nesc_hton_leuint16(__nesc_temp42, __nesc_ntoh_leuint16(__nesc_temp42) & (1 << IEEE154_FCF_ACK_REQ)));
  (__nesc_temp43 = header->fcf.data, __nesc_hton_leuint16(__nesc_temp43, __nesc_ntoh_leuint16(__nesc_temp43) | ((((IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE) | (
  1 << IEEE154_FCF_INTRAPAN)) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE)) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE))));

  __nesc_hton_int8(metadata->ack.data, FALSE);
  __nesc_hton_uint8(metadata->rssi.data, 0);
  __nesc_hton_uint8(metadata->lqi.data, 0);
  __nesc_hton_int8(metadata->timesync.data, FALSE);
  __nesc_hton_uint32(metadata->timestamp.data, CC2420_INVALID_TIMESTAMP);

  CC2420CsmaP_ccaOn = TRUE;
  CC2420CsmaP_RadioBackoff_requestCca(CC2420CsmaP_m_msg);

  CC2420CsmaP_CC2420Transmit_send(CC2420CsmaP_m_msg, CC2420CsmaP_ccaOn);
  return SUCCESS;
}

/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
inline static error_t CC2420TinyosNetworkP_SubSend_send(message_t * msg, uint8_t len){
/* #line 64 */
  unsigned char result;
/* #line 64 */

/* #line 64 */
  result = CC2420CsmaP_Send_send(msg, len);
/* #line 64 */

/* #line 64 */
  return result;
/* #line 64 */
}
/* #line 64 */
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
inline static cc2420_header_t * CC2420TinyosNetworkP_CC2420PacketBody_getHeader(message_t * msg){
/* #line 42 */
  nx_struct cc2420_header_t *result;
/* #line 42 */

/* #line 42 */
  result = CC2420PacketP_CC2420PacketBody_getHeader(msg);
/* #line 42 */

/* #line 42 */
  return result;
/* #line 42 */
}
/* #line 42 */
/* # 61 "/Users/doina/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc" */
static inline error_t CC2420TinyosNetworkP_Send_send(message_t *msg, uint8_t len)
/* #line 61 */
{
  __nesc_hton_leuint8(CC2420TinyosNetworkP_CC2420PacketBody_getHeader(msg)->network.data, 0x3f);
  return CC2420TinyosNetworkP_SubSend_send(msg, len);
}

/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
inline static error_t UniqueSendP_SubSend_send(message_t * msg, uint8_t len){
/* #line 64 */
  unsigned char result;
/* #line 64 */

/* #line 64 */
  result = CC2420TinyosNetworkP_Send_send(msg, len);
/* #line 64 */

/* #line 64 */
  return result;
/* #line 64 */
}
/* #line 64 */
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
inline static cc2420_header_t * UniqueSendP_CC2420PacketBody_getHeader(message_t * msg){
/* #line 42 */
  nx_struct cc2420_header_t *result;
/* #line 42 */

/* #line 42 */
  result = CC2420PacketP_CC2420PacketBody_getHeader(msg);
/* #line 42 */

/* #line 42 */
  return result;
/* #line 42 */
}
/* #line 42 */
/* # 45 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
inline static error_t UniqueSendP_State_requestState(uint8_t reqState){
/* #line 45 */
  unsigned char result;
/* #line 45 */

/* #line 45 */
  result = StateImplP_State_requestState(2U, reqState);
/* #line 45 */

/* #line 45 */
  return result;
/* #line 45 */
}
/* #line 45 */
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc" */
static inline error_t UniqueSendP_Send_send(message_t *msg, uint8_t len)
/* #line 75 */
{
  error_t error;

/* #line 77 */
  if (UniqueSendP_State_requestState(UniqueSendP_S_SENDING) == SUCCESS) {
      __nesc_hton_leuint8(UniqueSendP_CC2420PacketBody_getHeader(msg)->dsn.data, UniqueSendP_localSendId++);

      if ((error = UniqueSendP_SubSend_send(msg, len)) != SUCCESS) {
          UniqueSendP_State_toIdle();
        }

      return error;
    }

  return EBUSY;
}

/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
inline static error_t CC2420ActiveMessageP_SubSend_send(message_t * msg, uint8_t len){
/* #line 64 */
  unsigned char result;
/* #line 64 */

/* #line 64 */
  result = UniqueSendP_Send_send(msg, len);
/* #line 64 */

/* #line 64 */
  return result;
/* #line 64 */
}
/* #line 64 */
/* # 55 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc" */
inline static cc2420_status_t CC2420TransmitP_TXCTRL_write(uint16_t data){
/* #line 55 */
  unsigned char result;
/* #line 55 */

/* #line 55 */
  result = CC2420SpiP_Reg_write(CC2420_TXCTRL, data);
/* #line 55 */

/* #line 55 */
  return result;
/* #line 55 */
}
/* #line 55 */
/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/SpiPacket.nc" */
inline static error_t CC2420SpiP_SpiPacket_send(uint8_t * txBuf, uint8_t * rxBuf, uint16_t len){
/* #line 59 */
  unsigned char result;
/* #line 59 */

/* #line 59 */
  result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiPacket_send(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0_CLIENT_ID, txBuf, rxBuf, len);
/* #line 59 */

/* #line 59 */
  return result;
/* #line 59 */
}
/* #line 59 */
/* # 34 "/Users/doina/tinyos-2.x/tos/interfaces/SpiByte.nc" */
inline static uint8_t CC2420SpiP_SpiByte_write(uint8_t tx){
/* #line 34 */
  unsigned char result;
/* #line 34 */

/* #line 34 */
  result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiByte_write(tx);
/* #line 34 */

/* #line 34 */
  return result;
/* #line 34 */
}
/* #line 34 */
/* # 126 "/Users/doina/tinyos-2.x/tos/system/StateImplP.nc" */
static inline bool StateImplP_State_isIdle(uint8_t id)
/* #line 126 */
{
  return StateImplP_State_isState(id, StateImplP_S_IDLE);
}

/* # 61 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
inline static bool CC2420SpiP_WorkingState_isIdle(void ){
/* #line 61 */
  unsigned char result;
/* #line 61 */

/* #line 61 */
  result = StateImplP_State_isIdle(0U);
/* #line 61 */

/* #line 61 */
  return result;
/* #line 61 */
}
/* #line 61 */
/* # 214 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static inline cc2420_status_t CC2420SpiP_Fifo_write(uint8_t addr, uint8_t *data, 
uint8_t len)
/* #line 215 */
{

  uint8_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 219 */
    {
      if (CC2420SpiP_WorkingState_isIdle()) {
          {
            unsigned char __nesc_temp = 
/* #line 221 */
            status;

            {
/* #line 221 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 221 */
              return __nesc_temp;
            }
          }
        }
    }
/* #line 225 */
    __nesc_atomic_end(__nesc_atomic); }
/* #line 225 */
  CC2420SpiP_m_addr = addr;

  status = CC2420SpiP_SpiByte_write(CC2420SpiP_m_addr);
  CC2420SpiP_SpiPacket_send(data, (void *)0, len);

  return status;
}

/* # 82 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
inline static cc2420_status_t CC2420TransmitP_TXFIFO_write(uint8_t * data, uint8_t length){
/* #line 82 */
  unsigned char result;
/* #line 82 */

/* #line 82 */
  result = CC2420SpiP_Fifo_write(CC2420_TXFIFO, data, length);
/* #line 82 */

/* #line 82 */
  return result;
/* #line 82 */
}
/* #line 82 */
/* # 361 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc" */
static inline void HplMsp430Usart0P_Usart_enableRxIntr(void )
/* #line 361 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 362 */
    {
      HplMsp430Usart0P_IFG1 &= ~(1 << 6);
      HplMsp430Usart0P_IE1 |= 1 << 6;
    }
/* #line 365 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 180 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc" */
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_enableRxIntr(void ){
/* #line 180 */
  HplMsp430Usart0P_Usart_enableRxIntr();
/* #line 180 */
}
/* #line 180 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone_task_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone_task);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 269 "/opt/local/lib/ncc/nesc_nx.h" */
static __inline  uint16_t __nesc_hton_uint16(void * target, uint16_t value)
/* #line 269 */
{
  uint8_t *base = target;

/* #line 271 */
  base[1] = value;
  base[0] = value >> 8;
  return value;
}

/* # 61 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationValue.nc" */
inline static void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_changed(void ){
/* #line 61 */
  TestDisseminationC_Value32_changed();
/* #line 61 */
}
/* #line 61 */
/* # 62 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminatorP.nc" */
static inline void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_changedTask_runTask(void )
/* #line 62 */
{
  /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_changed();
}

/* # 159 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static inline void DisseminationEngineImplP_AMSend_sendDone(message_t *msg, error_t error)
/* #line 159 */
{
  DisseminationEngineImplP_m_bufBusy = FALSE;
}

/* # 99 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
inline static void /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_sendDone(message_t * msg, error_t error){
/* #line 99 */
  DisseminationEngineImplP_AMSend_sendDone(msg, error);
/* #line 99 */
}
/* #line 99 */
/* # 57 "/Users/doina/tinyos-2.x/tos/system/AMQueueEntryP.nc" */
static inline void /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_Send_sendDone(message_t *m, error_t err)
/* #line 57 */
{
  /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_AMSend_sendDone(m, err);
}

/* # 155 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static inline void DisseminationEngineImplP_ProbeAMSend_sendDone(message_t *msg, error_t error)
/* #line 155 */
{
  DisseminationEngineImplP_m_bufBusy = FALSE;
}

/* # 99 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
inline static void /*DisseminationEngineP.DisseminationProbeSendC.AMQueueEntryP*/AMQueueEntryP_1_AMSend_sendDone(message_t * msg, error_t error){
/* #line 99 */
  DisseminationEngineImplP_ProbeAMSend_sendDone(msg, error);
/* #line 99 */
}
/* #line 99 */
/* # 57 "/Users/doina/tinyos-2.x/tos/system/AMQueueEntryP.nc" */
static inline void /*DisseminationEngineP.DisseminationProbeSendC.AMQueueEntryP*/AMQueueEntryP_1_Send_sendDone(message_t *m, error_t err)
/* #line 57 */
{
  /*DisseminationEngineP.DisseminationProbeSendC.AMQueueEntryP*/AMQueueEntryP_1_AMSend_sendDone(m, err);
}

/* # 207 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_default_sendDone(uint8_t id, message_t *msg, error_t err)
/* #line 207 */
{
}

/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_sendDone(uint8_t arg_0x22476f8, message_t * msg, error_t error){
/* #line 89 */
  switch (arg_0x22476f8) {
/* #line 89 */
    case 0U:
/* #line 89 */
      /*DisseminationEngineP.DisseminationSendC.AMQueueEntryP*/AMQueueEntryP_0_Send_sendDone(msg, error);
/* #line 89 */
      break;
/* #line 89 */
    case 1U:
/* #line 89 */
      /*DisseminationEngineP.DisseminationProbeSendC.AMQueueEntryP*/AMQueueEntryP_1_Send_sendDone(msg, error);
/* #line 89 */
      break;
/* #line 89 */
    default:
/* #line 89 */
      /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_default_sendDone(arg_0x22476f8, msg, error);
/* #line 89 */
      break;
/* #line 89 */
    }
/* #line 89 */
}
/* #line 89 */
/* # 118 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_CancelTask_runTask(void )
/* #line 118 */
{
  uint8_t i;
/* #line 119 */
  uint8_t j;
/* #line 119 */
  uint8_t mask;
/* #line 119 */
  uint8_t last;
  message_t *msg;

/* #line 121 */
  for (i = 0; i < 2 / 8 + 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_cancelMask[i]) {
          for (mask = 1, j = 0; j < 8; j++) {
              if (/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_cancelMask[i] & mask) {
                  last = i * 8 + j;
                  msg = /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_queue[last].msg;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_queue[last].msg = (void *)0;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_cancelMask[i] &= ~mask;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_sendDone(last, msg, ECANCEL);
                }
              mask <<= 1;
            }
        }
    }
}

/* #line 161 */
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_errorTask_runTask(void )
/* #line 161 */
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current, /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current].msg, FAIL);
}

/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_errorTask_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_errorTask);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 152 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline uint8_t CC2420ActiveMessageP_Packet_payloadLength(message_t *msg)
/* #line 152 */
{
  return __nesc_ntoh_leuint8(CC2420ActiveMessageP_CC2420PacketBody_getHeader(msg)->length.data) - CC2420_SIZE;
}

/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Packet.nc" */
inline static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Packet_payloadLength(message_t * msg){
/* #line 67 */
  unsigned char result;
/* #line 67 */

/* #line 67 */
  result = CC2420ActiveMessageP_Packet_payloadLength(msg);
/* #line 67 */

/* #line 67 */
  return result;
/* #line 67 */
}
/* #line 67 */
/* # 57 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_nextPacket(void )
/* #line 57 */
{
  uint8_t i;

/* #line 59 */
  /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current + 1) % 2;
  for (i = 0; i < 2; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current].msg == (void *)0 || 
      /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_cancelMask[/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current / 8] & (1 << /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current % 8)) 
        {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current + 1) % 2;
        }
      else {
          break;
        }
    }
  if (i >= 2) {
/* #line 70 */
    /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current = 2;
    }
}

/* #line 166 */
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_tryToSend(void )
/* #line 166 */
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_nextPacket();
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current < 2) {
      error_t nextErr;
      message_t *nextMsg = /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current].msg;
      am_id_t nextId = /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMPacket_type(nextMsg);
      am_addr_t nextDest = /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMPacket_destination(nextMsg);
      uint8_t len = /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Packet_payloadLength(nextMsg);

/* #line 174 */
      nextErr = /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMSend_send(nextId, nextDest, nextMsg, len);
      if (nextErr != SUCCESS) {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_errorTask_postTask();
        }
    }
}

/* # 92 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_size_type dt){
/* #line 92 */
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_startAt(t0, dt);
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
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_disableEvents();
}

/* # 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_stop(void ){
/* #line 62 */
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Alarm_stop();
/* #line 62 */
}
/* #line 62 */
/* # 91 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_stop();
}

/* # 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_stop(void ){
/* #line 62 */
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_stop();
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
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Counter_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Counter_get(void ){
/* #line 53 */
  unsigned long result;
/* #line 53 */

/* #line 53 */
  result = /*CounterMilli32C.Transform*/TransformCounterC_1_Counter_get();
/* #line 53 */

/* #line 53 */
  return result;
/* #line 53 */
}
/* #line 53 */
/* # 75 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Counter_get();
}

/* # 98 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_getNow(void ){
/* #line 98 */
  unsigned long result;
/* #line 98 */

/* #line 98 */
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_getNow();
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

/* # 81 "/Users/doina/tinyos-2.x/tos/system/BitVectorC.nc" */
static inline void /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_set(uint16_t bitnum)
{
  /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_m_bits[/*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_getIndex(bitnum)] |= /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_getMask(bitnum);
}

/* # 52 "/Users/doina/tinyos-2.x/tos/interfaces/BitVector.nc" */
inline static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Pending_set(uint16_t bitnum){
/* #line 52 */
  /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_set(bitnum);
/* #line 52 */
}
/* #line 52 */
/* # 188 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static inline uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_getdt(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_m_timers[num].dt;
}

/* # 140 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static uint32_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_getdt(void ){
/* #line 140 */
  unsigned long result;
/* #line 140 */

/* #line 140 */
  result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_getdt(1U);
/* #line 140 */

/* #line 140 */
  return result;
/* #line 140 */
}
/* #line 140 */
/* # 168 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimerImplP.nc" */
static inline void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_fired(void )
/* #line 168 */
{
  uint8_t i;
  uint32_t dt = /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_getdt();

  for (i = 0; i < 2U; i++) {
      uint32_t remaining = /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[i].time;

/* #line 174 */
      if (remaining != 0) {
          remaining -= dt;
          if (remaining == 0) {
              if (/*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[i].count < 1) {
                  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 178 */
                    {
                      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Pending_set(i);
                    }
/* #line 180 */
                    __nesc_atomic_end(__nesc_atomic); }
                  /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_timerTask_postTask();
                }

              /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_generateTime(i);







              /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[i].count = 0;
            }
          else {
              /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[i].time = remaining;
            }
        }
    }
  /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_adjustTimer();
}

/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_toggle(void )
/* #line 47 */
{
/* #line 47 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 47 */
    * (volatile uint8_t * )49U ^= 0x01 << 6;
/* #line 47 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_HplGeneralIO_toggle(void ){
/* #line 44 */
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_toggle();
/* #line 44 */
}
/* #line 44 */
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_GeneralIO_toggle(void )
/* #line 39 */
{
/* #line 39 */
  /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_HplGeneralIO_toggle();
}

/* # 31 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led2_toggle(void ){
/* #line 31 */
  /*PlatformLedsC.Led2Impl*/Msp430GpioC_9_GeneralIO_toggle();
/* #line 31 */
}
/* #line 31 */
/* # 103 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc" */
static inline void LedsP_Leds_led2Toggle(void )
/* #line 103 */
{
  LedsP_Led2_toggle();
  ;
/* #line 105 */
  ;
}

/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
inline static void TestDisseminationC_Leds_led2Toggle(void ){
/* #line 89 */
  LedsP_Leds_led2Toggle();
/* #line 89 */
}
/* #line 89 */
/* # 78 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminatorP.nc" */
static inline const /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_t */*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_get(void )
/* #line 78 */
{
  return &/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_valueCache;
}

/* # 47 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationValue.nc" */
inline static const TestDisseminationC_Value32_t *TestDisseminationC_Value32_get(void ){
/* #line 47 */
  unsigned long const *result;
/* #line 47 */

/* #line 47 */
  result = /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_get();
/* #line 47 */

/* #line 47 */
  return result;
/* #line 47 */
}
/* #line 47 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_changedTask_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_changedTask);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
inline static void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_newData(void ){
/* #line 50 */
  DisseminationEngineImplP_DisseminationCache_newData(9029U);
/* #line 50 */
}
/* #line 50 */
/* # 88 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminatorP.nc" */
static inline void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationUpdate_change(/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_t *newVal)
/* #line 88 */
{
  if (!/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_m_running) {
/* #line 89 */
      return;
    }
/* #line 90 */
  memcpy(&/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_valueCache, newVal, sizeof(/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_t ));

  /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_seqno = /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_seqno >> 16;
  /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_seqno++;
  if (/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_seqno == DISSEMINATION_SEQNO_UNKNOWN) {
/* #line 94 */
      /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_seqno++;
    }
/* #line 95 */
  /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_seqno = /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_seqno << 16;
  /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_seqno += TOS_NODE_ID;
  /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_newData();
  /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_changedTask_postTask();
}

/* # 52 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationUpdate.nc" */
inline static void TestDisseminationC_Update16_change(TestDisseminationC_Update16_t * newVal){
/* #line 52 */
  /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationUpdate_change(newVal);
/* #line 52 */
}
/* #line 52 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_changedTask_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_changedTask);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 50 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
inline static void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_newData(void ){
/* #line 50 */
  DisseminationEngineImplP_DisseminationCache_newData(4660U);
/* #line 50 */
}
/* #line 50 */
/* # 88 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminatorP.nc" */
static inline void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationUpdate_change(/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_t *newVal)
/* #line 88 */
{
  if (!/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_m_running) {
/* #line 89 */
      return;
    }
/* #line 90 */
  memcpy(&/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_valueCache, newVal, sizeof(/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_t ));

  /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_seqno = /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_seqno >> 16;
  /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_seqno++;
  if (/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_seqno == DISSEMINATION_SEQNO_UNKNOWN) {
/* #line 94 */
      /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_seqno++;
    }
/* #line 95 */
  /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_seqno = /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_seqno << 16;
  /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_seqno += TOS_NODE_ID;
  /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_newData();
  /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_changedTask_postTask();
}

/* # 52 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationUpdate.nc" */
inline static void TestDisseminationC_Update32_change(TestDisseminationC_Update32_t * newVal){
/* #line 52 */
  /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationUpdate_change(newVal);
/* #line 52 */
}
/* #line 52 */
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_HplGeneralIO_toggle(void ){
/* #line 44 */
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_toggle();
/* #line 44 */
}
/* #line 44 */
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_GeneralIO_toggle(void )
/* #line 39 */
{
/* #line 39 */
  /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_HplGeneralIO_toggle();
}

/* # 31 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led1_toggle(void ){
/* #line 31 */
  /*PlatformLedsC.Led1Impl*/Msp430GpioC_8_GeneralIO_toggle();
/* #line 31 */
}
/* #line 31 */
/* # 88 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc" */
static inline void LedsP_Leds_led1Toggle(void )
/* #line 88 */
{
  LedsP_Led1_toggle();
  ;
/* #line 90 */
  ;
}

/* # 72 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
inline static void TestDisseminationC_Leds_led1Toggle(void ){
/* #line 72 */
  LedsP_Leds_led1Toggle();
/* #line 72 */
}
/* #line 72 */
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_HplGeneralIO_toggle(void ){
/* #line 44 */
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_toggle();
/* #line 44 */
}
/* #line 44 */
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_GeneralIO_toggle(void )
/* #line 39 */
{
/* #line 39 */
  /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_HplGeneralIO_toggle();
}

/* # 31 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led0_toggle(void ){
/* #line 31 */
  /*PlatformLedsC.Led0Impl*/Msp430GpioC_7_GeneralIO_toggle();
/* #line 31 */
}
/* #line 31 */
/* # 73 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc" */
static inline void LedsP_Leds_led0Toggle(void )
/* #line 73 */
{
  LedsP_Led0_toggle();
  ;
/* #line 75 */
  ;
}

/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
inline static void TestDisseminationC_Leds_led0Toggle(void ){
/* #line 56 */
  LedsP_Leds_led0Toggle();
/* #line 56 */
}
/* #line 56 */
/* # 98 "TestDisseminationC.nc" */
static inline void TestDisseminationC_Timer_fired(void )
/* #line 98 */
{
  uint32_t newVal32 = 0xDEADBEEF;
  uint16_t newVal16 = 0xABCD;

  if (TOS_NODE_ID % 4 == 1) {
      TestDisseminationC_Leds_led0Toggle();
      TestDisseminationC_Leds_led1Toggle();
      TestDisseminationC_Update32_change(&newVal32);
      TestDisseminationC_Update16_change(&newVal16);
      ;
    }
  else 
/* #line 108 */
    {
      const uint32_t *newVal = TestDisseminationC_Value32_get();

/* #line 110 */
      if (*newVal == 123456) {
          TestDisseminationC_Leds_led2Toggle();
        }
    }
}

/* # 193 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_default_fired(uint8_t num)
{
}

/* # 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_fired(uint8_t arg_0x20a3dd8){
/* #line 72 */
  switch (arg_0x20a3dd8) {
/* #line 72 */
    case 1U:
/* #line 72 */
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_fired();
/* #line 72 */
      break;
/* #line 72 */
    case 2U:
/* #line 72 */
      TestDisseminationC_Timer_fired();
/* #line 72 */
      break;
/* #line 72 */
    default:
/* #line 72 */
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_default_fired(arg_0x20a3dd8);
/* #line 72 */
      break;
/* #line 72 */
    }
/* #line 72 */
}
/* #line 72 */
/* # 55 "/Users/doina/tinyos-2.x/tos/system/BitVectorC.nc" */
static inline uint16_t /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_getMask(uint16_t bitnum)
{
  return 1 << bitnum % /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_ELEMENT_SIZE;
}

/* #line 50 */
static inline uint16_t /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_getIndex(uint16_t bitnum)
{
  return bitnum / /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_ELEMENT_SIZE;
}

/* #line 81 */
static inline void /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_set(uint16_t bitnum)
{
  /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_m_bits[/*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_getIndex(bitnum)] |= /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_getMask(bitnum);
}

/* # 52 "/Users/doina/tinyos-2.x/tos/interfaces/BitVector.nc" */
inline static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Changed_set(uint16_t bitnum){
/* #line 52 */
  /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_set(bitnum);
/* #line 52 */
}
/* #line 52 */
/* # 41 "/Users/doina/tinyos-2.x/tos/interfaces/Random.nc" */
inline static uint16_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Random_rand16(void ){
/* #line 41 */
  unsigned int result;
/* #line 41 */

/* #line 41 */
  result = RandomMlcgC_Random_rand16();
/* #line 41 */

/* #line 41 */
  return result;
/* #line 41 */
}
/* #line 41 */
/* # 178 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static inline uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_getNow(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_getNow();
}

/* # 125 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static uint32_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_getNow(void ){
/* #line 125 */
  unsigned long result;
/* #line 125 */

/* #line 125 */
  result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_getNow(1U);
/* #line 125 */

/* #line 125 */
  return result;
/* #line 125 */
}
/* #line 125 */
/* # 183 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static inline uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_gett0(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_m_timers[num].t0;
}

/* # 133 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static uint32_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_gett0(void ){
/* #line 133 */
  unsigned long result;
/* #line 133 */

/* #line 133 */
  result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_gett0(1U);
/* #line 133 */

/* #line 133 */
  return result;
/* #line 133 */
}
/* #line 133 */
/* # 76 "/Users/doina/tinyos-2.x/tos/system/BitVectorC.nc" */
static inline bool /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_get(uint16_t bitnum)
{
  return /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_m_bits[/*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_getIndex(bitnum)] & /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_getMask(bitnum) ? TRUE : FALSE;
}

/* # 46 "/Users/doina/tinyos-2.x/tos/interfaces/BitVector.nc" */
inline static bool /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Changed_get(uint16_t bitnum){
/* #line 46 */
  unsigned char result;
/* #line 46 */

/* #line 46 */
  result = /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_get(bitnum);
/* #line 46 */

/* #line 46 */
  return result;
/* #line 46 */
}
/* #line 46 */
/* # 86 "/Users/doina/tinyos-2.x/tos/system/BitVectorC.nc" */
static inline void /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_clear(uint16_t bitnum)
{
  /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_m_bits[/*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_getIndex(bitnum)] &= ~/*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_getMask(bitnum);
}

/* # 58 "/Users/doina/tinyos-2.x/tos/interfaces/BitVector.nc" */
inline static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Changed_clear(uint16_t bitnum){
/* #line 58 */
  /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_clear(bitnum);
/* #line 58 */
}
/* #line 58 */
/* # 148 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_getNow(), dt, TRUE);
}

/* # 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_startOneShot(uint32_t dt){
/* #line 62 */
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startOneShot(1U, dt);
/* #line 62 */
}
/* #line 62 */
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
/* # 153 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_m_timers[num].isrunning = FALSE;
}

/* # 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_stop(void ){
/* #line 67 */
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_stop(1U);
/* #line 67 */
}
/* #line 67 */
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
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 82 */
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type __nesc_temp = 
/* #line 82 */
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_dt;

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
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_getAlarm();
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

/* # 94 "/Users/doina/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc" */
static inline message_t *CC2420TinyosNetworkP_NonTinyosReceive_default_receive(uint8_t networkId, message_t *msg, void *payload, uint8_t len)
/* #line 94 */
{
  return msg;
}

/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
inline static message_t * CC2420TinyosNetworkP_NonTinyosReceive_receive(uint8_t arg_0x218d9b8, message_t * msg, void * payload, uint8_t len){
/* #line 67 */
  nx_struct message_t *result;
/* #line 67 */

/* #line 67 */
    result = CC2420TinyosNetworkP_NonTinyosReceive_default_receive(arg_0x218d9b8, msg, payload, len);
/* #line 67 */

/* #line 67 */
  return result;
/* #line 67 */
}
/* #line 67 */
/* # 246 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline message_t *CC2420ActiveMessageP_Snoop_default_receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
/* #line 246 */
{
  return msg;
}

/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
inline static message_t * CC2420ActiveMessageP_Snoop_receive(am_id_t arg_0x16df220, message_t * msg, void * payload, uint8_t len){
/* #line 67 */
  nx_struct message_t *result;
/* #line 67 */

/* #line 67 */
    result = CC2420ActiveMessageP_Snoop_default_receive(arg_0x16df220, msg, payload, len);
/* #line 67 */

/* #line 67 */
  return result;
/* #line 67 */
}
/* #line 67 */
/* # 264 "/opt/local/lib/ncc/nesc_nx.h" */
static __inline  uint16_t __nesc_ntoh_uint16(const void * source)
/* #line 264 */
{
  const uint8_t *base = source;

/* #line 266 */
  return ((uint16_t )base[0] << 8) | base[1];
}

/* # 142 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimerImplP.nc" */
static inline void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_incrementCounter(uint8_t id)
/* #line 142 */
{
  /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].count++;
}

/* # 253 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static inline void DisseminationEngineImplP_TrickleTimer_default_incrementCounter(uint16_t key)
/* #line 253 */
{
}

/* # 77 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
inline static void DisseminationEngineImplP_TrickleTimer_incrementCounter(uint16_t arg_0x21c4c10){
/* #line 77 */
  switch (arg_0x21c4c10) {
/* #line 77 */
    case 4660U:
/* #line 77 */
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_incrementCounter(/*TestDisseminationAppC.Object32C*/DisseminatorC_0_TIMER_ID);
/* #line 77 */
      break;
/* #line 77 */
    case 9029U:
/* #line 77 */
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_incrementCounter(/*TestDisseminationAppC.Object16C*/DisseminatorC_1_TIMER_ID);
/* #line 77 */
      break;
/* #line 77 */
    default:
/* #line 77 */
      DisseminationEngineImplP_TrickleTimer_default_incrementCounter(arg_0x21c4c10);
/* #line 77 */
      break;
/* #line 77 */
    }
/* #line 77 */
}
/* #line 77 */
/* # 251 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static inline void DisseminationEngineImplP_TrickleTimer_default_reset(uint16_t key)
/* #line 251 */
{
}

/* # 72 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
inline static void DisseminationEngineImplP_TrickleTimer_reset(uint16_t arg_0x21c4c10){
/* #line 72 */
  switch (arg_0x21c4c10) {
/* #line 72 */
    case 4660U:
/* #line 72 */
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_reset(/*TestDisseminationAppC.Object32C*/DisseminatorC_0_TIMER_ID);
/* #line 72 */
      break;
/* #line 72 */
    case 9029U:
/* #line 72 */
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_reset(/*TestDisseminationAppC.Object16C*/DisseminatorC_1_TIMER_ID);
/* #line 72 */
      break;
/* #line 72 */
    default:
/* #line 72 */
      DisseminationEngineImplP_TrickleTimer_default_reset(arg_0x21c4c10);
/* #line 72 */
      break;
/* #line 72 */
    }
/* #line 72 */
}
/* #line 72 */
/* # 240 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static inline 
/* #line 239 */
void 
DisseminationEngineImplP_DisseminationCache_default_storeData(uint16_t key, void *data, 
uint8_t size, 
uint32_t seqno)
/* #line 242 */
{
}

/* # 48 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
inline static void DisseminationEngineImplP_DisseminationCache_storeData(uint16_t arg_0x21c4120, void * data, uint8_t size, uint32_t seqno){
/* #line 48 */
  switch (arg_0x21c4120) {
/* #line 48 */
    case 4660U:
/* #line 48 */
      /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_storeData(data, size, seqno);
/* #line 48 */
      break;
/* #line 48 */
    case 9029U:
/* #line 48 */
      /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_storeData(data, size, seqno);
/* #line 48 */
      break;
/* #line 48 */
    default:
/* #line 48 */
      DisseminationEngineImplP_DisseminationCache_default_storeData(arg_0x21c4120, data, size, seqno);
/* #line 48 */
      break;
/* #line 48 */
    }
/* #line 48 */
}
/* #line 48 */
/* # 117 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminatorP.nc" */
static inline uint32_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_requestSeqno(void )
/* #line 117 */
{
  return /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_seqno;
}

/* #line 117 */
static inline uint32_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_requestSeqno(void )
/* #line 117 */
{
  return /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_seqno;
}

/* # 245 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static inline 
/* #line 244 */
uint32_t 
DisseminationEngineImplP_DisseminationCache_default_requestSeqno(uint16_t key)
/* #line 245 */
{
/* #line 245 */
  return DISSEMINATION_SEQNO_UNKNOWN;
}

/* # 49 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
inline static uint32_t DisseminationEngineImplP_DisseminationCache_requestSeqno(uint16_t arg_0x21c4120){
/* #line 49 */
  unsigned long result;
/* #line 49 */

/* #line 49 */
  switch (arg_0x21c4120) {
/* #line 49 */
    case 4660U:
/* #line 49 */
      result = /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_requestSeqno();
/* #line 49 */
      break;
/* #line 49 */
    case 9029U:
/* #line 49 */
      result = /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_requestSeqno();
/* #line 49 */
      break;
/* #line 49 */
    default:
/* #line 49 */
      result = DisseminationEngineImplP_DisseminationCache_default_requestSeqno(arg_0x21c4120);
/* #line 49 */
      break;
/* #line 49 */
    }
/* #line 49 */

/* #line 49 */
  return result;
/* #line 49 */
}
/* #line 49 */
/* # 163 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static inline message_t *DisseminationEngineImplP_Receive_receive(message_t *msg, 
void *payload, 
uint8_t len)
/* #line 165 */
{

  dissemination_message_t *dMsg = 
  (dissemination_message_t *)payload;

  uint16_t key = __nesc_ntoh_uint16(dMsg->key.data);
  uint32_t incomingSeqno = __nesc_ntoh_uint32(dMsg->seqno.data);
  uint32_t currentSeqno = DisseminationEngineImplP_DisseminationCache_requestSeqno(key);

  if (!DisseminationEngineImplP_m_running) {
/* #line 174 */
      return msg;
    }
  if (currentSeqno == DISSEMINATION_SEQNO_UNKNOWN && 
  incomingSeqno != DISSEMINATION_SEQNO_UNKNOWN) {

      DisseminationEngineImplP_DisseminationCache_storeData(key, 
      dMsg->data, 
      len - sizeof(dissemination_message_t ), 
      incomingSeqno);

      DisseminationEngineImplP_TrickleTimer_reset(key);
      return msg;
    }

  if (incomingSeqno == DISSEMINATION_SEQNO_UNKNOWN && 
  currentSeqno != DISSEMINATION_SEQNO_UNKNOWN) {

      DisseminationEngineImplP_TrickleTimer_reset(key);
      return msg;
    }

  if ((int32_t )(incomingSeqno - currentSeqno) > 0) {

      DisseminationEngineImplP_DisseminationCache_storeData(key, 
      dMsg->data, 
      len - sizeof(dissemination_message_t ), 
      incomingSeqno);
      ;
      DisseminationEngineImplP_TrickleTimer_reset(key);
    }
  else {
/* #line 204 */
    if ((int32_t )(incomingSeqno - currentSeqno) == 0) {

        DisseminationEngineImplP_TrickleTimer_incrementCounter(key);
      }
    else {


        DisseminationEngineImplP_sendObject(key);
      }
    }


  return msg;
}

static inline message_t *DisseminationEngineImplP_ProbeReceive_receive(message_t *msg, 
void *payload, 
uint8_t len)
/* #line 221 */
{

  dissemination_probe_message_t *dpMsg = 
  (dissemination_probe_message_t *)payload;

  if (!DisseminationEngineImplP_m_running) {
/* #line 226 */
      return msg;
    }
  if (DisseminationEngineImplP_DisseminationCache_requestSeqno(__nesc_ntoh_uint16(dpMsg->key.data)) != 
  DISSEMINATION_SEQNO_UNKNOWN) {
      DisseminationEngineImplP_sendObject(__nesc_ntoh_uint16(dpMsg->key.data));
    }

  return msg;
}

/* # 242 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline message_t *CC2420ActiveMessageP_Receive_default_receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
/* #line 242 */
{
  return msg;
}

/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
inline static message_t * CC2420ActiveMessageP_Receive_receive(am_id_t arg_0x16e1b78, message_t * msg, void * payload, uint8_t len){
/* #line 67 */
  nx_struct message_t *result;
/* #line 67 */

/* #line 67 */
  switch (arg_0x16e1b78) {
/* #line 67 */
    case 96:
/* #line 67 */
      result = DisseminationEngineImplP_Receive_receive(msg, payload, len);
/* #line 67 */
      break;
/* #line 67 */
    case 97:
/* #line 67 */
      result = DisseminationEngineImplP_ProbeReceive_receive(msg, payload, len);
/* #line 67 */
      break;
/* #line 67 */
    default:
/* #line 67 */
      result = CC2420ActiveMessageP_Receive_default_receive(arg_0x16e1b78, msg, payload, len);
/* #line 67 */
      break;
/* #line 67 */
    }
/* #line 67 */

/* #line 67 */
  return result;
/* #line 67 */
}
/* #line 67 */
/* # 61 "/Users/doina/tinyos-2.x/tos/system/ActiveMessageAddressC.nc" */
static inline am_addr_t ActiveMessageAddressC_ActiveMessageAddress_amAddress(void )
/* #line 61 */
{
  return ActiveMessageAddressC_amAddress();
}

/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc" */
inline static am_addr_t CC2420ActiveMessageP_ActiveMessageAddress_amAddress(void ){
/* #line 50 */
  unsigned int result;
/* #line 50 */

/* #line 50 */
  result = ActiveMessageAddressC_ActiveMessageAddress_amAddress();
/* #line 50 */

/* #line 50 */
  return result;
/* #line 50 */
}
/* #line 50 */
/* # 93 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline am_addr_t CC2420ActiveMessageP_AMPacket_address(void )
/* #line 93 */
{
  return CC2420ActiveMessageP_ActiveMessageAddress_amAddress();
}

/* #line 117 */
static inline bool CC2420ActiveMessageP_AMPacket_isForMe(message_t *amsg)
/* #line 117 */
{
  return CC2420ActiveMessageP_AMPacket_destination(amsg) == CC2420ActiveMessageP_AMPacket_address() || 
  CC2420ActiveMessageP_AMPacket_destination(amsg) == AM_BROADCAST_ADDR;
}

/* # 47 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
inline static cc2420_metadata_t * CC2420ActiveMessageP_CC2420PacketBody_getMetadata(message_t * msg){
/* #line 47 */
  nx_struct cc2420_metadata_t *result;
/* #line 47 */

/* #line 47 */
  result = CC2420PacketP_CC2420PacketBody_getMetadata(msg);
/* #line 47 */

/* #line 47 */
  return result;
/* #line 47 */
}
/* #line 47 */
/* # 176 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline message_t *CC2420ActiveMessageP_SubReceive_receive(message_t *msg, void *payload, uint8_t len)
/* #line 176 */
{

  if (! __nesc_ntoh_int8(CC2420ActiveMessageP_CC2420PacketBody_getMetadata(msg)->crc.data)) {
      return msg;
    }

  if (CC2420ActiveMessageP_AMPacket_isForMe(msg)) {
      return CC2420ActiveMessageP_Receive_receive(CC2420ActiveMessageP_AMPacket_type(msg), msg, payload, len);
    }
  else {
      return CC2420ActiveMessageP_Snoop_receive(CC2420ActiveMessageP_AMPacket_type(msg), msg, payload, len);
    }
}

/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
inline static message_t * UniqueReceiveP_Receive_receive(message_t * msg, void * payload, uint8_t len){
/* #line 67 */
  nx_struct message_t *result;
/* #line 67 */

/* #line 67 */
  result = CC2420ActiveMessageP_SubReceive_receive(msg, payload, len);
/* #line 67 */

/* #line 67 */
  return result;
/* #line 67 */
}
/* #line 67 */
/* # 137 "/Users/doina/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc" */
static inline void UniqueReceiveP_insert(uint16_t msgSource, uint8_t msgDsn)
/* #line 137 */
{
  uint8_t element = UniqueReceiveP_recycleSourceElement;
  bool increment = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 141 */
    {
      if (element == UniqueReceiveP_INVALID_ELEMENT || UniqueReceiveP_writeIndex == element) {

          element = UniqueReceiveP_writeIndex;
          increment = TRUE;
        }

      UniqueReceiveP_receivedMessages[element].source = msgSource;
      UniqueReceiveP_receivedMessages[element].dsn = msgDsn;
      if (increment) {
          UniqueReceiveP_writeIndex++;
          UniqueReceiveP_writeIndex %= 4;
        }
    }
/* #line 154 */
    __nesc_atomic_end(__nesc_atomic); }
}


static inline message_t *UniqueReceiveP_DuplicateReceive_default_receive(message_t *msg, void *payload, uint8_t len)
/* #line 158 */
{
  return msg;
}

/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
inline static message_t * UniqueReceiveP_DuplicateReceive_receive(message_t * msg, void * payload, uint8_t len){
/* #line 67 */
  nx_struct message_t *result;
/* #line 67 */

/* #line 67 */
  result = UniqueReceiveP_DuplicateReceive_default_receive(msg, payload, len);
/* #line 67 */

/* #line 67 */
  return result;
/* #line 67 */
}
/* #line 67 */
/* # 111 "/Users/doina/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc" */
static inline bool UniqueReceiveP_hasSeen(uint16_t msgSource, uint8_t msgDsn)
/* #line 111 */
{
  int i;

/* #line 113 */
  UniqueReceiveP_recycleSourceElement = UniqueReceiveP_INVALID_ELEMENT;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 115 */
    {
      for (i = 0; i < 4; i++) {
          if (UniqueReceiveP_receivedMessages[i].source == msgSource) {
              if (UniqueReceiveP_receivedMessages[i].dsn == msgDsn) {

                  {
                    unsigned char __nesc_temp = 
/* #line 120 */
                    TRUE;

                    {
/* #line 120 */
                      __nesc_atomic_end(__nesc_atomic); 
/* #line 120 */
                      return __nesc_temp;
                    }
                  }
                }
/* #line 123 */
              UniqueReceiveP_recycleSourceElement = i;
            }
        }
    }
/* #line 126 */
    __nesc_atomic_end(__nesc_atomic); }

  return FALSE;
}

/* # 42 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
inline static cc2420_header_t * UniqueReceiveP_CC2420PacketBody_getHeader(message_t * msg){
/* #line 42 */
  nx_struct cc2420_header_t *result;
/* #line 42 */

/* #line 42 */
  result = CC2420PacketP_CC2420PacketBody_getHeader(msg);
/* #line 42 */

/* #line 42 */
  return result;
/* #line 42 */
}
/* #line 42 */
/* # 85 "/Users/doina/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc" */
static inline message_t *UniqueReceiveP_SubReceive_receive(message_t *msg, void *payload, 
uint8_t len)
/* #line 86 */
{
  uint16_t msgSource = __nesc_ntoh_leuint16(UniqueReceiveP_CC2420PacketBody_getHeader(msg)->src.data);
  uint8_t msgDsn = __nesc_ntoh_leuint8(UniqueReceiveP_CC2420PacketBody_getHeader(msg)->dsn.data);

  if (UniqueReceiveP_hasSeen(msgSource, msgDsn)) {
      return UniqueReceiveP_DuplicateReceive_receive(msg, payload, len);
    }
  else {
      UniqueReceiveP_insert(msgSource, msgDsn);
      return UniqueReceiveP_Receive_receive(msg, payload, len);
    }
}

/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
inline static message_t * CC2420TinyosNetworkP_Receive_receive(message_t * msg, void * payload, uint8_t len){
/* #line 67 */
  nx_struct message_t *result;
/* #line 67 */

/* #line 67 */
  result = UniqueReceiveP_SubReceive_receive(msg, payload, len);
/* #line 67 */

/* #line 67 */
  return result;
/* #line 67 */
}
/* #line 67 */
/* # 84 "/Users/doina/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc" */
static inline message_t *CC2420TinyosNetworkP_SubReceive_receive(message_t *msg, void *payload, uint8_t len)
/* #line 84 */
{
  if (__nesc_ntoh_leuint8(CC2420TinyosNetworkP_CC2420PacketBody_getHeader(msg)->network.data) == 0x3f) {
      return CC2420TinyosNetworkP_Receive_receive(msg, payload, len);
    }
  else {
      return CC2420TinyosNetworkP_NonTinyosReceive_receive(__nesc_ntoh_leuint8(CC2420TinyosNetworkP_CC2420PacketBody_getHeader(msg)->network.data), msg, payload, len);
    }
}

/* # 67 "/Users/doina/tinyos-2.x/tos/interfaces/Receive.nc" */
inline static message_t * CC2420ReceiveP_Receive_receive(message_t * msg, void * payload, uint8_t len){
/* #line 67 */
  nx_struct message_t *result;
/* #line 67 */

/* #line 67 */
  result = CC2420TinyosNetworkP_SubReceive_receive(msg, payload, len);
/* #line 67 */

/* #line 67 */
  return result;
/* #line 67 */
}
/* #line 67 */
/* # 64 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc" */
inline static uint16_t CC2420ReceiveP_CC2420Config_getShortAddr(void ){
/* #line 64 */
  unsigned int result;
/* #line 64 */

/* #line 64 */
  result = CC2420ControlP_CC2420Config_getShortAddr();
/* #line 64 */

/* #line 64 */
  return result;
/* #line 64 */
}
/* #line 64 */
/* # 332 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline bool CC2420ControlP_CC2420Config_isAddressRecognitionEnabled(void )
/* #line 332 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 333 */
    {
      unsigned char __nesc_temp = 
/* #line 333 */
      CC2420ControlP_addressRecognition;

      {
/* #line 333 */
        __nesc_atomic_end(__nesc_atomic); 
/* #line 333 */
        return __nesc_temp;
      }
    }
/* #line 335 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 86 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc" */
inline static bool CC2420ReceiveP_CC2420Config_isAddressRecognitionEnabled(void ){
/* #line 86 */
  unsigned char result;
/* #line 86 */

/* #line 86 */
  result = CC2420ControlP_CC2420Config_isAddressRecognitionEnabled();
/* #line 86 */

/* #line 86 */
  return result;
/* #line 86 */
}
/* #line 86 */
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
inline static cc2420_header_t * CC2420ReceiveP_CC2420PacketBody_getHeader(message_t * msg){
/* #line 42 */
  nx_struct cc2420_header_t *result;
/* #line 42 */

/* #line 42 */
  result = CC2420PacketP_CC2420PacketBody_getHeader(msg);
/* #line 42 */

/* #line 42 */
  return result;
/* #line 42 */
}
/* #line 42 */
/* # 461 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
static inline bool CC2420ReceiveP_passesAddressCheck(message_t *msg)
/* #line 461 */
{
  cc2420_header_t *header = CC2420ReceiveP_CC2420PacketBody_getHeader(msg);

  if (!CC2420ReceiveP_CC2420Config_isAddressRecognitionEnabled()) {
      return TRUE;
    }

  return __nesc_ntoh_leuint16(header->dest.data) == CC2420ReceiveP_CC2420Config_getShortAddr()
   || __nesc_ntoh_leuint16(header->dest.data) == AM_BROADCAST_ADDR;
}

/* # 47 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
inline static cc2420_metadata_t * CC2420ReceiveP_CC2420PacketBody_getMetadata(message_t * msg){
/* #line 47 */
  nx_struct cc2420_metadata_t *result;
/* #line 47 */

/* #line 47 */
  result = CC2420PacketP_CC2420PacketBody_getMetadata(msg);
/* #line 47 */

/* #line 47 */
  return result;
/* #line 47 */
}
/* #line 47 */
/* # 339 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
static inline void CC2420ReceiveP_receiveDone_task_runTask(void )
/* #line 339 */
{
  cc2420_metadata_t *metadata = CC2420ReceiveP_CC2420PacketBody_getMetadata(CC2420ReceiveP_m_p_rx_buf);
  cc2420_header_t *header = CC2420ReceiveP_CC2420PacketBody_getHeader(CC2420ReceiveP_m_p_rx_buf);
  uint8_t length = __nesc_ntoh_leuint8(header->length.data);
  uint8_t tmpLen  = sizeof(message_t ) - ((size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

  __nesc_hton_int8(metadata->crc.data, buf[length] >> 7);
  __nesc_hton_uint8(metadata->lqi.data, buf[length] & 0x7f);
  __nesc_hton_uint8(metadata->rssi.data, buf[length - 1]);

  if (CC2420ReceiveP_passesAddressCheck(CC2420ReceiveP_m_p_rx_buf) && length >= CC2420_SIZE) {
      CC2420ReceiveP_m_p_rx_buf = CC2420ReceiveP_Receive_receive(CC2420ReceiveP_m_p_rx_buf, CC2420ReceiveP_m_p_rx_buf->data, 
      length - CC2420_SIZE);
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 355 */
    CC2420ReceiveP_receivingPacket = FALSE;
/* #line 355 */
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ReceiveP_waitForNextPacket();
}

/* # 178 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static inline uint8_t CC2420SpiP_Resource_isOwner(uint8_t id)
/* #line 178 */
{
  /* atomic removed: atomic calls only */
/* #line 179 */
  {
    unsigned char __nesc_temp = 
/* #line 179 */
    CC2420SpiP_m_holder == id;

/* #line 179 */
    return __nesc_temp;
  }
}

/* # 118 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static bool CC2420ReceiveP_SpiResource_isOwner(void ){
/* #line 118 */
  unsigned char result;
/* #line 118 */

/* #line 118 */
  result = CC2420SpiP_Resource_isOwner(/*CC2420ReceiveC.Spi*/CC2420SpiC_4_CLIENT_ID);
/* #line 118 */

/* #line 118 */
  return result;
/* #line 118 */
}
/* #line 118 */
/* #line 87 */
inline static error_t CC2420ReceiveP_SpiResource_immediateRequest(void ){
/* #line 87 */
  unsigned char result;
/* #line 87 */

/* #line 87 */
  result = CC2420SpiP_Resource_immediateRequest(/*CC2420ReceiveC.Spi*/CC2420SpiC_4_CLIENT_ID);
/* #line 87 */

/* #line 87 */
  return result;
/* #line 87 */
}
/* #line 87 */
/* #line 78 */
inline static error_t CC2420ReceiveP_SpiResource_request(void ){
/* #line 78 */
  unsigned char result;
/* #line 78 */

/* #line 78 */
  result = CC2420SpiP_Resource_request(/*CC2420ReceiveC.Spi*/CC2420SpiC_4_CLIENT_ID);
/* #line 78 */

/* #line 78 */
  return result;
/* #line 78 */
}
/* #line 78 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t CC2420SpiP_grant_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(CC2420SpiP_grant);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 184 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static inline void CC2420SpiP_SpiResource_granted(void )
/* #line 184 */
{
  CC2420SpiP_grant_postTask();
}

/* # 119 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_default_granted(uint8_t id)
/* #line 119 */
{
}

/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_granted(uint8_t arg_0x1c691b8){
/* #line 92 */
  switch (arg_0x1c691b8) {
/* #line 92 */
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0_CLIENT_ID:
/* #line 92 */
      CC2420SpiP_SpiResource_granted();
/* #line 92 */
      break;
/* #line 92 */
    default:
/* #line 92 */
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_default_granted(arg_0x1c691b8);
/* #line 92 */
      break;
/* #line 92 */
    }
/* #line 92 */
}
/* #line 92 */
/* # 95 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_granted(uint8_t id)
/* #line 95 */
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Resource_granted(id);
}

/* # 199 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_default_granted(uint8_t id)
/* #line 199 */
{
}

/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_granted(uint8_t arg_0x1db7230){
/* #line 92 */
  switch (arg_0x1db7230) {
/* #line 92 */
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C_0_CLIENT_ID:
/* #line 92 */
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartResource_granted(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0_CLIENT_ID);
/* #line 92 */
      break;
/* #line 92 */
    default:
/* #line 92 */
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_default_granted(arg_0x1db7230);
/* #line 92 */
      break;
/* #line 92 */
    }
/* #line 92 */
}
/* #line 92 */
/* # 187 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_grantedTask_runTask(void )
/* #line 187 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 188 */
    {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_reqResId;
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_BUSY;
    }
/* #line 191 */
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceConfigure_configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId);
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_granted(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId);
}

/* # 190 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiPacket_default_sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
/* #line 190 */
{
}

/* # 71 "/Users/doina/tinyos-2.x/tos/interfaces/SpiPacket.nc" */
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiPacket_sendDone(uint8_t arg_0x1c67740, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
/* #line 71 */
  switch (arg_0x1c67740) {
/* #line 71 */
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C_0_CLIENT_ID:
/* #line 71 */
      CC2420SpiP_SpiPacket_sendDone(txBuf, rxBuf, len, error);
/* #line 71 */
      break;
/* #line 71 */
    default:
/* #line 71 */
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiPacket_default_sendDone(arg_0x1c67740, txBuf, rxBuf, len, error);
/* #line 71 */
      break;
/* #line 71 */
    }
/* #line 71 */
}
/* #line 71 */
/* # 183 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone(void )
/* #line 183 */
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiPacket_sendDone(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_client, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_tx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_rx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_len, 
  SUCCESS);
}

/* #line 166 */
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone_task_runTask(void )
/* #line 166 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 167 */
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone();
/* #line 167 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 463 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static inline void CC2420TransmitP_TXFIFO_readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
/* #line 464 */
{
}

/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t CC2420ReceiveP_SpiResource_release(void ){
/* #line 110 */
  unsigned char result;
/* #line 110 */

/* #line 110 */
  result = CC2420SpiP_Resource_release(/*CC2420ReceiveC.Spi*/CC2420SpiC_4_CLIENT_ID);
/* #line 110 */

/* #line 110 */
  return result;
/* #line 110 */
}
/* #line 110 */
/* # 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420ReceiveP_CSN_set(void ){
/* #line 29 */
  /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_set();
/* #line 29 */
}
/* #line 29 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t CC2420ReceiveP_receiveDone_task_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(CC2420ReceiveP_receiveDone_task);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc" */
inline static cc2420_metadata_t * CC2420TransmitP_CC2420PacketBody_getMetadata(message_t * msg){
/* #line 47 */
  nx_struct cc2420_metadata_t *result;
/* #line 47 */

/* #line 47 */
  result = CC2420PacketP_CC2420PacketBody_getMetadata(msg);
/* #line 47 */

/* #line 47 */
  return result;
/* #line 47 */
}
/* #line 47 */
/* # 365 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static inline void CC2420TransmitP_CC2420Receive_receive(uint8_t type, message_t *ack_msg)
/* #line 365 */
{
  cc2420_header_t *ack_header;
  cc2420_header_t *msg_header;
  cc2420_metadata_t *msg_metadata;
  uint8_t *ack_buf;
  uint8_t length;

  if (type == IEEE154_TYPE_ACK && CC2420TransmitP_m_msg) {
      ack_header = CC2420TransmitP_CC2420PacketBody_getHeader(ack_msg);
      msg_header = CC2420TransmitP_CC2420PacketBody_getHeader(CC2420TransmitP_m_msg);


      if (CC2420TransmitP_m_state == CC2420TransmitP_S_ACK_WAIT && __nesc_ntoh_leuint8(msg_header->dsn.data) == __nesc_ntoh_leuint8(ack_header->dsn.data)) {
          CC2420TransmitP_BackoffTimer_stop();

          msg_metadata = CC2420TransmitP_CC2420PacketBody_getMetadata(CC2420TransmitP_m_msg);
          ack_buf = (uint8_t *)ack_header;
          length = __nesc_ntoh_leuint8(ack_header->length.data);

          __nesc_hton_int8(msg_metadata->ack.data, TRUE);
          __nesc_hton_uint8(msg_metadata->rssi.data, ack_buf[length - 1]);
          __nesc_hton_uint8(msg_metadata->lqi.data, ack_buf[length] & 0x7f);
          CC2420TransmitP_signalDone(SUCCESS);
        }
    }
}

/* # 63 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc" */
inline static void CC2420ReceiveP_CC2420Receive_receive(uint8_t type, message_t * message){
/* #line 63 */
  CC2420TransmitP_CC2420Receive_receive(type, message);
/* #line 63 */
}
/* #line 63 */
/* # 59 "/Users/doina/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc" */
inline static void CC2420ReceiveP_PacketTimeStamp_clear(message_t * msg){
/* #line 59 */
  CC2420PacketP_PacketTimeStamp32khz_clear(msg);
/* #line 59 */
}
/* #line 59 */








inline static void CC2420ReceiveP_PacketTimeStamp_set(message_t * msg, CC2420ReceiveP_PacketTimeStamp_size_type value){
/* #line 67 */
  CC2420PacketP_PacketTimeStamp32khz_set(msg, value);
/* #line 67 */
}
/* #line 67 */
/* # 48 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP_0_IO_getRaw(void )
/* #line 48 */
{
/* #line 48 */
  return * (volatile uint8_t * )32U & (0x01 << 0);
}

/* #line 49 */
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP_0_IO_get(void )
/* #line 49 */
{
/* #line 49 */
  return /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP_0_IO_getRaw() != 0;
}

/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC_3_HplGeneralIO_get(void ){
/* #line 59 */
  unsigned char result;
/* #line 59 */

/* #line 59 */
  result = /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP_0_IO_get();
/* #line 59 */

/* #line 59 */
  return result;
/* #line 59 */
}
/* #line 59 */
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC_3_GeneralIO_get(void )
/* #line 40 */
{
/* #line 40 */
  return /*HplCC2420PinsC.FIFOPM*/Msp430GpioC_3_HplGeneralIO_get();
}

/* # 32 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static bool CC2420ReceiveP_FIFOP_get(void ){
/* #line 32 */
  unsigned char result;
/* #line 32 */

/* #line 32 */
  result = /*HplCC2420PinsC.FIFOPM*/Msp430GpioC_3_GeneralIO_get();
/* #line 32 */

/* #line 32 */
  return result;
/* #line 32 */
}
/* #line 32 */
/* # 48 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP_3_IO_getRaw(void )
/* #line 48 */
{
/* #line 48 */
  return * (volatile uint8_t * )32U & (0x01 << 3);
}

/* #line 49 */
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP_3_IO_get(void )
/* #line 49 */
{
/* #line 49 */
  return /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP_3_IO_getRaw() != 0;
}

/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC_2_HplGeneralIO_get(void ){
/* #line 59 */
  unsigned char result;
/* #line 59 */

/* #line 59 */
  result = /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP_3_IO_get();
/* #line 59 */

/* #line 59 */
  return result;
/* #line 59 */
}
/* #line 59 */
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC_2_GeneralIO_get(void )
/* #line 40 */
{
/* #line 40 */
  return /*HplCC2420PinsC.FIFOM*/Msp430GpioC_2_HplGeneralIO_get();
}

/* # 32 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static bool CC2420ReceiveP_FIFO_get(void ){
/* #line 32 */
  unsigned char result;
/* #line 32 */

/* #line 32 */
  result = /*HplCC2420PinsC.FIFOM*/Msp430GpioC_2_GeneralIO_get();
/* #line 32 */

/* #line 32 */
  return result;
/* #line 32 */
}
/* #line 32 */
/* # 209 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static inline error_t CC2420SpiP_Fifo_continueRead(uint8_t addr, uint8_t *data, 
uint8_t len)
/* #line 210 */
{
  return CC2420SpiP_SpiPacket_send((void *)0, data, len);
}

/* # 62 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
inline static error_t CC2420ReceiveP_RXFIFO_continueRead(uint8_t * data, uint8_t length){
/* #line 62 */
  unsigned char result;
/* #line 62 */

/* #line 62 */
  result = CC2420SpiP_Fifo_continueRead(CC2420_RXFIFO, data, length);
/* #line 62 */

/* #line 62 */
  return result;
/* #line 62 */
}
/* #line 62 */
/* #line 51 */
inline static cc2420_status_t CC2420ReceiveP_RXFIFO_beginRead(uint8_t * data, uint8_t length){
/* #line 51 */
  unsigned char result;
/* #line 51 */

/* #line 51 */
  result = CC2420SpiP_Fifo_beginRead(CC2420_RXFIFO, data, length);
/* #line 51 */

/* #line 51 */
  return result;
/* #line 51 */
}
/* #line 51 */
/* # 30 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420ReceiveP_CSN_clr(void ){
/* #line 30 */
  /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_clr();
/* #line 30 */
}
/* #line 30 */
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
inline static cc2420_status_t CC2420ReceiveP_SACK_strobe(void ){
/* #line 45 */
  unsigned char result;
/* #line 45 */

/* #line 45 */
  result = CC2420SpiP_Strobe_strobe(CC2420_SACK);
/* #line 45 */

/* #line 45 */
  return result;
/* #line 45 */
}
/* #line 45 */
/* # 359 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline bool CC2420ControlP_CC2420Config_isHwAutoAckDefault(void )
/* #line 359 */
{
  /* atomic removed: atomic calls only */
/* #line 360 */
  {
    unsigned char __nesc_temp = 
/* #line 360 */
    CC2420ControlP_hwAutoAckDefault;

/* #line 360 */
    return __nesc_temp;
  }
}

/* # 105 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc" */
inline static bool CC2420ReceiveP_CC2420Config_isHwAutoAckDefault(void ){
/* #line 105 */
  unsigned char result;
/* #line 105 */

/* #line 105 */
  result = CC2420ControlP_CC2420Config_isHwAutoAckDefault();
/* #line 105 */

/* #line 105 */
  return result;
/* #line 105 */
}
/* #line 105 */
/* # 366 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline bool CC2420ControlP_CC2420Config_isAutoAckEnabled(void )
/* #line 366 */
{
  /* atomic removed: atomic calls only */
/* #line 367 */
  {
    unsigned char __nesc_temp = 
/* #line 367 */
    CC2420ControlP_autoAckEnabled;

/* #line 367 */
    return __nesc_temp;
  }
}

/* # 110 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc" */
inline static bool CC2420ReceiveP_CC2420Config_isAutoAckEnabled(void ){
/* #line 110 */
  unsigned char result;
/* #line 110 */

/* #line 110 */
  result = CC2420ControlP_CC2420Config_isAutoAckEnabled();
/* #line 110 */

/* #line 110 */
  return result;
/* #line 110 */
}
/* #line 110 */
/* # 199 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
static inline void CC2420ReceiveP_RXFIFO_readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error)
/* #line 200 */
{
  cc2420_header_t *header = CC2420ReceiveP_CC2420PacketBody_getHeader(CC2420ReceiveP_m_p_rx_buf);
  uint8_t tmpLen  = sizeof(message_t ) - ((size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

/* #line 204 */
  CC2420ReceiveP_rxFrameLength = buf[0];

  switch (CC2420ReceiveP_m_state) {

      case CC2420ReceiveP_S_RX_LENGTH: 
        CC2420ReceiveP_m_state = CC2420ReceiveP_S_RX_FCF;
      if (CC2420ReceiveP_rxFrameLength + 1 > CC2420ReceiveP_m_bytes_left) {

          CC2420ReceiveP_flush();
        }
      else {
          if (!CC2420ReceiveP_FIFO_get() && !CC2420ReceiveP_FIFOP_get()) {
              CC2420ReceiveP_m_bytes_left -= CC2420ReceiveP_rxFrameLength + 1;
            }

          if (CC2420ReceiveP_rxFrameLength <= MAC_PACKET_SIZE) {
              if (CC2420ReceiveP_rxFrameLength > 0) {
                  if (CC2420ReceiveP_rxFrameLength > CC2420ReceiveP_SACK_HEADER_LENGTH) {

                      CC2420ReceiveP_RXFIFO_continueRead(buf + 1, CC2420ReceiveP_SACK_HEADER_LENGTH);
                    }
                  else {

                      CC2420ReceiveP_m_state = CC2420ReceiveP_S_RX_PAYLOAD;
                      CC2420ReceiveP_RXFIFO_continueRead(buf + 1, CC2420ReceiveP_rxFrameLength);
                    }
                }
              else {
                  /* atomic removed: atomic calls only */
                  CC2420ReceiveP_receivingPacket = FALSE;
                  CC2420ReceiveP_CSN_set();
                  CC2420ReceiveP_SpiResource_release();
                  CC2420ReceiveP_waitForNextPacket();
                }
            }
          else {

              CC2420ReceiveP_flush();
            }
        }
      break;

      case CC2420ReceiveP_S_RX_FCF: 
        CC2420ReceiveP_m_state = CC2420ReceiveP_S_RX_PAYLOAD;










      if (CC2420ReceiveP_CC2420Config_isAutoAckEnabled() && !CC2420ReceiveP_CC2420Config_isHwAutoAckDefault()) {



          if (((__nesc_ntoh_leuint16(
/* #line 259 */
          header->fcf.data) >> IEEE154_FCF_ACK_REQ) & 0x01) == 1
           && (__nesc_ntoh_leuint16(header->dest.data) == CC2420ReceiveP_CC2420Config_getShortAddr()
           || __nesc_ntoh_leuint16(header->dest.data) == AM_BROADCAST_ADDR)
           && ((__nesc_ntoh_leuint16(header->fcf.data) >> IEEE154_FCF_FRAME_TYPE) & 7) == IEEE154_TYPE_DATA) {

              CC2420ReceiveP_CSN_set();
              CC2420ReceiveP_CSN_clr();
              CC2420ReceiveP_SACK_strobe();
              CC2420ReceiveP_CSN_set();
              CC2420ReceiveP_CSN_clr();
              CC2420ReceiveP_RXFIFO_beginRead(buf + 1 + CC2420ReceiveP_SACK_HEADER_LENGTH, 
              CC2420ReceiveP_rxFrameLength - CC2420ReceiveP_SACK_HEADER_LENGTH);
              return;
            }
        }


      CC2420ReceiveP_RXFIFO_continueRead(buf + 1 + CC2420ReceiveP_SACK_HEADER_LENGTH, 
      CC2420ReceiveP_rxFrameLength - CC2420ReceiveP_SACK_HEADER_LENGTH);
      break;

      case CC2420ReceiveP_S_RX_PAYLOAD: 
        CC2420ReceiveP_CSN_set();

      if (!CC2420ReceiveP_m_missed_packets) {

          CC2420ReceiveP_SpiResource_release();
        }




      if ((((
/* #line 289 */
      CC2420ReceiveP_m_missed_packets && CC2420ReceiveP_FIFO_get()) || !CC2420ReceiveP_FIFOP_get())
       || !CC2420ReceiveP_m_timestamp_size)
       || CC2420ReceiveP_rxFrameLength <= 10) {
          CC2420ReceiveP_PacketTimeStamp_clear(CC2420ReceiveP_m_p_rx_buf);
        }
      else {
          if (CC2420ReceiveP_m_timestamp_size == 1) {
            CC2420ReceiveP_PacketTimeStamp_set(CC2420ReceiveP_m_p_rx_buf, CC2420ReceiveP_m_timestamp_queue[CC2420ReceiveP_m_timestamp_head]);
            }
/* #line 297 */
          CC2420ReceiveP_m_timestamp_head = (CC2420ReceiveP_m_timestamp_head + 1) % CC2420ReceiveP_TIMESTAMP_QUEUE_SIZE;
          CC2420ReceiveP_m_timestamp_size--;

          if (CC2420ReceiveP_m_timestamp_size > 0) {
              CC2420ReceiveP_PacketTimeStamp_clear(CC2420ReceiveP_m_p_rx_buf);
              CC2420ReceiveP_m_timestamp_head = 0;
              CC2420ReceiveP_m_timestamp_size = 0;
            }
        }



      if (buf[CC2420ReceiveP_rxFrameLength] >> 7 && rx_buf) {
          uint8_t type = (__nesc_ntoh_leuint16(header->fcf.data) >> IEEE154_FCF_FRAME_TYPE) & 7;

/* #line 311 */
          CC2420ReceiveP_CC2420Receive_receive(type, CC2420ReceiveP_m_p_rx_buf);
          if (type == IEEE154_TYPE_DATA) {
              CC2420ReceiveP_receiveDone_task_postTask();
              return;
            }
        }

      CC2420ReceiveP_waitForNextPacket();
      break;

      default: /* atomic removed: atomic calls only */
        CC2420ReceiveP_receivingPacket = FALSE;
      CC2420ReceiveP_CSN_set();
      CC2420ReceiveP_SpiResource_release();
      break;
    }
}

/* # 370 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static inline void CC2420SpiP_Fifo_default_readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error)
/* #line 370 */
{
}

/* # 71 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
inline static void CC2420SpiP_Fifo_readDone(uint8_t arg_0x1b79698, uint8_t * data, uint8_t length, error_t error){
/* #line 71 */
  switch (arg_0x1b79698) {
/* #line 71 */
    case CC2420_TXFIFO:
/* #line 71 */
      CC2420TransmitP_TXFIFO_readDone(data, length, error);
/* #line 71 */
      break;
/* #line 71 */
    case CC2420_RXFIFO:
/* #line 71 */
      CC2420ReceiveP_RXFIFO_readDone(data, length, error);
/* #line 71 */
      break;
/* #line 71 */
    default:
/* #line 71 */
      CC2420SpiP_Fifo_default_readDone(arg_0x1b79698, data, length, error);
/* #line 71 */
      break;
/* #line 71 */
    }
/* #line 71 */
}
/* #line 71 */
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
inline static cc2420_status_t CC2420ReceiveP_SFLUSHRX_strobe(void ){
/* #line 45 */
  unsigned char result;
/* #line 45 */

/* #line 45 */
  result = CC2420SpiP_Strobe_strobe(CC2420_SFLUSHRX);
/* #line 45 */

/* #line 45 */
  return result;
/* #line 45 */
}
/* #line 45 */
/* # 255 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline void CC2420ActiveMessageP_RadioBackoff_default_requestInitialBackoff(am_id_t id, 
message_t *msg)
/* #line 256 */
{
}

/* # 81 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
inline static void CC2420ActiveMessageP_RadioBackoff_requestInitialBackoff(am_id_t arg_0x16fd2e8, message_t * msg){
/* #line 81 */
    CC2420ActiveMessageP_RadioBackoff_default_requestInitialBackoff(arg_0x16fd2e8, msg);
/* #line 81 */
}
/* #line 81 */
/* # 202 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline void CC2420ActiveMessageP_SubBackoff_requestInitialBackoff(message_t *msg)
/* #line 202 */
{
  CC2420ActiveMessageP_RadioBackoff_requestInitialBackoff(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.data), msg);
}

/* # 81 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
inline static void CC2420CsmaP_RadioBackoff_requestInitialBackoff(message_t * msg){
/* #line 81 */
  CC2420ActiveMessageP_SubBackoff_requestInitialBackoff(msg);
/* #line 81 */
}
/* #line 81 */
/* # 223 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static inline void CC2420TransmitP_RadioBackoff_setInitialBackoff(uint16_t backoffTime)
/* #line 223 */
{
  CC2420TransmitP_myInitialBackoff = backoffTime + 1;
}

/* # 60 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
inline static void CC2420CsmaP_SubBackoff_setInitialBackoff(uint16_t backoffTime){
/* #line 60 */
  CC2420TransmitP_RadioBackoff_setInitialBackoff(backoffTime);
/* #line 60 */
}
/* #line 60 */
/* # 216 "/Users/doina/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc" */
static inline void CC2420CsmaP_SubBackoff_requestInitialBackoff(message_t *msg)
/* #line 216 */
{
  CC2420CsmaP_SubBackoff_setInitialBackoff(CC2420CsmaP_Random_rand16()
   % (0x1F * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP_RadioBackoff_requestInitialBackoff(msg);
}

/* # 81 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc" */
inline static void CC2420TransmitP_RadioBackoff_requestInitialBackoff(message_t * msg){
/* #line 81 */
  CC2420CsmaP_SubBackoff_requestInitialBackoff(msg);
/* #line 81 */
}
/* #line 81 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t CC2420CsmaP_sendDone_task_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(CC2420CsmaP_sendDone_task);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 198 "/Users/doina/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc" */
static inline void CC2420CsmaP_CC2420Transmit_sendDone(message_t *p_msg, error_t err)
/* #line 198 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 199 */
    CC2420CsmaP_sendErr = err;
/* #line 199 */
    __nesc_atomic_end(__nesc_atomic); }
  CC2420CsmaP_sendDone_task_postTask();
}

/* # 73 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc" */
inline static void CC2420TransmitP_Send_sendDone(message_t * p_msg, error_t error){
/* #line 73 */
  CC2420CsmaP_CC2420Transmit_sendDone(p_msg, error);
/* #line 73 */
}
/* #line 73 */
/* # 431 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static inline void CC2420TransmitP_TXFIFO_writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
/* #line 432 */
{

  CC2420TransmitP_CSN_set();
  if (CC2420TransmitP_m_state == CC2420TransmitP_S_CANCEL) {
      /* atomic removed: atomic calls only */
/* #line 436 */
      {
        CC2420TransmitP_CSN_clr();
        CC2420TransmitP_SFLUSHTX_strobe();
        CC2420TransmitP_CSN_set();
      }
      CC2420TransmitP_releaseSpiResource();
      CC2420TransmitP_m_state = CC2420TransmitP_S_STARTED;
      CC2420TransmitP_Send_sendDone(CC2420TransmitP_m_msg, ECANCEL);
    }
  else {
/* #line 445 */
    if (!CC2420TransmitP_m_cca) {
        /* atomic removed: atomic calls only */
/* #line 446 */
        {
          CC2420TransmitP_m_state = CC2420TransmitP_S_BEGIN_TRANSMIT;
        }
        CC2420TransmitP_attemptSend();
      }
    else {
        CC2420TransmitP_releaseSpiResource();
        /* atomic removed: atomic calls only */
/* #line 453 */
        {
          CC2420TransmitP_m_state = CC2420TransmitP_S_SAMPLE_CCA;
        }

        CC2420TransmitP_RadioBackoff_requestInitialBackoff(CC2420TransmitP_m_msg);
        CC2420TransmitP_BackoffTimer_start(CC2420TransmitP_myInitialBackoff);
      }
    }
}

/* # 331 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
static inline void CC2420ReceiveP_RXFIFO_writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
/* #line 331 */
{
}

/* # 373 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static inline void CC2420SpiP_Fifo_default_writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error)
/* #line 373 */
{
}

/* # 91 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc" */
inline static void CC2420SpiP_Fifo_writeDone(uint8_t arg_0x1b79698, uint8_t * data, uint8_t length, error_t error){
/* #line 91 */
  switch (arg_0x1b79698) {
/* #line 91 */
    case CC2420_TXFIFO:
/* #line 91 */
      CC2420TransmitP_TXFIFO_writeDone(data, length, error);
/* #line 91 */
      break;
/* #line 91 */
    case CC2420_RXFIFO:
/* #line 91 */
      CC2420ReceiveP_RXFIFO_writeDone(data, length, error);
/* #line 91 */
      break;
/* #line 91 */
    default:
/* #line 91 */
      CC2420SpiP_Fifo_default_writeDone(arg_0x1b79698, data, length, error);
/* #line 91 */
      break;
/* #line 91 */
    }
/* #line 91 */
}
/* #line 91 */
/* # 55 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc" */
inline static cc2420_status_t CC2420ControlP_RXCTRL1_write(uint16_t data){
/* #line 55 */
  unsigned char result;
/* #line 55 */

/* #line 55 */
  result = CC2420SpiP_Reg_write(CC2420_RXCTRL1, data);
/* #line 55 */

/* #line 55 */
  return result;
/* #line 55 */
}
/* #line 55 */
inline static cc2420_status_t CC2420ControlP_IOCFG0_write(uint16_t data){
/* #line 55 */
  unsigned char result;
/* #line 55 */

/* #line 55 */
  result = CC2420SpiP_Reg_write(CC2420_IOCFG0, data);
/* #line 55 */

/* #line 55 */
  return result;
/* #line 55 */
}
/* #line 55 */
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
inline static cc2420_status_t CC2420ControlP_SXOSCON_strobe(void ){
/* #line 45 */
  unsigned char result;
/* #line 45 */

/* #line 45 */
  result = CC2420SpiP_Strobe_strobe(CC2420_SXOSCON);
/* #line 45 */

/* #line 45 */
  return result;
/* #line 45 */
}
/* #line 45 */
/* # 79 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port14_enable(void )
/* #line 79 */
{
/* #line 79 */
  P1IE |= 1 << 4;
}

/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_enable(void ){
/* #line 31 */
  HplMsp430InterruptP_Port14_enable();
/* #line 31 */
}
/* #line 31 */
/* # 131 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port14_edge(bool l2h)
/* #line 131 */
{
  /* atomic removed: atomic calls only */
/* #line 132 */
  {
    if (l2h) {
/* #line 133 */
      P1IES &= ~(1 << 4);
      }
    else {
/* #line 134 */
      P1IES |= 1 << 4;
      }
  }
}

/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_edge(bool low_to_high){
/* #line 56 */
  HplMsp430InterruptP_Port14_edge(low_to_high);
/* #line 56 */
}
/* #line 56 */
/* # 95 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port14_clear(void )
/* #line 95 */
{
/* #line 95 */
  P1IFG &= ~(1 << 4);
}

/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_clear(void ){
/* #line 41 */
  HplMsp430InterruptP_Port14_clear();
/* #line 41 */
}
/* #line 41 */
/* # 87 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port14_disable(void )
/* #line 87 */
{
/* #line 87 */
  P1IE &= ~(1 << 4);
}

/* # 36 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_disable(void ){
/* #line 36 */
  HplMsp430InterruptP_Port14_disable();
/* #line 36 */
}
/* #line 36 */
/* # 58 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc" */
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_Interrupt_disable(void )
/* #line 58 */
{
  /* atomic removed: atomic calls only */
/* #line 59 */
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_clear();
  }
  return SUCCESS;
}

/* #line 41 */
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_enable(bool rising)
/* #line 41 */
{
  /* atomic removed: atomic calls only */
/* #line 42 */
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_Interrupt_disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_edge(rising);
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_enable();
  }
  return SUCCESS;
}

static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_Interrupt_enableRisingEdge(void )
/* #line 50 */
{
  return /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_enable(TRUE);
}

/* # 42 "/Users/doina/tinyos-2.x/tos/interfaces/GpioInterrupt.nc" */
inline static error_t CC2420ControlP_InterruptCCA_enableRisingEdge(void ){
/* #line 42 */
  unsigned char result;
/* #line 42 */

/* #line 42 */
  result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_Interrupt_enableRisingEdge();
/* #line 42 */

/* #line 42 */
  return result;
/* #line 42 */
}
/* #line 42 */
/* # 55 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc" */
inline static cc2420_status_t CC2420ControlP_IOCFG1_write(uint16_t data){
/* #line 55 */
  unsigned char result;
/* #line 55 */

/* #line 55 */
  result = CC2420SpiP_Reg_write(CC2420_IOCFG1, data);
/* #line 55 */

/* #line 55 */
  return result;
/* #line 55 */
}
/* #line 55 */
/* # 207 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline error_t CC2420ControlP_CC2420Power_startOscillator(void )
/* #line 207 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 208 */
    {
      if (CC2420ControlP_m_state != CC2420ControlP_S_VREG_STARTED) {
          {
            unsigned char __nesc_temp = 
/* #line 210 */
            FAIL;

            {
/* #line 210 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 210 */
              return __nesc_temp;
            }
          }
        }
/* #line 213 */
      CC2420ControlP_m_state = CC2420ControlP_S_XOSC_STARTING;
      CC2420ControlP_IOCFG1_write(CC2420_SFDMUX_XOSC16M_STABLE << 
      CC2420_IOCFG1_CCAMUX);

      CC2420ControlP_InterruptCCA_enableRisingEdge();
      CC2420ControlP_SXOSCON_strobe();

      CC2420ControlP_IOCFG0_write((1 << CC2420_IOCFG0_FIFOP_POLARITY) | (
      127 << CC2420_IOCFG0_FIFOP_THR));

      CC2420ControlP_writeFsctrl();
      CC2420ControlP_writeMdmctrl0();

      CC2420ControlP_RXCTRL1_write(((((((1 << CC2420_RXCTRL1_RXBPF_LOCUR) | (
      1 << CC2420_RXCTRL1_LOW_LOWGAIN)) | (
      1 << CC2420_RXCTRL1_HIGH_HGM)) | (
      1 << CC2420_RXCTRL1_LNA_CAP_ARRAY)) | (
      1 << CC2420_RXCTRL1_RXMIX_TAIL)) | (
      1 << CC2420_RXCTRL1_RXMIX_VCM)) | (
      2 << CC2420_RXCTRL1_RXMIX_CURRENT));
    }
/* #line 233 */
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

/* # 71 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc" */
inline static error_t CC2420CsmaP_CC2420Power_startOscillator(void ){
/* #line 71 */
  unsigned char result;
/* #line 71 */

/* #line 71 */
  result = CC2420ControlP_CC2420Power_startOscillator();
/* #line 71 */

/* #line 71 */
  return result;
/* #line 71 */
}
/* #line 71 */
/* # 207 "/Users/doina/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc" */
static inline void CC2420CsmaP_Resource_granted(void )
/* #line 207 */
{
  CC2420CsmaP_CC2420Power_startOscillator();
}

/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static void CC2420ControlP_Resource_granted(void ){
/* #line 92 */
  CC2420CsmaP_Resource_granted();
/* #line 92 */
}
/* #line 92 */
/* # 30 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420ControlP_CSN_clr(void ){
/* #line 30 */
  /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_clr();
/* #line 30 */
}
/* #line 30 */
/* # 390 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline void CC2420ControlP_SpiResource_granted(void )
/* #line 390 */
{
  CC2420ControlP_CSN_clr();
  CC2420ControlP_Resource_granted();
}

/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t CC2420ControlP_syncDone_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(CC2420ControlP_syncDone);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t CC2420ControlP_SyncResource_release(void ){
/* #line 110 */
  unsigned char result;
/* #line 110 */

/* #line 110 */
  result = CC2420SpiP_Resource_release(/*CC2420ControlC.SyncSpiC*/CC2420SpiC_1_CLIENT_ID);
/* #line 110 */

/* #line 110 */
  return result;
/* #line 110 */
}
/* #line 110 */
/* # 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420ControlP_CSN_set(void ){
/* #line 29 */
  /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_set();
/* #line 29 */
}
/* #line 29 */
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc" */
inline static cc2420_status_t CC2420ControlP_SRXON_strobe(void ){
/* #line 45 */
  unsigned char result;
/* #line 45 */

/* #line 45 */
  result = CC2420SpiP_Strobe_strobe(CC2420_SRXON);
/* #line 45 */

/* #line 45 */
  return result;
/* #line 45 */
}
/* #line 45 */
inline static cc2420_status_t CC2420ControlP_SRFOFF_strobe(void ){
/* #line 45 */
  unsigned char result;
/* #line 45 */

/* #line 45 */
  result = CC2420SpiP_Strobe_strobe(CC2420_SRFOFF);
/* #line 45 */

/* #line 45 */
  return result;
/* #line 45 */
}
/* #line 45 */
/* # 376 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline void CC2420ControlP_SyncResource_granted(void )
/* #line 376 */
{
  CC2420ControlP_CSN_clr();
  CC2420ControlP_SRFOFF_strobe();
  CC2420ControlP_writeFsctrl();
  CC2420ControlP_writeMdmctrl0();
  CC2420ControlP_writeId();
  CC2420ControlP_CSN_set();
  CC2420ControlP_CSN_clr();
  CC2420ControlP_SRXON_strobe();
  CC2420ControlP_CSN_set();
  CC2420ControlP_SyncResource_release();
  CC2420ControlP_syncDone_postTask();
}

/* #line 509 */
static inline void CC2420ControlP_ReadRssi_default_readDone(error_t error, uint16_t data)
/* #line 509 */
{
}

/* # 63 "/Users/doina/tinyos-2.x/tos/interfaces/Read.nc" */
inline static void CC2420ControlP_ReadRssi_readDone(error_t result, CC2420ControlP_ReadRssi_val_t val){
/* #line 63 */
  CC2420ControlP_ReadRssi_default_readDone(result, val);
/* #line 63 */
}
/* #line 63 */
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t CC2420ControlP_RssiResource_release(void ){
/* #line 110 */
  unsigned char result;
/* #line 110 */

/* #line 110 */
  result = CC2420SpiP_Resource_release(/*CC2420ControlC.RssiResource*/CC2420SpiC_2_CLIENT_ID);
/* #line 110 */

/* #line 110 */
  return result;
/* #line 110 */
}
/* #line 110 */
/* # 287 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static inline cc2420_status_t CC2420SpiP_Reg_read(uint8_t addr, uint16_t *data)
/* #line 287 */
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 291 */
    {
      if (CC2420SpiP_WorkingState_isIdle()) {
          {
            unsigned char __nesc_temp = 
/* #line 293 */
            status;

            {
/* #line 293 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 293 */
              return __nesc_temp;
            }
          }
        }
    }
/* #line 297 */
    __nesc_atomic_end(__nesc_atomic); }
/* #line 297 */
  status = CC2420SpiP_SpiByte_write(addr | 0x40);
  *data = (uint16_t )CC2420SpiP_SpiByte_write(0) << 8;
  *data |= CC2420SpiP_SpiByte_write(0);

  return status;
}

/* # 47 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc" */
inline static cc2420_status_t CC2420ControlP_RSSI_read(uint16_t *data){
/* #line 47 */
  unsigned char result;
/* #line 47 */

/* #line 47 */
  result = CC2420SpiP_Reg_read(CC2420_RSSI, data);
/* #line 47 */

/* #line 47 */
  return result;
/* #line 47 */
}
/* #line 47 */
/* # 395 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline void CC2420ControlP_RssiResource_granted(void )
/* #line 395 */
{
  uint16_t data;

/* #line 397 */
  CC2420ControlP_CSN_clr();
  CC2420ControlP_RSSI_read(&data);
  CC2420ControlP_CSN_set();

  CC2420ControlP_RssiResource_release();
  data += 0x7f;
  data &= 0x00ff;
  CC2420ControlP_ReadRssi_readDone(SUCCESS, data);
}

/* # 393 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static inline void CC2420TransmitP_SpiResource_granted(void )
/* #line 393 */
{
  uint8_t cur_state;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 396 */
    {
      cur_state = CC2420TransmitP_m_state;
    }
/* #line 398 */
    __nesc_atomic_end(__nesc_atomic); }

  switch (cur_state) {
      case CC2420TransmitP_S_LOAD: 
        CC2420TransmitP_loadTXFIFO();
      break;

      case CC2420TransmitP_S_BEGIN_TRANSMIT: 
        CC2420TransmitP_attemptSend();
      break;

      case CC2420TransmitP_S_CANCEL: 
        CC2420TransmitP_CSN_clr();
      CC2420TransmitP_SFLUSHTX_strobe();
      CC2420TransmitP_CSN_set();
      CC2420TransmitP_releaseSpiResource();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 414 */
        {
          CC2420TransmitP_m_state = CC2420TransmitP_S_STARTED;
        }
/* #line 416 */
        __nesc_atomic_end(__nesc_atomic); }
      CC2420TransmitP_Send_sendDone(CC2420TransmitP_m_msg, ECANCEL);
      break;

      default: 
        CC2420TransmitP_releaseSpiResource();
      break;
    }
}

/* # 190 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
static inline void CC2420ReceiveP_SpiResource_granted(void )
/* #line 190 */
{
  CC2420ReceiveP_receive();
}

/* # 367 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static inline void CC2420SpiP_Resource_default_granted(uint8_t id)
/* #line 367 */
{
}

/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static void CC2420SpiP_Resource_granted(uint8_t arg_0x1b7abd0){
/* #line 92 */
  switch (arg_0x1b7abd0) {
/* #line 92 */
    case /*CC2420ControlC.Spi*/CC2420SpiC_0_CLIENT_ID:
/* #line 92 */
      CC2420ControlP_SpiResource_granted();
/* #line 92 */
      break;
/* #line 92 */
    case /*CC2420ControlC.SyncSpiC*/CC2420SpiC_1_CLIENT_ID:
/* #line 92 */
      CC2420ControlP_SyncResource_granted();
/* #line 92 */
      break;
/* #line 92 */
    case /*CC2420ControlC.RssiResource*/CC2420SpiC_2_CLIENT_ID:
/* #line 92 */
      CC2420ControlP_RssiResource_granted();
/* #line 92 */
      break;
/* #line 92 */
    case /*CC2420TransmitC.Spi*/CC2420SpiC_3_CLIENT_ID:
/* #line 92 */
      CC2420TransmitP_SpiResource_granted();
/* #line 92 */
      break;
/* #line 92 */
    case /*CC2420ReceiveC.Spi*/CC2420SpiC_4_CLIENT_ID:
/* #line 92 */
      CC2420ReceiveP_SpiResource_granted();
/* #line 92 */
      break;
/* #line 92 */
    default:
/* #line 92 */
      CC2420SpiP_Resource_default_granted(arg_0x1b7abd0);
/* #line 92 */
      break;
/* #line 92 */
    }
/* #line 92 */
}
/* #line 92 */
/* # 358 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static inline void CC2420SpiP_grant_runTask(void )
/* #line 358 */
{
  uint8_t holder;

/* #line 360 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 360 */
    {
      holder = CC2420SpiP_m_holder;
    }
/* #line 362 */
    __nesc_atomic_end(__nesc_atomic); }
  CC2420SpiP_Resource_granted(holder);
}

/* # 55 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc" */
inline static cc2420_status_t CC2420ControlP_FSCTRL_write(uint16_t data){
/* #line 55 */
  unsigned char result;
/* #line 55 */

/* #line 55 */
  result = CC2420SpiP_Reg_write(CC2420_FSCTRL, data);
/* #line 55 */

/* #line 55 */
  return result;
/* #line 55 */
}
/* #line 55 */
inline static cc2420_status_t CC2420ControlP_MDMCTRL0_write(uint16_t data){
/* #line 55 */
  unsigned char result;
/* #line 55 */

/* #line 55 */
  result = CC2420SpiP_Reg_write(CC2420_MDMCTRL0, data);
/* #line 55 */

/* #line 55 */
  return result;
/* #line 55 */
}
/* #line 55 */
/* # 63 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc" */
inline static cc2420_status_t CC2420ControlP_PANID_write(uint8_t offset, uint8_t * data, uint8_t length){
/* #line 63 */
  unsigned char result;
/* #line 63 */

/* #line 63 */
  result = CC2420SpiP_Ram_write(CC2420_RAM_PANID, offset, data, length);
/* #line 63 */

/* #line 63 */
  return result;
/* #line 63 */
}
/* #line 63 */
/* # 196 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline void CC2420ActiveMessageP_CC2420Config_syncDone(error_t error)
/* #line 196 */
{
}

/* # 360 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
static inline void CC2420ReceiveP_CC2420Config_syncDone(error_t error)
/* #line 360 */
{
}

/* # 53 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc" */
inline static void CC2420ControlP_CC2420Config_syncDone(error_t error){
/* #line 53 */
  CC2420ReceiveP_CC2420Config_syncDone(error);
/* #line 53 */
  CC2420ActiveMessageP_CC2420Config_syncDone(error);
/* #line 53 */
}
/* #line 53 */
/* # 446 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline void CC2420ControlP_syncDone_runTask(void )
/* #line 446 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 447 */
    CC2420ControlP_m_sync_busy = FALSE;
/* #line 447 */
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP_CC2420Config_syncDone(SUCCESS);
}

/* # 78 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t CC2420ControlP_SyncResource_request(void ){
/* #line 78 */
  unsigned char result;
/* #line 78 */

/* #line 78 */
  result = CC2420SpiP_Resource_request(/*CC2420ControlC.SyncSpiC*/CC2420SpiC_1_CLIENT_ID);
/* #line 78 */

/* #line 78 */
  return result;
/* #line 78 */
}
/* #line 78 */
/* # 300 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline error_t CC2420ControlP_CC2420Config_sync(void )
/* #line 300 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 301 */
    {
      if (CC2420ControlP_m_sync_busy) {
          {
            unsigned char __nesc_temp = 
/* #line 303 */
            FAIL;

            {
/* #line 303 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 303 */
              return __nesc_temp;
            }
          }
        }
/* #line 306 */
      CC2420ControlP_m_sync_busy = TRUE;
      if (CC2420ControlP_m_state == CC2420ControlP_S_XOSC_STARTED) {
          CC2420ControlP_SyncResource_request();
        }
      else 
/* #line 309 */
        {
          CC2420ControlP_syncDone_postTask();
        }
    }
/* #line 312 */
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

/* #line 442 */
static inline void CC2420ControlP_sync_runTask(void )
/* #line 442 */
{
  CC2420ControlP_CC2420Config_sync();
}

/* # 181 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMSend_sendDone(am_id_t id, message_t *msg, error_t err)
/* #line 181 */
{





  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current >= 2) {
      return;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current].msg == msg) {
      /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP_0_current, msg, err);
    }
  else {
      ;
    }
}

/* # 99 "/Users/doina/tinyos-2.x/tos/interfaces/AMSend.nc" */
inline static void CC2420ActiveMessageP_AMSend_sendDone(am_id_t arg_0x16e11b8, message_t * msg, error_t error){
/* #line 99 */
  /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_AMSend_sendDone(arg_0x16e11b8, msg, error);
/* #line 99 */
}
/* #line 99 */
/* # 170 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static inline void CC2420ActiveMessageP_SubSend_sendDone(message_t *msg, error_t result)
/* #line 170 */
{
  CC2420ActiveMessageP_AMSend_sendDone(CC2420ActiveMessageP_AMPacket_type(msg), msg, result);
}

/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
inline static void UniqueSendP_Send_sendDone(message_t * msg, error_t error){
/* #line 89 */
  CC2420ActiveMessageP_SubSend_sendDone(msg, error);
/* #line 89 */
}
/* #line 89 */
/* # 104 "/Users/doina/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc" */
static inline void UniqueSendP_SubSend_sendDone(message_t *msg, error_t error)
/* #line 104 */
{
  UniqueSendP_State_toIdle();
  UniqueSendP_Send_sendDone(msg, error);
}

/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
inline static void CC2420TinyosNetworkP_Send_sendDone(message_t * msg, error_t error){
/* #line 89 */
  UniqueSendP_SubSend_sendDone(msg, error);
/* #line 89 */
}
/* #line 89 */
/* # 79 "/Users/doina/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc" */
static inline void CC2420TinyosNetworkP_SubSend_sendDone(message_t *msg, error_t error)
/* #line 79 */
{
  CC2420TinyosNetworkP_Send_sendDone(msg, error);
}

/* # 89 "/Users/doina/tinyos-2.x/tos/interfaces/Send.nc" */
inline static void CC2420CsmaP_Send_sendDone(message_t * msg, error_t error){
/* #line 89 */
  CC2420TinyosNetworkP_SubSend_sendDone(msg, error);
/* #line 89 */
}
/* #line 89 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t CC2420CsmaP_stopDone_task_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(CC2420CsmaP_stopDone_task);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP_29_IO_clr(void )
/* #line 46 */
{
/* #line 46 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 46 */
    * (volatile uint8_t * )29U &= ~(0x01 << 5);
/* #line 46 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_HplGeneralIO_clr(void ){
/* #line 39 */
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP_29_IO_clr();
/* #line 39 */
}
/* #line 39 */
/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_GeneralIO_clr(void )
/* #line 38 */
{
/* #line 38 */
  /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_HplGeneralIO_clr();
}

/* # 30 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420ControlP_VREN_clr(void ){
/* #line 30 */
  /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_GeneralIO_clr();
/* #line 30 */
}
/* #line 30 */
/* # 199 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline error_t CC2420ControlP_CC2420Power_stopVReg(void )
/* #line 199 */
{
  CC2420ControlP_m_state = CC2420ControlP_S_VREG_STOPPED;
  CC2420ControlP_RSTN_clr();
  CC2420ControlP_VREN_clr();
  CC2420ControlP_RSTN_set();
  return SUCCESS;
}

/* # 63 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc" */
inline static error_t CC2420CsmaP_CC2420Power_stopVReg(void ){
/* #line 63 */
  unsigned char result;
/* #line 63 */

/* #line 63 */
  result = CC2420ControlP_CC2420Power_stopVReg();
/* #line 63 */

/* #line 63 */
  return result;
/* #line 63 */
}
/* #line 63 */
/* # 58 "/Users/doina/tinyos-2.x/tos/types/TinyError.h" */
static inline error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

/* # 91 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port10_clear(void )
/* #line 91 */
{
/* #line 91 */
  P1IFG &= ~(1 << 0);
}

/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_clear(void ){
/* #line 41 */
  HplMsp430InterruptP_Port10_clear();
/* #line 41 */
}
/* #line 41 */
/* # 83 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port10_disable(void )
/* #line 83 */
{
/* #line 83 */
  P1IE &= ~(1 << 0);
}

/* # 36 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_disable(void ){
/* #line 36 */
  HplMsp430InterruptP_Port10_disable();
/* #line 36 */
}
/* #line 36 */
/* # 58 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc" */
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_Interrupt_disable(void )
/* #line 58 */
{
  /* atomic removed: atomic calls only */
/* #line 59 */
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_clear();
  }
  return SUCCESS;
}

/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/GpioInterrupt.nc" */
inline static error_t CC2420ReceiveP_InterruptFIFOP_disable(void ){
/* #line 50 */
  unsigned char result;
/* #line 50 */

/* #line 50 */
  result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_Interrupt_disable();
/* #line 50 */

/* #line 50 */
  return result;
/* #line 50 */
}
/* #line 50 */
/* # 138 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
static inline error_t CC2420ReceiveP_StdControl_stop(void )
/* #line 138 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 139 */
    {
      CC2420ReceiveP_m_state = CC2420ReceiveP_S_STOPPED;
      CC2420ReceiveP_reset_state();
      CC2420ReceiveP_CSN_set();
      CC2420ReceiveP_InterruptFIFOP_disable();
    }
/* #line 144 */
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_selectIOFunc(void )
/* #line 56 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  * (volatile uint8_t * )31U &= ~(0x01 << 1);
}

/* # 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_GeneralIO_selectIOFunc(void ){
/* #line 85 */
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_selectIOFunc();
/* #line 85 */
}
/* #line 85 */
/* # 124 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_disableEvents(void )
{
  * (volatile uint16_t * )388U &= ~0x0010;
}

/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430TimerControl_disableEvents(void ){
/* #line 47 */
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4_Control_disableEvents();
/* #line 47 */
}
/* #line 47 */
/* # 58 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc" */
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_disable(void )
/* #line 58 */
{
  /* atomic removed: atomic calls only */
/* #line 59 */
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430TimerControl_disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_GeneralIO_selectIOFunc();
  }
}

/* # 55 "/Users/doina/tinyos-2.x/tos/interfaces/GpioCapture.nc" */
inline static void CC2420TransmitP_CaptureSFD_disable(void ){
/* #line 55 */
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Capture_disable();
/* #line 55 */
}
/* #line 55 */
/* # 159 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static inline error_t CC2420TransmitP_StdControl_stop(void )
/* #line 159 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 160 */
    {
      CC2420TransmitP_m_state = CC2420TransmitP_S_STOPPED;
      CC2420TransmitP_BackoffTimer_stop();
      CC2420TransmitP_CaptureSFD_disable();
      CC2420TransmitP_SpiResource_release();
      CC2420TransmitP_CSN_set();
    }
/* #line 166 */
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

/* # 84 "/Users/doina/tinyos-2.x/tos/interfaces/StdControl.nc" */
inline static error_t CC2420CsmaP_SubControl_stop(void ){
/* #line 84 */
  unsigned char result;
/* #line 84 */

/* #line 84 */
  result = CC2420TransmitP_StdControl_stop();
/* #line 84 */
  result = ecombine(result, CC2420ReceiveP_StdControl_stop());
/* #line 84 */

/* #line 84 */
  return result;
/* #line 84 */
}
/* #line 84 */
/* # 268 "/Users/doina/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc" */
static inline void CC2420CsmaP_shutdown(void )
/* #line 268 */
{
  CC2420CsmaP_SubControl_stop();
  CC2420CsmaP_CC2420Power_stopVReg();
  CC2420CsmaP_stopDone_task_postTask();
}

/* #line 237 */
static inline void CC2420CsmaP_sendDone_task_runTask(void )
/* #line 237 */
{
  error_t packetErr;

/* #line 239 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 239 */
    packetErr = CC2420CsmaP_sendErr;
/* #line 239 */
    __nesc_atomic_end(__nesc_atomic); }
  if (CC2420CsmaP_SplitControlState_isState(CC2420CsmaP_S_STOPPING)) {
      CC2420CsmaP_shutdown();
    }
  else {
      CC2420CsmaP_SplitControlState_forceState(CC2420CsmaP_S_STARTED);
    }

  CC2420CsmaP_Send_sendDone(CC2420CsmaP_m_msg, packetErr);
}

/* # 96 "TestDisseminationC.nc" */
static inline void TestDisseminationC_RadioControl_stopDone(error_t result)
/* #line 96 */
{
}

/* # 117 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
inline static void CC2420CsmaP_SplitControl_stopDone(error_t error){
/* #line 117 */
  TestDisseminationC_RadioControl_stopDone(error);
/* #line 117 */
}
/* #line 117 */
/* # 258 "/Users/doina/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc" */
static inline void CC2420CsmaP_stopDone_task_runTask(void )
/* #line 258 */
{
  CC2420CsmaP_SplitControlState_forceState(CC2420CsmaP_S_STOPPED);
  CC2420CsmaP_SplitControl_stopDone(SUCCESS);
}

/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static void TestDisseminationC_Timer_startPeriodic(uint32_t dt){
/* #line 53 */
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startPeriodic(2U, dt);
/* #line 53 */
}
/* #line 53 */
/* # 45 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
inline static error_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_start(void ){
/* #line 45 */
  unsigned char result;
/* #line 45 */

/* #line 45 */
  result = DisseminationEngineImplP_DisseminationCache_start(4660U);
/* #line 45 */

/* #line 45 */
  return result;
/* #line 45 */
}
/* #line 45 */
/* # 66 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminatorP.nc" */
static inline error_t /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_StdControl_start(void )
/* #line 66 */
{
  error_t result = /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_start();

/* #line 68 */
  if (result == SUCCESS) {
/* #line 68 */
      /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_m_running = TRUE;
    }
/* #line 69 */
  return result;
}

/* # 45 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationCache.nc" */
inline static error_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_start(void ){
/* #line 45 */
  unsigned char result;
/* #line 45 */

/* #line 45 */
  result = DisseminationEngineImplP_DisseminationCache_start(9029U);
/* #line 45 */

/* #line 45 */
  return result;
/* #line 45 */
}
/* #line 45 */
/* # 66 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminatorP.nc" */
static inline error_t /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_StdControl_start(void )
/* #line 66 */
{
  error_t result = /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_start();

/* #line 68 */
  if (result == SUCCESS) {
/* #line 68 */
      /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_m_running = TRUE;
    }
/* #line 69 */
  return result;
}

/* # 255 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static inline error_t DisseminationEngineImplP_DisseminatorControl_default_start(uint16_t id)
/* #line 255 */
{
/* #line 255 */
  return FAIL;
}

/* # 74 "/Users/doina/tinyos-2.x/tos/interfaces/StdControl.nc" */
inline static error_t DisseminationEngineImplP_DisseminatorControl_start(uint16_t arg_0x21c2738){
/* #line 74 */
  unsigned char result;
/* #line 74 */

/* #line 74 */
  switch (arg_0x21c2738) {
/* #line 74 */
    case /*TestDisseminationAppC.Object32C*/DisseminatorC_0_TIMER_ID:
/* #line 74 */
      result = /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_StdControl_start();
/* #line 74 */
      break;
/* #line 74 */
    case /*TestDisseminationAppC.Object16C*/DisseminatorC_1_TIMER_ID:
/* #line 74 */
      result = /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_StdControl_start();
/* #line 74 */
      break;
/* #line 74 */
    default:
/* #line 74 */
      result = DisseminationEngineImplP_DisseminatorControl_default_start(arg_0x21c2738);
/* #line 74 */
      break;
/* #line 74 */
    }
/* #line 74 */

/* #line 74 */
  return result;
/* #line 74 */
}
/* #line 74 */
/* # 73 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static inline error_t DisseminationEngineImplP_StdControl_start(void )
/* #line 73 */
{
  uint8_t i;

/* #line 75 */
  for (i = 0; i < DisseminationEngineImplP_NUM_DISSEMINATORS; i++) {
      DisseminationEngineImplP_DisseminatorControl_start(i);
    }
  DisseminationEngineImplP_m_running = TRUE;
  return SUCCESS;
}

/* # 74 "/Users/doina/tinyos-2.x/tos/interfaces/StdControl.nc" */
inline static error_t TestDisseminationC_DisseminationControl_start(void ){
/* #line 74 */
  unsigned char result;
/* #line 74 */

/* #line 74 */
  result = DisseminationEngineImplP_StdControl_start();
/* #line 74 */

/* #line 74 */
  return result;
/* #line 74 */
}
/* #line 74 */
/* # 83 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
inline static error_t TestDisseminationC_RadioControl_start(void ){
/* #line 83 */
  unsigned char result;
/* #line 83 */

/* #line 83 */
  result = CC2420CsmaP_SplitControl_start();
/* #line 83 */

/* #line 83 */
  return result;
/* #line 83 */
}
/* #line 83 */
/* # 78 "TestDisseminationC.nc" */
static inline void TestDisseminationC_RadioControl_startDone(error_t result)
/* #line 78 */
{

  if (result != SUCCESS) {

      TestDisseminationC_RadioControl_start();
    }
  else {

      TestDisseminationC_DisseminationControl_start();

      if (TOS_NODE_ID % 4 == 1) {
          TestDisseminationC_Timer_startPeriodic(1024 * 20);
        }
      else 
/* #line 90 */
        {
          TestDisseminationC_Timer_startPeriodic(1024);
        }
    }
}

/* # 92 "/Users/doina/tinyos-2.x/tos/interfaces/SplitControl.nc" */
inline static void CC2420CsmaP_SplitControl_startDone(error_t error){
/* #line 92 */
  TestDisseminationC_RadioControl_startDone(error);
/* #line 92 */
}
/* #line 92 */
/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t CC2420ControlP_SpiResource_release(void ){
/* #line 110 */
  unsigned char result;
/* #line 110 */

/* #line 110 */
  result = CC2420SpiP_Resource_release(/*CC2420ControlC.Spi*/CC2420SpiC_0_CLIENT_ID);
/* #line 110 */

/* #line 110 */
  return result;
/* #line 110 */
}
/* #line 110 */
/* # 179 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline error_t CC2420ControlP_Resource_release(void )
/* #line 179 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 180 */
    {
      CC2420ControlP_CSN_set();
      {
        unsigned char __nesc_temp = 
/* #line 182 */
        CC2420ControlP_SpiResource_release();

        {
/* #line 182 */
          __nesc_atomic_end(__nesc_atomic); 
/* #line 182 */
          return __nesc_temp;
        }
      }
    }
/* #line 185 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 110 "/Users/doina/tinyos-2.x/tos/interfaces/Resource.nc" */
inline static error_t CC2420CsmaP_Resource_release(void ){
/* #line 110 */
  unsigned char result;
/* #line 110 */

/* #line 110 */
  result = CC2420ControlP_Resource_release();
/* #line 110 */

/* #line 110 */
  return result;
/* #line 110 */
}
/* #line 110 */
/* # 249 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline error_t CC2420ControlP_CC2420Power_rxOn(void )
/* #line 249 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 250 */
    {
      if (CC2420ControlP_m_state != CC2420ControlP_S_XOSC_STARTED) {
          {
            unsigned char __nesc_temp = 
/* #line 252 */
            FAIL;

            {
/* #line 252 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 252 */
              return __nesc_temp;
            }
          }
        }
/* #line 254 */
      CC2420ControlP_SRXON_strobe();
    }
/* #line 255 */
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

/* # 90 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc" */
inline static error_t CC2420CsmaP_CC2420Power_rxOn(void ){
/* #line 90 */
  unsigned char result;
/* #line 90 */

/* #line 90 */
  result = CC2420ControlP_CC2420Power_rxOn();
/* #line 90 */

/* #line 90 */
  return result;
/* #line 90 */
}
/* #line 90 */
/* # 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port10_enable(void )
/* #line 75 */
{
/* #line 75 */
  P1IE |= 1 << 0;
}

/* # 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_enable(void ){
/* #line 31 */
  HplMsp430InterruptP_Port10_enable();
/* #line 31 */
}
/* #line 31 */
/* # 107 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port10_edge(bool l2h)
/* #line 107 */
{
  /* atomic removed: atomic calls only */
/* #line 108 */
  {
    if (l2h) {
/* #line 109 */
      P1IES &= ~(1 << 0);
      }
    else {
/* #line 110 */
      P1IES |= 1 << 0;
      }
  }
}

/* # 56 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_edge(bool low_to_high){
/* #line 56 */
  HplMsp430InterruptP_Port10_edge(low_to_high);
/* #line 56 */
}
/* #line 56 */
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc" */
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_enable(bool rising)
/* #line 41 */
{
  /* atomic removed: atomic calls only */
/* #line 42 */
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_Interrupt_disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_edge(rising);
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_enable();
  }
  return SUCCESS;
}





static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_Interrupt_enableFallingEdge(void )
/* #line 54 */
{
  return /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_enable(FALSE);
}

/* # 43 "/Users/doina/tinyos-2.x/tos/interfaces/GpioInterrupt.nc" */
inline static error_t CC2420ReceiveP_InterruptFIFOP_enableFallingEdge(void ){
/* #line 43 */
  unsigned char result;
/* #line 43 */

/* #line 43 */
  result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_Interrupt_enableFallingEdge();
/* #line 43 */

/* #line 43 */
  return result;
/* #line 43 */
}
/* #line 43 */
/* # 124 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
static inline error_t CC2420ReceiveP_StdControl_start(void )
/* #line 124 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 125 */
    {
      CC2420ReceiveP_reset_state();
      CC2420ReceiveP_m_state = CC2420ReceiveP_S_STARTED;
      CC2420ReceiveP_receivingPacket = FALSE;




      CC2420ReceiveP_InterruptFIFOP_enableFallingEdge();
    }
/* #line 134 */
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

/* # 148 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static inline error_t CC2420TransmitP_StdControl_start(void )
/* #line 148 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 149 */
    {
      CC2420TransmitP_CaptureSFD_captureRisingEdge();
      CC2420TransmitP_m_state = CC2420TransmitP_S_STARTED;
      CC2420TransmitP_m_receiving = FALSE;
      CC2420TransmitP_abortSpiRelease = FALSE;
      CC2420TransmitP_m_tx_power = 0;
    }
/* #line 155 */
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

/* # 74 "/Users/doina/tinyos-2.x/tos/interfaces/StdControl.nc" */
inline static error_t CC2420CsmaP_SubControl_start(void ){
/* #line 74 */
  unsigned char result;
/* #line 74 */

/* #line 74 */
  result = CC2420TransmitP_StdControl_start();
/* #line 74 */
  result = ecombine(result, CC2420ReceiveP_StdControl_start());
/* #line 74 */

/* #line 74 */
  return result;
/* #line 74 */
}
/* #line 74 */
/* # 250 "/Users/doina/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc" */
static inline void CC2420CsmaP_startDone_task_runTask(void )
/* #line 250 */
{
  CC2420CsmaP_SubControl_start();
  CC2420CsmaP_CC2420Power_rxOn();
  CC2420CsmaP_Resource_release();
  CC2420CsmaP_SplitControlState_forceState(CC2420CsmaP_S_STARTED);
  CC2420CsmaP_SplitControl_startDone(SUCCESS);
}

/* # 45 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
inline static error_t CC2420CsmaP_SplitControlState_requestState(uint8_t reqState){
/* #line 45 */
  unsigned char result;
/* #line 45 */

/* #line 45 */
  result = StateImplP_State_requestState(1U, reqState);
/* #line 45 */

/* #line 45 */
  return result;
/* #line 45 */
}
/* #line 45 */
/* # 55 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc" */
inline static void CC2420ControlP_StartupTimer_start(CC2420ControlP_StartupTimer_size_type dt){
/* #line 55 */
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_start(dt);
/* #line 55 */
}
/* #line 55 */
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP_29_IO_set(void )
/* #line 45 */
{
/* #line 45 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 45 */
    * (volatile uint8_t * )29U |= 0x01 << 5;
/* #line 45 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_HplGeneralIO_set(void ){
/* #line 34 */
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP_29_IO_set();
/* #line 34 */
}
/* #line 34 */
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_GeneralIO_set(void )
/* #line 37 */
{
/* #line 37 */
  /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_HplGeneralIO_set();
}

/* # 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420ControlP_VREN_set(void ){
/* #line 29 */
  /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_GeneralIO_set();
/* #line 29 */
}
/* #line 29 */
/* # 187 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline error_t CC2420ControlP_CC2420Power_startVReg(void )
/* #line 187 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 188 */
    {
      if (CC2420ControlP_m_state != CC2420ControlP_S_VREG_STOPPED) {
          {
            unsigned char __nesc_temp = 
/* #line 190 */
            FAIL;

            {
/* #line 190 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 190 */
              return __nesc_temp;
            }
          }
        }
/* #line 192 */
      CC2420ControlP_m_state = CC2420ControlP_S_VREG_STARTING;
    }
/* #line 193 */
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP_VREN_set();
  CC2420ControlP_StartupTimer_start(CC2420_TIME_VREN);
  return SUCCESS;
}

/* # 51 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc" */
inline static error_t CC2420CsmaP_CC2420Power_startVReg(void ){
/* #line 51 */
  unsigned char result;
/* #line 51 */

/* #line 51 */
  result = CC2420ControlP_CC2420Power_startVReg();
/* #line 51 */

/* #line 51 */
  return result;
/* #line 51 */
}
/* #line 51 */
/* # 247 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static inline error_t DisseminationEngineImplP_TrickleTimer_default_start(uint16_t key)
/* #line 247 */
{
/* #line 247 */
  return FAIL;
}

/* # 60 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimer.nc" */
inline static error_t DisseminationEngineImplP_TrickleTimer_start(uint16_t arg_0x21c4c10){
/* #line 60 */
  unsigned char result;
/* #line 60 */

/* #line 60 */
  switch (arg_0x21c4c10) {
/* #line 60 */
    case 4660U:
/* #line 60 */
      result = /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_start(/*TestDisseminationAppC.Object32C*/DisseminatorC_0_TIMER_ID);
/* #line 60 */
      break;
/* #line 60 */
    case 9029U:
/* #line 60 */
      result = /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_start(/*TestDisseminationAppC.Object16C*/DisseminatorC_1_TIMER_ID);
/* #line 60 */
      break;
/* #line 60 */
    default:
/* #line 60 */
      result = DisseminationEngineImplP_TrickleTimer_default_start(arg_0x21c4c10);
/* #line 60 */
      break;
/* #line 60 */
    }
/* #line 60 */

/* #line 60 */
  return result;
/* #line 60 */
}
/* #line 60 */
/* # 82 "/Users/doina/tinyos-2.x/tos/system/ActiveMessageAddressC.nc" */
static inline am_group_t ActiveMessageAddressC_ActiveMessageAddress_amGroup(void )
/* #line 82 */
{
  am_group_t myGroup;

  /* atomic removed: atomic calls only */
/* #line 84 */
  myGroup = ActiveMessageAddressC_group;
  return myGroup;
}

/* # 55 "/Users/doina/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc" */
inline static am_group_t CC2420ControlP_ActiveMessageAddress_amGroup(void ){
/* #line 55 */
  unsigned char result;
/* #line 55 */

/* #line 55 */
  result = ActiveMessageAddressC_ActiveMessageAddress_amGroup();
/* #line 55 */

/* #line 55 */
  return result;
/* #line 55 */
}
/* #line 55 */
/* #line 50 */
inline static am_addr_t CC2420ControlP_ActiveMessageAddress_amAddress(void ){
/* #line 50 */
  unsigned int result;
/* #line 50 */

/* #line 50 */
  result = ActiveMessageAddressC_ActiveMessageAddress_amAddress();
/* #line 50 */

/* #line 50 */
  return result;
/* #line 50 */
}
/* #line 50 */
/* # 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP_29_IO_makeOutput(void )
/* #line 52 */
{
  /* atomic removed: atomic calls only */
/* #line 52 */
  * (volatile uint8_t * )30U |= 0x01 << 5;
}

/* # 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_HplGeneralIO_makeOutput(void ){
/* #line 71 */
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP_29_IO_makeOutput();
/* #line 71 */
}
/* #line 71 */
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_GeneralIO_makeOutput(void )
/* #line 43 */
{
/* #line 43 */
  /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_HplGeneralIO_makeOutput();
}

/* # 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420ControlP_VREN_makeOutput(void ){
/* #line 35 */
  /*HplCC2420PinsC.VRENM*/Msp430GpioC_6_GeneralIO_makeOutput();
/* #line 35 */
}
/* #line 35 */
/* # 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP_30_IO_makeOutput(void )
/* #line 52 */
{
  /* atomic removed: atomic calls only */
/* #line 52 */
  * (volatile uint8_t * )30U |= 0x01 << 6;
}

/* # 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_HplGeneralIO_makeOutput(void ){
/* #line 71 */
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP_30_IO_makeOutput();
/* #line 71 */
}
/* #line 71 */
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_GeneralIO_makeOutput(void )
/* #line 43 */
{
/* #line 43 */
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_HplGeneralIO_makeOutput();
}

/* # 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420ControlP_RSTN_makeOutput(void ){
/* #line 35 */
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC_4_GeneralIO_makeOutput();
/* #line 35 */
}
/* #line 35 */
/* # 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP_26_IO_makeOutput(void )
/* #line 52 */
{
  /* atomic removed: atomic calls only */
/* #line 52 */
  * (volatile uint8_t * )30U |= 0x01 << 2;
}

/* # 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_HplGeneralIO_makeOutput(void ){
/* #line 71 */
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP_26_IO_makeOutput();
/* #line 71 */
}
/* #line 71 */
/* # 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_makeOutput(void )
/* #line 43 */
{
/* #line 43 */
  /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_HplGeneralIO_makeOutput();
}

/* # 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420ControlP_CSN_makeOutput(void ){
/* #line 35 */
  /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_makeOutput();
/* #line 35 */
}
/* #line 35 */
/* # 121 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline error_t CC2420ControlP_Init_init(void )
/* #line 121 */
{
  CC2420ControlP_CSN_makeOutput();
  CC2420ControlP_RSTN_makeOutput();
  CC2420ControlP_VREN_makeOutput();

  CC2420ControlP_m_short_addr = CC2420ControlP_ActiveMessageAddress_amAddress();
  CC2420ControlP_m_pan = CC2420ControlP_ActiveMessageAddress_amGroup();
  CC2420ControlP_m_tx_power = 31;
  CC2420ControlP_m_channel = 26;





  CC2420ControlP_addressRecognition = TRUE;





  CC2420ControlP_hwAddressRecognition = FALSE;






  CC2420ControlP_autoAckEnabled = TRUE;






  CC2420ControlP_hwAutoAckDefault = FALSE;



  return SUCCESS;
}

/* # 81 "/Users/doina/tinyos-2.x/tos/system/StateImplP.nc" */
static inline error_t StateImplP_Init_init(void )
/* #line 81 */
{
  int i;

/* #line 83 */
  for (i = 0; i < 4U; i++) {
      StateImplP_state[i] = StateImplP_S_IDLE;
    }
  return SUCCESS;
}

/* # 45 "/Users/doina/tinyos-2.x/tos/system/FcfsResourceQueueC.nc" */
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_Init_init(void )
/* #line 45 */
{
  memset(/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_resQ, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_NO_ENTRY, sizeof /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_resQ);
  return SUCCESS;
}

/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_cc_t x)
/* #line 46 */
{
/* #line 46 */
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3___nesc_unnamed4359 {
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
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_setControlAsCompare(void ){
/* #line 36 */
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_Control_setControlAsCompare();
/* #line 36 */
}
/* #line 36 */
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Init_init(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Msp430TimerControl_setControlAsCompare();
  return SUCCESS;
}

/* # 50 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_makeInput(void )
/* #line 50 */
{
  /* atomic removed: atomic calls only */
/* #line 50 */
  * (volatile uint8_t * )30U &= ~(0x01 << 1);
}

/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*HplCC2420PinsC.SFDM*/Msp430GpioC_5_HplGeneralIO_makeInput(void ){
/* #line 64 */
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP_25_IO_makeInput();
/* #line 64 */
}
/* #line 64 */
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC_5_GeneralIO_makeInput(void )
/* #line 41 */
{
/* #line 41 */
  /*HplCC2420PinsC.SFDM*/Msp430GpioC_5_HplGeneralIO_makeInput();
}

/* # 33 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420TransmitP_SFD_makeInput(void ){
/* #line 33 */
  /*HplCC2420PinsC.SFDM*/Msp430GpioC_5_GeneralIO_makeInput();
/* #line 33 */
}
/* #line 33 */


inline static void CC2420TransmitP_CSN_makeOutput(void ){
/* #line 35 */
  /*HplCC2420PinsC.CSNM*/Msp430GpioC_1_GeneralIO_makeOutput();
/* #line 35 */
}
/* #line 35 */
/* # 50 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP_4_IO_makeInput(void )
/* #line 50 */
{
  /* atomic removed: atomic calls only */
/* #line 50 */
  * (volatile uint8_t * )34U &= ~(0x01 << 4);
}

/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*HplCC2420PinsC.CCAM*/Msp430GpioC_0_HplGeneralIO_makeInput(void ){
/* #line 64 */
  /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP_4_IO_makeInput();
/* #line 64 */
}
/* #line 64 */
/* # 41 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC_0_GeneralIO_makeInput(void )
/* #line 41 */
{
/* #line 41 */
  /*HplCC2420PinsC.CCAM*/Msp430GpioC_0_HplGeneralIO_makeInput();
}

/* # 33 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void CC2420TransmitP_CCA_makeInput(void ){
/* #line 33 */
  /*HplCC2420PinsC.CCAM*/Msp430GpioC_0_GeneralIO_makeInput();
/* #line 33 */
}
/* #line 33 */
/* # 140 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static inline error_t CC2420TransmitP_Init_init(void )
/* #line 140 */
{
  CC2420TransmitP_CCA_makeInput();
  CC2420TransmitP_CSN_makeOutput();
  CC2420TransmitP_SFD_makeInput();
  return SUCCESS;
}

/* # 118 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
static inline error_t CC2420ReceiveP_Init_init(void )
/* #line 118 */
{
  CC2420ReceiveP_m_p_rx_buf = &CC2420ReceiveP_m_rx_buf;
  return SUCCESS;
}

/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_CC2int(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t x)
/* #line 46 */
{
/* #line 46 */
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5___nesc_unnamed4360 {
/* #line 46 */
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t f;
/* #line 46 */
    uint16_t t;
  } 
/* #line 46 */
  c = { .f = x };

/* #line 46 */
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_CC2int(x);
}

/* #line 94 */
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_setControlAsCompare(void )
{
  * (volatile uint16_t * )390U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_compareControl();
}

/* # 36 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc" */
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_setControlAsCompare(void ){
/* #line 36 */
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_setControlAsCompare();
/* #line 36 */
}
/* #line 36 */
/* # 42 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc" */
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Init_init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Msp430TimerControl_setControlAsCompare();
  return SUCCESS;
}

/* # 44 "/Users/doina/tinyos-2.x/tos/system/RandomMlcgC.nc" */
static inline error_t RandomMlcgC_Init_init(void )
/* #line 44 */
{
  /* atomic removed: atomic calls only */
/* #line 45 */
  RandomMlcgC_seed = (uint32_t )(TOS_NODE_ID + 1);

  return SUCCESS;
}

/* # 41 "/Users/doina/tinyos-2.x/tos/interfaces/Random.nc" */
inline static uint16_t UniqueSendP_Random_rand16(void ){
/* #line 41 */
  unsigned int result;
/* #line 41 */

/* #line 41 */
  result = RandomMlcgC_Random_rand16();
/* #line 41 */

/* #line 41 */
  return result;
/* #line 41 */
}
/* #line 41 */
/* # 62 "/Users/doina/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc" */
static inline error_t UniqueSendP_Init_init(void )
/* #line 62 */
{
  UniqueSendP_localSendId = UniqueSendP_Random_rand16();
  return SUCCESS;
}

/* # 71 "/Users/doina/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc" */
static inline error_t UniqueReceiveP_Init_init(void )
/* #line 71 */
{
  int i;

/* #line 73 */
  for (i = 0; i < 4; i++) {
      UniqueReceiveP_receivedMessages[i].source = (am_addr_t )0xFFFF;
      UniqueReceiveP_receivedMessages[i].dsn = 0;
    }
  return SUCCESS;
}

/* # 66 "/Users/doina/tinyos-2.x/tos/system/BitVectorC.nc" */
static inline void /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_clearAll(void )
{
  memset(/*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_m_bits, 0, sizeof /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_m_bits);
}

/* # 34 "/Users/doina/tinyos-2.x/tos/interfaces/BitVector.nc" */
inline static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Changed_clearAll(void ){
/* #line 34 */
  /*DisseminationTimerP.TrickleTimerMilliC.ChangeVector*/BitVectorC_1_BitVector_clearAll();
/* #line 34 */
}
/* #line 34 */
/* # 66 "/Users/doina/tinyos-2.x/tos/system/BitVectorC.nc" */
static inline void /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_clearAll(void )
{
  memset(/*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_m_bits, 0, sizeof /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_m_bits);
}

/* # 34 "/Users/doina/tinyos-2.x/tos/interfaces/BitVector.nc" */
inline static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Pending_clearAll(void ){
/* #line 34 */
  /*DisseminationTimerP.TrickleTimerMilliC.PendingVector*/BitVectorC_0_BitVector_clearAll();
/* #line 34 */
}
/* #line 34 */
/* # 74 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimerImplP.nc" */
static inline error_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Init_init(void )
/* #line 74 */
{
  int i;

/* #line 76 */
  for (i = 0; i < 2U; i++) {
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[i].period = 1024;
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[i].count = 0;
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[i].time = 0;
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[i].remainder = 0;
    }
  /* atomic removed: atomic calls only */
/* #line 82 */
  {
    /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Pending_clearAll();
    /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Changed_clearAll();
  }
  return SUCCESS;
}

/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
inline static error_t RealMainP_SoftwareInit_init(void ){
/* #line 51 */
  unsigned char result;
/* #line 51 */

/* #line 51 */
  result = /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Init_init();
/* #line 51 */
  result = ecombine(result, UniqueReceiveP_Init_init());
/* #line 51 */
  result = ecombine(result, UniqueSendP_Init_init());
/* #line 51 */
  result = ecombine(result, RandomMlcgC_Init_init());
/* #line 51 */
  result = ecombine(result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_1_Init_init());
/* #line 51 */
  result = ecombine(result, CC2420ReceiveP_Init_init());
/* #line 51 */
  result = ecombine(result, CC2420TransmitP_Init_init());
/* #line 51 */
  result = ecombine(result, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC_0_Init_init());
/* #line 51 */
  result = ecombine(result, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC_0_Init_init());
/* #line 51 */
  result = ecombine(result, StateImplP_Init_init());
/* #line 51 */
  result = ecombine(result, CC2420ControlP_Init_init());
/* #line 51 */

/* #line 51 */
  return result;
/* #line 51 */
}
/* #line 51 */
/* # 82 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminatorP.nc" */
static inline void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_set(const /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_t *val)
/* #line 82 */
{
  if (/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_seqno == DISSEMINATION_SEQNO_UNKNOWN) {
      /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_valueCache = *val;
    }
}

/* # 56 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationValue.nc" */
inline static void TestDisseminationC_Value16_set(const TestDisseminationC_Value16_t *arg_0x14ad7a8){
/* #line 56 */
  /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_set(arg_0x14ad7a8);
/* #line 56 */
}
/* #line 56 */
/* # 82 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminatorP.nc" */
static inline void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_set(const /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_t *val)
/* #line 82 */
{
  if (/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_seqno == DISSEMINATION_SEQNO_UNKNOWN) {
      /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_valueCache = *val;
    }
}

/* # 56 "/Users/doina/tinyos-2.x/tos/lib/net/DisseminationValue.nc" */
inline static void TestDisseminationC_Value32_set(const TestDisseminationC_Value32_t *arg_0x14ad7a8){
/* #line 56 */
  /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_set(arg_0x14ad7a8);
/* #line 56 */
}
/* #line 56 */
/* # 68 "TestDisseminationC.nc" */
static inline void TestDisseminationC_Boot_booted(void )
/* #line 68 */
{
  uint32_t initialVal32 = 123456;
  uint16_t initialVal16 = 1234;

  TestDisseminationC_Value32_set(&initialVal32);
  TestDisseminationC_Value16_set(&initialVal16);

  TestDisseminationC_RadioControl_start();
}

/* # 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc" */
inline static void RealMainP_Boot_booted(void ){
/* #line 49 */
  TestDisseminationC_Boot_booted();
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
/* # 179 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
static inline void CC2420ReceiveP_InterruptFIFOP_fired(void )
/* #line 179 */
{
  if (CC2420ReceiveP_m_state == CC2420ReceiveP_S_STARTED) {
      CC2420ReceiveP_beginReceive();
    }
  else {
      CC2420ReceiveP_m_missed_packets++;
    }
}

/* # 57 "/Users/doina/tinyos-2.x/tos/interfaces/GpioInterrupt.nc" */
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_Interrupt_fired(void ){
/* #line 57 */
  CC2420ReceiveP_InterruptFIFOP_fired();
/* #line 57 */
}
/* #line 57 */
/* # 66 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc" */
static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_fired(void )
/* #line 66 */
{
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_clear();
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_Interrupt_fired();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port10_fired(void ){
/* #line 61 */
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC_1_HplInterrupt_fired();
/* #line 61 */
}
/* #line 61 */
/* # 92 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port11_clear(void )
/* #line 92 */
{
/* #line 92 */
  P1IFG &= ~(1 << 1);
}

/* #line 68 */
static inline void HplMsp430InterruptP_Port11_default_fired(void )
/* #line 68 */
{
/* #line 68 */
  HplMsp430InterruptP_Port11_clear();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port11_fired(void ){
/* #line 61 */
  HplMsp430InterruptP_Port11_default_fired();
/* #line 61 */
}
/* #line 61 */
/* # 93 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port12_clear(void )
/* #line 93 */
{
/* #line 93 */
  P1IFG &= ~(1 << 2);
}

/* #line 69 */
static inline void HplMsp430InterruptP_Port12_default_fired(void )
/* #line 69 */
{
/* #line 69 */
  HplMsp430InterruptP_Port12_clear();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port12_fired(void ){
/* #line 61 */
  HplMsp430InterruptP_Port12_default_fired();
/* #line 61 */
}
/* #line 61 */
/* # 94 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port13_clear(void )
/* #line 94 */
{
/* #line 94 */
  P1IFG &= ~(1 << 3);
}

/* #line 70 */
static inline void HplMsp430InterruptP_Port13_default_fired(void )
/* #line 70 */
{
/* #line 70 */
  HplMsp430InterruptP_Port13_clear();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port13_fired(void ){
/* #line 61 */
  HplMsp430InterruptP_Port13_default_fired();
/* #line 61 */
}
/* #line 61 */
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static error_t CC2420CsmaP_startDone_task_postTask(void ){
/* #line 56 */
  unsigned char result;
/* #line 56 */

/* #line 56 */
  result = SchedulerBasicP_TaskBasic_postTask(CC2420CsmaP_startDone_task);
/* #line 56 */

/* #line 56 */
  return result;
/* #line 56 */
}
/* #line 56 */
/* # 211 "/Users/doina/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc" */
static inline void CC2420CsmaP_CC2420Power_startOscillatorDone(void )
/* #line 211 */
{
  CC2420CsmaP_startDone_task_postTask();
}

/* # 76 "/Users/doina/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc" */
inline static void CC2420ControlP_CC2420Power_startOscillatorDone(void ){
/* #line 76 */
  CC2420CsmaP_CC2420Power_startOscillatorDone();
/* #line 76 */
}
/* #line 76 */
/* # 50 "/Users/doina/tinyos-2.x/tos/interfaces/GpioInterrupt.nc" */
inline static error_t CC2420ControlP_InterruptCCA_disable(void ){
/* #line 50 */
  unsigned char result;
/* #line 50 */

/* #line 50 */
  result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_Interrupt_disable();
/* #line 50 */

/* #line 50 */
  return result;
/* #line 50 */
}
/* #line 50 */
/* # 418 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static inline void CC2420ControlP_InterruptCCA_fired(void )
/* #line 418 */
{
  CC2420ControlP_m_state = CC2420ControlP_S_XOSC_STARTED;
  CC2420ControlP_InterruptCCA_disable();
  CC2420ControlP_IOCFG1_write(0);
  CC2420ControlP_writeId();
  CC2420ControlP_CSN_set();
  CC2420ControlP_CSN_clr();
  CC2420ControlP_CC2420Power_startOscillatorDone();
}

/* # 57 "/Users/doina/tinyos-2.x/tos/interfaces/GpioInterrupt.nc" */
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_Interrupt_fired(void ){
/* #line 57 */
  CC2420ControlP_InterruptCCA_fired();
/* #line 57 */
}
/* #line 57 */
/* # 66 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc" */
static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_fired(void )
/* #line 66 */
{
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_clear();
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_Interrupt_fired();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port14_fired(void ){
/* #line 61 */
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC_0_HplInterrupt_fired();
/* #line 61 */
}
/* #line 61 */
/* # 96 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port15_clear(void )
/* #line 96 */
{
/* #line 96 */
  P1IFG &= ~(1 << 5);
}

/* #line 72 */
static inline void HplMsp430InterruptP_Port15_default_fired(void )
/* #line 72 */
{
/* #line 72 */
  HplMsp430InterruptP_Port15_clear();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port15_fired(void ){
/* #line 61 */
  HplMsp430InterruptP_Port15_default_fired();
/* #line 61 */
}
/* #line 61 */
/* # 97 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port16_clear(void )
/* #line 97 */
{
/* #line 97 */
  P1IFG &= ~(1 << 6);
}

/* #line 73 */
static inline void HplMsp430InterruptP_Port16_default_fired(void )
/* #line 73 */
{
/* #line 73 */
  HplMsp430InterruptP_Port16_clear();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port16_fired(void ){
/* #line 61 */
  HplMsp430InterruptP_Port16_default_fired();
/* #line 61 */
}
/* #line 61 */
/* # 98 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port17_clear(void )
/* #line 98 */
{
/* #line 98 */
  P1IFG &= ~(1 << 7);
}

/* #line 74 */
static inline void HplMsp430InterruptP_Port17_default_fired(void )
/* #line 74 */
{
/* #line 74 */
  HplMsp430InterruptP_Port17_clear();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port17_fired(void ){
/* #line 61 */
  HplMsp430InterruptP_Port17_default_fired();
/* #line 61 */
}
/* #line 61 */
/* # 195 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port20_clear(void )
/* #line 195 */
{
/* #line 195 */
  P2IFG &= ~(1 << 0);
}

/* #line 171 */
static inline void HplMsp430InterruptP_Port20_default_fired(void )
/* #line 171 */
{
/* #line 171 */
  HplMsp430InterruptP_Port20_clear();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port20_fired(void ){
/* #line 61 */
  HplMsp430InterruptP_Port20_default_fired();
/* #line 61 */
}
/* #line 61 */
/* # 196 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port21_clear(void )
/* #line 196 */
{
/* #line 196 */
  P2IFG &= ~(1 << 1);
}

/* #line 172 */
static inline void HplMsp430InterruptP_Port21_default_fired(void )
/* #line 172 */
{
/* #line 172 */
  HplMsp430InterruptP_Port21_clear();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port21_fired(void ){
/* #line 61 */
  HplMsp430InterruptP_Port21_default_fired();
/* #line 61 */
}
/* #line 61 */
/* # 197 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port22_clear(void )
/* #line 197 */
{
/* #line 197 */
  P2IFG &= ~(1 << 2);
}

/* #line 173 */
static inline void HplMsp430InterruptP_Port22_default_fired(void )
/* #line 173 */
{
/* #line 173 */
  HplMsp430InterruptP_Port22_clear();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port22_fired(void ){
/* #line 61 */
  HplMsp430InterruptP_Port22_default_fired();
/* #line 61 */
}
/* #line 61 */
/* # 198 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port23_clear(void )
/* #line 198 */
{
/* #line 198 */
  P2IFG &= ~(1 << 3);
}

/* #line 174 */
static inline void HplMsp430InterruptP_Port23_default_fired(void )
/* #line 174 */
{
/* #line 174 */
  HplMsp430InterruptP_Port23_clear();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port23_fired(void ){
/* #line 61 */
  HplMsp430InterruptP_Port23_default_fired();
/* #line 61 */
}
/* #line 61 */
/* # 199 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port24_clear(void )
/* #line 199 */
{
/* #line 199 */
  P2IFG &= ~(1 << 4);
}

/* #line 175 */
static inline void HplMsp430InterruptP_Port24_default_fired(void )
/* #line 175 */
{
/* #line 175 */
  HplMsp430InterruptP_Port24_clear();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port24_fired(void ){
/* #line 61 */
  HplMsp430InterruptP_Port24_default_fired();
/* #line 61 */
}
/* #line 61 */
/* # 200 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port25_clear(void )
/* #line 200 */
{
/* #line 200 */
  P2IFG &= ~(1 << 5);
}

/* #line 176 */
static inline void HplMsp430InterruptP_Port25_default_fired(void )
/* #line 176 */
{
/* #line 176 */
  HplMsp430InterruptP_Port25_clear();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port25_fired(void ){
/* #line 61 */
  HplMsp430InterruptP_Port25_default_fired();
/* #line 61 */
}
/* #line 61 */
/* # 201 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port26_clear(void )
/* #line 201 */
{
/* #line 201 */
  P2IFG &= ~(1 << 6);
}

/* #line 177 */
static inline void HplMsp430InterruptP_Port26_default_fired(void )
/* #line 177 */
{
/* #line 177 */
  HplMsp430InterruptP_Port26_clear();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port26_fired(void ){
/* #line 61 */
  HplMsp430InterruptP_Port26_default_fired();
/* #line 61 */
}
/* #line 61 */
/* # 202 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
static inline void HplMsp430InterruptP_Port27_clear(void )
/* #line 202 */
{
/* #line 202 */
  P2IFG &= ~(1 << 7);
}

/* #line 178 */
static inline void HplMsp430InterruptP_Port27_default_fired(void )
/* #line 178 */
{
/* #line 178 */
  HplMsp430InterruptP_Port27_clear();
}

/* # 61 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc" */
inline static void HplMsp430InterruptP_Port27_fired(void ){
/* #line 61 */
  HplMsp430InterruptP_Port27_default_fired();
/* #line 61 */
}
/* #line 61 */
/* # 88 "/Users/doina/tinyos-2.x/tos/interfaces/ArbiterInfo.nc" */
inline static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_ArbiterInfo_userId(void ){
/* #line 88 */
  unsigned char result;
/* #line 88 */

/* #line 88 */
  result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ArbiterInfo_userId();
/* #line 88 */

/* #line 88 */
  return result;
/* #line 88 */
}
/* #line 88 */
/* # 349 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc" */
static inline void HplMsp430Usart0P_Usart_disableRxIntr(void )
/* #line 349 */
{
  HplMsp430Usart0P_IE1 &= ~(1 << 6);
}

/* # 177 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc" */
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_disableRxIntr(void ){
/* #line 177 */
  HplMsp430Usart0P_Usart_disableRxIntr();
/* #line 177 */
}
/* #line 177 */
/* # 170 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartInterrupts_rxDone(uint8_t data)
/* #line 170 */
{

  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_rx_buf) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_pos - 1] = data;
    }
  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_pos < /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_len) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_continueOp();
    }
  else 
/* #line 177 */
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_disableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone();
    }
}

/* # 65 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc" */
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_Interrupts_default_rxDone(uint8_t id, uint8_t data)
/* #line 65 */
{
}

/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc" */
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_Interrupts_rxDone(uint8_t arg_0x1dac1d8, uint8_t data){
/* #line 54 */
  switch (arg_0x1dac1d8) {
/* #line 54 */
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C_0_CLIENT_ID:
/* #line 54 */
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartInterrupts_rxDone(data);
/* #line 54 */
      break;
/* #line 54 */
    default:
/* #line 54 */
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_Interrupts_default_rxDone(arg_0x1dac1d8, data);
/* #line 54 */
      break;
/* #line 54 */
    }
/* #line 54 */
}
/* #line 54 */
/* # 80 "/Users/doina/tinyos-2.x/tos/interfaces/ArbiterInfo.nc" */
inline static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_ArbiterInfo_inUse(void ){
/* #line 80 */
  unsigned char result;
/* #line 80 */

/* #line 80 */
  result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ArbiterInfo_inUse();
/* #line 80 */

/* #line 80 */
  return result;
/* #line 80 */
}
/* #line 80 */
/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc" */
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_RawInterrupts_rxDone(uint8_t data)
/* #line 54 */
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_ArbiterInfo_inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_Interrupts_rxDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_ArbiterInfo_userId(), data);
    }
}

/* # 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc" */
inline static void HplMsp430Usart0P_Interrupts_rxDone(uint8_t data){
/* #line 54 */
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_RawInterrupts_rxDone(data);
/* #line 54 */
}
/* #line 54 */
/* # 55 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C0P.nc" */
static inline bool HplMsp430I2C0P_HplI2C_isI2C(void )
/* #line 55 */
{
  /* atomic removed: atomic calls only */
/* #line 56 */
  {
    unsigned char __nesc_temp = 
/* #line 56 */
    HplMsp430I2C0P_U0CTL & 0x20 && HplMsp430I2C0P_U0CTL & 0x04 && HplMsp430I2C0P_U0CTL & 0x01;

/* #line 56 */
    return __nesc_temp;
  }
}

/* # 6 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc" */
inline static bool HplMsp430Usart0P_HplI2C_isI2C(void ){
/* #line 6 */
  unsigned char result;
/* #line 6 */

/* #line 6 */
  result = HplMsp430I2C0P_HplI2C_isI2C();
/* #line 6 */

/* #line 6 */
  return result;
/* #line 6 */
}
/* #line 6 */
/* # 66 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc" */
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_I2CInterrupts_default_fired(uint8_t id)
/* #line 66 */
{
}

/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc" */
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_I2CInterrupts_fired(uint8_t arg_0x1daca58){
/* #line 39 */
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_I2CInterrupts_default_fired(arg_0x1daca58);
/* #line 39 */
}
/* #line 39 */
/* # 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc" */
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_RawI2CInterrupts_fired(void )
/* #line 59 */
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_ArbiterInfo_inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_I2CInterrupts_fired(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_ArbiterInfo_userId());
    }
}

/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc" */
inline static void HplMsp430Usart0P_I2CInterrupts_fired(void ){
/* #line 39 */
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_RawI2CInterrupts_fired();
/* #line 39 */
}
/* #line 39 */
/* # 188 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartInterrupts_txDone(void )
/* #line 188 */
{
}

/* # 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc" */
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_Interrupts_default_txDone(uint8_t id)
/* #line 64 */
{
}

/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc" */
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_Interrupts_txDone(uint8_t arg_0x1dac1d8){
/* #line 49 */
  switch (arg_0x1dac1d8) {
/* #line 49 */
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C_0_CLIENT_ID:
/* #line 49 */
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_UsartInterrupts_txDone();
/* #line 49 */
      break;
/* #line 49 */
    default:
/* #line 49 */
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_Interrupts_default_txDone(arg_0x1dac1d8);
/* #line 49 */
      break;
/* #line 49 */
    }
/* #line 49 */
}
/* #line 49 */
/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc" */
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_RawInterrupts_txDone(void )
/* #line 49 */
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_ArbiterInfo_inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_Interrupts_txDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_ArbiterInfo_userId());
    }
}

/* # 49 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc" */
inline static void HplMsp430Usart0P_Interrupts_txDone(void ){
/* #line 49 */
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP_0_RawInterrupts_txDone();
/* #line 49 */
}
/* #line 49 */
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
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_fired(uint8_t arg_0x1593680){
/* #line 28 */
  switch (arg_0x1593680) {
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
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_default_fired(arg_0x1593680);
/* #line 28 */
      break;
/* #line 28 */
    }
/* #line 28 */
}
/* #line 28 */
/* # 136 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Alarm_startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_t0 = t0;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_dt = dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_set_alarm();
    }
/* #line 143 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* #line 96 */
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_set_alarm(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_Counter_get();
/* #line 98 */
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type expires;
/* #line 98 */
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type remaining;




  expires = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_t0 + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_dt;


  remaining = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_to_size_type )(expires - now);


  if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_t0 <= now) 
    {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
/* #line 121 */
  if (remaining > /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_MAX_DELAY) 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_t0 = now + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_MAX_DELAY;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_dt = remaining - /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_MAX_DELAY;
      remaining = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_MAX_DELAY;
    }
  else 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_t0 += /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_m_dt = 0;
    }
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_AlarmFrom_startAt((/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_from_size_type )now << 0, 
  (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC_0_from_size_type )remaining << 0);
}

/* # 69 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformCounterC.nc" */
static /*Counter32khz32C.Transform*/TransformCounterC_0_to_size_type /*Counter32khz32C.Transform*/TransformCounterC_0_Counter_get(void )
{
  /*Counter32khz32C.Transform*/TransformCounterC_0_to_size_type rv = 0;

/* #line 72 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*Counter32khz32C.Transform*/TransformCounterC_0_upper_count_type high = /*Counter32khz32C.Transform*/TransformCounterC_0_m_upper;
      /*Counter32khz32C.Transform*/TransformCounterC_0_from_size_type low = /*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_get();

/* #line 76 */
      if (/*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_isOverflowPending()) 
        {






          high++;
          low = /*Counter32khz32C.Transform*/TransformCounterC_0_CounterFrom_get();
        }
      {
        /*Counter32khz32C.Transform*/TransformCounterC_0_to_size_type high_to = high;
        /*Counter32khz32C.Transform*/TransformCounterC_0_to_size_type low_to = low >> /*Counter32khz32C.Transform*/TransformCounterC_0_LOW_SHIFT_RIGHT;

/* #line 90 */
        rv = (high_to << /*Counter32khz32C.Transform*/TransformCounterC_0_HIGH_SHIFT_LEFT) | low_to;
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

/* # 634 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static void CC2420TransmitP_congestionBackoff(void )
/* #line 634 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 635 */
    {
      CC2420TransmitP_RadioBackoff_requestCongestionBackoff(CC2420TransmitP_m_msg);
      CC2420TransmitP_BackoffTimer_start(CC2420TransmitP_myCongestionBackoff);
    }
/* #line 638 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 58 "/Users/doina/tinyos-2.x/tos/system/RandomMlcgC.nc" */
static uint32_t RandomMlcgC_Random_rand32(void )
/* #line 58 */
{
  uint32_t mlcg;
/* #line 59 */
  uint32_t p;
/* #line 59 */
  uint32_t q;
  uint64_t tmpseed;

/* #line 61 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      tmpseed = (uint64_t )33614U * (uint64_t )RandomMlcgC_seed;
      q = tmpseed;
      q = q >> 1;
      p = tmpseed >> 32;
      mlcg = p + q;
      if (mlcg & 0x80000000) {
          mlcg = mlcg & 0x7FFFFFFF;
          mlcg++;
        }
      RandomMlcgC_seed = mlcg;
    }
/* #line 73 */
    __nesc_atomic_end(__nesc_atomic); }
  return mlcg;
}

/* # 641 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static error_t CC2420TransmitP_acquireSpiResource(void )
/* #line 641 */
{
  error_t error = CC2420TransmitP_SpiResource_immediateRequest();

/* #line 643 */
  if (error != SUCCESS) {
      CC2420TransmitP_SpiResource_request();
    }
  return error;
}

/* # 126 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static error_t CC2420SpiP_Resource_immediateRequest(uint8_t id)
/* #line 126 */
{
  error_t error;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 129 */
    {
      if (CC2420SpiP_WorkingState_requestState(CC2420SpiP_S_BUSY) != SUCCESS) {
          {
            unsigned char __nesc_temp = 
/* #line 131 */
            EBUSY;

            {
/* #line 131 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 131 */
              return __nesc_temp;
            }
          }
        }
      if (CC2420SpiP_SpiResource_isOwner()) {
          CC2420SpiP_m_holder = id;
          error = SUCCESS;
        }
      else {
/* #line 139 */
        if ((error = CC2420SpiP_SpiResource_immediateRequest()) == SUCCESS) {
            CC2420SpiP_m_holder = id;
          }
        else {
            CC2420SpiP_WorkingState_toIdle();
          }
        }
    }
/* #line 146 */
    __nesc_atomic_end(__nesc_atomic); }
/* #line 146 */
  return error;
}

/* # 96 "/Users/doina/tinyos-2.x/tos/system/StateImplP.nc" */
static error_t StateImplP_State_requestState(uint8_t id, uint8_t reqState)
/* #line 96 */
{
  error_t returnVal = FAIL;

/* #line 98 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 98 */
    {
      if (reqState == StateImplP_S_IDLE || StateImplP_state[id] == StateImplP_S_IDLE) {
          StateImplP_state[id] = reqState;
          returnVal = SUCCESS;
        }
    }
/* #line 103 */
    __nesc_atomic_end(__nesc_atomic); }
  return returnVal;
}

/* # 174 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_Resource_isOwner(uint8_t id)
/* #line 174 */
{
  /* atomic removed: atomic calls only */
/* #line 175 */
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId == id && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_BUSY) {
        unsigned char __nesc_temp = 
/* #line 176 */
        TRUE;

/* #line 176 */
        return __nesc_temp;
      }
    else 
/* #line 177 */
      {
        unsigned char __nesc_temp = 
/* #line 177 */
        FALSE;

/* #line 177 */
        return __nesc_temp;
      }
  }
}

/* #line 130 */
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ResourceDefaultOwner_release(void )
/* #line 130 */
{
  /* atomic removed: atomic calls only */
/* #line 131 */
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_default_owner_id) {
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_GRANTING) {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_grantedTask_postTask();
            {
              unsigned char __nesc_temp = 
/* #line 135 */
              SUCCESS;

/* #line 135 */
              return __nesc_temp;
            }
          }
        else {
/* #line 137 */
          if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_IMM_GRANTING) {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_reqResId;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_BUSY;
              {
                unsigned char __nesc_temp = 
/* #line 140 */
                SUCCESS;

/* #line 140 */
                return __nesc_temp;
              }
            }
          }
      }
  }
/* #line 144 */
  return FAIL;
}

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

/* # 265 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc" */
static void HplMsp430Usart0P_Usart_setModeSpi(msp430_spi_union_config_t *config)
/* #line 265 */
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 267 */
    {
      HplMsp430Usart0P_Usart_resetUsart(TRUE);
      HplMsp430Usart0P_HplI2C_clearModeI2C();
      HplMsp430Usart0P_Usart_disableUart();
      HplMsp430Usart0P_configSpi(config);
      HplMsp430Usart0P_Usart_enableSpi();
      HplMsp430Usart0P_Usart_resetUsart(FALSE);
      HplMsp430Usart0P_Usart_clrIntr();
      HplMsp430Usart0P_Usart_disableIntr();
    }
/* #line 276 */
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

/* # 107 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static error_t CC2420SpiP_Resource_request(uint8_t id)
/* #line 107 */
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 109 */
    {
      if (CC2420SpiP_WorkingState_requestState(CC2420SpiP_S_BUSY) == SUCCESS) {
          CC2420SpiP_m_holder = id;
          if (CC2420SpiP_SpiResource_isOwner()) {
              CC2420SpiP_grant_postTask();
            }
          else {
              CC2420SpiP_SpiResource_request();
            }
        }
      else {
          CC2420SpiP_m_requests |= 1 << id;
        }
    }
/* #line 122 */
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

/* # 592 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static void CC2420TransmitP_attemptSend(void )
/* #line 592 */
{
  uint8_t status;
  bool congestion = TRUE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 596 */
    {
      if (CC2420TransmitP_m_state == CC2420TransmitP_S_CANCEL) {
          CC2420TransmitP_SFLUSHTX_strobe();
          CC2420TransmitP_releaseSpiResource();
          CC2420TransmitP_CSN_set();
          CC2420TransmitP_m_state = CC2420TransmitP_S_STARTED;
          CC2420TransmitP_Send_sendDone(CC2420TransmitP_m_msg, ECANCEL);
          {
/* #line 603 */
            __nesc_atomic_end(__nesc_atomic); 
/* #line 603 */
            return;
          }
        }

      CC2420TransmitP_CSN_clr();

      status = CC2420TransmitP_m_cca ? CC2420TransmitP_STXONCCA_strobe() : CC2420TransmitP_STXON_strobe();
      if (!(status & CC2420_STATUS_TX_ACTIVE)) {
          status = CC2420TransmitP_SNOP_strobe();
          if (status & CC2420_STATUS_TX_ACTIVE) {
              congestion = FALSE;
            }
        }

      CC2420TransmitP_m_state = congestion ? CC2420TransmitP_S_SAMPLE_CCA : CC2420TransmitP_S_SFD;
      CC2420TransmitP_CSN_set();
    }
/* #line 619 */
    __nesc_atomic_end(__nesc_atomic); }

  if (congestion) {
      CC2420TransmitP_totalCcaChecks = 0;
      CC2420TransmitP_releaseSpiResource();
      CC2420TransmitP_congestionBackoff();
    }
  else 
/* #line 625 */
    {
      CC2420TransmitP_BackoffTimer_start(CC2420TransmitP_CC2420_ABORT_PERIOD);
    }
}

/* # 318 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static cc2420_status_t CC2420SpiP_Strobe_strobe(uint8_t addr)
/* #line 318 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 319 */
    {
      if (CC2420SpiP_WorkingState_isIdle()) {
          {
            unsigned char __nesc_temp = 
/* #line 321 */
            0;

            {
/* #line 321 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 321 */
              return __nesc_temp;
            }
          }
        }
    }
/* #line 325 */
    __nesc_atomic_end(__nesc_atomic); }
/* #line 325 */
  return CC2420SpiP_SpiByte_write(addr);
}

/* # 133 "/Users/doina/tinyos-2.x/tos/system/StateImplP.nc" */
static bool StateImplP_State_isState(uint8_t id, uint8_t myState)
/* #line 133 */
{
  bool isState;

/* #line 135 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 135 */
    isState = StateImplP_state[id] == myState;
/* #line 135 */
    __nesc_atomic_end(__nesc_atomic); }
  return isState;
}

/* # 99 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiByte_write(uint8_t tx)
/* #line 99 */
{
  uint8_t byte;


  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_tx(tx);
  while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_isRxIntrPending()) ;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_clrRxIntr();
  byte = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_rx();

  return byte;
}

/* # 386 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc" */
static uint8_t HplMsp430Usart0P_Usart_rx(void )
/* #line 386 */
{
  uint8_t value;

/* #line 388 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 388 */
    value = U0RXBUF;
/* #line 388 */
    __nesc_atomic_end(__nesc_atomic); }
  return value;
}

/* # 149 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static error_t CC2420SpiP_Resource_release(uint8_t id)
/* #line 149 */
{
  uint8_t i;

/* #line 151 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 151 */
    {
      if (CC2420SpiP_m_holder != id) {
          {
            unsigned char __nesc_temp = 
/* #line 153 */
            FAIL;

            {
/* #line 153 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 153 */
              return __nesc_temp;
            }
          }
        }
/* #line 156 */
      CC2420SpiP_m_holder = CC2420SpiP_NO_HOLDER;
      if (!CC2420SpiP_m_requests) {
          CC2420SpiP_WorkingState_toIdle();
          CC2420SpiP_attemptRelease();
        }
      else {
          for (i = CC2420SpiP_m_holder + 1; ; i++) {
              i %= CC2420SpiP_RESOURCE_COUNT;

              if (CC2420SpiP_m_requests & (1 << i)) {
                  CC2420SpiP_m_holder = i;
                  CC2420SpiP_m_requests &= ~(1 << i);
                  CC2420SpiP_grant_postTask();
                  {
                    unsigned char __nesc_temp = 
/* #line 169 */
                    SUCCESS;

                    {
/* #line 169 */
                      __nesc_atomic_end(__nesc_atomic); 
/* #line 169 */
                      return __nesc_temp;
                    }
                  }
                }
            }
        }
    }
/* #line 175 */
    __nesc_atomic_end(__nesc_atomic); }
/* #line 175 */
  return SUCCESS;
}

/* #line 339 */
static error_t CC2420SpiP_attemptRelease(void )
/* #line 339 */
{


  if ((
/* #line 340 */
  CC2420SpiP_m_requests > 0
   || CC2420SpiP_m_holder != CC2420SpiP_NO_HOLDER)
   || !CC2420SpiP_WorkingState_isIdle()) {
      return FAIL;
    }
  /* atomic removed: atomic calls only */
  CC2420SpiP_release = TRUE;
  CC2420SpiP_ChipSpiResource_releasing();
  /* atomic removed: atomic calls only */
/* #line 348 */
  {
    if (CC2420SpiP_release) {
        CC2420SpiP_SpiResource_release();
        {
          unsigned char __nesc_temp = 
/* #line 351 */
          SUCCESS;

/* #line 351 */
          return __nesc_temp;
        }
      }
  }
  return EBUSY;
}

/* # 247 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc" */
static void HplMsp430Usart0P_Usart_disableSpi(void )
/* #line 247 */
{
  /* atomic removed: atomic calls only */
/* #line 248 */
  {
    HplMsp430Usart0P_ME1 &= ~(1 << 6);
    HplMsp430Usart0P_SIMO_selectIOFunc();
    HplMsp430Usart0P_SOMI_selectIOFunc();
    HplMsp430Usart0P_UCLK_selectIOFunc();
  }
}

/* # 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP_26_IO_set(void )
/* #line 45 */
{
/* #line 45 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 45 */
    * (volatile uint8_t * )29U |= 0x01 << 2;
/* #line 45 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* #line 46 */
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP_26_IO_clr(void )
/* #line 46 */
{
/* #line 46 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 46 */
    * (volatile uint8_t * )29U &= ~(0x01 << 2);
/* #line 46 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 696 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static void CC2420TransmitP_signalDone(error_t err)
/* #line 696 */
{
  /* atomic removed: atomic calls only */
/* #line 697 */
  CC2420TransmitP_m_state = CC2420TransmitP_S_STARTED;
  CC2420TransmitP_abortSpiRelease = FALSE;
  CC2420TransmitP_ChipSpiResource_attemptRelease();
  CC2420TransmitP_Send_sendDone(CC2420TransmitP_m_msg, err);
}

/* # 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc" */
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_enableCapture(uint8_t mode)
/* #line 38 */
{
  /* atomic removed: atomic calls only */
/* #line 39 */
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430TimerControl_disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_GeneralIO_selectModuleFunc();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430TimerControl_clearPendingInterrupt();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430Capture_clearOverflow();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430TimerControl_setControlAsCapture(mode);
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC_0_Msp430TimerControl_enableEvents();
  }
  return SUCCESS;
}

/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP_30_IO_clr(void )
/* #line 46 */
{
/* #line 46 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 46 */
    * (volatile uint8_t * )29U &= ~(0x01 << 6);
/* #line 46 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* #line 45 */
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP_30_IO_set(void )
/* #line 45 */
{
/* #line 45 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 45 */
    * (volatile uint8_t * )29U |= 0x01 << 6;
/* #line 45 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 260 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static cc2420_status_t CC2420SpiP_Ram_write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len)
/* #line 262 */
{

  cc2420_status_t status = 0;
  uint8_t tmpLen = len;
  uint8_t * tmpData = (uint8_t * )data;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 268 */
    {
      if (CC2420SpiP_WorkingState_isIdle()) {
          {
            unsigned char __nesc_temp = 
/* #line 270 */
            status;

            {
/* #line 270 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 270 */
              return __nesc_temp;
            }
          }
        }
    }
/* #line 274 */
    __nesc_atomic_end(__nesc_atomic); }
/* #line 274 */
  addr += offset;

  status = CC2420SpiP_SpiByte_write(addr | 0x80);
  CC2420SpiP_SpiByte_write((addr >> 1) & 0xc0);
  for (; len; len--) {
      CC2420SpiP_SpiByte_write(tmpData[tmpLen - len]);
    }

  return status;
}

/* # 122 "/Users/doina/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc" */
static void CC2420PacketP_PacketTimeStamp32khz_clear(message_t *msg)
{
  __nesc_hton_int8(CC2420PacketP_CC2420PacketBody_getMetadata(msg)->timesync.data, FALSE);
  __nesc_hton_uint32(CC2420PacketP_CC2420PacketBody_getMetadata(msg)->timestamp.data, CC2420_INVALID_TIMESTAMP);
}

/* # 96 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Counter_get();
/* #line 98 */
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type expires;
/* #line 98 */
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
/* #line 121 */
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_AlarmFrom_startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_from_size_type )remaining << 5);
}

/* # 69 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformCounterC.nc" */
static /*CounterMilli32C.Transform*/TransformCounterC_1_to_size_type /*CounterMilli32C.Transform*/TransformCounterC_1_Counter_get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC_1_to_size_type rv = 0;

/* #line 72 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC_1_upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC_1_m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC_1_from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_get();

/* #line 76 */
      if (/*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC_1_CounterFrom_get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC_1_to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC_1_to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC_1_LOW_SHIFT_RIGHT;

/* #line 90 */
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC_1_HIGH_SHIFT_LEFT) | low_to;
      }
    }
/* #line 92 */
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
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
    case CC2420CsmaP_startDone_task:
/* #line 64 */
      CC2420CsmaP_startDone_task_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case CC2420CsmaP_stopDone_task:
/* #line 64 */
      CC2420CsmaP_stopDone_task_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case CC2420CsmaP_sendDone_task:
/* #line 64 */
      CC2420CsmaP_sendDone_task_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case CC2420ControlP_sync:
/* #line 64 */
      CC2420ControlP_sync_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case CC2420ControlP_syncDone:
/* #line 64 */
      CC2420ControlP_syncDone_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case CC2420SpiP_grant:
/* #line 64 */
      CC2420SpiP_grant_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone_task:
/* #line 64 */
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone_task_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_grantedTask:
/* #line 64 */
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_grantedTask_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case CC2420ReceiveP_receiveDone_task:
/* #line 64 */
      CC2420ReceiveP_receiveDone_task_runTask();
/* #line 64 */
      break;
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
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_CancelTask:
/* #line 64 */
      /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_CancelTask_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_errorTask:
/* #line 64 */
      /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_errorTask_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_changedTask:
/* #line 64 */
      /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_changedTask_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_timerTask:
/* #line 64 */
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_timerTask_runTask();
/* #line 64 */
      break;
/* #line 64 */
    case /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_changedTask:
/* #line 64 */
      /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_changedTask_runTask();
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
/* # 127 "TestDisseminationC.nc" */
static void TestDisseminationC_Value16_changed(void )
/* #line 127 */
{
  const uint16_t *newVal = TestDisseminationC_Value16_get();

/* #line 129 */
  if (*newVal == 0xABCD) {
      TestDisseminationC_Leds_led1Toggle();
      ;
    }
  else {
      ;
    }
}

/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_toggle(void )
/* #line 47 */
{
/* #line 47 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 47 */
    * (volatile uint8_t * )49U ^= 0x01 << 5;
/* #line 47 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 130 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static void DisseminationEngineImplP_sendObject(uint16_t key)
/* #line 130 */
{
  void *object;
  uint8_t objectSize = 0;

  dissemination_message_t *dMsg = 
  (dissemination_message_t *)DisseminationEngineImplP_AMSend_getPayload(&DisseminationEngineImplP_m_buf, sizeof(dissemination_message_t ));

/* #line 136 */
  if (dMsg != (void *)0) {
      DisseminationEngineImplP_m_bufBusy = TRUE;

      __nesc_hton_uint16(dMsg->key.data, key);
      __nesc_hton_uint32(dMsg->seqno.data, DisseminationEngineImplP_DisseminationCache_requestSeqno(key));

      if (__nesc_ntoh_uint32(dMsg->seqno.data) != DISSEMINATION_SEQNO_UNKNOWN) {
          object = DisseminationEngineImplP_DisseminationCache_requestData(key, &objectSize);
          if (objectSize + sizeof(dissemination_message_t ) > 
          DisseminationEngineImplP_AMSend_maxPayloadLength()) {
              objectSize = DisseminationEngineImplP_AMSend_maxPayloadLength() - sizeof(dissemination_message_t );
            }
          memcpy(dMsg->data, object, objectSize);
        }
      DisseminationEngineImplP_AMSend_send(AM_BROADCAST_ADDR, 
      &DisseminationEngineImplP_m_buf, sizeof(dissemination_message_t ) + objectSize);
    }
}

/* # 157 "/Users/doina/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc" */
static void *CC2420CsmaP_Send_getPayload(message_t *m, uint8_t len)
/* #line 157 */
{
  if (len <= CC2420CsmaP_Send_maxPayloadLength()) {
      return (void * )m->data;
    }
  else {
      return (void *)0;
    }
}

/* # 122 "/Users/doina/tinyos-2.x/tos/chips/cc2420/CC2420ActiveMessageP.nc" */
static am_id_t CC2420ActiveMessageP_AMPacket_type(message_t *amsg)
/* #line 122 */
{
  cc2420_header_t *header = CC2420ActiveMessageP_CC2420PacketBody_getHeader(amsg);

/* #line 124 */
  return __nesc_ntoh_leuint8(header->type.data);
}

/* #line 97 */
static am_addr_t CC2420ActiveMessageP_AMPacket_destination(message_t *amsg)
/* #line 97 */
{
  cc2420_header_t *header = CC2420ActiveMessageP_CC2420PacketBody_getHeader(amsg);

/* #line 99 */
  return __nesc_ntoh_leuint16(header->dest.data);
}

/* #line 61 */
static error_t CC2420ActiveMessageP_AMSend_send(am_id_t id, am_addr_t addr, 
message_t *msg, 
uint8_t len)
/* #line 63 */
{
  cc2420_header_t *header = CC2420ActiveMessageP_CC2420PacketBody_getHeader(msg);

  if (len > CC2420ActiveMessageP_Packet_maxPayloadLength()) {
      return ESIZE;
    }

  __nesc_hton_leuint8(header->type.data, id);
  __nesc_hton_leuint16(header->dest.data, addr);
  __nesc_hton_leuint16(header->destpan.data, CC2420ActiveMessageP_CC2420Config_getPanAddr());
  __nesc_hton_leuint16(header->src.data, CC2420ActiveMessageP_AMPacket_address());

  CC2420ActiveMessageP_SendNotifier_aboutToSend(id, addr, msg);

  return CC2420ActiveMessageP_SubSend_send(msg, len);
}

/* # 95 "/Users/doina/tinyos-2.x/tos/system/ActiveMessageAddressC.nc" */
static am_addr_t ActiveMessageAddressC_amAddress(void )
/* #line 95 */
{
  am_addr_t myAddr;

/* #line 97 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 97 */
    myAddr = ActiveMessageAddressC_addr;
/* #line 97 */
    __nesc_atomic_end(__nesc_atomic); }
  return myAddr;
}

/* # 671 "/Users/doina/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc" */
static void CC2420TransmitP_loadTXFIFO(void )
/* #line 671 */
{
  cc2420_header_t *header = CC2420TransmitP_CC2420PacketBody_getHeader(CC2420TransmitP_m_msg);
  uint8_t tx_power = __nesc_ntoh_uint8(CC2420TransmitP_CC2420PacketBody_getMetadata(CC2420TransmitP_m_msg)->tx_power.data);

  if (!tx_power) {
      tx_power = 31;
    }

  CC2420TransmitP_CSN_clr();

  if (CC2420TransmitP_m_tx_power != tx_power) {
      CC2420TransmitP_TXCTRL_write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
      3 << CC2420_TXCTRL_PA_CURRENT)) | (
      1 << CC2420_TXCTRL_RESERVED)) | ((
      tx_power & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
    }

  CC2420TransmitP_m_tx_power = tx_power;

  {
    uint8_t tmpLen  = __nesc_ntoh_leuint8(header->length.data) - 1;

/* #line 692 */
    CC2420TransmitP_TXFIFO_write((uint8_t * )header, __nesc_ntoh_leuint8(header->length.data) - 1);
  }
}

/* # 305 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static cc2420_status_t CC2420SpiP_Reg_write(uint8_t addr, uint16_t data)
/* #line 305 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 306 */
    {
      if (CC2420SpiP_WorkingState_isIdle()) {
          {
            unsigned char __nesc_temp = 
/* #line 308 */
            0;

            {
/* #line 308 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 308 */
              return __nesc_temp;
            }
          }
        }
    }
/* #line 312 */
    __nesc_atomic_end(__nesc_atomic); }
/* #line 311 */
  CC2420SpiP_SpiByte_write(addr);
  CC2420SpiP_SpiByte_write(data >> 8);
  return CC2420SpiP_SpiByte_write(data & 0xff);
}

/* # 144 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc" */
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SpiPacket_send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len)
/* #line 146 */
{

  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_client = id;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_tx_buf = tx_buf;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_rx_buf = rx_buf;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_len = len;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_pos = 0;

  if (len) {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_enableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_continueOp();
    }
  else {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_signalDone_task_postTask();
    }

  return SUCCESS;
}

/* #line 121 */
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_continueOp(void )
/* #line 121 */
{

  uint8_t end;
  uint8_t tmp;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 126 */
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_pos] : 0);

      end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_pos + /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_SPI_ATOMIC_SIZE;
      if (end > /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_len) {
        end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_len;
        }
      while (++/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_pos < end) {
          while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_isRxIntrPending()) ;
          tmp = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_rx();
          if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_rx_buf) {
            /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_pos - 1] = tmp;
            }
/* #line 138 */
          /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_Usart_tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP_0_m_pos] : 0);
        }
    }
/* #line 140 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/State.nc" */
static void UniqueSendP_State_toIdle(void ){
/* #line 56 */
  StateImplP_State_toIdle(2U);
/* #line 56 */
}
/* #line 56 */
/* # 116 "TestDisseminationC.nc" */
static void TestDisseminationC_Value32_changed(void )
/* #line 116 */
{
  const uint32_t *newVal = TestDisseminationC_Value32_get();

/* #line 118 */
  if (*newVal == 0xDEADBEEF) {
      TestDisseminationC_Leds_led0Toggle();
      ;
    }
  else {
      ;
    }
}

/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_toggle(void )
/* #line 47 */
{
/* #line 47 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 47 */
    * (volatile uint8_t * )49U ^= 0x01 << 4;
/* #line 47 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 155 "/Users/doina/tinyos-2.x/tos/system/AMQueueImplP.nc" */
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_sendDone(uint8_t last, message_t * msg, error_t err)
/* #line 155 */
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_queue[last].msg = (void *)0;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_tryToSend();
  /*AMQueueP.AMQueueImplP*/AMQueueImplP_0_Send_sendDone(last, msg, err);
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

/* # 102 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static void DisseminationEngineImplP_DisseminationCache_newData(uint16_t key)
/* #line 102 */
{

  if (!DisseminationEngineImplP_m_running || DisseminationEngineImplP_m_bufBusy) {
/* #line 104 */
      return;
    }
  DisseminationEngineImplP_sendObject(key);
  DisseminationEngineImplP_TrickleTimer_reset(key);
}

/* # 122 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimerImplP.nc" */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_reset(uint8_t id)
/* #line 122 */
{
  /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].period = 1;
  /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].count = 0;
  if (/*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].time != 0) {
      ;
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 127 */
        {
          /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Changed_set(id);
        }
/* #line 129 */
        __nesc_atomic_end(__nesc_atomic); }
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].time = 0;
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].remainder = 0;
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_generateTime(id);
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_adjustTimer();
    }
  else 
/* #line 134 */
    {
      ;
    }
}

/* #line 246 */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_generateTime(uint8_t id)
/* #line 246 */
{
  uint32_t newTime;
  uint16_t rval;

  if (/*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].time != 0) {
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].period *= 2;
      if (/*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].period > 1024) {
          /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].period = 1024;
        }
    }

  /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].time = /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].remainder;

  newTime = /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].period;
  newTime = newTime << (10 - 1);

  rval = /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Random_rand16() % (/*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].period << (10 - 1));
  newTime += rval;

  /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].remainder = (/*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].period << 10) - newTime;
  /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].time += newTime;
  ;
}

/* #line 203 */
static void /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_adjustTimer(void )
/* #line 203 */
{
  uint8_t i;
  uint32_t lowest = 0;
  bool set = FALSE;





  uint32_t elapsed = /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_getNow() - /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_gett0();

  for (i = 0; i < 2U; i++) {
      uint32_t timeRemaining = /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[i].time;

/* #line 216 */
      if (timeRemaining != 0) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 217 */
            {
              if (!/*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Changed_get(i)) {
                  /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Changed_clear(i);
                  timeRemaining -= elapsed;
                }
            }
/* #line 222 */
            __nesc_atomic_end(__nesc_atomic); }
          if (!set) {
              lowest = timeRemaining;
              set = TRUE;
            }
          else {
/* #line 227 */
            if (timeRemaining < lowest) {
                lowest = timeRemaining;
              }
            }
        }
    }
/* #line 232 */
  if (set) {
      uint32_t timerVal = lowest;

/* #line 234 */
      timerVal = timerVal;
      ;
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_startOneShot(timerVal);
    }
  else {
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Timer_stop();
    }
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

/* # 136 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc" */
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_Alarm_startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_1_set_alarm();
    }
/* #line 143 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 279 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static uint16_t CC2420ControlP_CC2420Config_getShortAddr(void )
/* #line 279 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 280 */
    {
      unsigned int __nesc_temp = 
/* #line 280 */
      CC2420ControlP_m_short_addr;

      {
/* #line 280 */
        __nesc_atomic_end(__nesc_atomic); 
/* #line 280 */
        return __nesc_temp;
      }
    }
/* #line 282 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 106 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminatorP.nc" */
static void /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationCache_storeData(void *data, uint8_t size, 
uint32_t newSeqno)
/* #line 107 */
{
  memcpy(&/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_valueCache, data, size < sizeof(/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_t ) ? size : sizeof(/*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_t ));
  /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_seqno = newSeqno;




  /*TestDisseminationAppC.Object16C.DisseminatorP*/DisseminatorP_1_DisseminationValue_changed();
}

/* #line 106 */
static void /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationCache_storeData(void *data, uint8_t size, 
uint32_t newSeqno)
/* #line 107 */
{
  memcpy(&/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_valueCache, data, size < sizeof(/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_t ) ? size : sizeof(/*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_t ));
  /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_seqno = newSeqno;




  /*TestDisseminationAppC.Object32C.DisseminatorP*/DisseminatorP_0_DisseminationValue_changed();
}

/* # 412 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
static void CC2420ReceiveP_waitForNextPacket(void )
/* #line 412 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 413 */
    {
      if (CC2420ReceiveP_m_state == CC2420ReceiveP_S_STOPPED) {
          CC2420ReceiveP_SpiResource_release();
          {
/* #line 416 */
            __nesc_atomic_end(__nesc_atomic); 
/* #line 416 */
            return;
          }
        }
      CC2420ReceiveP_receivingPacket = FALSE;










      if ((CC2420ReceiveP_m_missed_packets && CC2420ReceiveP_FIFO_get()) || !CC2420ReceiveP_FIFOP_get()) {

          if (CC2420ReceiveP_m_missed_packets) {
              CC2420ReceiveP_m_missed_packets--;
            }

          CC2420ReceiveP_beginReceive();
        }
      else {

          CC2420ReceiveP_m_state = CC2420ReceiveP_S_STARTED;
          CC2420ReceiveP_m_missed_packets = 0;
          CC2420ReceiveP_SpiResource_release();
        }
    }
/* #line 444 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* #line 367 */
static void CC2420ReceiveP_beginReceive(void )
/* #line 367 */
{
  CC2420ReceiveP_m_state = CC2420ReceiveP_S_RX_LENGTH;
  /* atomic removed: atomic calls only */
  CC2420ReceiveP_receivingPacket = TRUE;
  if (CC2420ReceiveP_SpiResource_isOwner()) {
      CC2420ReceiveP_receive();
    }
  else {
/* #line 374 */
    if (CC2420ReceiveP_SpiResource_immediateRequest() == SUCCESS) {
        CC2420ReceiveP_receive();
      }
    else {
        CC2420ReceiveP_SpiResource_request();
      }
    }
}

/* #line 402 */
static void CC2420ReceiveP_receive(void )
/* #line 402 */
{
  CC2420ReceiveP_CSN_clr();
  CC2420ReceiveP_RXFIFO_beginRead((uint8_t *)CC2420ReceiveP_CC2420PacketBody_getHeader(CC2420ReceiveP_m_p_rx_buf), 1);
}

/* # 189 "/Users/doina/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc" */
static cc2420_status_t CC2420SpiP_Fifo_beginRead(uint8_t addr, uint8_t *data, 
uint8_t len)
/* #line 190 */
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 194 */
    {
      if (CC2420SpiP_WorkingState_isIdle()) {
          {
            unsigned char __nesc_temp = 
/* #line 196 */
            status;

            {
/* #line 196 */
              __nesc_atomic_end(__nesc_atomic); 
/* #line 196 */
              return __nesc_temp;
            }
          }
        }
    }
/* #line 200 */
    __nesc_atomic_end(__nesc_atomic); }
/* #line 200 */
  CC2420SpiP_m_addr = addr | 0x40;

  status = CC2420SpiP_SpiByte_write(CC2420SpiP_m_addr);
  CC2420SpiP_Fifo_continueRead(addr, data, len);

  return status;
}

/* #line 329 */
static void CC2420SpiP_SpiPacket_sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error)
/* #line 330 */
{
  if (CC2420SpiP_m_addr & 0x40) {
      CC2420SpiP_Fifo_readDone(CC2420SpiP_m_addr & ~0x40, rx_buf, len, error);
    }
  else 
/* #line 333 */
    {
      CC2420SpiP_Fifo_writeDone(CC2420SpiP_m_addr, tx_buf, len, error);
    }
}

/* # 385 "/Users/doina/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc" */
static void CC2420ReceiveP_flush(void )
/* #line 385 */
{
  CC2420ReceiveP_reset_state();
  CC2420ReceiveP_CSN_set();
  CC2420ReceiveP_CSN_clr();
  CC2420ReceiveP_SFLUSHRX_strobe();
  CC2420ReceiveP_SFLUSHRX_strobe();
  CC2420ReceiveP_CSN_set();
  CC2420ReceiveP_SpiResource_release();
  CC2420ReceiveP_waitForNextPacket();
}

/* #line 450 */
static void CC2420ReceiveP_reset_state(void )
/* #line 450 */
{
  CC2420ReceiveP_m_bytes_left = CC2420ReceiveP_RXFIFO_SIZE;
  /* atomic removed: atomic calls only */
/* #line 452 */
  CC2420ReceiveP_receivingPacket = FALSE;
  CC2420ReceiveP_m_timestamp_head = 0;
  CC2420ReceiveP_m_timestamp_size = 0;
  CC2420ReceiveP_m_missed_packets = 0;
}

/* # 456 "/Users/doina/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc" */
static void CC2420ControlP_writeFsctrl(void )
/* #line 456 */
{
  uint8_t channel;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 459 */
    {
      channel = CC2420ControlP_m_channel;
    }
/* #line 461 */
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP_FSCTRL_write((1 << CC2420_FSCTRL_LOCK_THR) | (((
  channel - 11) * 5 + 357) << CC2420_FSCTRL_FREQ));
}







static void CC2420ControlP_writeMdmctrl0(void )
/* #line 473 */
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 474 */
    {
      CC2420ControlP_MDMCTRL0_write((((((((1 << CC2420_MDMCTRL0_RESERVED_FRAME_MODE) | ((
      CC2420ControlP_addressRecognition && CC2420ControlP_hwAddressRecognition) << CC2420_MDMCTRL0_ADR_DECODE)) | (
      2 << CC2420_MDMCTRL0_CCA_HYST)) | (
      3 << CC2420_MDMCTRL0_CCA_MOD)) | (
      1 << CC2420_MDMCTRL0_AUTOCRC)) | ((
      CC2420ControlP_autoAckEnabled && CC2420ControlP_hwAutoAckDefault) << CC2420_MDMCTRL0_AUTOACK)) | (
      0 << CC2420_MDMCTRL0_AUTOACK)) | (
      2 << CC2420_MDMCTRL0_PREAMBLE_LENGTH));
    }
/* #line 483 */
    __nesc_atomic_end(__nesc_atomic); }
}







static void CC2420ControlP_writeId(void )
/* #line 492 */
{
  nxle_uint16_t id[2];

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 495 */
    {
      __nesc_hton_leuint16(id[0].data, CC2420ControlP_m_pan);
      __nesc_hton_leuint16(id[1].data, CC2420ControlP_m_short_addr);
    }
/* #line 498 */
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP_PANID_write(0, (uint8_t *)&id, sizeof id);
}

/* # 81 "/Users/doina/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc" */
static error_t CC2420CsmaP_SplitControl_start(void )
/* #line 81 */
{
  if (CC2420CsmaP_SplitControlState_requestState(CC2420CsmaP_S_STARTING) == SUCCESS) {
      CC2420CsmaP_CC2420Power_startVReg();
      return SUCCESS;
    }
  else {
/* #line 86 */
    if (CC2420CsmaP_SplitControlState_isState(CC2420CsmaP_S_STARTED)) {
        return EALREADY;
      }
    else {
/* #line 89 */
      if (CC2420CsmaP_SplitControlState_isState(CC2420CsmaP_S_STARTING)) {
          return SUCCESS;
        }
      }
    }
/* #line 93 */
  return EBUSY;
}

/* # 91 "/Users/doina/tinyos-2.x/tos/lib/net/drip/DisseminationEngineImplP.nc" */
static error_t DisseminationEngineImplP_DisseminationCache_start(uint16_t key)
/* #line 91 */
{
  error_t result = DisseminationEngineImplP_TrickleTimer_start(key);

/* #line 93 */
  DisseminationEngineImplP_TrickleTimer_reset(key);
  return result;
}

/* # 92 "/Users/doina/tinyos-2.x/tos/lib/net/TrickleTimerImplP.nc" */
static error_t /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_TrickleTimer_start(uint8_t id)
/* #line 92 */
{
  if (/*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].time != 0) {
      return EBUSY;
    }
  /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].time = 0;
  /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].remainder = 0;
  /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_trickles[id].count = 0;
  /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_generateTime(id);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 100 */
    {
      /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_Changed_set(id);
    }
/* #line 102 */
    __nesc_atomic_end(__nesc_atomic); }
  /*DisseminationTimerP.TrickleTimerMilliC.TrickleTimerImplP*/TrickleTimerImplP_0_adjustTimer();
  ;
  return SUCCESS;
}

/* # 143 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_getNow(), dt, FALSE);
}

/* # 53 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc" */
  void sig_PORT1_VECTOR(void )
{
  volatile int n = P1IFG & P1IE;

  if (n & (1 << 0)) {
/* #line 57 */
      HplMsp430InterruptP_Port10_fired();
/* #line 57 */
      return;
    }
/* #line 58 */
  if (n & (1 << 1)) {
/* #line 58 */
      HplMsp430InterruptP_Port11_fired();
/* #line 58 */
      return;
    }
/* #line 59 */
  if (n & (1 << 2)) {
/* #line 59 */
      HplMsp430InterruptP_Port12_fired();
/* #line 59 */
      return;
    }
/* #line 60 */
  if (n & (1 << 3)) {
/* #line 60 */
      HplMsp430InterruptP_Port13_fired();
/* #line 60 */
      return;
    }
/* #line 61 */
  if (n & (1 << 4)) {
/* #line 61 */
      HplMsp430InterruptP_Port14_fired();
/* #line 61 */
      return;
    }
/* #line 62 */
  if (n & (1 << 5)) {
/* #line 62 */
      HplMsp430InterruptP_Port15_fired();
/* #line 62 */
      return;
    }
/* #line 63 */
  if (n & (1 << 6)) {
/* #line 63 */
      HplMsp430InterruptP_Port16_fired();
/* #line 63 */
      return;
    }
/* #line 64 */
  if (n & (1 << 7)) {
/* #line 64 */
      HplMsp430InterruptP_Port17_fired();
/* #line 64 */
      return;
    }
}

/* #line 158 */
  void sig_PORT2_VECTOR(void )
{
  volatile int n = P2IFG & P2IE;

  if (n & (1 << 0)) {
/* #line 162 */
      HplMsp430InterruptP_Port20_fired();
/* #line 162 */
      return;
    }
/* #line 163 */
  if (n & (1 << 1)) {
/* #line 163 */
      HplMsp430InterruptP_Port21_fired();
/* #line 163 */
      return;
    }
/* #line 164 */
  if (n & (1 << 2)) {
/* #line 164 */
      HplMsp430InterruptP_Port22_fired();
/* #line 164 */
      return;
    }
/* #line 165 */
  if (n & (1 << 3)) {
/* #line 165 */
      HplMsp430InterruptP_Port23_fired();
/* #line 165 */
      return;
    }
/* #line 166 */
  if (n & (1 << 4)) {
/* #line 166 */
      HplMsp430InterruptP_Port24_fired();
/* #line 166 */
      return;
    }
/* #line 167 */
  if (n & (1 << 5)) {
/* #line 167 */
      HplMsp430InterruptP_Port25_fired();
/* #line 167 */
      return;
    }
/* #line 168 */
  if (n & (1 << 6)) {
/* #line 168 */
      HplMsp430InterruptP_Port26_fired();
/* #line 168 */
      return;
    }
/* #line 169 */
  if (n & (1 << 7)) {
/* #line 169 */
      HplMsp430InterruptP_Port27_fired();
/* #line 169 */
      return;
    }
}

/* # 96 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc" */
  void sig_UART0RX_VECTOR(void )
/* #line 96 */
{
  uint8_t temp = U0RXBUF;

/* #line 98 */
  HplMsp430Usart0P_Interrupts_rxDone(temp);
}

/* # 150 "/Users/doina/tinyos-2.x/tos/system/ArbiterP.nc" */
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ArbiterInfo_inUse(void )
/* #line 150 */
{
  /* atomic removed: atomic calls only */
/* #line 151 */
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
/* #line 153 */
        FALSE;

/* #line 153 */
        return __nesc_temp;
      }
  }
/* #line 155 */
  return TRUE;
}






static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_ArbiterInfo_userId(void )
/* #line 163 */
{
  /* atomic removed: atomic calls only */
/* #line 164 */
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_state != /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_RES_BUSY) 
      {
        unsigned char __nesc_temp = 
/* #line 166 */
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_NO_RES;

/* #line 166 */
        return __nesc_temp;
      }
/* #line 167 */
    {
      unsigned char __nesc_temp = 
/* #line 167 */
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP_0_resId;

/* #line 167 */
      return __nesc_temp;
    }
  }
}

/* # 101 "/Users/doina/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc" */
  void sig_UART0TX_VECTOR(void )
/* #line 101 */
{
  if (HplMsp430Usart0P_HplI2C_isI2C()) {
    HplMsp430Usart0P_I2CInterrupts_fired();
    }
  else {
/* #line 105 */
    HplMsp430Usart0P_Interrupts_txDone();
    }
}

