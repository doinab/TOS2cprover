#define nx_struct struct
#define nx_union union
/* #define dbg(mode, format, ...) ((void)0)
   #define dbg_clear(mode, format, ...) ((void)0)
   #define dbg_active(mode) 0 */
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
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.h" */
enum __nesc_unnamed4259 {
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
typedef struct __nesc_unnamed4260 {

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
typedef struct __nesc_unnamed4261 {

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
typedef struct __nesc_unnamed4262 {

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
/* # 29 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.h" */
typedef struct __nesc_unnamed4263 {
/* #line 29 */
  int notUsed;
} 
/* #line 29 */
TMilli;
typedef struct __nesc_unnamed4264 {
/* #line 30 */
  int notUsed;
} 
/* #line 30 */
T32khz;
typedef struct __nesc_unnamed4265 {
/* #line 31 */
  int notUsed;
} 
/* #line 31 */
TMicro;
/* # 32 "/Users/doina/tinyos-2.x/tos/types/Leds.h" */
enum __nesc_unnamed4266 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
typedef TMilli BlinkC_Timer0_precision_tag;
typedef TMilli BlinkC_Timer1_precision_tag;
typedef TMilli BlinkC_Timer2_precision_tag;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC_0___nesc_unnamed4267 {
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
uint8_t arg_0x1561800);
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_VectorTimerX0_fired(void );
/* #line 28 */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Overflow_fired(void );
/* #line 28 */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_VectorTimerX1_fired(void );
/* #line 28 */
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_default_fired(
/* # 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc" */
uint8_t arg_0x1561800);
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
/* # 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc" */
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Event_fired(void );
/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_default_fired(void );
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
/* # 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void BlinkC_Timer0_fired(void );
/* # 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc" */
static void BlinkC_Boot_booted(void );
/* # 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void BlinkC_Timer1_fired(void );
/* #line 72 */
static void BlinkC_Timer2_fired(void );
/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
static error_t LedsP_Init_init(void );
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
static void LedsP_Leds_led0Toggle(void );
/* #line 72 */
static void LedsP_Leds_led1Toggle(void );
/* #line 89 */
static void LedsP_Leds_led2Toggle(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
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
/* # 31 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_toggle(void );



static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_makeOutput(void );
/* #line 29 */
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_set(void );

static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_toggle(void );



static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_makeOutput(void );
/* #line 29 */
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_set(void );

static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_toggle(void );



static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_makeOutput(void );
/* #line 29 */
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_set(void );
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
uint8_t arg_0x18a3ea0);
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startPeriodic(
/* # 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
uint8_t arg_0x18a3ea0, 
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
uint32_t dt);
/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_0_Counter_overflow(void );
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

enum Msp430ClockP___nesc_unnamed4268 {

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
uint8_t arg_0x1561800);
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
uint8_t arg_0x1561800);
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
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_int2CC(uint16_t x)  ;
/* #line 74 */
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Control_getControl(void );
/* #line 139 */
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Capture_getEvent(void );
/* #line 169 */
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Event_fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Capture_default_captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_default_fired(void );



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
enum SchedulerBasicP___nesc_unnamed4269 {

  SchedulerBasicP_NUM_TASKS = 2U, 
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




static inline void SchedulerBasicP_TaskBasic_default_runTask(uint8_t id);
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
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void BlinkC_Timer0_startPeriodic(uint32_t dt);
/* #line 53 */
static void BlinkC_Timer1_startPeriodic(uint32_t dt);
/* # 56 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc" */
static void BlinkC_Leds_led0Toggle(void );
/* #line 72 */
static void BlinkC_Leds_led1Toggle(void );
/* #line 89 */
static void BlinkC_Leds_led2Toggle(void );
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
static void BlinkC_Timer2_startPeriodic(uint32_t dt);
/* # 49 "BlinkC.nc" */
static inline void BlinkC_Boot_booted(void );






static inline void BlinkC_Timer0_fired(void );





static inline void BlinkC_Timer1_fired(void );





static inline void BlinkC_Timer2_fired(void );
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
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_set(void );

static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_toggle(void );




static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_makeOutput(void );
/* #line 45 */
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_set(void );

static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_toggle(void );




static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_makeOutput(void );
/* #line 45 */
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_set(void );

static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_toggle(void );




static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_makeOutput(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_HplGeneralIO_toggle(void );
/* #line 71 */
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_HplGeneralIO_makeOutput(void );
/* #line 34 */
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_HplGeneralIO_set(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_set(void );

static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_toggle(void );



static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_makeOutput(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_HplGeneralIO_toggle(void );
/* #line 71 */
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_HplGeneralIO_makeOutput(void );
/* #line 34 */
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_HplGeneralIO_set(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_set(void );

static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_toggle(void );



static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_makeOutput(void );
/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_HplGeneralIO_toggle(void );
/* #line 71 */
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_HplGeneralIO_makeOutput(void );
/* #line 34 */
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_HplGeneralIO_set(void );
/* # 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_set(void );

static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_toggle(void );



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

enum /*CounterMilli32C.Transform*/TransformCounterC_0___nesc_unnamed4270 {

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

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC_0___nesc_unnamed4271 {

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
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0___nesc_unnamed4272 {
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


static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_fired_runTask(void );






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
uint8_t arg_0x18a3ea0);
/* #line 60 */
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0___nesc_unnamed4273 {
/* #line 60 */
  VirtualizeTimerC_0_updateFromTimer = 1U
};
/* #line 60 */
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0___nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer];
/* #line 42 */
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0___nesc_unnamed4274 {

  VirtualizeTimerC_0_NUM_TIMERS = 3U, 
  VirtualizeTimerC_0_END_OF_LIST = 255
};








/* #line 48 */
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0___nesc_unnamed4275 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_fireTimers(uint32_t now);
/* #line 89 */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer_runTask(void );
/* #line 128 */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_fired(void );




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startPeriodic(uint8_t num, uint32_t dt);
/* #line 193 */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_default_fired(uint8_t num);
/* # 47 "/Users/doina/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc" */
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC_0_Counter_overflow(void );
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
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Event_fired(uint8_t arg_0x1561800){
/* #line 28 */
  switch (arg_0x1561800) {
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
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP_0_Event_default_fired(arg_0x1561800);
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
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP_0___nesc_unnamed4276 {
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
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP_1___nesc_unnamed4277 {
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
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP_2___nesc_unnamed4278 {
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

/* # 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc" */
inline static void /*CounterMilli32C.Transform*/TransformCounterC_0_Counter_overflow(void ){
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
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3___nesc_unnamed4279 {
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
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP_4___nesc_unnamed4280 {
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




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_default_fired(void )
{
}

/* # 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc" */
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_fired(void ){
/* #line 34 */
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5_Compare_default_fired();
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
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP_5___nesc_unnamed4281 {
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
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP_6___nesc_unnamed4282 {
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
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP_7___nesc_unnamed4283 {
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
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP_8___nesc_unnamed4284 {
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
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP_9___nesc_unnamed4285 {
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
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_set(void )
/* #line 45 */
{
  /* atomic removed: atomic calls only */
/* #line 45 */
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

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
/* # 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_set(void )
/* #line 45 */
{
  /* atomic removed: atomic calls only */
/* #line 45 */
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

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






    /* MotePlatformC_uwait(1024 * 10); */

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
/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_toggle(void )
/* #line 47 */
{
/* #line 47 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 47 */
    * (volatile uint8_t * )49U ^= 0x01 << 4;
/* #line 47 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_HplGeneralIO_toggle(void ){
/* #line 44 */
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP_36_IO_toggle();
/* #line 44 */
}
/* #line 44 */
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_toggle(void )
/* #line 39 */
{
/* #line 39 */
  /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_HplGeneralIO_toggle();
}

/* # 31 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led0_toggle(void ){
/* #line 31 */
  /*PlatformLedsC.Led0Impl*/Msp430GpioC_0_GeneralIO_toggle();
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
inline static void BlinkC_Leds_led0Toggle(void ){
/* #line 56 */
  LedsP_Leds_led0Toggle();
/* #line 56 */
}
/* #line 56 */
/* # 56 "BlinkC.nc" */
static inline void BlinkC_Timer0_fired(void )
{
  ;
  BlinkC_Leds_led0Toggle();
}

/* # 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc" */
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_toggle(void )
/* #line 47 */
{
/* #line 47 */
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
/* #line 47 */
    * (volatile uint8_t * )49U ^= 0x01 << 5;
/* #line 47 */
    __nesc_atomic_end(__nesc_atomic); }
}

/* # 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc" */
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_HplGeneralIO_toggle(void ){
/* #line 44 */
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP_37_IO_toggle();
/* #line 44 */
}
/* #line 44 */
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_toggle(void )
/* #line 39 */
{
/* #line 39 */
  /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_HplGeneralIO_toggle();
}

/* # 31 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led1_toggle(void ){
/* #line 31 */
  /*PlatformLedsC.Led1Impl*/Msp430GpioC_1_GeneralIO_toggle();
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
inline static void BlinkC_Leds_led1Toggle(void ){
/* #line 72 */
  LedsP_Leds_led1Toggle();
/* #line 72 */
}
/* #line 72 */
/* # 62 "BlinkC.nc" */
static inline void BlinkC_Timer1_fired(void )
{
  ;
  BlinkC_Leds_led1Toggle();
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
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_HplGeneralIO_toggle(void ){
/* #line 44 */
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP_38_IO_toggle();
/* #line 44 */
}
/* #line 44 */
/* # 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc" */
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_toggle(void )
/* #line 39 */
{
/* #line 39 */
  /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_HplGeneralIO_toggle();
}

/* # 31 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc" */
inline static void LedsP_Led2_toggle(void ){
/* #line 31 */
  /*PlatformLedsC.Led2Impl*/Msp430GpioC_2_GeneralIO_toggle();
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
inline static void BlinkC_Leds_led2Toggle(void ){
/* #line 89 */
  LedsP_Leds_led2Toggle();
/* #line 89 */
}
/* #line 89 */
/* # 68 "BlinkC.nc" */
static inline void BlinkC_Timer2_fired(void )
{
  ;
  BlinkC_Leds_led2Toggle();
}

/* # 193 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_default_fired(uint8_t num)
{
}

/* # 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_fired(uint8_t arg_0x18a3ea0){
/* #line 72 */
  switch (arg_0x18a3ea0) {
/* #line 72 */
    case 0U:
/* #line 72 */
      BlinkC_Timer0_fired();
/* #line 72 */
      break;
/* #line 72 */
    case 1U:
/* #line 72 */
      BlinkC_Timer1_fired();
/* #line 72 */
      break;
/* #line 72 */
    case 2U:
/* #line 72 */
      BlinkC_Timer2_fired();
/* #line 72 */
      break;
/* #line 72 */
    default:
/* #line 72 */
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_default_fired(arg_0x18a3ea0);
/* #line 72 */
      break;
/* #line 72 */
    }
/* #line 72 */
}
/* #line 72 */
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
/* # 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc" */
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3_cc_t x)
/* #line 46 */
{
/* #line 46 */
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP_3___nesc_unnamed4286 {
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

/* # 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc" */
inline static error_t RealMainP_SoftwareInit_init(void ){
/* #line 51 */
  unsigned char result;
/* #line 51 */

/* #line 51 */
  result = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC_0_Init_init();
/* #line 51 */

/* #line 51 */
  return result;
/* #line 51 */
}
/* #line 51 */
/* # 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc" */
inline static void BlinkC_Timer2_startPeriodic(uint32_t dt){
/* #line 53 */
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startPeriodic(2U, dt);
/* #line 53 */
}
/* #line 53 */
inline static void BlinkC_Timer1_startPeriodic(uint32_t dt){
/* #line 53 */
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startPeriodic(1U, dt);
/* #line 53 */
}
/* #line 53 */
inline static void BlinkC_Timer0_startPeriodic(uint32_t dt){
/* #line 53 */
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startPeriodic(0U, dt);
/* #line 53 */
}
/* #line 53 */
/* # 49 "BlinkC.nc" */
static inline void BlinkC_Boot_booted(void )
{
  BlinkC_Timer0_startPeriodic(250);
  BlinkC_Timer1_startPeriodic(500);
  BlinkC_Timer2_startPeriodic(1000);
}

/* # 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc" */
inline static void RealMainP_Boot_booted(void ){
/* #line 49 */
  BlinkC_Boot_booted();
/* #line 49 */
}
/* #line 49 */
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
/* # 133 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_m_timers[num];

/* #line 136 */
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer_postTask();
}

/* # 164 "/Users/doina/tinyos-2.x/tos/system/SchedulerBasicP.nc" */
static inline void SchedulerBasicP_TaskBasic_default_runTask(uint8_t id)
{
}

/* # 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc" */
inline static void SchedulerBasicP_TaskBasic_runTask(uint8_t arg_0x147bb38){
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
  assert(0);
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
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_fired(uint8_t arg_0x1561800){
/* #line 28 */
  switch (arg_0x1561800) {
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
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP_1_Event_default_fired(arg_0x1561800);
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




/*  if (1) { */
      /* atomic removed: atomic calls only */
/* #line 58 */
/*      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )400U;

        do {
            t0 = t1;
            t1 = * (volatile uint16_t * )400U;
          }
        while (
        t0 != t1);
        {
          unsigned int __nesc_temp = 
          t1;

          return __nesc_temp;
        }
      }
    }
  else 
    {*/
      return * (volatile uint16_t * )400U;
/*    } */
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

  return -1; }

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

/* # 89 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_updateFromTimer_runTask(void )
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

/* #line 62 */
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

/* # 63 "/Users/doina/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc" */
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_fired_runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Alarm_getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_m_dt, FALSE);
    }
/* #line 67 */
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC_0_Timer_fired();
}

/* # 143 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc" */
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_Timer_startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC_0_TimerFrom_getNow(), dt, FALSE);
}

