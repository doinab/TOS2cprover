#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 151 "/opt/local/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 8 "/opt/local/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
}  ;
#line 14
struct __nesc_attr_one_nok {
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
}  ;
# 38 "/opt/local/msp430/include/sys/inttypes.h" 3
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
# 385 "/opt/local/lib/ncc/nesc_nx.h"
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 41 "/opt/local/msp430/include/sys/types.h" 3
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
# 42 "/opt/local/msp430/include/string.h" 3
extern void *memset(void *arg_0x701690, int arg_0x7017e8, size_t arg_0x701980);
#line 63
extern void *memset(void *arg_0x104a650, int arg_0x104a7a8, size_t arg_0x104a940);
# 59 "/opt/local/msp430/include/stdlib.h" 3
#line 56
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;







#line 64
typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;
# 122 "/opt/local/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/opt/local/msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 28 "/opt/local/msp430/include/sys/reent.h" 3
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

  void (*__cleanup)(struct _reent *arg_0x107aab8);


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


  void (**_sig_func)(int arg_0x107ec98);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/opt/local/msp430/include/math.h" 3
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
# 23 "/Users/doina/tinyos-2.x/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 24
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;







struct __nesc_attr_atmostonce {
};
#line 35
struct __nesc_attr_atleastonce {
};
#line 36
struct __nesc_attr_exactlyonce {
};
# 40 "/Users/doina/tinyos-2.x/tos/types/TinyError.h"
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
# 59 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/chips/msp430/chip_thread.h"
#line 45
typedef struct thread_regs {
  uint16_t status;
  uint16_t r4;
  uint16_t r5;
  uint16_t r6;
  uint16_t r7;
  uint16_t r8;
  uint16_t r9;
  uint16_t r10;
  uint16_t r11;
  uint16_t r12;
  uint16_t r13;
  uint16_t r14;
  uint16_t r15;
} thread_regs_t;

typedef uint16_t *stack_ptr_t;
# 42 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/types/linked_list.h"
#line 39
typedef struct list_element {
  struct list_element *next;
  uint8_t *element_data;
} list_element_t;




#line 44
typedef struct linked_list {
  list_element_t *head;
  volatile uint8_t size;
} linked_list_t;
# 43 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/types/thread_queue.h"
#line 41
typedef struct thread_queue {
  linked_list_t l;
} thread_queue_t;
# 45 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/types/refcounter.h"
#line 42
typedef struct refcounter {
  uint8_t count;
  thread_queue_t thread_queue;
} refcounter_t;
# 42 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/types/thread.h"
typedef uint8_t thread_id_t;
typedef uint8_t syscall_id_t;
typedef thread_id_t tosthread_t;










enum __nesc_unnamed4249 {



  TOSTHREAD_MAX_NUM_THREADS = 33, 

  TOSTHREAD_NUM_STATIC_THREADS = 4U, 
  TOSTHREAD_MAX_DYNAMIC_THREADS = TOSTHREAD_MAX_NUM_THREADS - TOSTHREAD_NUM_STATIC_THREADS, 
  TOSTHREAD_TOS_THREAD_ID = TOSTHREAD_MAX_NUM_THREADS, 
  TOSTHREAD_INVALID_THREAD_ID = TOSTHREAD_MAX_NUM_THREADS, 
  TOSTHREAD_PREEMPTION_PERIOD = 5
};

enum __nesc_unnamed4250 {
  INVALID_ID = 0xFF, 
  SYSCALL_WAIT_ON_EVENT = 0
};

typedef struct syscall syscall_t;
typedef struct thread thread_t;
typedef struct init_block init_block_t;



struct init_block {
  void *globals;
  void (*init_ptr)(void *arg_0x1106990);
  void *init_arg;
  refcounter_t thread_counter;
};


struct syscall {

  struct syscall *next_call;
  syscall_id_t id;
  thread_t *thread;
  void (*syscall_ptr)(struct syscall *arg_0x1105b00);
  void *params;
};


struct thread {

  volatile struct thread *next_thread;
  thread_id_t id;
  init_block_t *init_block;
  stack_ptr_t stack_ptr;
  volatile uint8_t state;
  volatile uint8_t mutex_count;
  uint8_t joinedOnMe[(TOSTHREAD_MAX_NUM_THREADS - 1) / 8 + 1];
  void (*start_ptr)(void *arg_0x110b6b0);
  void *start_arg_ptr;
  syscall_t *syscall;
  thread_regs_t regs;
};

enum __nesc_unnamed4251 {
  TOSTHREAD_STATE_INACTIVE = 0, 
  TOSTHREAD_STATE_ACTIVE = 1, 
  TOSTHREAD_STATE_READY = 2, 
  TOSTHREAD_STATE_SUSPENDED = 3
};
# 39 "/opt/local/msp430/include/msp430/iostructures.h" 3
#line 27
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4252 {
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
#line 108
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
# 116 "/opt/local/msp430/include/msp430/gpio.h" 3
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
# 94 "/opt/local/msp430/include/msp430/usart.h" 3
volatile unsigned char U0TCTL __asm ("0x0071");
#line 277
volatile unsigned char U1TCTL __asm ("0x0079");
# 27 "/opt/local/msp430/include/msp430/timera.h" 3
volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");


volatile unsigned int TA0CCTL0 __asm ("0x0162");

volatile unsigned int TA0CCTL1 __asm ("0x0164");
#line 70
volatile unsigned int TA0CCTL2 __asm ("0x0166");
#line 127
#line 118
typedef struct __nesc_unnamed4253 {
  volatile unsigned 
  taifg : 1, 
  taie : 1, 
  taclr : 1, 
  dummy : 1, 
  tamc : 2, 
  taid : 2, 
  tassel : 2;
} __attribute((packed))  tactl_t;
#line 143
#line 129
typedef struct __nesc_unnamed4254 {
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
# 26 "/opt/local/msp430/include/msp430/timerb.h" 3
volatile unsigned int TBR __asm ("0x0190");


volatile unsigned int TBCCTL0 __asm ("0x0182");





volatile unsigned int TBCCR0 __asm ("0x0192");
#line 76
#line 64
typedef struct __nesc_unnamed4255 {
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
#line 91
#line 78
typedef struct __nesc_unnamed4256 {
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
# 20 "/opt/local/msp430/include/msp430/basic_clock.h" 3
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 18 "/opt/local/msp430/include/msp430/adc12.h" 3
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");
#line 42
#line 30
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
#line 54
#line 44
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
#line 74
#line 56
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


struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};




struct adc12_t;
# 83 "/opt/local/msp430/include/msp430x16x.h" 3
volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
# 158 "/Users/doina/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");
#line 193
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4260 {
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
#line 248
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
# 33 "/Users/doina/tinyos-2.x/tos/platforms/telosb/hardware.h"
static inline void TOSH_SET_SIMO0_PIN()  ;
#line 33
static inline void TOSH_CLR_SIMO0_PIN()  ;
#line 33
static inline void TOSH_MAKE_SIMO0_OUTPUT()  ;
static inline void TOSH_SET_UCLK0_PIN()  ;
#line 34
static inline void TOSH_CLR_UCLK0_PIN()  ;
#line 34
static inline void TOSH_MAKE_UCLK0_OUTPUT()  ;
#line 76
enum __nesc_unnamed4261 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN()  ;
#line 85
static inline void TOSH_CLR_FLASH_CS_PIN()  ;
#line 85
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()  ;
static inline void TOSH_SET_FLASH_HOLD_PIN()  ;
#line 86
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()  ;
# 32 "/Users/doina/tinyos-2.x/tos/types/Leds.h"
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
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.h"
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
#line 64
#line 51
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
#line 76
#line 66
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
#line 91
#line 78
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
# 29 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4267 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4268 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4269 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
typedef TMilli TinyThreadSchedulerP$PreemptionAlarm$precision_tag;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC$0$__nesc_unnamed4270 {
  Msp430Timer32khzC$0$ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC$0$to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC$0$from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$LocalTime$precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$size_type;
typedef TMilli /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$precision_tag;
typedef /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$precision_tag /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$precision_tag;
typedef /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$precision_tag /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$Timer$precision_tag;
typedef TMilli ThreadSleepP$TimerMilli$precision_tag;
enum /*BlinkAppC.NullThread*/ThreadC$0$__nesc_unnamed4271 {
  ThreadC$0$THREAD_ID = 0U
};
enum /*BlinkAppC.TinyThread0*/ThreadC$1$__nesc_unnamed4272 {
  ThreadC$1$THREAD_ID = 1U
};
enum /*BlinkAppC.TinyThread1*/ThreadC$2$__nesc_unnamed4273 {
  ThreadC$2$THREAD_ID = 2U
};
enum /*BlinkAppC.TinyThread2*/ThreadC$3$__nesc_unnamed4274 {
  ThreadC$3$THREAD_ID = 3U
};
# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP$Init$init(void );
#line 51
static error_t MotePlatformC$Init$init(void );
# 35 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void );
#line 32
static void Msp430ClockP$Msp430ClockInit$default$initTimerB(void );



static void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void );
#line 31
static void Msp430ClockP$Msp430ClockInit$default$initTimerA(void );





static void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void );
#line 34
static void Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate(void );
#line 29
static void Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate(void );
static void Msp430ClockP$Msp430ClockInit$default$initClocks(void );
# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
static error_t Msp430ClockP$Init$init(void );
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(
# 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x16e4eb0);
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(
# 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x16e4eb0);
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void );
# 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t time);
# 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void );
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void );
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$default$fired(void );
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void );
# 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t time);
# 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void );
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void );
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$default$fired(void );
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void );
# 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t time);
# 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void );
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void );
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void );
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void );
# 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t time);
# 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void );
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void );
# 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t delta);
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void );
# 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(uint16_t time);
# 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void );
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void );
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void );
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void );
# 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t time);
# 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void );
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void );
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$default$fired(void );
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void );
# 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t time);
# 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void );
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void );
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void );
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void );
# 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t time);
# 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void );
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void );
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void );
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void );
# 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t time);
# 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void );
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void );
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void );
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void );
# 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t time);
# 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void );
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void );
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void );
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void );
# 47 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
static error_t TinyThreadSchedulerP$ThreadScheduler$suspendCurrentThread(void );
#line 39
static uint8_t TinyThreadSchedulerP$ThreadScheduler$currentThreadId(void );
static thread_t *TinyThreadSchedulerP$ThreadScheduler$currentThreadInfo(void );



static error_t TinyThreadSchedulerP$ThreadScheduler$startThread(thread_id_t id);
#line 41
static thread_t *TinyThreadSchedulerP$ThreadScheduler$threadInfo(thread_id_t id);

static error_t TinyThreadSchedulerP$ThreadScheduler$initThread(thread_id_t id);






static error_t TinyThreadSchedulerP$ThreadScheduler$wakeupThread(thread_id_t id);
#line 48
static error_t TinyThreadSchedulerP$ThreadScheduler$interruptCurrentThread(void );
# 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
static void TinyThreadSchedulerP$PreemptionAlarm$fired(void );
# 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void TinyThreadSchedulerP$alarmTask$runTask(void );
# 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc"
static void TinyThreadSchedulerP$ThreadSchedulerBoot$booted(void );
# 54 "/Users/doina/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void );
# 59 "/Users/doina/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void McuSleepC$McuSleep$sleep(void );
# 29 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/BitArrayUtils.nc"
static void BitArrayUtilsC$BitArrayUtils$clrArray(uint8_t *array, uint8_t numBytes);
# 54 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/LinkedList.nc"
static list_element_t *LinkedListC$LinkedList$removeAt(linked_list_t *l, uint8_t i);
#line 39
static void LinkedListC$LinkedList$init(linked_list_t *l);



static error_t LinkedListC$LinkedList$addFirst(linked_list_t *l, list_element_t *e);
#line 56
static list_element_t *LinkedListC$LinkedList$removeLast(linked_list_t *l);
#line 55
static list_element_t *LinkedListC$LinkedList$removeFirst(linked_list_t *l);
#line 41
static uint8_t LinkedListC$LinkedList$size(linked_list_t *l);
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
static void ThreadQueueP$ThreadQueue$enqueue(thread_queue_t *q, thread_t *t);


static bool ThreadQueueP$ThreadQueue$isEmpty(thread_queue_t *q);
#line 39
static void ThreadQueueP$ThreadQueue$init(thread_queue_t *q);

static thread_t *ThreadQueueP$ThreadQueue$dequeue(thread_queue_t *q);
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void );
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void );
# 92 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void );
# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void );
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void );
# 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void );
#line 53
static /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void );
# 98 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type dt);
#line 105
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void );
# 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void );
# 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void );
# 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void );
# 125 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void );
#line 118
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void );
# 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void );
# 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void );
#line 125
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$getNow(
# 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x19c83c8);
# 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(
# 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x19c83c8);
# 118 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShotAt(
# 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x19c83c8, 
# 118 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t t0, uint32_t dt);
#line 62
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(
# 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x19c83c8, 
# 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(
# 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x19c83c8);
# 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void );
# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
static error_t LedsP$Init$init(void );
# 56 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc"
static void LedsP$Leds$led0Toggle(void );
#line 72
static void LedsP$Leds$led1Toggle(void );
#line 89
static void LedsP$Leds$led2Toggle(void );
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void );









static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void );









static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void );
# 31 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle(void );



static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void );
#line 29
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void );

static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$toggle(void );



static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void );
#line 29
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void );

static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$toggle(void );



static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void );
#line 29
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void );
# 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP$TaskBasic$postTask(
# 47 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/SchedulerBasicP.nc"
uint8_t arg_0x14a22b0);
# 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$default$runTask(
# 47 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/SchedulerBasicP.nc"
uint8_t arg_0x14a22b0);
# 62 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/TaskScheduler.nc"
static bool SchedulerBasicP$TaskScheduler$hasTasks(void );
#line 47
static void SchedulerBasicP$TaskScheduler$init(void );
#line 69
static void SchedulerBasicP$TaskScheduler$taskLoop(void );
#line 55
static bool SchedulerBasicP$TaskScheduler$runNextTask(void );
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/PlatformInterrupt.nc"
static void TOSThreadsInterruptP$PlatformInterrupt$postAmble(void );
# 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc"
static void TinyOSMainP$TinyOSBoot$booted(void );
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *TinyOSMainP$ThreadInfo$get(void );
#line 39
static error_t TinyOSMainP$ThreadInfo$reset(void );
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadNotification.nc"
static void StaticThreadP$ThreadNotification$default$justCreated(
# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x1a486a0);
# 38 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadNotification.nc"
static void StaticThreadP$ThreadNotification$default$aboutToDestroy(
# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x1a486a0);
# 42 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/Thread.nc"
static void StaticThreadP$Thread$default$run(
# 38 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x1a4ca98, 
# 42 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/Thread.nc"
void *arg);
#line 37
static error_t StaticThreadP$Thread$start(
# 38 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x1a4ca98, 
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/Thread.nc"
void *arg);



static error_t StaticThreadP$Thread$sleep(
# 38 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x1a4ca98, 
# 41 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/Thread.nc"
uint32_t milli);
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void StaticThreadP$ThreadCleanup$cleanup(
# 46 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x1a713c8);
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadFunction.nc"
static void StaticThreadP$ThreadFunction$signalThreadRun(
# 45 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x1a45d18, 
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadFunction.nc"
void *arg);
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *StaticThreadP$ThreadInfo$default$get(
# 44 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x1a455b0);
# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static error_t StaticThreadP$ThreadInfo$default$reset(
# 44 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x1a455b0);
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void ThreadMapP$DynamicThreadCleanup$default$cleanup(
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x1a78ed0);
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *ThreadMapP$StaticThreadInfo$default$get(
# 43 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x1a77688);
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void ThreadMapP$ThreadCleanup$cleanup(
# 45 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x1a75680);
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void ThreadMapP$StaticThreadCleanup$default$cleanup(
# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x1a78760);
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *ThreadMapP$DynamicThreadInfo$default$get(
# 44 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x1a77e98);
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *ThreadMapP$ThreadInfo$get(
# 38 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x1a7ce48);
# 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$updateFromTimer$runTask(void );
# 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$fired(void );
#line 62
static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$Timer$startOneShot(
# 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x19c83c8, 
# 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t dt);









static void ThreadSleepP$TimerMilli$fired(
# 43 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadSleepP.nc"
uint8_t arg_0x1aa1770);
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadSleep.nc"
static error_t ThreadSleepP$ThreadSleep$sleep(uint32_t milli);
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/SystemCall.nc"
static error_t SystemCallP$SystemCall$finish(syscall_t *s);
#line 39
static error_t SystemCallP$SystemCall$start(void *syscall_ptr, syscall_t *s, syscall_id_t id, void *params);
# 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SystemCallP$threadTask$runTask(void );
# 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc"
static void BlinkC$Boot$booted(void );
# 42 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/Thread.nc"
static void BlinkC$TinyThread0$run(void *arg);
#line 42
static void BlinkC$TinyThread1$run(void *arg);
#line 42
static void BlinkC$TinyThread2$run(void *arg);
#line 42
static void BlinkC$NullThread$run(void *arg);
# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$Init$init(void );
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t */*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$ThreadInfo$get(void );
#line 39
static error_t /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$ThreadInfo$reset(void );
# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$Init$init(void );
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t */*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$ThreadInfo$get(void );
#line 39
static error_t /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$ThreadInfo$reset(void );
# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$Init$init(void );
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t */*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$ThreadInfo$get(void );
#line 39
static error_t /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$ThreadInfo$reset(void );
# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$Init$init(void );
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t */*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$ThreadInfo$get(void );
#line 39
static error_t /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$ThreadInfo$reset(void );
# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP$MoteInit$init(void );
#line 51
static error_t PlatformP$MoteClockInit$init(void );
#line 51
static error_t PlatformP$LedsInit$init(void );
# 10 "/Users/doina/tinyos-2.x/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP$Init$init(void );
# 6 "/Users/doina/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC$uwait(uint16_t u);




static __inline void MotePlatformC$TOSH_wait(void );




static void MotePlatformC$TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC$TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC$Init$init(void );
# 32 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP$Msp430ClockInit$initTimerB(void );
#line 31
static void Msp430ClockP$Msp430ClockInit$initTimerA(void );
#line 29
static void Msp430ClockP$Msp430ClockInit$setupDcoCalibrate(void );
static void Msp430ClockP$Msp430ClockInit$initClocks(void );
# 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP$IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP$TA0CTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP$TA0IV __asm ("0x012E");
static volatile uint16_t Msp430ClockP$TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP$TBIV __asm ("0x011E");

enum Msp430ClockP$__nesc_unnamed4275 {

  Msp430ClockP$ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP$TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP$ACLK_CALIB_PERIOD
};


static inline void Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate(void );
#line 64
static inline void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void );
#line 85
static inline void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void );
#line 100
static inline void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void );
#line 115
static inline void Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate(void );




static inline void Msp430ClockP$Msp430ClockInit$default$initClocks(void );




static inline void Msp430ClockP$Msp430ClockInit$default$initTimerA(void );




static inline void Msp430ClockP$Msp430ClockInit$default$initTimerB(void );





static inline void Msp430ClockP$startTimerA(void );
#line 148
static inline void Msp430ClockP$startTimerB(void );
#line 160
static void Msp430ClockP$set_dco_calib(int calib);





static inline uint16_t Msp430ClockP$test_calib_busywait_delta(int calib);
#line 189
static inline void Msp430ClockP$busyCalibrateDco(void );
#line 214
static inline error_t Msp430ClockP$Init$init(void );
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(
# 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x16e4eb0);
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow(void );
# 115 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(uint8_t n);
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(
# 40 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x16e4eb0);
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow(void );
# 51 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void );
#line 70
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void );
#line 115
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(uint8_t n);
# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$captured(uint16_t time);
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$fired(void );
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t;


static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void );
# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$captured(uint16_t time);
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$fired(void );
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void );
# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(uint16_t time);
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired(void );
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void );
# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(uint16_t time);
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired(void );
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get(void );
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl(void );
#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void );
# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(uint16_t time);
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired(void );
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t;


static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void );
# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(uint16_t time);
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired(void );
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void );
# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(uint16_t time);
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired(void );
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void );
# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(uint16_t time);
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired(void );
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void );
# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(uint16_t time);
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired(void );
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void );
# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(uint16_t time);
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired(void );
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void );
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP$VectorTimerB1$fired(void );
#line 28
static void Msp430TimerCommonP$VectorTimerA0$fired(void );
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/PlatformInterrupt.nc"
static void Msp430TimerCommonP$PlatformInterrupt$postAmble(void );
# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP$VectorTimerA1$fired(void );
#line 28
static void Msp430TimerCommonP$VectorTimerB0$fired(void );
# 12 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/chips/msp430/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(12)))  ;



void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(10)))  ;



void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(26)))  ;



void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(24)))  ;
# 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
static void TinyThreadSchedulerP$PreemptionAlarm$startOneShot(uint32_t dt);




static void TinyThreadSchedulerP$PreemptionAlarm$stop(void );
# 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc"
static void TinyThreadSchedulerP$TinyOSBoot$booted(void );
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
static void TinyThreadSchedulerP$ThreadQueue$enqueue(thread_queue_t *q, thread_t *t);


static bool TinyThreadSchedulerP$ThreadQueue$isEmpty(thread_queue_t *q);
#line 39
static void TinyThreadSchedulerP$ThreadQueue$init(thread_queue_t *q);

static thread_t *TinyThreadSchedulerP$ThreadQueue$dequeue(thread_queue_t *q);
# 29 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/BitArrayUtils.nc"
static void TinyThreadSchedulerP$BitArrayUtils$clrArray(uint8_t *array, uint8_t numBytes);
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void TinyThreadSchedulerP$ThreadCleanup$cleanup(
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
uint8_t arg_0x17b0878);
# 59 "/Users/doina/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void TinyThreadSchedulerP$McuSleep$sleep(void );
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *TinyThreadSchedulerP$ThreadInfo$get(
# 44 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
uint8_t arg_0x17af3a8);
#line 64
enum TinyThreadSchedulerP$__nesc_unnamed4276 {
#line 64
  TinyThreadSchedulerP$alarmTask = 0U
};
#line 64
typedef int TinyThreadSchedulerP$__nesc_sillytask_alarmTask[TinyThreadSchedulerP$alarmTask];
#line 54
thread_t *TinyThreadSchedulerP$current_thread;

thread_t *TinyThreadSchedulerP$tos_thread;

thread_t *TinyThreadSchedulerP$yielding_thread;

uint8_t TinyThreadSchedulerP$num_runnable_threads;

thread_queue_t TinyThreadSchedulerP$ready_queue;

static inline void TinyThreadSchedulerP$alarmTask$runTask(void );
#line 84
static void TinyThreadSchedulerP$switchThreads(void ) __attribute((noinline)) ;


static void TinyThreadSchedulerP$restoreThread(void ) __attribute((noinline)) ;









static void TinyThreadSchedulerP$sleepWhileIdle(void );
#line 111
static void TinyThreadSchedulerP$scheduleNextThread(void );
#line 124
static void TinyThreadSchedulerP$interrupt(thread_t *thread);
#line 138
static inline void TinyThreadSchedulerP$suspend(thread_t *thread);










static inline void TinyThreadSchedulerP$wakeupJoined(thread_t *t);
#line 171
static inline void TinyThreadSchedulerP$stop(thread_t *t);
#line 186
static void TinyThreadSchedulerP$threadWrapper(void ) __attribute((noinline)) __attribute((naked)) ;
#line 201
static inline void TinyThreadSchedulerP$ThreadSchedulerBoot$booted(void );
#line 213
static inline error_t TinyThreadSchedulerP$ThreadScheduler$initThread(uint8_t id);








static inline error_t TinyThreadSchedulerP$ThreadScheduler$startThread(uint8_t id);
#line 253
static error_t TinyThreadSchedulerP$ThreadScheduler$suspendCurrentThread(void );










static error_t TinyThreadSchedulerP$ThreadScheduler$interruptCurrentThread(void );
#line 291
static error_t TinyThreadSchedulerP$ThreadScheduler$wakeupThread(uint8_t id);
#line 307
static inline uint8_t TinyThreadSchedulerP$ThreadScheduler$currentThreadId(void );



static inline thread_t *TinyThreadSchedulerP$ThreadScheduler$threadInfo(uint8_t id);



static inline thread_t *TinyThreadSchedulerP$ThreadScheduler$currentThreadInfo(void );



static inline void TinyThreadSchedulerP$PreemptionAlarm$fired(void );
# 54 "/Users/doina/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC$McuPowerOverride$lowestState(void );
# 51 "/Users/doina/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
bool McuSleepC$dirty = TRUE;
mcu_power_t McuSleepC$powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC$msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC$getPowerState(void );
#line 104
static inline void McuSleepC$computePowerState(void );




static inline void McuSleepC$McuSleep$sleep(void );
#line 126
static inline mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void );
# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/BitArrayUtilsC.nc"
static inline void BitArrayUtilsC$BitArrayUtils$clrArray(uint8_t *array, uint8_t size);
# 42 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/LinkedListC.nc"
static inline list_element_t *LinkedListC$get_elementAt(linked_list_t *l, uint8_t i);
#line 84
static error_t LinkedListC$insert_element(linked_list_t *l, list_element_t **previous_next, list_element_t *e);







static list_element_t *LinkedListC$remove_element(linked_list_t *l, list_element_t **previous_next);







static inline void LinkedListC$LinkedList$init(linked_list_t *l);










static inline uint8_t LinkedListC$LinkedList$size(linked_list_t *l);


static inline error_t LinkedListC$LinkedList$addFirst(linked_list_t *l, list_element_t *e);






static inline list_element_t *LinkedListC$LinkedList$removeFirst(linked_list_t *l);









static inline list_element_t *LinkedListC$LinkedList$removeLast(linked_list_t *l);
#line 148
static list_element_t *LinkedListC$LinkedList$removeAt(linked_list_t *l, uint8_t i);
# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/LinkedList.nc"
static void ThreadQueueP$LinkedList$init(linked_list_t *l);



static error_t ThreadQueueP$LinkedList$addFirst(linked_list_t *l, list_element_t *e);
#line 56
static list_element_t *ThreadQueueP$LinkedList$removeLast(linked_list_t *l);
#line 41
static uint8_t ThreadQueueP$LinkedList$size(linked_list_t *l);
# 45 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadQueueP.nc"
static inline void ThreadQueueP$ThreadQueue$init(thread_queue_t *q);


static inline void ThreadQueueP$ThreadQueue$enqueue(thread_queue_t *q, thread_t *t);


static inline thread_t *ThreadQueueP$ThreadQueue$dequeue(thread_queue_t *q);





static inline bool ThreadQueueP$ThreadQueue$isEmpty(thread_queue_t *q);
# 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(uint16_t delta);
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$get(void );
# 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired(void );
# 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$enableEvents(void );
#line 36
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents(void );
#line 33
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$clearPendingInterrupt(void );
# 42 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void );
#line 54
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void );
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending(void );
# 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow(void );
# 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void );
# 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow(void );
# 56 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type /*CounterMilli32C.Transform*/TransformCounterC$0$m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC$0$__nesc_unnamed4277 {

  TransformCounterC$0$LOW_SHIFT_RIGHT = 5, 
  TransformCounterC$0$HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC$0$LOW_SHIFT_RIGHT, 
  TransformCounterC$0$NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type ) + 5, 



  TransformCounterC$0$OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC$0$NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC$0$NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void );
#line 122
static inline void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void );
# 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$fired(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop(void );
# 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get(void );
# 66 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$__nesc_unnamed4278 {

  TransformAlarmC$0$MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type ) - 1 - 5, 
  TransformAlarmC$0$MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm(void );
#line 136
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type dt);
#line 151
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void );
#line 166
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void );
# 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask(void );
# 98 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type dt);
#line 105
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop(void );
# 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired(void );
# 63 "/Users/doina/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$__nesc_unnamed4279 {
#line 63
  AlarmToTimerC$0$fired = 1U
};
#line 63
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$__nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired];
#line 44
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot);
#line 60
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void );
#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void );
# 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask(void );
# 125 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(void );
#line 118
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(
# 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x19c83c8);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4280 {
#line 60
  VirtualizeTimerC$0$updateFromTimer = 2U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4281 {

  VirtualizeTimerC$0$NUM_TIMERS = 2U, 
  VirtualizeTimerC$0$END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4282 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(uint32_t now);
#line 89
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void );
#line 128
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);
#line 148
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(uint8_t num);
#line 173
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShotAt(uint8_t num, uint32_t t0, uint32_t dt);




static inline uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$getNow(uint8_t num);
#line 193
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num);
# 47 "/Users/doina/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void );
# 31 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void LedsP$Led0$toggle(void );



static void LedsP$Led0$makeOutput(void );
#line 29
static void LedsP$Led0$set(void );

static void LedsP$Led1$toggle(void );



static void LedsP$Led1$makeOutput(void );
#line 29
static void LedsP$Led1$set(void );

static void LedsP$Led2$toggle(void );



static void LedsP$Led2$makeOutput(void );
#line 29
static void LedsP$Led2$set(void );
# 45 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc"
static inline error_t LedsP$Init$init(void );
#line 73
static inline void LedsP$Leds$led0Toggle(void );
#line 88
static inline void LedsP$Leds$led1Toggle(void );
#line 103
static inline void LedsP$Leds$led2Toggle(void );
# 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void );

static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle(void );




static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void );

static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$toggle(void );




static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void );

static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$toggle(void );




static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void );
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$toggle(void );
#line 71
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput(void );
#line 34
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set(void );
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void );

static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle(void );



static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void );
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$toggle(void );
#line 71
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput(void );
#line 34
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set(void );
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void );

static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$toggle(void );



static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void );
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$toggle(void );
#line 71
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput(void );
#line 34
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set(void );
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void );

static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$toggle(void );



static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void );
# 47 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
static error_t SchedulerBasicP$ThreadScheduler$suspendCurrentThread(void );
# 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$runTask(
# 47 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/SchedulerBasicP.nc"
uint8_t arg_0x14a22b0);




enum SchedulerBasicP$__nesc_unnamed4283 {
  SchedulerBasicP$NUM_TASKS = 5U, 
  SchedulerBasicP$NO_TASK = 255
};

volatile uint8_t SchedulerBasicP$m_head;
volatile uint8_t SchedulerBasicP$m_tail;
volatile uint8_t SchedulerBasicP$m_next[SchedulerBasicP$NUM_TASKS];








static __inline uint8_t SchedulerBasicP$popTask(void );
#line 83
static inline bool SchedulerBasicP$isWaiting(uint8_t id);



static inline bool SchedulerBasicP$TaskScheduler$hasTasks(void );



static inline bool SchedulerBasicP$pushTask(uint8_t id);
#line 108
static inline void SchedulerBasicP$TaskScheduler$init(void );







static bool SchedulerBasicP$TaskScheduler$runNextTask(void );
#line 128
static inline void SchedulerBasicP$TaskScheduler$taskLoop(void );
#line 145
static error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id);



static void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id);
# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
static uint8_t TOSThreadsInterruptP$ThreadScheduler$currentThreadId(void );










static error_t TOSThreadsInterruptP$ThreadScheduler$wakeupThread(thread_id_t id);
#line 48
static error_t TOSThreadsInterruptP$ThreadScheduler$interruptCurrentThread(void );
# 62 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/TaskScheduler.nc"
static bool TOSThreadsInterruptP$TaskScheduler$hasTasks(void );
# 46 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TOSThreadsInterruptP.nc"
static void TOSThreadsInterruptP$interruptThread(void ) __attribute((noinline)) ;





static __inline void TOSThreadsInterruptP$PlatformInterrupt$postAmble(void );
# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
static error_t TinyOSMainP$SoftwareInit$init(void );
# 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc"
static void TinyOSMainP$Boot$booted(void );
# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
static error_t TinyOSMainP$PlatformInit$init(void );
# 47 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/TaskScheduler.nc"
static void TinyOSMainP$TaskScheduler$init(void );
#line 69
static void TinyOSMainP$TaskScheduler$taskLoop(void );
#line 55
static bool TinyOSMainP$TaskScheduler$runNextTask(void );
# 65 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyOSMainP.nc"
thread_t TinyOSMainP$thread_info;

static inline void TinyOSMainP$TinyOSBoot$booted(void );
#line 103
static inline error_t TinyOSMainP$ThreadInfo$reset(void );



static inline thread_t *TinyOSMainP$ThreadInfo$get(void );
# 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc"
static void RealMainImplP$ThreadSchedulerBoot$booted(void );
# 49 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/RealMainImplP.nc"
int main(void )   ;
# 44 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
static error_t StaticThreadP$ThreadScheduler$startThread(thread_id_t id);
#line 43
static error_t StaticThreadP$ThreadScheduler$initThread(thread_id_t id);
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadNotification.nc"
static void StaticThreadP$ThreadNotification$justCreated(
# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x1a486a0);
# 38 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadNotification.nc"
static void StaticThreadP$ThreadNotification$aboutToDestroy(
# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x1a486a0);
# 42 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/Thread.nc"
static void StaticThreadP$Thread$run(
# 38 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x1a4ca98, 
# 42 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/Thread.nc"
void *arg);
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadSleep.nc"
static error_t StaticThreadP$ThreadSleep$sleep(uint32_t milli);
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *StaticThreadP$ThreadInfo$get(
# 44 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x1a455b0);
# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static error_t StaticThreadP$ThreadInfo$reset(
# 44 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
uint8_t arg_0x1a455b0);







static inline error_t StaticThreadP$init(uint8_t id, void *arg);










static error_t StaticThreadP$Thread$start(uint8_t id, void *arg);
#line 89
static inline error_t StaticThreadP$Thread$sleep(uint8_t id, uint32_t milli);







static inline void StaticThreadP$ThreadFunction$signalThreadRun(uint8_t id, void *arg);



static inline void StaticThreadP$ThreadCleanup$cleanup(uint8_t id);



static inline void StaticThreadP$Thread$default$run(uint8_t id, void *arg);
static inline thread_t *StaticThreadP$ThreadInfo$default$get(uint8_t id);
static inline error_t StaticThreadP$ThreadInfo$default$reset(uint8_t id);
static inline void StaticThreadP$ThreadNotification$default$justCreated(uint8_t id);
static inline void StaticThreadP$ThreadNotification$default$aboutToDestroy(uint8_t id);
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void ThreadMapP$DynamicThreadCleanup$cleanup(
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x1a78ed0);
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *ThreadMapP$StaticThreadInfo$get(
# 43 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x1a77688);
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void ThreadMapP$StaticThreadCleanup$cleanup(
# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x1a78760);
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *ThreadMapP$DynamicThreadInfo$get(
# 44 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
uint8_t arg_0x1a77e98);





static inline thread_t *ThreadMapP$ThreadInfo$get(uint8_t id);





static thread_t *ThreadMapP$StaticThreadInfo$default$get(uint8_t id);





static inline thread_t *ThreadMapP$DynamicThreadInfo$default$get(uint8_t id);





static inline void ThreadMapP$ThreadCleanup$cleanup(uint8_t id);


static void ThreadMapP$StaticThreadCleanup$default$cleanup(uint8_t id);


static inline void ThreadMapP$DynamicThreadCleanup$default$cleanup(uint8_t id);
# 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$updateFromTimer$postTask(void );
# 125 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
static uint32_t /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$getNow(void );
#line 118
static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$stop(void );




static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$Timer$fired(
# 37 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x19c83c8);
#line 60
enum /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$__nesc_unnamed4284 {
#line 60
  VirtualizeTimerC$1$updateFromTimer = 3U
};
#line 60
typedef int /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$__nesc_sillytask_updateFromTimer[/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$updateFromTimer];
#line 42
enum /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$__nesc_unnamed4285 {

  VirtualizeTimerC$1$NUM_TIMERS = 33, 
  VirtualizeTimerC$1$END_OF_LIST = 255
};








#line 48
typedef struct /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$__nesc_unnamed4286 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$Timer_t;

/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$Timer_t /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$m_timers[/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$NUM_TIMERS];




static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$fireTimers(uint32_t now);
#line 89
static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$updateFromTimer$runTask(void );
#line 128
static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$fired(void );




static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);
#line 148
static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$Timer$startOneShot(uint8_t num, uint32_t dt);
# 41 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
static thread_t *ThreadSleepP$ThreadScheduler$threadInfo(thread_id_t id);
# 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
static void ThreadSleepP$TimerMilli$startOneShot(
# 43 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadSleepP.nc"
uint8_t arg_0x1aa1770, 
# 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t dt);
# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/SystemCall.nc"
static error_t ThreadSleepP$SystemCall$finish(syscall_t *s);
#line 39
static error_t ThreadSleepP$SystemCall$start(void *syscall_ptr, syscall_t *s, syscall_id_t id, void *params);
# 50 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadSleepP.nc"
#line 48
typedef struct ThreadSleepP$sleep_params {
  uint32_t *milli;
} ThreadSleepP$sleep_params_t;

static void ThreadSleepP$sleepTask(syscall_t *s);




static error_t ThreadSleepP$ThreadSleep$sleep(uint32_t milli);
#line 76
static inline void ThreadSleepP$TimerMilli$fired(uint8_t id);
# 47 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
static error_t SystemCallP$ThreadScheduler$suspendCurrentThread(void );
#line 40
static thread_t *SystemCallP$ThreadScheduler$currentThreadInfo(void );









static error_t SystemCallP$ThreadScheduler$wakeupThread(thread_id_t id);
# 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SystemCallP$threadTask$postTask(void );
# 48 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/SystemCallP.nc"
enum SystemCallP$__nesc_unnamed4287 {
#line 48
  SystemCallP$threadTask = 4U
};
#line 48
typedef int SystemCallP$__nesc_sillytask_threadTask[SystemCallP$threadTask];
#line 46
syscall_t *SystemCallP$current_call = (void *)0;

static inline void SystemCallP$threadTask$runTask(void );
#line 63
static inline error_t SystemCallP$SystemCall$start(void *syscall_ptr, syscall_t *s, syscall_id_t id, void *p);
#line 82
static inline error_t SystemCallP$SystemCall$finish(syscall_t *s);
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/Thread.nc"
static error_t BlinkC$TinyThread0$start(void *arg);



static error_t BlinkC$TinyThread0$sleep(uint32_t milli);
#line 37
static error_t BlinkC$TinyThread1$start(void *arg);



static error_t BlinkC$TinyThread1$sleep(uint32_t milli);
#line 37
static error_t BlinkC$TinyThread2$start(void *arg);



static error_t BlinkC$TinyThread2$sleep(uint32_t milli);
# 56 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc"
static void BlinkC$Leds$led0Toggle(void );
#line 72
static void BlinkC$Leds$led1Toggle(void );
#line 89
static void BlinkC$Leds$led2Toggle(void );
# 48 "BlinkC.nc"
static inline void BlinkC$Boot$booted(void );






static inline void BlinkC$NullThread$run(void *arg);



static inline void BlinkC$TinyThread0$run(void *arg);





static inline void BlinkC$TinyThread1$run(void *arg);





static inline void BlinkC$TinyThread2$run(void *arg);
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadFunction.nc"
static void /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$ThreadFunction$signalThreadRun(void *arg);
# 47 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadInfoP.nc"
uint8_t /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$stack[100];
thread_t /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$thread_info;

static void /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$run_thread(void *arg) __attribute((noinline)) ;



static error_t /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$init(void );
#line 67
static inline error_t /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$Init$init(void );



static inline error_t /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$ThreadInfo$reset(void );



static inline thread_t */*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$ThreadInfo$get(void );
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadFunction.nc"
static void /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$ThreadFunction$signalThreadRun(void *arg);
# 47 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadInfoP.nc"
uint8_t /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$stack[100];
thread_t /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$thread_info;

static void /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$run_thread(void *arg) __attribute((noinline)) ;



static error_t /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$init(void );
#line 67
static inline error_t /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$Init$init(void );



static inline error_t /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$ThreadInfo$reset(void );



static inline thread_t */*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$ThreadInfo$get(void );
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadFunction.nc"
static void /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$ThreadFunction$signalThreadRun(void *arg);
# 47 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadInfoP.nc"
uint8_t /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$stack[100];
thread_t /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$thread_info;

static void /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$run_thread(void *arg) __attribute((noinline)) ;



static error_t /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$init(void );
#line 67
static inline error_t /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$Init$init(void );



static inline error_t /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$ThreadInfo$reset(void );



static inline thread_t */*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$ThreadInfo$get(void );
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadFunction.nc"
static void /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$ThreadFunction$signalThreadRun(void *arg);
# 47 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadInfoP.nc"
uint8_t /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$stack[100];
thread_t /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$thread_info;

static void /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$run_thread(void *arg) __attribute((noinline)) ;



static error_t /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$init(void );
#line 67
static inline error_t /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$Init$init(void );



static inline error_t /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$ThreadInfo$reset(void );



static inline thread_t */*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$ThreadInfo$get(void );
# 185 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void )
{
}

# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow(void ){
#line 37
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow();
#line 37
}
#line 37
# 126 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(uint8_t n)
{
}

# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(uint8_t arg_0x16e4eb0){
#line 28
  switch (arg_0x16e4eb0) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(arg_0x16e4eb0);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(0);
}

# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP$VectorTimerA0$fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired();
#line 28
}
#line 28
# 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$__nesc_unnamed4288 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(* (volatile uint16_t * )354U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t n)
{
}

# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(time);
#line 75
}
#line 75
# 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$default$fired(void )
{
}

# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$default$fired();
#line 34
}
#line 34
# 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$__nesc_unnamed4289 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(* (volatile uint16_t * )356U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t n)
{
}

# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(time);
#line 75
}
#line 75
# 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$default$fired(void )
{
}

# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$default$fired();
#line 34
}
#line 34
# 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$__nesc_unnamed4290 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(* (volatile uint16_t * )358U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t n)
{
}

# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(time);
#line 75
}
#line 75
# 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void )
{
}

# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired();
#line 34
}
#line 34
# 87 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP$TaskScheduler$hasTasks(void )
#line 87
{
  /* atomic removed: atomic calls only */
#line 88
  {
    unsigned char __nesc_temp = 
#line 88
    SchedulerBasicP$m_head != SchedulerBasicP$NO_TASK;

#line 88
    return __nesc_temp;
  }
}

# 62 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/TaskScheduler.nc"
inline static bool TOSThreadsInterruptP$TaskScheduler$hasTasks(void ){
#line 62
  unsigned char result;
#line 62

#line 62
  result = SchedulerBasicP$TaskScheduler$hasTasks();
#line 62

#line 62
  return result;
#line 62
}
#line 62
# 52 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TOSThreadsInterruptP.nc"
static __inline void TOSThreadsInterruptP$PlatformInterrupt$postAmble(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 53
  {
    if (TOSThreadsInterruptP$TaskScheduler$hasTasks() == TRUE) {
      TOSThreadsInterruptP$interruptThread();
      }
  }
}

# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/PlatformInterrupt.nc"
inline static void Msp430TimerCommonP$PlatformInterrupt$postAmble(void ){
#line 37
  TOSThreadsInterruptP$PlatformInterrupt$postAmble();
#line 37
}
#line 37
# 50 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static error_t TOSThreadsInterruptP$ThreadScheduler$wakeupThread(thread_id_t id){
#line 50
  unsigned char result;
#line 50

#line 50
  result = TinyThreadSchedulerP$ThreadScheduler$wakeupThread(id);
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 62 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
static inline thread_t *ThreadMapP$DynamicThreadInfo$default$get(uint8_t id)
#line 62
{
  return ThreadMapP$StaticThreadInfo$get(id);
}

# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
inline static thread_t *ThreadMapP$DynamicThreadInfo$get(uint8_t arg_0x1a77e98){
#line 40
  struct thread *result;
#line 40

#line 40
    result = ThreadMapP$DynamicThreadInfo$default$get(arg_0x1a77e98);
#line 40

#line 40
  return result;
#line 40
}
#line 40
# 307 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline uint8_t TinyThreadSchedulerP$ThreadScheduler$currentThreadId(void )
#line 307
{
  /* atomic removed: atomic calls only */
#line 308
  {
    unsigned char __nesc_temp = 
#line 308
    TinyThreadSchedulerP$current_thread->id;

#line 308
    return __nesc_temp;
  }
}

# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static uint8_t TOSThreadsInterruptP$ThreadScheduler$currentThreadId(void ){
#line 39
  unsigned char result;
#line 39

#line 39
  result = TinyThreadSchedulerP$ThreadScheduler$currentThreadId();
#line 39

#line 39
  return result;
#line 39
}
#line 39









inline static error_t TOSThreadsInterruptP$ThreadScheduler$interruptCurrentThread(void ){
#line 48
  unsigned char result;
#line 48

#line 48
  result = TinyThreadSchedulerP$ThreadScheduler$interruptCurrentThread();
#line 48

#line 48
  return result;
#line 48
}
#line 48
# 131 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/LinkedListC.nc"
static inline list_element_t *LinkedListC$LinkedList$removeLast(linked_list_t *l)
#line 131
{
  return LinkedListC$LinkedList$removeAt(l, l->size - 1);
}

# 56 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/LinkedList.nc"
inline static list_element_t *ThreadQueueP$LinkedList$removeLast(linked_list_t *l){
#line 56
  struct list_element *result;
#line 56

#line 56
  result = LinkedListC$LinkedList$removeLast(l);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 51 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadQueueP.nc"
static inline thread_t *ThreadQueueP$ThreadQueue$dequeue(thread_queue_t *q)
#line 51
{
  return (thread_t *)ThreadQueueP$LinkedList$removeLast(& q->l);
}

# 41 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
inline static thread_t *TinyThreadSchedulerP$ThreadQueue$dequeue(thread_queue_t *q){
#line 41
  struct thread *result;
#line 41

#line 41
  result = ThreadQueueP$ThreadQueue$dequeue(q);
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 121 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/LinkedListC.nc"
static inline list_element_t *LinkedListC$LinkedList$removeFirst(linked_list_t *l)
#line 121
{
  if (l->head == (void *)0) {
#line 122
    return (void *)0;
    }
  else {
#line 123
    return LinkedListC$remove_element(l, & l->head);
    }
}

#line 42
static inline list_element_t *LinkedListC$get_elementAt(linked_list_t *l, uint8_t i)
#line 42
{
  if (i >= l->size) {
#line 43
    return (void *)0;
    }
  else {
#line 44
    if (l->head == (void *)0) {
#line 44
      return (void *)0;
      }
    else 
#line 45
      {
        list_element_t *temp = l->head;

#line 47
        while (i-- > 0) {
            temp = temp->next;
          }
        return temp;
      }
    }
}

# 120 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 123
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(n >> 1);
}

# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP$VectorTimerA1$fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired();
#line 28
}
#line 28
# 115 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(0);
}

# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP$VectorTimerB0$fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired();
#line 28
}
#line 28
# 185 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void )
{
}

# 103 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void )
{
}

# 47 "/Users/doina/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void )
{
}

# 166 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void )
{
}

# 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow(void ){
#line 71
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow();
#line 71
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow();
#line 71
}
#line 71
# 122 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC$0$m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC$0$m_upper & /*CounterMilli32C.Transform*/TransformCounterC$0$OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow();
      }
  }
}

# 71 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow(void ){
#line 71
  /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow();
#line 71
}
#line 71
# 53 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow();
}

# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow(void ){
#line 37
  /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow();
#line 37
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow();
#line 37
}
#line 37
# 126 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow();
}

# 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 70 "/Users/doina/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void )
{
#line 71
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask();
}

# 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$fired(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired();
#line 67
}
#line 67
# 151 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm();
      }
  }
}

# 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired(void ){
#line 67
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired();
#line 67
}
#line 67
# 124 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents();
#line 47
}
#line 47
# 59 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired();
}

# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired(void ){
#line 34
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired();
#line 34
}
#line 34
# 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t n)
{
}

# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$__nesc_unnamed4291 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(* (volatile uint16_t * )386U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired();
    }
}

# 83 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP$isWaiting(uint8_t id)
#line 83
{
  return SchedulerBasicP$m_next[id] != SchedulerBasicP$NO_TASK || SchedulerBasicP$m_tail == id;
}





static inline bool SchedulerBasicP$pushTask(uint8_t id)
#line 91
{
  if (!SchedulerBasicP$isWaiting(id)) {
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) {
          SchedulerBasicP$m_head = id;
          SchedulerBasicP$m_tail = id;
        }
      else {
          SchedulerBasicP$m_next[SchedulerBasicP$m_tail] = id;
          SchedulerBasicP$m_tail = id;
        }
      return TRUE;
    }
  else {
      return FALSE;
    }
}

# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 38 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get();
}

# 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get(void ){
#line 53
  unsigned int result;
#line 53

#line 53
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 70 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 35 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending(void ){
#line 35
  unsigned char result;
#line 35

#line 35
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending();
#line 35

#line 35
  return result;
#line 35
}
#line 35
# 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending();
}

# 60 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static bool /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending(void ){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 119 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents();
#line 46
}
#line 46
# 84 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 33 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(time);
#line 30
}
#line 30
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 154 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get() + x;
}

# 32 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 70 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(2);
          }
        else {
#line 86
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEvent(now + remaining);
          }
      }
#line 88
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$enableEvents();
  }
}

# 92 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(t0, dt);
#line 92
}
#line 92
# 181 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void )
{
}

# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired();
#line 34
}
#line 34
# 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(uint16_t n)
{
}

# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$__nesc_unnamed4292 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(* (volatile uint16_t * )388U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$default$fired(void )
{
}

# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$default$fired();
#line 34
}
#line 34
# 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t n)
{
}

# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$__nesc_unnamed4293 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(* (volatile uint16_t * )390U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void )
{
}

# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired();
#line 34
}
#line 34
# 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t n)
{
}

# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$__nesc_unnamed4294 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(* (volatile uint16_t * )392U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void )
{
}

# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired();
#line 34
}
#line 34
# 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t n)
{
}

# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$__nesc_unnamed4295 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(* (volatile uint16_t * )394U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void )
{
}

# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired();
#line 34
}
#line 34
# 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t n)
{
}

# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$__nesc_unnamed4296 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(* (volatile uint16_t * )396U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void )
{
}

# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired();
#line 34
}
#line 34
# 139 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t n)
{
}

# 75 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$__nesc_unnamed4297 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(* (volatile uint16_t * )398U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired();
    }
}

# 120 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 123
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(n >> 1);
}

# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP$VectorTimerB1$fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired();
#line 28
}
#line 28
# 47 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static error_t SchedulerBasicP$ThreadScheduler$suspendCurrentThread(void ){
#line 47
  unsigned char result;
#line 47

#line 47
  result = TinyThreadSchedulerP$ThreadScheduler$suspendCurrentThread();
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 68 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP$popTask(void )
#line 68
{
  if (SchedulerBasicP$m_head != SchedulerBasicP$NO_TASK) {
      uint8_t id = SchedulerBasicP$m_head;

#line 71
      SchedulerBasicP$m_head = SchedulerBasicP$m_next[SchedulerBasicP$m_head];
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) {
          SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
        }
      SchedulerBasicP$m_next[id] = SchedulerBasicP$NO_TASK;
      return id;
    }
  else {
      return SchedulerBasicP$NO_TASK;
    }
}

#line 128
static inline void SchedulerBasicP$TaskScheduler$taskLoop(void )
#line 128
{
  for (; ; ) {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 132
        {
          while ((nextTask = SchedulerBasicP$popTask()) == SchedulerBasicP$NO_TASK) {
              SchedulerBasicP$ThreadScheduler$suspendCurrentThread();
            }
        }
#line 136
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP$TaskBasic$runTask(nextTask);
    }
}

# 69 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/TaskScheduler.nc"
inline static void TinyOSMainP$TaskScheduler$taskLoop(void ){
#line 69
  SchedulerBasicP$TaskScheduler$taskLoop();
#line 69
}
#line 69
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/Thread.nc"
inline static error_t BlinkC$TinyThread2$start(void *arg){
#line 37
  unsigned char result;
#line 37

#line 37
  result = StaticThreadP$Thread$start(/*BlinkAppC.TinyThread2*/ThreadC$3$THREAD_ID, arg);
#line 37

#line 37
  return result;
#line 37
}
#line 37
inline static error_t BlinkC$TinyThread1$start(void *arg){
#line 37
  unsigned char result;
#line 37

#line 37
  result = StaticThreadP$Thread$start(/*BlinkAppC.TinyThread1*/ThreadC$2$THREAD_ID, arg);
#line 37

#line 37
  return result;
#line 37
}
#line 37
inline static error_t BlinkC$TinyThread0$start(void *arg){
#line 37
  unsigned char result;
#line 37

#line 37
  result = StaticThreadP$Thread$start(/*BlinkAppC.TinyThread0*/ThreadC$1$THREAD_ID, arg);
#line 37

#line 37
  return result;
#line 37
}
#line 37
# 48 "BlinkC.nc"
static inline void BlinkC$Boot$booted(void )
#line 48
{

  BlinkC$TinyThread0$start((void *)0);
  BlinkC$TinyThread1$start((void *)0);
  BlinkC$TinyThread2$start((void *)0);
}

# 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc"
inline static void TinyOSMainP$Boot$booted(void ){
#line 49
  BlinkC$Boot$booted();
#line 49
}
#line 49
# 212 "/Users/doina/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
   __asm volatile ("eint");}

# 55 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/TaskScheduler.nc"
inline static bool TinyOSMainP$TaskScheduler$runNextTask(void ){
#line 55
  unsigned char result;
#line 55

#line 55
  result = SchedulerBasicP$TaskScheduler$runNextTask();
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 58 "/Users/doina/tinyos-2.x/tos/types/TinyError.h"
static inline error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 46 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$__nesc_unnamed4298 {
#line 46
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$CC2int(x);
}

#line 94
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl();
}

# 36 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare();
#line 36
}
#line 36
# 42 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare();
  return SUCCESS;
}

# 67 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadInfoP.nc"
static inline error_t /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$Init$init(void )
#line 67
{
  return /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$init();
}

#line 67
static inline error_t /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$Init$init(void )
#line 67
{
  return /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$init();
}

#line 67
static inline error_t /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$Init$init(void )
#line 67
{
  return /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$init();
}

#line 67
static inline error_t /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$Init$init(void )
#line 67
{
  return /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$init();
}

# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t TinyOSMainP$SoftwareInit$init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$Init$init();
#line 51
  result = ecombine(result, /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$Init$init());
#line 51
  result = ecombine(result, /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$Init$init());
#line 51
  result = ecombine(result, /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$Init$init());
#line 51
  result = ecombine(result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init());
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 6;
}

# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set(void ){
#line 34
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set();
#line 34
}
#line 34
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set();
}

# 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$set(void ){
#line 29
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set();
#line 29
}
#line 29
# 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set(void ){
#line 34
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set();
#line 34
}
#line 34
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set();
}

# 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$set(void ){
#line 29
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set();
#line 29
}
#line 29
# 45 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

# 34 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set(void ){
#line 34
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set();
#line 34
}
#line 34
# 37 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set();
}

# 29 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$set(void ){
#line 29
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set();
#line 29
}
#line 29
# 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput();
#line 71
}
#line 71
# 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput();
}

# 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$makeOutput(void ){
#line 35
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput();
#line 35
}
#line 35
# 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput();
#line 71
}
#line 71
# 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput();
}

# 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$makeOutput(void ){
#line 35
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput();
#line 35
}
#line 35
# 52 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 71 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput();
#line 71
}
#line 71
# 43 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput();
}

# 35 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$makeOutput(void ){
#line 35
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput();
#line 35
}
#line 35
# 45 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc"
static inline error_t LedsP$Init$init(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 46
  {
    ;
    LedsP$Led0$makeOutput();
    LedsP$Led1$makeOutput();
    LedsP$Led2$makeOutput();
    LedsP$Led0$set();
    LedsP$Led1$set();
    LedsP$Led2$set();
  }
  return SUCCESS;
}

# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP$LedsInit$init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = LedsP$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 33 "/Users/doina/tinyos-2.x/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_SIMO0_PIN()
#line 33
{
#line 33
  static volatile uint8_t r __asm ("0x0019");

#line 33
  r |= 1 << 1;
}

#line 34
static inline  void TOSH_SET_UCLK0_PIN()
#line 34
{
#line 34
  static volatile uint8_t r __asm ("0x0019");

#line 34
  r |= 1 << 3;
}

#line 85
static inline  void TOSH_SET_FLASH_CS_PIN()
#line 85
{
#line 85
  static volatile uint8_t r __asm ("0x001D");

#line 85
  r |= 1 << 4;
}

#line 34
static inline  void TOSH_CLR_UCLK0_PIN()
#line 34
{
#line 34
  static volatile uint8_t r __asm ("0x0019");

#line 34
  r &= ~(1 << 3);
}

#line 85
static inline  void TOSH_CLR_FLASH_CS_PIN()
#line 85
{
#line 85
  static volatile uint8_t r __asm ("0x001D");

#line 85
  r &= ~(1 << 4);
}

# 11 "/Users/doina/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC$TOSH_wait(void )
#line 11
{
   __asm volatile ("nop"); __asm volatile ("nop");}

# 86 "/Users/doina/tinyos-2.x/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_FLASH_HOLD_PIN()
#line 86
{
#line 86
  static volatile uint8_t r __asm ("0x001D");

#line 86
  r |= 1 << 7;
}

#line 85
static inline  void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 85
{
#line 85
  static volatile uint8_t r __asm ("0x001E");

#line 85
  r |= 1 << 4;
}

#line 86
static inline  void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 86
{
#line 86
  static volatile uint8_t r __asm ("0x001E");

#line 86
  r |= 1 << 7;
}

#line 34
static inline  void TOSH_MAKE_UCLK0_OUTPUT()
#line 34
{
#line 34
  static volatile uint8_t r __asm ("0x001A");

#line 34
  r |= 1 << 3;
}

#line 33
static inline  void TOSH_MAKE_SIMO0_OUTPUT()
#line 33
{
#line 33
  static volatile uint8_t r __asm ("0x001A");

#line 33
  r |= 1 << 1;
}

# 27 "/Users/doina/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc"
static inline void MotePlatformC$TOSH_FLASH_M25P_DP(void )
#line 27
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC$TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

#line 6
static __inline void MotePlatformC$uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TA0R;

#line 8
  while (TA0R - t0 <= u) ;
}

#line 56
static inline error_t MotePlatformC$Init$init(void )
#line 56
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






    MotePlatformC$uwait(1024 * 10);

    MotePlatformC$TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP$MoteInit$init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = MotePlatformC$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 148 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP$startTimerB(void )
{

  Msp430ClockP$TBCTL = 0x0020 | (Msp430ClockP$TBCTL & ~(0x0020 | 0x0010));
}

#line 136
static inline void Msp430ClockP$startTimerA(void )
{

  Msp430ClockP$TA0CTL = 0x0020 | (Msp430ClockP$TA0CTL & ~(0x0020 | 0x0010));
}

#line 100
static inline void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP$TBCTL = 0x0100 | 0x0002;
}

#line 130
static inline void Msp430ClockP$Msp430ClockInit$default$initTimerB(void )
{
  Msp430ClockP$Msp430ClockInit$defaultInitTimerB();
}

# 32 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP$Msp430ClockInit$initTimerB(void ){
#line 32
  Msp430ClockP$Msp430ClockInit$default$initTimerB();
#line 32
}
#line 32
# 85 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void )
{
  TA0R = 0;









  Msp430ClockP$TA0CTL = 0x0200 | 0x0002;
}

#line 125
static inline void Msp430ClockP$Msp430ClockInit$default$initTimerA(void )
{
  Msp430ClockP$Msp430ClockInit$defaultInitTimerA();
}

# 31 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP$Msp430ClockInit$initTimerA(void ){
#line 31
  Msp430ClockP$Msp430ClockInit$default$initTimerA();
#line 31
}
#line 31
# 64 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP$IE1 &= ~(1 << 1);
}

#line 120
static inline void Msp430ClockP$Msp430ClockInit$default$initClocks(void )
{
  Msp430ClockP$Msp430ClockInit$defaultInitClocks();
}

# 30 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP$Msp430ClockInit$initClocks(void ){
#line 30
  Msp430ClockP$Msp430ClockInit$default$initClocks();
#line 30
}
#line 30
# 166 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP$test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP$set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + Msp430ClockP$ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TA0R;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP$busyCalibrateDco(void )
{

  int calib;
  int step;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP$test_calib_busywait_delta(calib | step) <= Msp430ClockP$TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP$set_dco_calib(calib);
}

#line 52
static inline void Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate(void )
{



  Msp430ClockP$TA0CTL = 0x0200 | 0x0020;
  Msp430ClockP$TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 115
static inline void Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate(void )
{
  Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate();
}

# 29 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP$Msp430ClockInit$setupDcoCalibrate(void ){
#line 29
  Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate();
#line 29
}
#line 29
# 214 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline error_t Msp430ClockP$Init$init(void )
{

  Msp430ClockP$TA0CTL = 0x0004;
  Msp430ClockP$TA0IV = 0;
  Msp430ClockP$TBCTL = 0x0004;
  Msp430ClockP$TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP$Msp430ClockInit$setupDcoCalibrate();
    Msp430ClockP$busyCalibrateDco();
    Msp430ClockP$Msp430ClockInit$initClocks();
    Msp430ClockP$Msp430ClockInit$initTimerA();
    Msp430ClockP$Msp430ClockInit$initTimerB();
    Msp430ClockP$startTimerA();
    Msp430ClockP$startTimerB();
  }

  return SUCCESS;
}

# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP$MoteClockInit$init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = Msp430ClockP$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 10 "/Users/doina/tinyos-2.x/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP$Init$init(void )
#line 10
{
  PlatformP$MoteClockInit$init();
  PlatformP$MoteInit$init();
  PlatformP$LedsInit$init();
  return SUCCESS;
}

# 51 "/Users/doina/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t TinyOSMainP$PlatformInit$init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = PlatformP$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 108 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP$TaskScheduler$init(void )
#line 108
{
  /* atomic removed: atomic calls only */
#line 109
  {
    memset((void *)SchedulerBasicP$m_next, SchedulerBasicP$NO_TASK, sizeof SchedulerBasicP$m_next);
    SchedulerBasicP$m_head = SchedulerBasicP$NO_TASK;
    SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
  }
}

# 47 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/TaskScheduler.nc"
inline static void TinyOSMainP$TaskScheduler$init(void ){
#line 47
  SchedulerBasicP$TaskScheduler$init();
#line 47
}
#line 47
# 67 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyOSMainP.nc"
static inline void TinyOSMainP$TinyOSBoot$booted(void )
#line 67
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 68
    {




      {
      }
#line 73
      ;


      TinyOSMainP$TaskScheduler$init();





      TinyOSMainP$PlatformInit$init();
      while (TinyOSMainP$TaskScheduler$runNextTask()) ;





      TinyOSMainP$SoftwareInit$init();
      while (TinyOSMainP$TaskScheduler$runNextTask()) ;
    }
#line 91
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  TinyOSMainP$Boot$booted();


  TinyOSMainP$TaskScheduler$taskLoop();
}

# 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc"
inline static void TinyThreadSchedulerP$TinyOSBoot$booted(void ){
#line 49
  TinyOSMainP$TinyOSBoot$booted();
#line 49
}
#line 49
# 100 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/LinkedListC.nc"
static inline void LinkedListC$LinkedList$init(linked_list_t *l)
#line 100
{
  l->head = (void *)0;
  l->size = 0;
}

# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/LinkedList.nc"
inline static void ThreadQueueP$LinkedList$init(linked_list_t *l){
#line 39
  LinkedListC$LinkedList$init(l);
#line 39
}
#line 39
# 45 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadQueueP.nc"
static inline void ThreadQueueP$ThreadQueue$init(thread_queue_t *q)
#line 45
{
  ThreadQueueP$LinkedList$init(& q->l);
}

# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
inline static void TinyThreadSchedulerP$ThreadQueue$init(thread_queue_t *q){
#line 39
  ThreadQueueP$ThreadQueue$init(q);
#line 39
}
#line 39
# 50 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
static inline thread_t *ThreadMapP$ThreadInfo$get(uint8_t id)
#line 50
{
  return ThreadMapP$StaticThreadInfo$get(id);
}

# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
inline static thread_t *TinyThreadSchedulerP$ThreadInfo$get(uint8_t arg_0x17af3a8){
#line 40
  struct thread *result;
#line 40

#line 40
  result = ThreadMapP$ThreadInfo$get(arg_0x17af3a8);
#line 40

#line 40
  return result;
#line 40
}
#line 40
# 201 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline void TinyThreadSchedulerP$ThreadSchedulerBoot$booted(void )
#line 201
{
  TinyThreadSchedulerP$num_runnable_threads = 0;
  TinyThreadSchedulerP$tos_thread = TinyThreadSchedulerP$ThreadInfo$get(TOSTHREAD_TOS_THREAD_ID);
  TinyThreadSchedulerP$tos_thread->id = TOSTHREAD_TOS_THREAD_ID;
  TinyThreadSchedulerP$ThreadQueue$init(&TinyThreadSchedulerP$ready_queue);

  TinyThreadSchedulerP$current_thread = TinyThreadSchedulerP$tos_thread;
  TinyThreadSchedulerP$current_thread->state = TOSTHREAD_STATE_ACTIVE;
  TinyThreadSchedulerP$current_thread->init_block = (void *)0;
  TinyThreadSchedulerP$TinyOSBoot$booted();
}

# 49 "/Users/doina/tinyos-2.x/tos/interfaces/Boot.nc"
inline static void RealMainImplP$ThreadSchedulerBoot$booted(void ){
#line 49
  TinyThreadSchedulerP$ThreadSchedulerBoot$booted();
#line 49
}
#line 49
# 33 "/Users/doina/tinyos-2.x/tos/platforms/telosb/hardware.h"
static inline  void TOSH_CLR_SIMO0_PIN()
#line 33
{
#line 33
  static volatile uint8_t r __asm ("0x0019");

#line 33
  r &= ~(1 << 1);
}

# 48 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/SystemCallP.nc"
static inline void SystemCallP$threadTask$runTask(void )
#line 48
{
  (* SystemCallP$current_call->syscall_ptr)(SystemCallP$current_call);
}

# 173 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShotAt(uint8_t num, uint32_t t0, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(num, t0, dt, TRUE);
}

# 118 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShotAt(1U, t0, dt);
#line 118
}
#line 118
# 153 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num].isrunning = FALSE;
}

# 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(1U);
#line 67
}
#line 67
# 53 "/Users/doina/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get(void ){
#line 53
  unsigned long result;
#line 53

#line 53
  result = /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 75 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get();
}

# 98 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow(void ){
#line 98
  unsigned long result;
#line 98

#line 98
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow();
#line 98

#line 98
  return result;
#line 98
}
#line 98
# 85 "/Users/doina/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void )
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow();
}

# 125 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(void ){
#line 125
  unsigned long result;
#line 125

#line 125
  result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow();
#line 125

#line 125
  return result;
#line 125
}
#line 125
# 178 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$getNow(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow();
}

# 125 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static uint32_t /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$getNow(void ){
#line 125
  unsigned long result;
#line 125

#line 125
  result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$getNow(1U);
#line 125

#line 125
  return result;
#line 125
}
#line 125
# 89 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$updateFromTimer$runTask(void )
{




  uint32_t now = /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$stop();

  for (num = 0; num < /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$NUM_TIMERS; num++) 
    {
      /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$Timer_t *timer = &/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$m_timers[num];

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
        /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$fireTimers(now);
        }
      else {
#line 124
        /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$startOneShotAt(now, min_remaining);
        }
    }
}

# 50 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static error_t SystemCallP$ThreadScheduler$wakeupThread(thread_id_t id){
#line 50
  unsigned char result;
#line 50

#line 50
  result = TinyThreadSchedulerP$ThreadScheduler$wakeupThread(id);
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 82 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/SystemCallP.nc"
static inline error_t SystemCallP$SystemCall$finish(syscall_t *s)
#line 82
{
  return SystemCallP$ThreadScheduler$wakeupThread(s->thread->id);
}

# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/SystemCall.nc"
inline static error_t ThreadSleepP$SystemCall$finish(syscall_t *s){
#line 40
  unsigned char result;
#line 40

#line 40
  result = SystemCallP$SystemCall$finish(s);
#line 40

#line 40
  return result;
#line 40
}
#line 40
# 311 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline thread_t *TinyThreadSchedulerP$ThreadScheduler$threadInfo(uint8_t id)
#line 311
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 312
    {
      struct thread *__nesc_temp = 
#line 312
      TinyThreadSchedulerP$ThreadInfo$get(id);

      {
#line 312
        __nesc_atomic_end(__nesc_atomic); 
#line 312
        return __nesc_temp;
      }
    }
#line 314
    __nesc_atomic_end(__nesc_atomic); }
}

# 41 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static thread_t *ThreadSleepP$ThreadScheduler$threadInfo(thread_id_t id){
#line 41
  struct thread *result;
#line 41

#line 41
  result = TinyThreadSchedulerP$ThreadScheduler$threadInfo(id);
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 76 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadSleepP.nc"
static inline void ThreadSleepP$TimerMilli$fired(uint8_t id)
#line 76
{
  thread_t *t = ThreadSleepP$ThreadScheduler$threadInfo(id);

#line 78
  if (t->syscall->syscall_ptr == ThreadSleepP$sleepTask) {
    ThreadSleepP$SystemCall$finish(t->syscall);
    }
}

# 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$Timer$fired(uint8_t arg_0x19c83c8){
#line 72
  ThreadSleepP$TimerMilli$fired(arg_0x19c83c8);
#line 72
}
#line 72
# 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$updateFromTimer$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$updateFromTimer);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 133 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$Timer_t *timer = &/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$m_timers[num];

#line 136
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$updateFromTimer$postTask();
}






static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$Timer$startOneShot(uint8_t num, uint32_t dt)
{
  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$startTimer(num, /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$getNow(), dt, TRUE);
}

# 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void ThreadSleepP$TimerMilli$startOneShot(uint8_t arg_0x1aa1770, uint32_t dt){
#line 62
  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$Timer$startOneShot(arg_0x1aa1770, dt);
#line 62
}
#line 62
# 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 92 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(t0, dt);
#line 92
}
#line 92
# 47 "/Users/doina/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(t0, dt);
}

#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 83
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(t0, dt, TRUE);
}

# 118 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(t0, dt);
#line 118
}
#line 118
# 54 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
}

# 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop();
#line 62
}
#line 62
# 91 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop();
}

# 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop();
#line 62
}
#line 62
# 60 "/Users/doina/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void )
{
#line 61
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop();
}

# 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop();
#line 67
}
#line 67
# 89 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num];

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
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(now);
        }
      else {
#line 124
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(now, min_remaining);
        }
    }
}

# 111 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/LinkedListC.nc"
static inline uint8_t LinkedListC$LinkedList$size(linked_list_t *l)
#line 111
{
  return l->size;
}

# 41 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/LinkedList.nc"
inline static uint8_t ThreadQueueP$LinkedList$size(linked_list_t *l){
#line 41
  unsigned char result;
#line 41

#line 41
  result = LinkedListC$LinkedList$size(l);
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 57 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadQueueP.nc"
static inline bool ThreadQueueP$ThreadQueue$isEmpty(thread_queue_t *q)
#line 57
{
  return ThreadQueueP$LinkedList$size(& q->l) == 0;
}

# 43 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
inline static bool TinyThreadSchedulerP$ThreadQueue$isEmpty(thread_queue_t *q){
#line 43
  unsigned char result;
#line 43

#line 43
  result = ThreadQueueP$ThreadQueue$isEmpty(q);
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 62 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void TinyThreadSchedulerP$PreemptionAlarm$startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(0U, dt);
#line 62
}
#line 62
# 319 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline void TinyThreadSchedulerP$PreemptionAlarm$fired(void )
#line 319
{
  TinyThreadSchedulerP$PreemptionAlarm$startOneShot(TOSTHREAD_PREEMPTION_PERIOD);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 321
    {
      if (TinyThreadSchedulerP$ThreadQueue$isEmpty(&TinyThreadSchedulerP$ready_queue) == FALSE) {
          TinyThreadSchedulerP$ThreadScheduler$interruptCurrentThread();
        }
    }
#line 325
    __nesc_atomic_end(__nesc_atomic); }
}

# 128 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$fired(void )
{
  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$fireTimers(/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$getNow());
}

#line 193
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num)
{
}

# 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(uint8_t arg_0x19c83c8){
#line 72
  switch (arg_0x19c83c8) {
#line 72
    case 0U:
#line 72
      TinyThreadSchedulerP$PreemptionAlarm$fired();
#line 72
      break;
#line 72
    case 1U:
#line 72
      /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$TimerFrom$fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(arg_0x19c83c8);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 128 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow());
}

# 72 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired(void ){
#line 72
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired();
#line 72
}
#line 72
# 80 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 82
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type __nesc_temp = 
#line 82
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;

      {
#line 82
        __nesc_atomic_end(__nesc_atomic); 
#line 82
        return __nesc_temp;
      }
    }
#line 84
    __nesc_atomic_end(__nesc_atomic); }
}

# 105 "/Users/doina/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(void ){
#line 105
  unsigned long result;
#line 105

#line 105
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm();
#line 105

#line 105
  return result;
#line 105
}
#line 105
# 63 "/Users/doina/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired();
}

# 67 "/Users/doina/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void TinyThreadSchedulerP$PreemptionAlarm$stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$stop(0U);
#line 67
}
#line 67
# 64 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline void TinyThreadSchedulerP$alarmTask$runTask(void )
#line 64
{
  uint8_t temp;

#line 66
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 66
    temp = TinyThreadSchedulerP$num_runnable_threads;
#line 66
    __nesc_atomic_end(__nesc_atomic); }
  if (temp <= 1) {
    TinyThreadSchedulerP$PreemptionAlarm$stop();
    }
  else {
#line 69
    if (temp > 1) {
      TinyThreadSchedulerP$PreemptionAlarm$startOneShot(TOSTHREAD_PREEMPTION_PERIOD);
      }
    }
}

# 55 "BlinkC.nc"
static inline void BlinkC$NullThread$run(void *arg)
#line 55
{
  for (; ; ) {
    }
}

# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadSleep.nc"
inline static error_t StaticThreadP$ThreadSleep$sleep(uint32_t milli){
#line 37
  unsigned char result;
#line 37

#line 37
  result = ThreadSleepP$ThreadSleep$sleep(milli);
#line 37

#line 37
  return result;
#line 37
}
#line 37
# 89 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline error_t StaticThreadP$Thread$sleep(uint8_t id, uint32_t milli)
#line 89
{
  return StaticThreadP$ThreadSleep$sleep(milli);
}

# 41 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/Thread.nc"
inline static error_t BlinkC$TinyThread0$sleep(uint32_t milli){
#line 41
  unsigned char result;
#line 41

#line 41
  result = StaticThreadP$Thread$sleep(/*BlinkAppC.TinyThread0*/ThreadC$1$THREAD_ID, milli);
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle();
#line 44
}
#line 44
# 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$toggle();
}

# 31 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$toggle(void ){
#line 31
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle();
#line 31
}
#line 31
# 73 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP$Leds$led0Toggle(void )
#line 73
{
  LedsP$Led0$toggle();
  ;
#line 75
  ;
}

# 56 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkC$Leds$led0Toggle(void ){
#line 56
  LedsP$Leds$led0Toggle();
#line 56
}
#line 56
# 59 "BlinkC.nc"
static inline void BlinkC$TinyThread0$run(void *arg)
#line 59
{
  for (; ; ) {
      BlinkC$Leds$led0Toggle();
      BlinkC$TinyThread0$sleep(200);
    }
}

# 41 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/Thread.nc"
inline static error_t BlinkC$TinyThread1$sleep(uint32_t milli){
#line 41
  unsigned char result;
#line 41

#line 41
  result = StaticThreadP$Thread$sleep(/*BlinkAppC.TinyThread1*/ThreadC$2$THREAD_ID, milli);
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$toggle();
#line 44
}
#line 44
# 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$toggle();
}

# 31 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$toggle(void ){
#line 31
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$toggle();
#line 31
}
#line 31
# 88 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP$Leds$led1Toggle(void )
#line 88
{
  LedsP$Led1$toggle();
  ;
#line 90
  ;
}

# 72 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkC$Leds$led1Toggle(void ){
#line 72
  LedsP$Leds$led1Toggle();
#line 72
}
#line 72
# 65 "BlinkC.nc"
static inline void BlinkC$TinyThread1$run(void *arg)
#line 65
{
  for (; ; ) {
      BlinkC$Leds$led1Toggle();
      BlinkC$TinyThread1$sleep(1000);
    }
}

# 41 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/Thread.nc"
inline static error_t BlinkC$TinyThread2$sleep(uint32_t milli){
#line 41
  unsigned char result;
#line 41

#line 41
  result = StaticThreadP$Thread$sleep(/*BlinkAppC.TinyThread2*/ThreadC$3$THREAD_ID, milli);
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 44 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$toggle();
#line 44
}
#line 44
# 39 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$toggle();
}

# 31 "/Users/doina/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$toggle(void ){
#line 31
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$toggle();
#line 31
}
#line 31
# 103 "/Users/doina/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP$Leds$led2Toggle(void )
#line 103
{
  LedsP$Led2$toggle();
  ;
#line 105
  ;
}

# 89 "/Users/doina/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkC$Leds$led2Toggle(void ){
#line 89
  LedsP$Leds$led2Toggle();
#line 89
}
#line 89
# 71 "BlinkC.nc"
static inline void BlinkC$TinyThread2$run(void *arg)
#line 71
{
  for (; ; ) {
      BlinkC$Leds$led2Toggle();
      BlinkC$TinyThread2$sleep(1000);
    }
}

# 105 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline void StaticThreadP$Thread$default$run(uint8_t id, void *arg)
#line 105
{
}

# 42 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/Thread.nc"
inline static void StaticThreadP$Thread$run(uint8_t arg_0x1a4ca98, void *arg){
#line 42
  switch (arg_0x1a4ca98) {
#line 42
    case /*BlinkAppC.NullThread*/ThreadC$0$THREAD_ID:
#line 42
      BlinkC$NullThread$run(arg);
#line 42
      break;
#line 42
    case /*BlinkAppC.TinyThread0*/ThreadC$1$THREAD_ID:
#line 42
      BlinkC$TinyThread0$run(arg);
#line 42
      break;
#line 42
    case /*BlinkAppC.TinyThread1*/ThreadC$2$THREAD_ID:
#line 42
      BlinkC$TinyThread1$run(arg);
#line 42
      break;
#line 42
    case /*BlinkAppC.TinyThread2*/ThreadC$3$THREAD_ID:
#line 42
      BlinkC$TinyThread2$run(arg);
#line 42
      break;
#line 42
    default:
#line 42
      StaticThreadP$Thread$default$run(arg_0x1a4ca98, arg);
#line 42
      break;
#line 42
    }
#line 42
}
#line 42
# 97 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline void StaticThreadP$ThreadFunction$signalThreadRun(uint8_t id, void *arg)
#line 97
{
  StaticThreadP$Thread$run(id, arg);
}

# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadFunction.nc"
inline static void /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$ThreadFunction$signalThreadRun(void *arg){
#line 37
  StaticThreadP$ThreadFunction$signalThreadRun(/*BlinkAppC.TinyThread2*/ThreadC$3$THREAD_ID, arg);
#line 37
}
#line 37
# 47 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static error_t SystemCallP$ThreadScheduler$suspendCurrentThread(void ){
#line 47
  unsigned char result;
#line 47

#line 47
  result = TinyThreadSchedulerP$ThreadScheduler$suspendCurrentThread();
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 56 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t SystemCallP$threadTask$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(SystemCallP$threadTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 315 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline thread_t *TinyThreadSchedulerP$ThreadScheduler$currentThreadInfo(void )
#line 315
{
  /* atomic removed: atomic calls only */
#line 316
  {
    struct thread *__nesc_temp = 
#line 316
    TinyThreadSchedulerP$current_thread;

#line 316
    return __nesc_temp;
  }
}

# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static thread_t *SystemCallP$ThreadScheduler$currentThreadInfo(void ){
#line 40
  struct thread *result;
#line 40

#line 40
  result = TinyThreadSchedulerP$ThreadScheduler$currentThreadInfo();
#line 40

#line 40
  return result;
#line 40
}
#line 40
# 63 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/SystemCallP.nc"
static inline error_t SystemCallP$SystemCall$start(void *syscall_ptr, syscall_t *s, syscall_id_t id, void *p)
#line 63
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 64
    {

      SystemCallP$current_call = s;
      SystemCallP$current_call->id = id;
      SystemCallP$current_call->thread = SystemCallP$ThreadScheduler$currentThreadInfo();
      SystemCallP$current_call->thread->syscall = s;
      SystemCallP$current_call->params = p;

      if (syscall_ptr != SYSCALL_WAIT_ON_EVENT) {
          SystemCallP$current_call->syscall_ptr = syscall_ptr;
          SystemCallP$threadTask$postTask();
          SystemCallP$ThreadScheduler$wakeupThread(TOSTHREAD_TOS_THREAD_ID);
        }

      {
        unsigned char __nesc_temp = 
#line 78
        SystemCallP$ThreadScheduler$suspendCurrentThread();

        {
#line 78
          __nesc_atomic_end(__nesc_atomic); 
#line 78
          return __nesc_temp;
        }
      }
    }
#line 81
    __nesc_atomic_end(__nesc_atomic); }
}

# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/SystemCall.nc"
inline static error_t ThreadSleepP$SystemCall$start(void *syscall_ptr, syscall_t *s, syscall_id_t id, void *params){
#line 39
  unsigned char result;
#line 39

#line 39
  result = SystemCallP$SystemCall$start(syscall_ptr, s, id, params);
#line 39

#line 39
  return result;
#line 39
}
#line 39
# 138 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline void TinyThreadSchedulerP$suspend(thread_t *thread)
#line 138
{






  TinyThreadSchedulerP$sleepWhileIdle();
  TinyThreadSchedulerP$interrupt(thread);
}

# 206 "/Users/doina/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

# 126 "/Users/doina/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void )
#line 126
{
  return MSP430_POWER_LPM4;
}

# 54 "/Users/doina/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC$McuPowerOverride$lowestState(void ){
#line 54
  unsigned char result;
#line 54

#line 54
  result = McuSleepC$McuPowerOverride$default$lowestState();
#line 54

#line 54
  return result;
#line 54
}
#line 54
# 66 "/Users/doina/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC$getPowerState(void )
#line 66
{
  mcu_power_t pState = MSP430_POWER_LPM3;









  if ((((((
#line 69
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
#line 91
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 92
        if (ADC12CTL1 & 0x0400 && (TA0CTL & (3 << 8)) == 2 << 8) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 194 "/Users/doina/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 194
{
  return m1 < m2 ? m1 : m2;
}

# 104 "/Users/doina/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC$computePowerState(void )
#line 104
{
  McuSleepC$powerState = mcombine(McuSleepC$getPowerState(), 
  McuSleepC$McuPowerOverride$lowestState());
}

static inline void McuSleepC$McuSleep$sleep(void )
#line 109
{
  uint16_t temp;

#line 111
  if (McuSleepC$dirty) {
      McuSleepC$computePowerState();
    }

  temp = McuSleepC$msp430PowerBits[McuSleepC$powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 59 "/Users/doina/tinyos-2.x/tos/interfaces/McuSleep.nc"
inline static void TinyThreadSchedulerP$McuSleep$sleep(void ){
#line 59
  McuSleepC$McuSleep$sleep();
#line 59
}
#line 59
# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadFunction.nc"
inline static void /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$ThreadFunction$signalThreadRun(void *arg){
#line 37
  StaticThreadP$ThreadFunction$signalThreadRun(/*BlinkAppC.TinyThread1*/ThreadC$2$THREAD_ID, arg);
#line 37
}
#line 37
inline static void /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$ThreadFunction$signalThreadRun(void *arg){
#line 37
  StaticThreadP$ThreadFunction$signalThreadRun(/*BlinkAppC.TinyThread0*/ThreadC$1$THREAD_ID, arg);
#line 37
}
#line 37
inline static void /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$ThreadFunction$signalThreadRun(void *arg){
#line 37
  StaticThreadP$ThreadFunction$signalThreadRun(/*BlinkAppC.NullThread*/ThreadC$0$THREAD_ID, arg);
#line 37
}
#line 37
# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/BitArrayUtilsC.nc"
static inline void BitArrayUtilsC$BitArrayUtils$clrArray(uint8_t *array, uint8_t size)
#line 39
{
  memset(array, 0, size);
}

# 29 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/BitArrayUtils.nc"
inline static void TinyThreadSchedulerP$BitArrayUtils$clrArray(uint8_t *array, uint8_t numBytes){
#line 29
  BitArrayUtilsC$BitArrayUtils$clrArray(array, numBytes);
#line 29
}
#line 29
# 213 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline error_t TinyThreadSchedulerP$ThreadScheduler$initThread(uint8_t id)
#line 213
{
  thread_t *t = TinyThreadSchedulerP$ThreadInfo$get(id);

#line 215
  t->state = TOSTHREAD_STATE_INACTIVE;
  t->init_block = TinyThreadSchedulerP$current_thread->init_block;
  TinyThreadSchedulerP$BitArrayUtils$clrArray(t->joinedOnMe, sizeof  t->joinedOnMe);
  * t->stack_ptr = (uint16_t )&TinyThreadSchedulerP$threadWrapper;
#line 218
   __asm ("mov.w r2,%0" : "=r"(t->regs.status));
  return SUCCESS;
}

# 43 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static error_t StaticThreadP$ThreadScheduler$initThread(thread_id_t id){
#line 43
  unsigned char result;
#line 43

#line 43
  result = TinyThreadSchedulerP$ThreadScheduler$initThread(id);
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 103 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyOSMainP.nc"
static inline error_t TinyOSMainP$ThreadInfo$reset(void )
#line 103
{
  return FAIL;
}

# 71 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadInfoP.nc"
static inline error_t /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$ThreadInfo$reset(void )
#line 71
{
  return /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$init();
}

#line 71
static inline error_t /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$ThreadInfo$reset(void )
#line 71
{
  return /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$init();
}

#line 71
static inline error_t /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$ThreadInfo$reset(void )
#line 71
{
  return /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$init();
}

#line 71
static inline error_t /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$ThreadInfo$reset(void )
#line 71
{
  return /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$init();
}

# 107 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline error_t StaticThreadP$ThreadInfo$default$reset(uint8_t id)
#line 107
{
#line 107
  return FAIL;
}

# 39 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
inline static error_t StaticThreadP$ThreadInfo$reset(uint8_t arg_0x1a455b0){
#line 39
  unsigned char result;
#line 39

#line 39
  switch (arg_0x1a455b0) {
#line 39
    case /*BlinkAppC.NullThread*/ThreadC$0$THREAD_ID:
#line 39
      result = /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$ThreadInfo$reset();
#line 39
      break;
#line 39
    case /*BlinkAppC.TinyThread0*/ThreadC$1$THREAD_ID:
#line 39
      result = /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$ThreadInfo$reset();
#line 39
      break;
#line 39
    case /*BlinkAppC.TinyThread1*/ThreadC$2$THREAD_ID:
#line 39
      result = /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$ThreadInfo$reset();
#line 39
      break;
#line 39
    case /*BlinkAppC.TinyThread2*/ThreadC$3$THREAD_ID:
#line 39
      result = /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$ThreadInfo$reset();
#line 39
      break;
#line 39
    case TOSTHREAD_TOS_THREAD_ID:
#line 39
      result = TinyOSMainP$ThreadInfo$reset();
#line 39
      break;
#line 39
    default:
#line 39
      result = StaticThreadP$ThreadInfo$default$reset(arg_0x1a455b0);
#line 39
      break;
#line 39
    }
#line 39

#line 39
  return result;
#line 39
}
#line 39
# 107 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyOSMainP.nc"
static inline thread_t *TinyOSMainP$ThreadInfo$get(void )
#line 107
{
  return &TinyOSMainP$thread_info;
}

# 75 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadInfoP.nc"
static inline thread_t */*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$ThreadInfo$get(void )
#line 75
{
  return &/*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$thread_info;
}

#line 75
static inline thread_t */*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$ThreadInfo$get(void )
#line 75
{
  return &/*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$thread_info;
}

#line 75
static inline thread_t */*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$ThreadInfo$get(void )
#line 75
{
  return &/*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$thread_info;
}

#line 75
static inline thread_t */*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$ThreadInfo$get(void )
#line 75
{
  return &/*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$thread_info;
}

# 106 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline thread_t *StaticThreadP$ThreadInfo$default$get(uint8_t id)
#line 106
{
#line 106
  return (void *)0;
}

# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
inline static thread_t *StaticThreadP$ThreadInfo$get(uint8_t arg_0x1a455b0){
#line 40
  struct thread *result;
#line 40

#line 40
  switch (arg_0x1a455b0) {
#line 40
    case /*BlinkAppC.NullThread*/ThreadC$0$THREAD_ID:
#line 40
      result = /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$ThreadInfo$get();
#line 40
      break;
#line 40
    case /*BlinkAppC.TinyThread0*/ThreadC$1$THREAD_ID:
#line 40
      result = /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$ThreadInfo$get();
#line 40
      break;
#line 40
    case /*BlinkAppC.TinyThread1*/ThreadC$2$THREAD_ID:
#line 40
      result = /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$ThreadInfo$get();
#line 40
      break;
#line 40
    case /*BlinkAppC.TinyThread2*/ThreadC$3$THREAD_ID:
#line 40
      result = /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$ThreadInfo$get();
#line 40
      break;
#line 40
    case TOSTHREAD_TOS_THREAD_ID:
#line 40
      result = TinyOSMainP$ThreadInfo$get();
#line 40
      break;
#line 40
    default:
#line 40
      result = StaticThreadP$ThreadInfo$default$get(arg_0x1a455b0);
#line 40
      break;
#line 40
    }
#line 40

#line 40
  return result;
#line 40
}
#line 40
# 52 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline error_t StaticThreadP$init(uint8_t id, void *arg)
#line 52
{
  error_t r1;
#line 53
  error_t r2;
  thread_t *thread_info = StaticThreadP$ThreadInfo$get(id);

#line 55
  thread_info->start_arg_ptr = arg;
  thread_info->mutex_count = 0;
  thread_info->next_thread = (void *)0;
  r1 = StaticThreadP$ThreadInfo$reset(id);
  r2 = StaticThreadP$ThreadScheduler$initThread(id);
  return ecombine(r1, r2);
}

# 68 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
static inline void ThreadMapP$ThreadCleanup$cleanup(uint8_t id)
#line 68
{
  ThreadMapP$StaticThreadCleanup$cleanup(id);
}

# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
inline static void TinyThreadSchedulerP$ThreadCleanup$cleanup(uint8_t arg_0x17b0878){
#line 37
  ThreadMapP$ThreadCleanup$cleanup(arg_0x17b0878);
#line 37
}
#line 37
# 149 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline void TinyThreadSchedulerP$wakeupJoined(thread_t *t)
#line 149
{
  int i;
#line 150
  int j;
#line 150
  int k;

#line 151
  k = 0;
  for (i = 0; i < sizeof  t->joinedOnMe; i++) {
      if (t->joinedOnMe[i] == 0) {
          k += 8;
          continue;
        }
      for (j = 0; j < 8; j++) {
          if (t->joinedOnMe[i] & 0x1) {
            TinyThreadSchedulerP$ThreadScheduler$wakeupThread(k);
            }
#line 160
          t->joinedOnMe[i] >>= 1;
          k++;
        }
    }
}






static inline void TinyThreadSchedulerP$stop(thread_t *t)
#line 171
{
  t->state = TOSTHREAD_STATE_INACTIVE;
  TinyThreadSchedulerP$num_runnable_threads--;
  TinyThreadSchedulerP$wakeupJoined(t);



  if (TinyThreadSchedulerP$num_runnable_threads == 1) {
    TinyThreadSchedulerP$PreemptionAlarm$stop();
    }
  TinyThreadSchedulerP$ThreadCleanup$cleanup(t->id);
}

# 109 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline void StaticThreadP$ThreadNotification$default$aboutToDestroy(uint8_t id)
#line 109
{
}

# 38 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadNotification.nc"
inline static void StaticThreadP$ThreadNotification$aboutToDestroy(uint8_t arg_0x1a486a0){
#line 38
    StaticThreadP$ThreadNotification$default$aboutToDestroy(arg_0x1a486a0);
#line 38
}
#line 38
# 101 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline void StaticThreadP$ThreadCleanup$cleanup(uint8_t id)
#line 101
{
  StaticThreadP$ThreadNotification$aboutToDestroy(id);
}

# 74 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
static inline void ThreadMapP$DynamicThreadCleanup$default$cleanup(uint8_t id)
#line 74
{
  ThreadMapP$StaticThreadCleanup$cleanup(id);
}

# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
inline static void ThreadMapP$DynamicThreadCleanup$cleanup(uint8_t arg_0x1a78ed0){
#line 37
    ThreadMapP$DynamicThreadCleanup$default$cleanup(arg_0x1a78ed0);
#line 37
}
#line 37
# 114 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/LinkedListC.nc"
static inline error_t LinkedListC$LinkedList$addFirst(linked_list_t *l, list_element_t *e)
#line 114
{
  return LinkedListC$insert_element(l, & l->head, e);
}

# 43 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/LinkedList.nc"
inline static error_t ThreadQueueP$LinkedList$addFirst(linked_list_t *l, list_element_t *e){
#line 43
  unsigned char result;
#line 43

#line 43
  result = LinkedListC$LinkedList$addFirst(l, e);
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 48 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadQueueP.nc"
static inline void ThreadQueueP$ThreadQueue$enqueue(thread_queue_t *q, thread_t *t)
#line 48
{
  ThreadQueueP$LinkedList$addFirst(& q->l, (list_element_t *)t);
}

# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadQueue.nc"
inline static void TinyThreadSchedulerP$ThreadQueue$enqueue(thread_queue_t *q, thread_t *t){
#line 40
  ThreadQueueP$ThreadQueue$enqueue(q, t);
#line 40
}
#line 40
# 222 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static inline error_t TinyThreadSchedulerP$ThreadScheduler$startThread(uint8_t id)
#line 222
{
  /* atomic removed: atomic calls only */
#line 223
  {
    thread_t *t = TinyThreadSchedulerP$ThreadInfo$get(id);

#line 225
    if (t->state == TOSTHREAD_STATE_INACTIVE) {
        TinyThreadSchedulerP$num_runnable_threads++;



        if (TinyThreadSchedulerP$num_runnable_threads == 2) {
          TinyThreadSchedulerP$PreemptionAlarm$startOneShot(TOSTHREAD_PREEMPTION_PERIOD);
          }
        t->state = TOSTHREAD_STATE_READY;
        TinyThreadSchedulerP$ThreadQueue$enqueue(&TinyThreadSchedulerP$ready_queue, t);
        {
          unsigned char __nesc_temp = 
#line 235
          SUCCESS;

#line 235
          return __nesc_temp;
        }
      }
  }
#line 238
  return FAIL;
}

# 44 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadScheduler.nc"
inline static error_t StaticThreadP$ThreadScheduler$startThread(thread_id_t id){
#line 44
  unsigned char result;
#line 44

#line 44
  result = TinyThreadSchedulerP$ThreadScheduler$startThread(id);
#line 44

#line 44
  return result;
#line 44
}
#line 44
# 108 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
static inline void StaticThreadP$ThreadNotification$default$justCreated(uint8_t id)
#line 108
{
}

# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadNotification.nc"
inline static void StaticThreadP$ThreadNotification$justCreated(uint8_t arg_0x1a486a0){
#line 37
    StaticThreadP$ThreadNotification$default$justCreated(arg_0x1a486a0);
#line 37
}
#line 37
# 226 "/Users/doina/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (({
#line 228
    uint16_t __x;

#line 228
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 228
   & 0x0008) != 0;

#line 229
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 12 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/chips/msp430/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(12)))  void sig_TIMERA0_VECTOR(void )
#line 12
{
  Msp430TimerCommonP$VectorTimerA0$fired();
  Msp430TimerCommonP$PlatformInterrupt$postAmble();
}

# 169 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired();
    }
}

# 46 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TOSThreadsInterruptP.nc"
static __attribute((noinline)) void TOSThreadsInterruptP$interruptThread(void )
#line 46
{
  if (TOSThreadsInterruptP$ThreadScheduler$wakeupThread(TOSTHREAD_TOS_THREAD_ID) == SUCCESS) {
    if (TOSThreadsInterruptP$ThreadScheduler$currentThreadId() != TOSTHREAD_TOS_THREAD_ID) {
      TOSThreadsInterruptP$ThreadScheduler$interruptCurrentThread();
      }
    }
}

# 291 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static error_t TinyThreadSchedulerP$ThreadScheduler$wakeupThread(uint8_t id)
#line 291
{
  thread_t *t = TinyThreadSchedulerP$ThreadInfo$get(id);

#line 293
  if (t->state == TOSTHREAD_STATE_SUSPENDED) {
      t->state = TOSTHREAD_STATE_READY;
      if (t != TinyThreadSchedulerP$tos_thread) {
          TinyThreadSchedulerP$ThreadQueue$enqueue(&TinyThreadSchedulerP$ready_queue, TinyThreadSchedulerP$ThreadInfo$get(id));
        }




      return SUCCESS;
    }
  return FAIL;
}

# 56 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
static thread_t *ThreadMapP$StaticThreadInfo$default$get(uint8_t id)
#line 56
{
  return ThreadMapP$DynamicThreadInfo$get(id);
}

# 40 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadInfo.nc"
static thread_t *ThreadMapP$StaticThreadInfo$get(uint8_t arg_0x1a77688){
#line 40
  struct thread *result;
#line 40

#line 40
  switch (arg_0x1a77688) {
#line 40
    case /*BlinkAppC.NullThread*/ThreadC$0$THREAD_ID:
#line 40
      result = /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$ThreadInfo$get();
#line 40
      break;
#line 40
    case /*BlinkAppC.TinyThread0*/ThreadC$1$THREAD_ID:
#line 40
      result = /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$ThreadInfo$get();
#line 40
      break;
#line 40
    case /*BlinkAppC.TinyThread1*/ThreadC$2$THREAD_ID:
#line 40
      result = /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$ThreadInfo$get();
#line 40
      break;
#line 40
    case /*BlinkAppC.TinyThread2*/ThreadC$3$THREAD_ID:
#line 40
      result = /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$ThreadInfo$get();
#line 40
      break;
#line 40
    case TOSTHREAD_TOS_THREAD_ID:
#line 40
      result = TinyOSMainP$ThreadInfo$get();
#line 40
      break;
#line 40
    default:
#line 40
      result = ThreadMapP$StaticThreadInfo$default$get(arg_0x1a77688);
#line 40
      break;
#line 40
    }
#line 40

#line 40
  return result;
#line 40
}
#line 40
# 84 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/LinkedListC.nc"
static error_t LinkedListC$insert_element(linked_list_t *l, list_element_t **previous_next, list_element_t *e)
#line 84
{
  if (e == (void *)0) {
#line 85
    return FAIL;
    }
#line 86
  e->next = *previous_next;
  *previous_next = e;
  l->size++;
  return SUCCESS;
}

# 264 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static error_t TinyThreadSchedulerP$ThreadScheduler$interruptCurrentThread(void )
#line 264
{
  /* atomic removed: atomic calls only */
#line 265
  {
    if (TinyThreadSchedulerP$current_thread->state == TOSTHREAD_STATE_ACTIVE) {
        TinyThreadSchedulerP$current_thread->state = TOSTHREAD_STATE_READY;
        if (TinyThreadSchedulerP$current_thread != TinyThreadSchedulerP$tos_thread) {
          TinyThreadSchedulerP$ThreadQueue$enqueue(&TinyThreadSchedulerP$ready_queue, TinyThreadSchedulerP$current_thread);
          }
#line 270
        TinyThreadSchedulerP$interrupt(TinyThreadSchedulerP$current_thread);
        {
          unsigned char __nesc_temp = 
#line 271
          SUCCESS;

#line 271
          return __nesc_temp;
        }
      }
#line 273
    {
      unsigned char __nesc_temp = 
#line 273
      FAIL;

#line 273
      return __nesc_temp;
    }
  }
}

#line 124
static void TinyThreadSchedulerP$interrupt(thread_t *thread)
#line 124
{
  TinyThreadSchedulerP$yielding_thread = thread;
  TinyThreadSchedulerP$scheduleNextThread();
  if (TinyThreadSchedulerP$current_thread != TinyThreadSchedulerP$yielding_thread) {
      TinyThreadSchedulerP$switchThreads();
    }
}

#line 111
static void TinyThreadSchedulerP$scheduleNextThread(void )
#line 111
{
  if (TinyThreadSchedulerP$tos_thread->state == TOSTHREAD_STATE_READY) {
    TinyThreadSchedulerP$current_thread = TinyThreadSchedulerP$tos_thread;
    }
  else {
#line 115
    TinyThreadSchedulerP$current_thread = TinyThreadSchedulerP$ThreadQueue$dequeue(&TinyThreadSchedulerP$ready_queue);
    }
  TinyThreadSchedulerP$current_thread->state = TOSTHREAD_STATE_ACTIVE;
}

# 148 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/LinkedListC.nc"
static list_element_t *LinkedListC$LinkedList$removeAt(linked_list_t *l, uint8_t i)
#line 148
{
  if (i == 0) {
    return LinkedListC$LinkedList$removeFirst(l);
    }
  else 
#line 151
    {
      list_element_t *temp = LinkedListC$get_elementAt(l, i - 1);

#line 153
      if (temp == (void *)0) {
#line 153
        return (void *)0;
        }
      else {
#line 154
        return LinkedListC$remove_element(l, & temp->next);
        }
    }
}

#line 92
static list_element_t *LinkedListC$remove_element(linked_list_t *l, list_element_t **previous_next)
#line 92
{
  list_element_t *e = *previous_next;

#line 94
  *previous_next = (*previous_next)->next;
  e->next = (void *)0;
  l->size--;
  return e;
}

# 84 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static __attribute((noinline)) void TinyThreadSchedulerP$switchThreads(void )
#line 84
{
   __asm ("mov.w r4,%0" : "=m"(TinyThreadSchedulerP$yielding_thread->regs.r4)); __asm ("mov.w r5,%0" : "=m"(TinyThreadSchedulerP$yielding_thread->regs.r5)); __asm ("mov.w r6,%0" : "=m"(TinyThreadSchedulerP$yielding_thread->regs.r6)); __asm ("mov.w r7,%0" : "=m"(TinyThreadSchedulerP$yielding_thread->regs.r7)); __asm ("mov.w r8,%0" : "=m"(TinyThreadSchedulerP$yielding_thread->regs.r8)); __asm ("mov.w r9,%0" : "=m"(TinyThreadSchedulerP$yielding_thread->regs.r9)); __asm ("mov.w r10,%0" : "=m"(TinyThreadSchedulerP$yielding_thread->regs.r10)); __asm ("mov.w r11,%0" : "=m"(TinyThreadSchedulerP$yielding_thread->regs.r11)); __asm ("mov.w r12,%0" : "=m"(TinyThreadSchedulerP$yielding_thread->regs.r12)); __asm ("mov.w r13,%0" : "=m"(TinyThreadSchedulerP$yielding_thread->regs.r13)); __asm ("mov.w r14,%0" : "=m"(TinyThreadSchedulerP$yielding_thread->regs.r14)); __asm ("mov.w r15,%0" : "=m"(TinyThreadSchedulerP$yielding_thread->regs.r15)); __asm ("mov.w r2,%0" : "=r"(TinyThreadSchedulerP$yielding_thread->regs.status)); __asm ("mov.w r1,%0" : "=m"(TinyThreadSchedulerP$yielding_thread->stack_ptr)); __asm ("mov.w %0,r1" :  : "m"(TinyThreadSchedulerP$current_thread->stack_ptr)); __asm ("mov.w %0,r2" :  : "r"(TinyThreadSchedulerP$current_thread->regs.status)); __asm ("mov.w %0,r4" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r4)); __asm ("mov.w %0,r5" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r5)); __asm ("mov.w %0,r6" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r6)); __asm ("mov.w %0,r7" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r7)); __asm ("mov.w %0,r8" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r8)); __asm ("mov.w %0,r9" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r9)); __asm ("mov.w %0,r10" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r10)); __asm ("mov.w %0,r11" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r11)); __asm ("mov.w %0,r12" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r12)); __asm ("mov.w %0,r13" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r13)); __asm ("mov.w %0,r14" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r14)); __asm ("mov.w %0,r15" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r15));}

# 16 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/chips/msp430/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(10)))  void sig_TIMERA1_VECTOR(void )
#line 16
{
  Msp430TimerCommonP$VectorTimerA1$fired();
  Msp430TimerCommonP$PlatformInterrupt$postAmble();
}

#line 20
__attribute((wakeup)) __attribute((interrupt(26)))  void sig_TIMERB0_VECTOR(void )
#line 20
{
  Msp430TimerCommonP$VectorTimerB0$fired();
  Msp430TimerCommonP$PlatformInterrupt$postAmble();
}

# 135 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(uint8_t n)
{
}

# 28 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(uint8_t arg_0x16e4eb0){
#line 28
  switch (arg_0x16e4eb0) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired();
#line 28
      break;
#line 28
    case 3:
#line 28
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired();
#line 28
      break;
#line 28
    case 4:
#line 28
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired();
#line 28
      break;
#line 28
    case 6:
#line 28
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired();
#line 28
      break;
#line 28
    case 7:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(arg_0x16e4eb0);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 145 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id)
#line 145
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 146
    {
#line 146
      {
        unsigned char __nesc_temp = 
#line 146
        SchedulerBasicP$pushTask(id) ? SUCCESS : EBUSY;

        {
#line 146
          __nesc_atomic_end(__nesc_atomic); 
#line 146
          return __nesc_temp;
        }
      }
    }
#line 149
    __nesc_atomic_end(__nesc_atomic); }
}

# 96 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get();
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type expires;
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type )remaining << 5);
}

# 69 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC$0$m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get();

#line 76
      if (/*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC$0$LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC$0$HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 51 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 58
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )400U;

#line 61
        do {
#line 61
            t0 = t1;
#line 61
            t1 = * (volatile uint16_t * )400U;
          }
        while (
#line 61
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 62
          t1;

#line 62
          return __nesc_temp;
        }
      }
    }
  else 
#line 65
    {
      return * (volatile uint16_t * )400U;
    }
}

# 24 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/chips/msp430/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(24)))  void sig_TIMERB1_VECTOR(void )
#line 24
{
  Msp430TimerCommonP$VectorTimerB1$fired();
  Msp430TimerCommonP$PlatformInterrupt$postAmble();
}

# 49 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/RealMainImplP.nc"
  int main(void )
#line 49
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 50
    {

      RealMainImplP$ThreadSchedulerBoot$booted();
    }
#line 53
    __nesc_atomic_end(__nesc_atomic); }




  return -1;
}

# 160 "/Users/doina/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP$set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/Users/doina/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc"
static void MotePlatformC$TOSH_FLASH_M25P_DP_bit(bool set)
#line 16
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 20
    TOSH_CLR_SIMO0_PIN();
    }
#line 21
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 116 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/SchedulerBasicP.nc"
static bool SchedulerBasicP$TaskScheduler$runNextTask(void )
#line 116
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 118
  {
    nextTask = SchedulerBasicP$popTask();
    if (nextTask == SchedulerBasicP$NO_TASK) {
        {
          unsigned char __nesc_temp = 
#line 121
          FALSE;

#line 121
          return __nesc_temp;
        }
      }
  }
#line 124
  SchedulerBasicP$TaskBasic$runTask(nextTask);
  return TRUE;
}

#line 149
static void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id)
#line 149
{
}

# 64 "/Users/doina/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$runTask(uint8_t arg_0x14a22b0){
#line 64
  switch (arg_0x14a22b0) {
#line 64
    case TinyThreadSchedulerP$alarmTask:
#line 64
      TinyThreadSchedulerP$alarmTask$runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired:
#line 64
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer:
#line 64
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask();
#line 64
      break;
#line 64
    case /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$updateFromTimer:
#line 64
      /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$updateFromTimer$runTask();
#line 64
      break;
#line 64
    case SystemCallP$threadTask:
#line 64
      SystemCallP$threadTask$runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP$TaskBasic$default$runTask(arg_0x14a22b0);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 62 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$NUM_TIMERS; num++) 
    {
      /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$Timer_t *timer = &/*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 79
                timer->t0 += timer->dt;
                }
              /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$Timer$fired(num);
              break;
            }
        }
    }
  /*ThreadTimersC.VirtualizeTimerC*/VirtualizeTimerC$1$updateFromTimer$postTask();
}

# 52 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadSleepP.nc"
static void ThreadSleepP$sleepTask(syscall_t *s)
#line 52
{
  ThreadSleepP$sleep_params_t *p = s->params;

#line 54
  ThreadSleepP$TimerMilli$startOneShot(s->thread->id, * p->milli);
}

# 133 "/Users/doina/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num];

#line 136
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

#line 62
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 79
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

#line 148
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(), dt, TRUE);
}

# 136 "/Users/doina/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

# 54 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadInfoP.nc"
static error_t /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$init(void )
#line 54
{
  /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$thread_info.next_thread = (void *)0;
  /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$thread_info.id = 3;
  /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$thread_info.init_block = (void *)0;
  /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$thread_info.stack_ptr = (stack_ptr_t )&((uint8_t *)/*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$stack)[sizeof /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$stack - sizeof(stack_ptr_t )];
  /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$thread_info.state = TOSTHREAD_STATE_INACTIVE;
  /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$thread_info.mutex_count = 0;
  /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$thread_info.start_ptr = /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$run_thread;
  /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$thread_info.start_arg_ptr = (void *)0;
  /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$thread_info.syscall = (void *)0;
  return SUCCESS;
}

#line 50
static __attribute((noinline)) void /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$run_thread(void *arg)
#line 50
{
  /*BlinkAppC.TinyThread2.ThreadInfoP*/ThreadInfoP$3$ThreadFunction$signalThreadRun(arg);
}

# 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$toggle(void )
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t * )49U ^= 0x01 << 6;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

# 57 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadSleepP.nc"
static error_t ThreadSleepP$ThreadSleep$sleep(uint32_t milli)
#line 57
{
  syscall_t s;
  ThreadSleepP$sleep_params_t p;

#line 60
  p.milli = &milli;
  ThreadSleepP$SystemCall$start(&ThreadSleepP$sleepTask, &s, INVALID_ID, &p);
  return SUCCESS;
}

# 253 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static error_t TinyThreadSchedulerP$ThreadScheduler$suspendCurrentThread(void )
#line 253
{
  /* atomic removed: atomic calls only */
#line 254
  {
    if (TinyThreadSchedulerP$current_thread->state == TOSTHREAD_STATE_ACTIVE) {
        TinyThreadSchedulerP$current_thread->state = TOSTHREAD_STATE_SUSPENDED;
        TinyThreadSchedulerP$suspend(TinyThreadSchedulerP$current_thread);
        {
          unsigned char __nesc_temp = 
#line 258
          SUCCESS;

#line 258
          return __nesc_temp;
        }
      }
#line 260
    {
      unsigned char __nesc_temp = 
#line 260
      FAIL;

#line 260
      return __nesc_temp;
    }
  }
}

#line 97
static void TinyThreadSchedulerP$sleepWhileIdle(void )
#line 97
{
  while (TRUE) {
      bool mt;

      /* atomic removed: atomic calls only */
#line 100
      mt = TinyThreadSchedulerP$ThreadQueue$isEmpty(&TinyThreadSchedulerP$ready_queue) == TRUE;
      if (!mt || TinyThreadSchedulerP$tos_thread->state == TOSTHREAD_STATE_READY) {
#line 101
        break;
        }
#line 102
      TinyThreadSchedulerP$McuSleep$sleep();
    }
}

# 47 "/Users/doina/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$toggle(void )
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t * )49U ^= 0x01 << 5;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

#line 47
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle(void )
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t * )49U ^= 0x01 << 4;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

# 54 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadInfoP.nc"
static error_t /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$init(void )
#line 54
{
  /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$thread_info.next_thread = (void *)0;
  /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$thread_info.id = 2;
  /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$thread_info.init_block = (void *)0;
  /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$thread_info.stack_ptr = (stack_ptr_t )&((uint8_t *)/*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$stack)[sizeof /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$stack - sizeof(stack_ptr_t )];
  /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$thread_info.state = TOSTHREAD_STATE_INACTIVE;
  /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$thread_info.mutex_count = 0;
  /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$thread_info.start_ptr = /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$run_thread;
  /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$thread_info.start_arg_ptr = (void *)0;
  /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$thread_info.syscall = (void *)0;
  return SUCCESS;
}

#line 50
static __attribute((noinline)) void /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$run_thread(void *arg)
#line 50
{
  /*BlinkAppC.TinyThread1.ThreadInfoP*/ThreadInfoP$2$ThreadFunction$signalThreadRun(arg);
}

static error_t /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$init(void )
#line 54
{
  /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$thread_info.next_thread = (void *)0;
  /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$thread_info.id = 1;
  /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$thread_info.init_block = (void *)0;
  /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$thread_info.stack_ptr = (stack_ptr_t )&((uint8_t *)/*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$stack)[sizeof /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$stack - sizeof(stack_ptr_t )];
  /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$thread_info.state = TOSTHREAD_STATE_INACTIVE;
  /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$thread_info.mutex_count = 0;
  /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$thread_info.start_ptr = /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$run_thread;
  /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$thread_info.start_arg_ptr = (void *)0;
  /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$thread_info.syscall = (void *)0;
  return SUCCESS;
}

#line 50
static __attribute((noinline)) void /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$run_thread(void *arg)
#line 50
{
  /*BlinkAppC.TinyThread0.ThreadInfoP*/ThreadInfoP$1$ThreadFunction$signalThreadRun(arg);
}

static error_t /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$init(void )
#line 54
{
  /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$thread_info.next_thread = (void *)0;
  /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$thread_info.id = 0;
  /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$thread_info.init_block = (void *)0;
  /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$thread_info.stack_ptr = (stack_ptr_t )&((uint8_t *)/*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$stack)[sizeof /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$stack - sizeof(stack_ptr_t )];
  /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$thread_info.state = TOSTHREAD_STATE_INACTIVE;
  /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$thread_info.mutex_count = 0;
  /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$thread_info.start_ptr = /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$run_thread;
  /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$thread_info.start_arg_ptr = (void *)0;
  /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$thread_info.syscall = (void *)0;
  return SUCCESS;
}

#line 50
static __attribute((noinline)) void /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$run_thread(void *arg)
#line 50
{
  /*BlinkAppC.NullThread.ThreadInfoP*/ThreadInfoP$0$ThreadFunction$signalThreadRun(arg);
}

# 63 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/StaticThreadP.nc"
static error_t StaticThreadP$Thread$start(uint8_t id, void *arg)
#line 63
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 64
    {
      if (StaticThreadP$init(id, arg) == SUCCESS) {
          error_t e = StaticThreadP$ThreadScheduler$startThread(id);

#line 67
          if (e == SUCCESS) {
            StaticThreadP$ThreadNotification$justCreated(id);
            }
#line 69
          {
            unsigned char __nesc_temp = 
#line 69
            e;

            {
#line 69
              __nesc_atomic_end(__nesc_atomic); 
#line 69
              return __nesc_temp;
            }
          }
        }
    }
#line 73
    __nesc_atomic_end(__nesc_atomic); }
#line 72
  return FAIL;
}

# 186 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static __attribute((noinline)) __attribute((naked)) void TinyThreadSchedulerP$threadWrapper(void )
#line 186
{
  thread_t *t;

#line 188
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    t = TinyThreadSchedulerP$current_thread;
#line 188
    __nesc_atomic_end(__nesc_atomic); }

  __nesc_enable_interrupt();
  (* t->start_ptr)(t->start_arg_ptr);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 193
    {
      TinyThreadSchedulerP$stop(t);
      TinyThreadSchedulerP$sleepWhileIdle();
      TinyThreadSchedulerP$scheduleNextThread();
      TinyThreadSchedulerP$restoreThread();
    }
#line 198
    __nesc_atomic_end(__nesc_atomic); }
}

# 71 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/ThreadMapP.nc"
static void ThreadMapP$StaticThreadCleanup$default$cleanup(uint8_t id)
#line 71
{
  ThreadMapP$DynamicThreadCleanup$cleanup(id);
}

# 37 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/interfaces/ThreadCleanup.nc"
static void ThreadMapP$StaticThreadCleanup$cleanup(uint8_t arg_0x1a78760){
#line 37
  switch (arg_0x1a78760) {
#line 37
    case /*BlinkAppC.NullThread*/ThreadC$0$THREAD_ID:
#line 37
      StaticThreadP$ThreadCleanup$cleanup(/*BlinkAppC.NullThread*/ThreadC$0$THREAD_ID);
#line 37
      break;
#line 37
    case /*BlinkAppC.TinyThread0*/ThreadC$1$THREAD_ID:
#line 37
      StaticThreadP$ThreadCleanup$cleanup(/*BlinkAppC.TinyThread0*/ThreadC$1$THREAD_ID);
#line 37
      break;
#line 37
    case /*BlinkAppC.TinyThread1*/ThreadC$2$THREAD_ID:
#line 37
      StaticThreadP$ThreadCleanup$cleanup(/*BlinkAppC.TinyThread1*/ThreadC$2$THREAD_ID);
#line 37
      break;
#line 37
    case /*BlinkAppC.TinyThread2*/ThreadC$3$THREAD_ID:
#line 37
      StaticThreadP$ThreadCleanup$cleanup(/*BlinkAppC.TinyThread2*/ThreadC$3$THREAD_ID);
#line 37
      break;
#line 37
    default:
#line 37
      ThreadMapP$StaticThreadCleanup$default$cleanup(arg_0x1a78760);
#line 37
      break;
#line 37
    }
#line 37
}
#line 37
# 87 "/Users/doina/tinyos-2.x/tos/lib/tosthreads/system/TinyThreadSchedulerP.nc"
static __attribute((noinline)) void TinyThreadSchedulerP$restoreThread(void )
#line 87
{
   __asm ("mov.w %0,r1" :  : "m"(TinyThreadSchedulerP$current_thread->stack_ptr)); __asm ("mov.w %0,r2" :  : "r"(TinyThreadSchedulerP$current_thread->regs.status)); __asm ("mov.w %0,r4" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r4)); __asm ("mov.w %0,r5" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r5)); __asm ("mov.w %0,r6" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r6)); __asm ("mov.w %0,r7" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r7)); __asm ("mov.w %0,r8" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r8)); __asm ("mov.w %0,r9" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r9)); __asm ("mov.w %0,r10" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r10)); __asm ("mov.w %0,r11" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r11)); __asm ("mov.w %0,r12" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r12)); __asm ("mov.w %0,r13" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r13)); __asm ("mov.w %0,r14" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r14)); __asm ("mov.w %0,r15" :  : "m"(TinyThreadSchedulerP$current_thread->regs.r15));}

