#include "lib/msp430_hw.h"
#include "lib/libc_string.h"

extern nondet_bv();

typedef int ptrdiff_t;

typedef int unsigned size_t;

typedef int wchar_t;

struct __nesc_attr_nonnull ;

struct __nesc_attr_bnd {
  void *lo;
  void *hi;
};

struct __nesc_attr_bnd_nok {
  void *lo;
  void *hi;
};

struct __nesc_attr_count {
  int n;
};

struct __nesc_attr_count_nok {
  int n;
};

struct __nesc_attr_one ;

struct __nesc_attr_one_nok ;

struct __nesc_attr_dmemset {
  int a1;
  int a2;
  int a3;
};

struct __nesc_attr_dmemcpy {
  int a1;
  int a2;
  int a3;
};

struct __nesc_attr_nts ;

typedef char signed int8_t;

typedef char unsigned uint8_t;

typedef int int16_t;

typedef int unsigned uint16_t;

typedef long int32_t;

typedef long unsigned uint32_t;

typedef long long int64_t;

typedef long long unsigned uint64_t;

typedef int16_t intptr_t;

typedef uint16_t uintptr_t;

typedef struct {
  char unsigned (data[1]);
} __attribute((packed)) nx_int8_t;

typedef int8_t __nesc_nxbase_nx_int8_t;

typedef struct {
  char unsigned (data[2]);
} __attribute((packed)) nx_int16_t;

typedef int16_t __nesc_nxbase_nx_int16_t;

typedef struct {
  char unsigned (data[4]);
} __attribute((packed)) nx_int32_t;

typedef int32_t __nesc_nxbase_nx_int32_t;

typedef struct {
  char unsigned (data[8]);
} __attribute((packed)) nx_int64_t;

typedef int64_t __nesc_nxbase_nx_int64_t;

typedef struct {
  char unsigned (data[1]);
} __attribute((packed)) nx_uint8_t;

typedef uint8_t __nesc_nxbase_nx_uint8_t;

typedef struct {
  char unsigned (data[2]);
} __attribute((packed)) nx_uint16_t;

typedef uint16_t __nesc_nxbase_nx_uint16_t;

typedef struct {
  char unsigned (data[4]);
} __attribute((packed)) nx_uint32_t;

typedef uint32_t __nesc_nxbase_nx_uint32_t;

typedef struct {
  char unsigned (data[8]);
} __attribute((packed)) nx_uint64_t;

typedef uint64_t __nesc_nxbase_nx_uint64_t;

typedef struct {
  char unsigned (data[1]);
} __attribute((packed)) nxle_int8_t;

typedef int8_t __nesc_nxbase_nxle_int8_t;

typedef struct {
  char unsigned (data[2]);
} __attribute((packed)) nxle_int16_t;

typedef int16_t __nesc_nxbase_nxle_int16_t;

typedef struct {
  char unsigned (data[4]);
} __attribute((packed)) nxle_int32_t;

typedef int32_t __nesc_nxbase_nxle_int32_t;

typedef struct {
  char unsigned (data[8]);
} __attribute((packed)) nxle_int64_t;

typedef int64_t __nesc_nxbase_nxle_int64_t;

typedef struct {
  char unsigned (data[1]);
} __attribute((packed)) nxle_uint8_t;

typedef uint8_t __nesc_nxbase_nxle_uint8_t;

typedef struct {
  char unsigned (data[2]);
} __attribute((packed)) nxle_uint16_t;

typedef uint16_t __nesc_nxbase_nxle_uint16_t;

typedef struct {
  char unsigned (data[4]);
} __attribute((packed)) nxle_uint32_t;

typedef uint32_t __nesc_nxbase_nxle_uint32_t;

typedef struct {
  char unsigned (data[8]);
} __attribute((packed)) nxle_uint64_t;

typedef uint64_t __nesc_nxbase_nxle_uint64_t;

typedef char unsigned u_char;

typedef short unsigned u_short;

typedef int unsigned u_int;

typedef long unsigned u_long;

typedef short unsigned ushort;

typedef int unsigned uint;

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

extern void *memset(void *arg_0x702690, int arg_0x7027e8, size_t arg_0x702980) ;

extern void *memset(void *arg_0x1045650, int arg_0x10457a8, size_t arg_0x1045940) ;

typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;

typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;

typedef int long __int32_t;

typedef int long unsigned __uint32_t;

typedef long _off_t;

typedef long _ssize_t;

typedef __uint32_t __ULong;

struct _glue {
  struct _glue  *_next;
  int _niobs;
  struct __sFILE  *_iobs;
};

struct _Bigint {
  struct _Bigint  *_next;
  int _k;
  int _maxwds;
  int _sign;
  int _wds;
  __ULong (_x[1]);
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
  struct _atexit  *_next;
  int _ind;
  void (*_fns[32])( void ) ;
};

struct __sbuf {
  char unsigned *_base;
  int _size;
};

typedef long _fpos_t;

struct __sFILE {
  char unsigned *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf  _bf;
  int _lbfsize;
  void *_cookie;
  int *_read(void *_cookie, char *_buf, int _n) ;
  int *_write(void *_cookie, const char *_buf, int _n) ;
  _fpos_t *_seek(void *_cookie, _fpos_t _offset, int _whence) ;
  int *_close(void *_cookie) ;
  struct __sbuf  _ub;
  char unsigned *_up;
  int _ur;
  char unsigned (_ubuf[3]);
  char unsigned (_nbuf[1]);
  struct __sbuf  _lb;
  int _blksize;
  int _offset;
  struct _reent  *_data;
};

struct _rand48 {
  short unsigned (_seed[3]);
  short unsigned (_mult[3]);
  short unsigned _add;
};

struct _reent {
  int _errno;
  struct __sFILE  *_stdin;
  struct __sFILE  *_stdout;
  struct __sFILE  *_stderr;
  int _inc;
  char (_emergency[25]);
  int _current_category;
  const char *_current_locale;
  int __sdidinit;
  void *__cleanup(struct _reent  *arg_0x1073ab8) ;
  struct _Bigint  *_result;
  int _result_k;
  struct _Bigint  *_p5s;
  struct _Bigint  *_freelist;
  int _cvtlen;
  char *_cvtbuf;
  union __nesc_unnamed4244  {
  struct __nesc_unnamed4245 {
  int unsigned _unused_rand;
  char *_strtok_last;
  char (_asctime_buf[26]);
  struct __tm  _localtime_buf;
  int _gamma_signgam;
  __extension__ long long unsigned _rand_next;
  struct _rand48  _r48;
} _reent;
  struct __nesc_unnamed4246 {
  char unsigned (*_nextf[30]);
  int unsigned (_nmalloc[30]);
} _unused;
} _new;
  struct _atexit  *_atexit;
  struct _atexit  _atexit0;
  void *_sig_func(int arg_0x1077c98) ;
  struct _glue  __sglue;
  struct __sFILE  (__sf[3]);
};

struct _reent ;

union __dmath  {
  __uint32_t (i[2]);
  double d;
};

union __dmath ;

struct exception {
  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};

enum __fdlibm_version { __fdlibm_ieee = -1, __fdlibm_svid, __fdlibm_xopen, __fdlibm_posix };

enum __fdlibm_version;

typedef uint8_t bool;

enum __nesc_unnamed4247 { FALSE = 0, TRUE = 1 };

typedef nx_int8_t nx_bool;

struct __nesc_attr_atmostonce ;

struct __nesc_attr_atleastonce ;

struct __nesc_attr_exactlyonce ;

enum __nesc_unnamed4248 { SUCCESS = 0, FAIL = 1, ESIZE = 2, ECANCEL = 3, EOFF = 4, EBUSY = 5, EINVAL = 6, ERETRY = 7, ERESERVE = 8, EALREADY = 9, ENOMEM = 10, ENOACK = 11, ELAST = 11 };

typedef uint8_t error_t;

inline static error_t ecombine(error_t r1, error_t r2) ;

typedef union port  {
  volatile char unsigned reg_p;
  volatile struct __nesc_unnamed4249 {
  unsigned char __p0 :1;
  char unsigned __p1 :1;
  unsigned char __p2 :1;
  char unsigned __p3 :1;
  unsigned char __p4 :1;
  char unsigned __p5 :1;
  unsigned char __p6 :1;
  char unsigned __p7 :1;
} __pin;
} ioregister_t;

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

struct port_full_t ;

struct port_full_t ;

struct port_simple_t ;

struct port_simple_t ;

struct port_simple_t ;

struct port_simple_t ;

typedef struct __nesc_unnamed4250 {
  volatile unsigned taifg :1;
  volatile unsigned taie :1;
  volatile unsigned taclr :1;
  volatile unsigned dummy :1;
  volatile unsigned tamc :2;
  volatile unsigned taid :2;
  volatile unsigned tassel :2;
} __attribute((packed)) tactl_t;

typedef struct __nesc_unnamed4251 {
  volatile unsigned ccifg :1;
  volatile unsigned cov :1;
  volatile unsigned out :1;
  volatile unsigned cci :1;
  volatile unsigned ccie :1;
  volatile unsigned outmod :3;
  volatile unsigned cap :1;
  volatile unsigned dummy :1;
  volatile unsigned scci :1;
  volatile unsigned scs :1;
  volatile unsigned ccis :2;
  volatile unsigned cm :2;
} __attribute((packed)) tacctl_t;

struct timera_t {
  tactl_t ctl;
  tacctl_t cctl0;
  tacctl_t cctl1;
  tacctl_t cctl2;
  volatile unsigned (dummy[4]);
  volatile unsigned tar;
  volatile unsigned taccr0;
  volatile unsigned taccr1;
  volatile unsigned taccr2;
};

struct timera_t ;

typedef struct __nesc_unnamed4252 {
  volatile unsigned tbifg :1;
  volatile unsigned tbie :1;
  volatile unsigned tbclr :1;
  volatile unsigned dummy1 :1;
  volatile unsigned tbmc :2;
  volatile unsigned tbid :2;
  volatile unsigned tbssel :2;
  volatile unsigned dummy2 :1;
  volatile unsigned tbcntl :2;
  volatile unsigned tbclgrp :2;
} __attribute((packed)) tbctl_t;

typedef struct __nesc_unnamed4253 {
  volatile unsigned ccifg :1;
  volatile unsigned cov :1;
  volatile unsigned out :1;
  volatile unsigned cci :1;
  volatile unsigned ccie :1;
  volatile unsigned outmod :3;
  volatile unsigned cap :1;
  volatile unsigned clld :2;
  volatile unsigned scs :1;
  volatile unsigned ccis :2;
  volatile unsigned cm :2;
} __attribute((packed)) tbcctl_t;

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

struct timerb_t ;

typedef struct __nesc_unnamed4254 {
  volatile unsigned adc12sc :1;
  volatile unsigned enc :1;
  volatile unsigned adc12tovie :1;
  volatile unsigned adc12ovie :1;
  volatile unsigned adc12on :1;
  volatile unsigned refon :1;
  volatile unsigned r2_5v :1;
  volatile unsigned msc :1;
  volatile unsigned sht0 :4;
  volatile unsigned sht1 :4;
} __attribute((packed)) adc12ctl0_t;

typedef struct __nesc_unnamed4255 {
  volatile unsigned adc12busy :1;
  volatile unsigned conseq :2;
  volatile unsigned adc12ssel :2;
  volatile unsigned adc12div :3;
  volatile unsigned issh :1;
  volatile unsigned shp :1;
  volatile unsigned shs :2;
  volatile unsigned cstartadd :4;
} __attribute((packed)) adc12ctl1_t;

typedef struct __nesc_unnamed4256 {
  volatile unsigned bit0 :1;
  volatile unsigned bit1 :1;
  volatile unsigned bit2 :1;
  volatile unsigned bit3 :1;
  volatile unsigned bit4 :1;
  volatile unsigned bit5 :1;
  volatile unsigned bit6 :1;
  volatile unsigned bit7 :1;
  volatile unsigned bit8 :1;
  volatile unsigned bit9 :1;
  volatile unsigned bit10 :1;
  volatile unsigned bit11 :1;
  volatile unsigned bit12 :1;
  volatile unsigned bit13 :1;
  volatile unsigned bit14 :1;
  volatile unsigned bit15 :1;
} __attribute((packed)) adc12xflg_t;

struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};

struct adc12_t ;

typedef uint8_t mcu_power_t;

inline static mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2) ;

enum __nesc_unnamed4257 { MSP430_POWER_ACTIVE = 0, MSP430_POWER_LPM0 = 1, MSP430_POWER_LPM1 = 2, MSP430_POWER_LPM2 = 3, MSP430_POWER_LPM3 = 4, MSP430_POWER_LPM4 = 5 };

inline static void __nesc_disable_interrupt( void ) ;

inline static void __nesc_enable_interrupt( void ) ;

typedef bool __nesc_atomic_t;

__nesc_atomic_t __nesc_atomic_start( void ) ;

void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts) ;

__nesc_atomic_t __nesc_atomic_start( void ) ;

void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts) ;

typedef struct {
  char unsigned (data[4]);
} __attribute((packed)) nx_float;

typedef float __nesc_nxbase_nx_float;

inline static void TOSH_SET_SIMO0_PIN() ;

inline static void TOSH_CLR_SIMO0_PIN() ;

inline static void TOSH_MAKE_SIMO0_OUTPUT() ;

inline static void TOSH_SET_UCLK0_PIN() ;

inline static void TOSH_CLR_UCLK0_PIN() ;

inline static void TOSH_MAKE_UCLK0_OUTPUT() ;

enum __nesc_unnamed4258 { TOSH_HUMIDITY_ADDR = 5, TOSH_HUMIDTEMP_ADDR = 3, TOSH_HUMIDITY_RESET = 0x1E };

inline static void TOSH_SET_FLASH_CS_PIN() ;

inline static void TOSH_CLR_FLASH_CS_PIN() ;

inline static void TOSH_MAKE_FLASH_CS_OUTPUT() ;

inline static void TOSH_SET_FLASH_HOLD_PIN() ;

inline static void TOSH_MAKE_FLASH_HOLD_OUTPUT() ;

typedef struct __nesc_unnamed4259 {
  int notUsed;
} TMilli;

typedef struct __nesc_unnamed4260 {
  int notUsed;
} T32khz;

typedef struct __nesc_unnamed4261 {
  int notUsed;
} TMicro;

enum __nesc_unnamed4262 { LEDS_LED0 = 1 << 0, LEDS_LED1 = 1 << 1, LEDS_LED2 = 1 << 2, LEDS_LED3 = 1 << 3, LEDS_LED4 = 1 << 4, LEDS_LED5 = 1 << 5, LEDS_LED6 = 1 << 6, LEDS_LED7 = 1 << 7 };

enum __nesc_unnamed4263 { MSP430TIMER_CM_NONE = 0, MSP430TIMER_CM_RISING = 1, MSP430TIMER_CM_FALLING = 2, MSP430TIMER_CM_BOTH = 3, MSP430TIMER_STOP_MODE = 0, MSP430TIMER_UP_MODE = 1, MSP430TIMER_CONTINUOUS_MODE = 2, MSP430TIMER_UPDOWN_MODE = 3, MSP430TIMER_TACLK = 0, MSP430TIMER_TBCLK = 0, MSP430TIMER_ACLK = 1, MSP430TIMER_SMCLK = 2, MSP430TIMER_INCLK = 3, MSP430TIMER_CLOCKDIV_1 = 0, MSP430TIMER_CLOCKDIV_2 = 1, MSP430TIMER_CLOCKDIV_4 = 2, MSP430TIMER_CLOCKDIV_8 = 3 };

typedef struct __nesc_unnamed4264 {
  int ccifg :1;
  int cov :1;
  int out :1;
  int cci :1;
  int ccie :1;
  int outmod :3;
  int cap :1;
  int clld :2;
  int scs :1;
  int ccis :2;
  int cm :2;
} msp430_compare_control_t;

typedef struct __nesc_unnamed4265 {
  int taifg :1;
  int taie :1;
  int taclr :1;
  int _unused0 :1;
  int mc :2;
  int id :2;
  int tassel :2;
  int _unused1 :6;
} msp430_timer_a_control_t;

typedef struct __nesc_unnamed4266 {
  int tbifg :1;
  int tbie :1;
  int tbclr :1;
  int _unused0 :1;
  int mc :2;
  int id :2;
  int tbssel :2;
  int _unused1 :1;
  int cntl :2;
  int tbclgrp :2;
  int _unused2 :1;
} msp430_timer_b_control_t;

typedef struct __nesc_unnamed4267 {
  int unsigned inch :4;
  int unsigned sref :3;
  int unsigned ref2_5v :1;
  int unsigned adc12ssel :2;
  int unsigned adc12div :3;
  int unsigned sht :4;
  int unsigned sampcon_ssel :2;
  int unsigned sampcon_id :2;
} msp430adc12_channel_config_t;

typedef struct __nesc_unnamed4268 {
  volatile unsigned inch :4;
  volatile unsigned sref :3;
  volatile unsigned eos :1;
} __attribute((packed)) adc12memctl_t;

enum inch_enum { INPUT_CHANNEL_A0 = 0, INPUT_CHANNEL_A1 = 1, INPUT_CHANNEL_A2 = 2, INPUT_CHANNEL_A3 = 3, INPUT_CHANNEL_A4 = 4, INPUT_CHANNEL_A5 = 5, INPUT_CHANNEL_A6 = 6, INPUT_CHANNEL_A7 = 7, EXTERNAL_REF_VOLTAGE_CHANNEL = 8, REF_VOLTAGE_NEG_TERMINAL_CHANNEL = 9, TEMPERATURE_DIODE_CHANNEL = 10, SUPPLY_VOLTAGE_HALF_CHANNEL = 11, INPUT_CHANNEL_NONE = 12 };

enum sref_enum { REFERENCE_AVcc_AVss = 0, REFERENCE_VREFplus_AVss = 1, REFERENCE_VeREFplus_AVss = 2, REFERENCE_AVcc_VREFnegterm = 4, REFERENCE_VREFplus_VREFnegterm = 5, REFERENCE_VeREFplus_VREFnegterm = 6 };

enum ref2_5v_enum { REFVOLT_LEVEL_1_5 = 0, REFVOLT_LEVEL_2_5 = 1, REFVOLT_LEVEL_NONE = 0 };

enum adc12ssel_enum { SHT_SOURCE_ADC12OSC = 0, SHT_SOURCE_ACLK = 1, SHT_SOURCE_MCLK = 2, SHT_SOURCE_SMCLK = 3 };

enum adc12div_enum { SHT_CLOCK_DIV_1 = 0, SHT_CLOCK_DIV_2 = 1, SHT_CLOCK_DIV_3 = 2, SHT_CLOCK_DIV_4 = 3, SHT_CLOCK_DIV_5 = 4, SHT_CLOCK_DIV_6 = 5, SHT_CLOCK_DIV_7 = 6, SHT_CLOCK_DIV_8 = 7 };

enum sht_enum { SAMPLE_HOLD_4_CYCLES = 0, SAMPLE_HOLD_8_CYCLES = 1, SAMPLE_HOLD_16_CYCLES = 2, SAMPLE_HOLD_32_CYCLES = 3, SAMPLE_HOLD_64_CYCLES = 4, SAMPLE_HOLD_96_CYCLES = 5, SAMPLE_HOLD_123_CYCLES = 6, SAMPLE_HOLD_192_CYCLES = 7, SAMPLE_HOLD_256_CYCLES = 8, SAMPLE_HOLD_384_CYCLES = 9, SAMPLE_HOLD_512_CYCLES = 10, SAMPLE_HOLD_768_CYCLES = 11, SAMPLE_HOLD_1024_CYCLES = 12 };

enum sampcon_ssel_enum { SAMPCON_SOURCE_TACLK = 0, SAMPCON_SOURCE_ACLK = 1, SAMPCON_SOURCE_SMCLK = 2, SAMPCON_SOURCE_INCLK = 3 };

enum sampcon_id_enum { SAMPCON_CLOCK_DIV_1 = 0, SAMPCON_CLOCK_DIV_2 = 1, SAMPCON_CLOCK_DIV_3 = 2, SAMPCON_CLOCK_DIV_4 = 3 };

typedef uint8_t resource_client_id_t;

typedef uint16_t SenseC_Read_val_t;

typedef TMilli SenseC_Timer_precision_tag;

enum Msp430Timer32khzC_0___nesc_unnamed4269 { Msp430Timer32khzC_0_ALARM_ID = 0U };

typedef T32khz Msp430AlarmC_0_frequency_tag;

typedef Msp430AlarmC_0_frequency_tag Msp430AlarmC_0_Alarm_precision_tag;

typedef uint16_t Msp430AlarmC_0_Alarm_size_type;

typedef T32khz Msp430CounterC_0_frequency_tag;

typedef Msp430CounterC_0_frequency_tag Msp430CounterC_0_Counter_precision_tag;

typedef uint16_t Msp430CounterC_0_Counter_size_type;

typedef TMilli TransformCounterC_0_to_precision_tag;

typedef uint32_t TransformCounterC_0_to_size_type;

typedef T32khz TransformCounterC_0_from_precision_tag;

typedef uint16_t TransformCounterC_0_from_size_type;

typedef uint32_t TransformCounterC_0_upper_count_type;

typedef TransformCounterC_0_from_precision_tag TransformCounterC_0_CounterFrom_precision_tag;

typedef TransformCounterC_0_from_size_type TransformCounterC_0_CounterFrom_size_type;

typedef TransformCounterC_0_to_precision_tag TransformCounterC_0_Counter_precision_tag;

typedef TransformCounterC_0_to_size_type TransformCounterC_0_Counter_size_type;

typedef TMilli TransformAlarmC_0_to_precision_tag;

typedef uint32_t TransformAlarmC_0_to_size_type;

typedef T32khz TransformAlarmC_0_from_precision_tag;

typedef uint16_t TransformAlarmC_0_from_size_type;

typedef TransformAlarmC_0_to_precision_tag TransformAlarmC_0_Alarm_precision_tag;

typedef TransformAlarmC_0_to_size_type TransformAlarmC_0_Alarm_size_type;

typedef TransformAlarmC_0_from_precision_tag TransformAlarmC_0_AlarmFrom_precision_tag;

typedef TransformAlarmC_0_from_size_type TransformAlarmC_0_AlarmFrom_size_type;

typedef TransformAlarmC_0_to_precision_tag TransformAlarmC_0_Counter_precision_tag;

typedef TransformAlarmC_0_to_size_type TransformAlarmC_0_Counter_size_type;

typedef TMilli AlarmToTimerC_0_precision_tag;

typedef AlarmToTimerC_0_precision_tag AlarmToTimerC_0_Alarm_precision_tag;

typedef uint32_t AlarmToTimerC_0_Alarm_size_type;

typedef AlarmToTimerC_0_precision_tag AlarmToTimerC_0_Timer_precision_tag;

typedef TMilli VirtualizeTimerC_0_precision_tag;

typedef VirtualizeTimerC_0_precision_tag VirtualizeTimerC_0_TimerFrom_precision_tag;

typedef VirtualizeTimerC_0_precision_tag VirtualizeTimerC_0_Timer_precision_tag;

typedef TMilli CounterToLocalTimeC_0_precision_tag;

typedef CounterToLocalTimeC_0_precision_tag CounterToLocalTimeC_0_LocalTime_precision_tag;

typedef CounterToLocalTimeC_0_precision_tag CounterToLocalTimeC_0_Counter_precision_tag;

typedef uint32_t CounterToLocalTimeC_0_Counter_size_type;

typedef uint16_t AdcP_Read_val_t;

typedef uint16_t AdcP_ReadNow_val_t;

typedef const msp430adc12_channel_config_t *AdcP_Config_adc_config_t;

typedef TMilli Msp430RefVoltGeneratorP_SwitchOffTimer_precision_tag;

typedef TMilli Msp430RefVoltGeneratorP_SwitchOnTimer_precision_tag;

typedef const msp430adc12_channel_config_t *Msp430RefVoltArbiterImplP_Config_adc_config_t;

enum Msp430Adc12ClientAutoRVGC_0___nesc_unnamed4270 { Msp430Adc12ClientAutoRVGC_0_ID = 0U };

typedef const msp430adc12_channel_config_t *Msp430Adc12ConfAlertC_0_ConfSub_adc_config_t;

typedef const msp430adc12_channel_config_t *Msp430Adc12ConfAlertC_0_ConfUp_adc_config_t;

enum AdcReadClientC_0___nesc_unnamed4271 { AdcReadClientC_0_CLIENT = 0U };

typedef TMilli AdcStreamP_Alarm_precision_tag;

typedef uint32_t AdcStreamP_Alarm_size_type;

typedef const msp430adc12_channel_config_t *AdcStreamP_AdcConfigure_adc_config_t;

typedef uint16_t AdcStreamP_ReadStream_val_t;

enum Msp430Timer32khzC_1___nesc_unnamed4272 { Msp430Timer32khzC_1_ALARM_ID = 1U };

typedef T32khz Msp430AlarmC_1_frequency_tag;

typedef Msp430AlarmC_1_frequency_tag Msp430AlarmC_1_Alarm_precision_tag;

typedef uint16_t Msp430AlarmC_1_Alarm_size_type;

typedef TMilli TransformAlarmC_1_to_precision_tag;

typedef uint32_t TransformAlarmC_1_to_size_type;

typedef T32khz TransformAlarmC_1_from_precision_tag;

typedef uint16_t TransformAlarmC_1_from_size_type;

typedef TransformAlarmC_1_to_precision_tag TransformAlarmC_1_Alarm_precision_tag;

typedef TransformAlarmC_1_to_size_type TransformAlarmC_1_Alarm_size_type;

typedef TransformAlarmC_1_from_precision_tag TransformAlarmC_1_AlarmFrom_precision_tag;

typedef TransformAlarmC_1_from_size_type TransformAlarmC_1_AlarmFrom_size_type;

typedef TransformAlarmC_1_to_precision_tag TransformAlarmC_1_Counter_precision_tag;

typedef TransformAlarmC_1_to_size_type TransformAlarmC_1_Counter_size_type;

typedef uint16_t ArbitratedReadStreamC_0_val_t;

typedef ArbitratedReadStreamC_0_val_t ArbitratedReadStreamC_0_Service_val_t;

typedef ArbitratedReadStreamC_0_val_t ArbitratedReadStreamC_0_ReadStream_val_t;

enum Msp430Adc12ClientAutoRVGC_1___nesc_unnamed4273 { Msp430Adc12ClientAutoRVGC_1_ID = 1U };

typedef const msp430adc12_channel_config_t *Msp430Adc12ConfAlertC_1_ConfSub_adc_config_t;

typedef const msp430adc12_channel_config_t *Msp430Adc12ConfAlertC_1_ConfUp_adc_config_t;

enum AdcReadStreamClientC_0___nesc_unnamed4274 { AdcReadStreamClientC_0_RSCLIENT = 0U };

typedef const msp430adc12_channel_config_t *Msp430InternalVoltageP_AdcConfigure_adc_config_t;

enum Msp430Adc12ClientAutoRVGC_2___nesc_unnamed4275 { Msp430Adc12ClientAutoRVGC_2_ID = 2U };

typedef const msp430adc12_channel_config_t *Msp430Adc12ConfAlertC_2_ConfSub_adc_config_t;

typedef const msp430adc12_channel_config_t *Msp430Adc12ConfAlertC_2_ConfUp_adc_config_t;

enum AdcReadNowClientC_0___nesc_unnamed4276 { AdcReadNowClientC_0_CLIENT = 1U };

static void SenseC_Boot_booted( void ) ;

static void SenseC_Read_readDone(error_t result, SenseC_Read_val_t val) ;

static void SenseC_Timer_fired( void ) ;

static error_t PlatformP_Init_init( void ) ;

static error_t MotePlatformC_Init_init( void ) ;

static void Msp430ClockP_Msp430ClockInit_defaultInitClocks( void ) ;

static void Msp430ClockP_Msp430ClockInit_default_initTimerB( void ) ;

static void Msp430ClockP_Msp430ClockInit_defaultInitTimerA( void ) ;

static void Msp430ClockP_Msp430ClockInit_default_initTimerA( void ) ;

static void Msp430ClockP_Msp430ClockInit_defaultInitTimerB( void ) ;

static void Msp430ClockP_Msp430ClockInit_default_initClocks( void ) ;

static error_t Msp430ClockP_Init_init( void ) ;

static void Msp430TimerP_0_Timer_clear( void ) ;

static void Msp430TimerP_0_Timer_setClockSource(uint16_t clockSource) ;

static void Msp430TimerP_0_Timer_disableEvents( void ) ;

static void Msp430TimerP_0_Timer_setMode(int mode) ;

static void Msp430TimerP_0_Timer_setInputDivider(uint16_t inputDivider) ;

static void Msp430TimerP_1_VectorTimerX0_fired( void ) ;

static void Msp430TimerP_1_Overflow_fired( void ) ;

static void Msp430TimerP_1_Event_default_fired(uint8_t arg_0x15734f0) ;

static uint16_t Msp430TimerP_1_Timer_get( void ) ;

static bool Msp430TimerP_1_Timer_isOverflowPending( void ) ;

static void Msp430TimerCapComP_0_Control_setControl(msp430_compare_control_t control) ;

static void Msp430TimerCapComP_0_Compare_setEvent(uint16_t time) ;

static void Msp430TimerCapComP_1_Control_setControl(msp430_compare_control_t control) ;

static void Msp430TimerCapComP_1_Compare_setEvent(uint16_t time) ;

static uint16_t Msp430TimerCapComP_3_Capture_getEvent( void ) ;

static void Msp430TimerCapComP_3_Capture_default_captured(uint16_t time) ;

static msp430_compare_control_t Msp430TimerCapComP_3_Control_getControl( void ) ;

static void Msp430TimerCapComP_3_Control_enableEvents( void ) ;

static void Msp430TimerCapComP_3_Control_setControlAsCompare( void ) ;

static void Msp430TimerCapComP_3_Control_disableEvents( void ) ;

static void Msp430TimerCapComP_3_Control_clearPendingInterrupt( void ) ;

static void Msp430TimerCapComP_3_Event_fired( void ) ;

static void Msp430TimerCapComP_3_Compare_setEvent(uint16_t time) ;

static void Msp430TimerCapComP_3_Compare_setEventFromNow(uint16_t delta) ;

static void Msp430TimerCapComP_3_Timer_overflow( void ) ;

static uint16_t Msp430TimerCapComP_4_Capture_getEvent( void ) ;

static void Msp430TimerCapComP_4_Capture_default_captured(uint16_t time) ;

static msp430_compare_control_t Msp430TimerCapComP_4_Control_getControl( void ) ;

static void Msp430TimerCapComP_4_Event_fired( void ) ;

static void Msp430TimerCapComP_4_Compare_default_fired( void ) ;

static void Msp430TimerCapComP_4_Timer_overflow( void ) ;

static uint16_t Msp430TimerCapComP_5_Capture_getEvent( void ) ;

static void Msp430TimerCapComP_5_Capture_default_captured(uint16_t time) ;

static msp430_compare_control_t Msp430TimerCapComP_5_Control_getControl( void ) ;

static void Msp430TimerCapComP_5_Control_enableEvents( void ) ;

static void Msp430TimerCapComP_5_Control_disableEvents( void ) ;

static void Msp430TimerCapComP_5_Control_clearPendingInterrupt( void ) ;

static void Msp430TimerCapComP_5_Event_fired( void ) ;

static void Msp430TimerCapComP_5_Compare_setEvent(uint16_t time) ;

static void Msp430TimerCapComP_5_Compare_setEventFromNow(uint16_t delta) ;

static void Msp430TimerCapComP_5_Timer_overflow( void ) ;

static uint16_t Msp430TimerCapComP_6_Capture_getEvent( void ) ;

static void Msp430TimerCapComP_6_Capture_default_captured(uint16_t time) ;

static msp430_compare_control_t Msp430TimerCapComP_6_Control_getControl( void ) ;

static void Msp430TimerCapComP_6_Event_fired( void ) ;

static void Msp430TimerCapComP_6_Compare_default_fired( void ) ;

static void Msp430TimerCapComP_6_Timer_overflow( void ) ;

static uint16_t Msp430TimerCapComP_7_Capture_getEvent( void ) ;

static void Msp430TimerCapComP_7_Capture_default_captured(uint16_t time) ;

static msp430_compare_control_t Msp430TimerCapComP_7_Control_getControl( void ) ;

static void Msp430TimerCapComP_7_Event_fired( void ) ;

static void Msp430TimerCapComP_7_Compare_default_fired( void ) ;

static void Msp430TimerCapComP_7_Timer_overflow( void ) ;

static uint16_t Msp430TimerCapComP_8_Capture_getEvent( void ) ;

static void Msp430TimerCapComP_8_Capture_default_captured(uint16_t time) ;

static msp430_compare_control_t Msp430TimerCapComP_8_Control_getControl( void ) ;

static void Msp430TimerCapComP_8_Event_fired( void ) ;

static void Msp430TimerCapComP_8_Compare_default_fired( void ) ;

static void Msp430TimerCapComP_8_Timer_overflow( void ) ;

static uint16_t Msp430TimerCapComP_9_Capture_getEvent( void ) ;

static void Msp430TimerCapComP_9_Capture_default_captured(uint16_t time) ;

static msp430_compare_control_t Msp430TimerCapComP_9_Control_getControl( void ) ;

static void Msp430TimerCapComP_9_Event_fired( void ) ;

static void Msp430TimerCapComP_9_Compare_default_fired( void ) ;

static void Msp430TimerCapComP_9_Timer_overflow( void ) ;

static error_t SchedulerBasicP_TaskBasic_postTask(uint8_t arg_0x147bb38) ;

static void SchedulerBasicP_TaskBasic_default_runTask(uint8_t arg_0x147bb38) ;

static void SchedulerBasicP_Scheduler_init( void ) ;

static void SchedulerBasicP_Scheduler_taskLoop( void ) ;

static bool SchedulerBasicP_Scheduler_runNextTask( void ) ;

static mcu_power_t McuSleepC_McuPowerOverride_default_lowestState( void ) ;

static void McuSleepC_McuSleep_sleep( void ) ;

static error_t LedsP_Init_init( void ) ;

static void LedsP_Leds_led0Off( void ) ;

static void LedsP_Leds_led1On( void ) ;

static void LedsP_Leds_led1Off( void ) ;

static void LedsP_Leds_led2Off( void ) ;

static void LedsP_Leds_led0On( void ) ;

static void LedsP_Leds_led2On( void ) ;

static void HplMsp430GeneralIOP_36_IO_makeOutput( void ) ;

static void HplMsp430GeneralIOP_36_IO_set( void ) ;

static void HplMsp430GeneralIOP_36_IO_clr( void ) ;

static void HplMsp430GeneralIOP_37_IO_makeOutput( void ) ;

static void HplMsp430GeneralIOP_37_IO_set( void ) ;

static void HplMsp430GeneralIOP_37_IO_clr( void ) ;

static void HplMsp430GeneralIOP_38_IO_makeOutput( void ) ;

static void HplMsp430GeneralIOP_38_IO_set( void ) ;

static void HplMsp430GeneralIOP_38_IO_clr( void ) ;

static void HplMsp430GeneralIOP_40_IO_makeInput( void ) ;

static void HplMsp430GeneralIOP_40_IO_selectIOFunc( void ) ;

static void HplMsp430GeneralIOP_40_IO_selectModuleFunc( void ) ;

static void HplMsp430GeneralIOP_41_IO_makeInput( void ) ;

static void HplMsp430GeneralIOP_41_IO_selectIOFunc( void ) ;

static void HplMsp430GeneralIOP_41_IO_selectModuleFunc( void ) ;

static void HplMsp430GeneralIOP_42_IO_makeInput( void ) ;

static void HplMsp430GeneralIOP_42_IO_selectIOFunc( void ) ;

static void HplMsp430GeneralIOP_42_IO_selectModuleFunc( void ) ;

static void HplMsp430GeneralIOP_43_IO_makeInput( void ) ;

static void HplMsp430GeneralIOP_43_IO_selectIOFunc( void ) ;

static void HplMsp430GeneralIOP_43_IO_selectModuleFunc( void ) ;

static void HplMsp430GeneralIOP_44_IO_makeInput( void ) ;

static void HplMsp430GeneralIOP_44_IO_selectIOFunc( void ) ;

static void HplMsp430GeneralIOP_44_IO_selectModuleFunc( void ) ;

static void HplMsp430GeneralIOP_45_IO_makeInput( void ) ;

static void HplMsp430GeneralIOP_45_IO_selectIOFunc( void ) ;

static void HplMsp430GeneralIOP_45_IO_selectModuleFunc( void ) ;

static void HplMsp430GeneralIOP_46_IO_makeInput( void ) ;

static void HplMsp430GeneralIOP_46_IO_selectIOFunc( void ) ;

static void HplMsp430GeneralIOP_46_IO_selectModuleFunc( void ) ;

static void HplMsp430GeneralIOP_47_IO_makeInput( void ) ;

static void HplMsp430GeneralIOP_47_IO_selectIOFunc( void ) ;

static void HplMsp430GeneralIOP_47_IO_selectModuleFunc( void ) ;

static void Msp430GpioC_0_GeneralIO_makeOutput( void ) ;

static void Msp430GpioC_0_GeneralIO_set( void ) ;

static void Msp430GpioC_0_GeneralIO_clr( void ) ;

static void Msp430GpioC_1_GeneralIO_makeOutput( void ) ;

static void Msp430GpioC_1_GeneralIO_set( void ) ;

static void Msp430GpioC_1_GeneralIO_clr( void ) ;

static void Msp430GpioC_2_GeneralIO_makeOutput( void ) ;

static void Msp430GpioC_2_GeneralIO_set( void ) ;

static void Msp430GpioC_2_GeneralIO_clr( void ) ;

static void Msp430AlarmC_0_Msp430Compare_fired( void ) ;

static void Msp430AlarmC_0_Msp430Timer_overflow( void ) ;

static void Msp430AlarmC_0_Alarm_startAt(Msp430AlarmC_0_Alarm_size_type t0, Msp430AlarmC_0_Alarm_size_type dt) ;

static void Msp430AlarmC_0_Alarm_stop( void ) ;

static error_t Msp430AlarmC_0_Init_init( void ) ;

static void Msp430CounterC_0_Msp430Timer_overflow( void ) ;

static Msp430CounterC_0_Counter_size_type Msp430CounterC_0_Counter_get( void ) ;

static bool Msp430CounterC_0_Counter_isOverflowPending( void ) ;

static void TransformCounterC_0_CounterFrom_overflow( void ) ;

static TransformCounterC_0_Counter_size_type TransformCounterC_0_Counter_get( void ) ;

static TransformAlarmC_0_Alarm_size_type TransformAlarmC_0_Alarm_getNow( void ) ;

static void TransformAlarmC_0_Alarm_startAt(TransformAlarmC_0_Alarm_size_type t0, TransformAlarmC_0_Alarm_size_type dt) ;

static TransformAlarmC_0_Alarm_size_type TransformAlarmC_0_Alarm_getAlarm( void ) ;

static void TransformAlarmC_0_Alarm_stop( void ) ;

static void TransformAlarmC_0_AlarmFrom_fired( void ) ;

static void TransformAlarmC_0_Counter_overflow( void ) ;

static void AlarmToTimerC_0_fired_runTask( void ) ;

static void AlarmToTimerC_0_Alarm_fired( void ) ;

static uint32_t AlarmToTimerC_0_Timer_getNow( void ) ;

static void AlarmToTimerC_0_Timer_startOneShotAt(uint32_t t0, uint32_t dt) ;

static void AlarmToTimerC_0_Timer_stop( void ) ;

static void VirtualizeTimerC_0_updateFromTimer_runTask( void ) ;

static void VirtualizeTimerC_0_TimerFrom_fired( void ) ;

static void VirtualizeTimerC_0_Timer_default_fired(uint8_t arg_0x18923c8) ;

static void VirtualizeTimerC_0_Timer_startPeriodic(uint8_t arg_0x18923c8, uint32_t dt) ;

static void VirtualizeTimerC_0_Timer_startOneShot(uint8_t arg_0x18923c8, uint32_t dt) ;

static void VirtualizeTimerC_0_Timer_stop(uint8_t arg_0x18923c8) ;

static void CounterToLocalTimeC_0_Counter_overflow( void ) ;

static void AdcP_SubResourceReadNow_granted(uint8_t arg_0x18fe908) ;

static error_t AdcP_Read_read(uint8_t arg_0x1903338) ;

static void AdcP_Read_default_readDone(uint8_t arg_0x1903338, error_t result, AdcP_Read_val_t val) ;

static void AdcP_ReadNow_default_readDone(uint8_t arg_0x1901350, error_t result, AdcP_ReadNow_val_t val) ;

static void AdcP_ResourceReadNow_default_granted(uint8_t arg_0x18ff330) ;

static AdcP_Config_adc_config_t AdcP_Config_default_getConfiguration(uint8_t arg_0x18fc648) ;

static error_t AdcP_SingleChannel_default_getData(uint8_t arg_0x191ee60) ;

static error_t AdcP_SingleChannel_default_configureSingle(uint8_t arg_0x191ee60, const msp430adc12_channel_config_t *config) ;

static uint16_t *AdcP_SingleChannel_multipleDataReady(uint8_t arg_0x191ee60, uint16_t *buffer, uint16_t numSamples) ;

static error_t AdcP_SingleChannel_singleDataReady(uint8_t arg_0x191ee60, uint16_t data) ;

static error_t AdcP_ResourceRead_default_release(uint8_t arg_0x18ffe18) ;

static error_t AdcP_ResourceRead_default_request(uint8_t arg_0x18ffe18) ;

static void AdcP_ResourceRead_granted(uint8_t arg_0x18ffe18) ;

static void AdcP_readDone_runTask( void ) ;

static void Msp430Adc12ImplP_MultiChannel_default_dataReady(uint8_t arg_0x1972108, uint16_t *buffer, uint16_t numSamples) ;

static void Msp430Adc12ImplP_HplAdc12_conversionDone(uint16_t iv) ;

static void Msp430Adc12ImplP_Overflow_default_memOverflow(uint8_t arg_0x19729f8) ;

static void Msp430Adc12ImplP_Overflow_default_conversionTimeOverflow(uint8_t arg_0x19729f8) ;

static error_t Msp430Adc12ImplP_Init_init( void ) ;

static error_t Msp430Adc12ImplP_SingleChannel_getData(uint8_t arg_0x19743f0) ;

static error_t Msp430Adc12ImplP_SingleChannel_configureSingle(uint8_t arg_0x19743f0, const msp430adc12_channel_config_t *config) ;

static uint16_t *Msp430Adc12ImplP_SingleChannel_default_multipleDataReady(uint8_t arg_0x19743f0, uint16_t *buffer, uint16_t numSamples) ;

static error_t Msp430Adc12ImplP_SingleChannel_configureMultiple(uint8_t arg_0x19743f0, const msp430adc12_channel_config_t *config, uint16_t *buffer, uint16_t numSamples, uint16_t jiffies) ;

static error_t Msp430Adc12ImplP_SingleChannel_default_singleDataReady(uint8_t arg_0x19743f0, uint16_t data) ;

static adc12ctl0_t HplAdc12P_HplAdc12_getCtl0( void ) ;

static adc12memctl_t HplAdc12P_HplAdc12_getMCtl(uint8_t idx) ;

static void HplAdc12P_HplAdc12_resetIFGs( void ) ;

static bool HplAdc12P_HplAdc12_isBusy( void ) ;

static void HplAdc12P_HplAdc12_setMCtl(uint8_t idx, adc12memctl_t memControl) ;

static void HplAdc12P_HplAdc12_startConversion( void ) ;

static void HplAdc12P_HplAdc12_setCtl0(adc12ctl0_t control0) ;

static uint16_t HplAdc12P_HplAdc12_getMem(uint8_t idx) ;

static void HplAdc12P_HplAdc12_setIEFlags(uint16_t mask) ;

static void HplAdc12P_HplAdc12_stopConversion( void ) ;

static void HplAdc12P_HplAdc12_setCtl1(adc12ctl1_t control1) ;

static error_t RoundRobinResourceQueueC_0_Init_init( void ) ;

static error_t RoundRobinResourceQueueC_0_RoundRobinQueue_enqueue(resource_client_id_t id) ;

static bool RoundRobinResourceQueueC_0_RoundRobinQueue_isEmpty( void ) ;

static bool RoundRobinResourceQueueC_0_RoundRobinQueue_isEnqueued(resource_client_id_t id) ;

static resource_client_id_t RoundRobinResourceQueueC_0_RoundRobinQueue_dequeue( void ) ;

static void SimpleArbiterP_0_ResourceRequested_default_requested(uint8_t arg_0x1a813f0) ;

static void SimpleArbiterP_0_ResourceConfigure_default_unconfigure(uint8_t arg_0x1a7f030) ;

static void SimpleArbiterP_0_ResourceConfigure_default_configure(uint8_t arg_0x1a7f030) ;

static error_t SimpleArbiterP_0_Resource_release(uint8_t arg_0x1a82930) ;

static error_t SimpleArbiterP_0_Resource_request(uint8_t arg_0x1a82930) ;

static void SimpleArbiterP_0_Resource_default_granted(uint8_t arg_0x1a82930) ;

static uint8_t SimpleArbiterP_0_ArbiterInfo_userId( void ) ;

static void SimpleArbiterP_0_grantedTask_runTask( void ) ;

static void Msp430RefVoltGeneratorP_HplAdc12_conversionDone(uint16_t iv) ;

static void Msp430RefVoltGeneratorP_SwitchOffTimer_fired( void ) ;

static error_t Msp430RefVoltGeneratorP_RefVolt_2_5V_start( void ) ;

static error_t Msp430RefVoltGeneratorP_RefVolt_1_5V_start( void ) ;

static error_t Msp430RefVoltGeneratorP_RefVolt_1_5V_stop( void ) ;

static void Msp430RefVoltGeneratorP_SwitchOnTimer_fired( void ) ;

static Msp430RefVoltArbiterImplP_Config_adc_config_t Msp430RefVoltArbiterImplP_Config_default_getConfiguration(uint8_t arg_0x1af6638) ;

static void Msp430RefVoltArbiterImplP_RefVolt_2_5V_startDone(error_t error) ;

static void Msp430RefVoltArbiterImplP_RefVolt_2_5V_stopDone(error_t error) ;

static error_t Msp430RefVoltArbiterImplP_AdcResource_default_release(uint8_t arg_0x1af9ac8) ;

static error_t Msp430RefVoltArbiterImplP_AdcResource_default_request(uint8_t arg_0x1af9ac8) ;

static void Msp430RefVoltArbiterImplP_AdcResource_granted(uint8_t arg_0x1af9ac8) ;

static error_t Msp430RefVoltArbiterImplP_ClientResource_release(uint8_t arg_0x1af9010) ;

static error_t Msp430RefVoltArbiterImplP_ClientResource_request(uint8_t arg_0x1af9010) ;

static void Msp430RefVoltArbiterImplP_ClientResource_default_granted(uint8_t arg_0x1af9010) ;

static void Msp430RefVoltArbiterImplP_switchOff_runTask( void ) ;

static void Msp430RefVoltArbiterImplP_RefVolt_1_5V_startDone(error_t error) ;

static void Msp430RefVoltArbiterImplP_RefVolt_1_5V_stopDone(error_t error) ;

static Msp430Adc12ConfAlertC_0_ConfSub_adc_config_t Msp430Adc12ConfAlertC_0_ConfSub_getConfiguration( void ) ;

static void AdcStreamP_bufferDone_runTask( void ) ;

static void AdcStreamP_readStreamDone_runTask( void ) ;

static void AdcStreamP_readStreamFail_runTask( void ) ;

static void AdcStreamP_Alarm_fired( void ) ;

static error_t AdcStreamP_Init_init( void ) ;

static AdcStreamP_AdcConfigure_adc_config_t AdcStreamP_AdcConfigure_default_getConfiguration(uint8_t arg_0x1b5cde0) ;

static error_t AdcStreamP_SingleChannel_default_getData(uint8_t arg_0x1b5c010) ;

static error_t AdcStreamP_SingleChannel_default_configureSingle(uint8_t arg_0x1b5c010, const msp430adc12_channel_config_t *config) ;

static uint16_t *AdcStreamP_SingleChannel_multipleDataReady(uint8_t arg_0x1b5c010, uint16_t *buffer, uint16_t numSamples) ;

static error_t AdcStreamP_SingleChannel_default_configureMultiple(uint8_t arg_0x1b5c010, const msp430adc12_channel_config_t *config, uint16_t *buffer, uint16_t numSamples, uint16_t jiffies) ;

static error_t AdcStreamP_SingleChannel_singleDataReady(uint8_t arg_0x1b5c010, uint16_t data) ;

static error_t AdcStreamP_ReadStream_read(uint8_t arg_0x1b5f340, uint32_t usPeriod) ;

static void Msp430AlarmC_1_Msp430Compare_fired( void ) ;

static void Msp430AlarmC_1_Msp430Timer_overflow( void ) ;

static void Msp430AlarmC_1_Alarm_startAt(Msp430AlarmC_1_Alarm_size_type t0, Msp430AlarmC_1_Alarm_size_type dt) ;

static TransformAlarmC_1_Alarm_size_type TransformAlarmC_1_Alarm_getNow( void ) ;

static void TransformAlarmC_1_Alarm_startAt(TransformAlarmC_1_Alarm_size_type t0, TransformAlarmC_1_Alarm_size_type dt) ;

static void TransformAlarmC_1_AlarmFrom_fired( void ) ;

static void TransformAlarmC_1_Counter_overflow( void ) ;

static void ArbitratedReadStreamC_0_Service_bufferDone(uint8_t arg_0x1bc37c8, error_t result, ArbitratedReadStreamC_0_Service_val_t *buf, uint16_t count) ;

static void ArbitratedReadStreamC_0_Service_readDone(uint8_t arg_0x1bc37c8, error_t result, uint32_t usActualPeriod) ;

static void ArbitratedReadStreamC_0_ReadStream_default_bufferDone(uint8_t arg_0x1bc7bb0, error_t result, ArbitratedReadStreamC_0_ReadStream_val_t *buf, uint16_t count) ;

static void ArbitratedReadStreamC_0_ReadStream_default_readDone(uint8_t arg_0x1bc7bb0, error_t result, uint32_t usActualPeriod) ;

static error_t ArbitratedReadStreamC_0_Resource_default_release(uint8_t arg_0x1bbf278) ;

static void ArbitratedReadStreamC_0_Resource_granted(uint8_t arg_0x1bbf278) ;

static Msp430Adc12ConfAlertC_1_ConfSub_adc_config_t Msp430Adc12ConfAlertC_1_ConfSub_getConfiguration( void ) ;

static Msp430InternalVoltageP_AdcConfigure_adc_config_t Msp430InternalVoltageP_AdcConfigure_getConfiguration( void ) ;

static Msp430Adc12ConfAlertC_2_ConfSub_adc_config_t Msp430Adc12ConfAlertC_2_ConfSub_getConfiguration( void ) ;

static error_t SenseC_Read_read( void ) ;

static void SenseC_Leds_led0Off( void ) ;

static void SenseC_Leds_led1On( void ) ;

static void SenseC_Leds_led1Off( void ) ;

static void SenseC_Leds_led2Off( void ) ;

static void SenseC_Leds_led0On( void ) ;

static void SenseC_Leds_led2On( void ) ;

static void SenseC_Timer_startPeriodic(uint32_t dt) ;

inline static void SenseC_Boot_booted( void ) ;

inline static void SenseC_Timer_fired( void ) ;

static void SenseC_Read_readDone(error_t result, uint16_t data) ;

static error_t PlatformP_MoteInit_init( void ) ;

static error_t PlatformP_MoteClockInit_init( void ) ;

static error_t PlatformP_LedsInit_init( void ) ;

inline static error_t PlatformP_Init_init( void ) ;

__inline static void MotePlatformC_TOSH_wait( void ) ;

static void MotePlatformC_TOSH_FLASH_M25P_DP_bit(bool set) ;

inline static void MotePlatformC_TOSH_FLASH_M25P_DP( void ) ;

inline static error_t MotePlatformC_Init_init( void ) ;

static void Msp430ClockP_Msp430ClockInit_initTimerB( void ) ;

static void Msp430ClockP_Msp430ClockInit_initTimerA( void ) ;

static void Msp430ClockP_Msp430ClockInit_initClocks( void ) ;

enum Msp430ClockP___nesc_unnamed4277 { Msp430ClockP_ACLK_CALIB_PERIOD = 8, Msp430ClockP_TARGET_DCO_DELTA = (4096 / 32) * Msp430ClockP_ACLK_CALIB_PERIOD };

inline static void Msp430ClockP_Msp430ClockInit_defaultInitClocks( void ) ;

inline static void Msp430ClockP_Msp430ClockInit_defaultInitTimerA( void ) ;

inline static void Msp430ClockP_Msp430ClockInit_defaultInitTimerB( void ) ;

inline static void Msp430ClockP_Msp430ClockInit_default_initClocks( void ) ;

inline static void Msp430ClockP_Msp430ClockInit_default_initTimerA( void ) ;

inline static void Msp430ClockP_Msp430ClockInit_default_initTimerB( void ) ;

inline static void Msp430ClockP_startTimerA( void ) ;

inline static void Msp430ClockP_startTimerB( void ) ;

inline static error_t Msp430ClockP_Init_init( void ) ;

static void Msp430TimerP_0_Timer_setMode(int mode) ;

inline static void Msp430TimerP_0_Timer_clear( void ) ;

inline static void Msp430TimerP_0_Timer_disableEvents( void ) ;

inline static void Msp430TimerP_0_Timer_setClockSource(uint16_t clockSource) ;

inline static void Msp430TimerP_0_Timer_setInputDivider(uint16_t inputDivider) ;

static void Msp430TimerP_1_Event_fired(uint8_t arg_0x15734f0) ;

static void Msp430TimerP_1_Timer_overflow( void ) ;

static uint16_t Msp430TimerP_1_Timer_get( void ) ;

inline static bool Msp430TimerP_1_Timer_isOverflowPending( void ) ;

inline static void Msp430TimerP_1_VectorTimerX0_fired( void ) ;

inline static void Msp430TimerP_1_Overflow_fired( void ) ;

static void Msp430TimerP_1_Event_default_fired(uint8_t n) ;

typedef msp430_compare_control_t Msp430TimerCapComP_0_cc_t;

inline static uint16_t Msp430TimerCapComP_0_CC2int(Msp430TimerCapComP_0_cc_t x) ;

inline static void Msp430TimerCapComP_0_Control_setControl(Msp430TimerCapComP_0_cc_t x) ;

inline static void Msp430TimerCapComP_0_Compare_setEvent(uint16_t x) ;

typedef msp430_compare_control_t Msp430TimerCapComP_1_cc_t;

inline static uint16_t Msp430TimerCapComP_1_CC2int(Msp430TimerCapComP_1_cc_t x) ;

inline static void Msp430TimerCapComP_1_Control_setControl(Msp430TimerCapComP_1_cc_t x) ;

inline static void Msp430TimerCapComP_1_Compare_setEvent(uint16_t x) ;

typedef msp430_compare_control_t Msp430TimerCapComP_2_cc_t;

static void Msp430TimerCapComP_3_Capture_captured(uint16_t time) ;

static void Msp430TimerCapComP_3_Compare_fired( void ) ;

static uint16_t Msp430TimerCapComP_3_Timer_get( void ) ;

typedef msp430_compare_control_t Msp430TimerCapComP_3_cc_t;

inline static uint16_t Msp430TimerCapComP_3_CC2int(Msp430TimerCapComP_3_cc_t x) ;

inline static Msp430TimerCapComP_3_cc_t Msp430TimerCapComP_3_int2CC(uint16_t x) ;

inline static uint16_t Msp430TimerCapComP_3_compareControl( void ) ;

inline static Msp430TimerCapComP_3_cc_t Msp430TimerCapComP_3_Control_getControl( void ) ;

inline static void Msp430TimerCapComP_3_Control_clearPendingInterrupt( void ) ;

inline static void Msp430TimerCapComP_3_Control_setControlAsCompare( void ) ;

inline static void Msp430TimerCapComP_3_Control_enableEvents( void ) ;

inline static void Msp430TimerCapComP_3_Control_disableEvents( void ) ;

inline static uint16_t Msp430TimerCapComP_3_Capture_getEvent( void ) ;

inline static void Msp430TimerCapComP_3_Compare_setEvent(uint16_t x) ;

inline static void Msp430TimerCapComP_3_Compare_setEventFromNow(uint16_t x) ;

inline static void Msp430TimerCapComP_3_Event_fired( void ) ;

inline static void Msp430TimerCapComP_3_Capture_default_captured(uint16_t n) ;

inline static void Msp430TimerCapComP_3_Timer_overflow( void ) ;

static void Msp430TimerCapComP_4_Capture_captured(uint16_t time) ;

static void Msp430TimerCapComP_4_Compare_fired( void ) ;

typedef msp430_compare_control_t Msp430TimerCapComP_4_cc_t;

inline static Msp430TimerCapComP_4_cc_t Msp430TimerCapComP_4_int2CC(uint16_t x) ;

inline static Msp430TimerCapComP_4_cc_t Msp430TimerCapComP_4_Control_getControl( void ) ;

inline static uint16_t Msp430TimerCapComP_4_Capture_getEvent( void ) ;

inline static void Msp430TimerCapComP_4_Event_fired( void ) ;

inline static void Msp430TimerCapComP_4_Capture_default_captured(uint16_t n) ;

inline static void Msp430TimerCapComP_4_Compare_default_fired( void ) ;

inline static void Msp430TimerCapComP_4_Timer_overflow( void ) ;

static void Msp430TimerCapComP_5_Capture_captured(uint16_t time) ;

static void Msp430TimerCapComP_5_Compare_fired( void ) ;

static uint16_t Msp430TimerCapComP_5_Timer_get( void ) ;

typedef msp430_compare_control_t Msp430TimerCapComP_5_cc_t;

inline static Msp430TimerCapComP_5_cc_t Msp430TimerCapComP_5_int2CC(uint16_t x) ;

inline static Msp430TimerCapComP_5_cc_t Msp430TimerCapComP_5_Control_getControl( void ) ;

inline static void Msp430TimerCapComP_5_Control_clearPendingInterrupt( void ) ;

inline static void Msp430TimerCapComP_5_Control_enableEvents( void ) ;

inline static void Msp430TimerCapComP_5_Control_disableEvents( void ) ;

inline static uint16_t Msp430TimerCapComP_5_Capture_getEvent( void ) ;

inline static void Msp430TimerCapComP_5_Compare_setEvent(uint16_t x) ;

inline static void Msp430TimerCapComP_5_Compare_setEventFromNow(uint16_t x) ;

inline static void Msp430TimerCapComP_5_Event_fired( void ) ;

inline static void Msp430TimerCapComP_5_Capture_default_captured(uint16_t n) ;

inline static void Msp430TimerCapComP_5_Timer_overflow( void ) ;

static void Msp430TimerCapComP_6_Capture_captured(uint16_t time) ;

static void Msp430TimerCapComP_6_Compare_fired( void ) ;

typedef msp430_compare_control_t Msp430TimerCapComP_6_cc_t;

inline static Msp430TimerCapComP_6_cc_t Msp430TimerCapComP_6_int2CC(uint16_t x) ;

inline static Msp430TimerCapComP_6_cc_t Msp430TimerCapComP_6_Control_getControl( void ) ;

inline static uint16_t Msp430TimerCapComP_6_Capture_getEvent( void ) ;

inline static void Msp430TimerCapComP_6_Event_fired( void ) ;

inline static void Msp430TimerCapComP_6_Capture_default_captured(uint16_t n) ;

inline static void Msp430TimerCapComP_6_Compare_default_fired( void ) ;

inline static void Msp430TimerCapComP_6_Timer_overflow( void ) ;

static void Msp430TimerCapComP_7_Capture_captured(uint16_t time) ;

static void Msp430TimerCapComP_7_Compare_fired( void ) ;

typedef msp430_compare_control_t Msp430TimerCapComP_7_cc_t;

inline static Msp430TimerCapComP_7_cc_t Msp430TimerCapComP_7_int2CC(uint16_t x) ;

inline static Msp430TimerCapComP_7_cc_t Msp430TimerCapComP_7_Control_getControl( void ) ;

inline static uint16_t Msp430TimerCapComP_7_Capture_getEvent( void ) ;

inline static void Msp430TimerCapComP_7_Event_fired( void ) ;

inline static void Msp430TimerCapComP_7_Capture_default_captured(uint16_t n) ;

inline static void Msp430TimerCapComP_7_Compare_default_fired( void ) ;

inline static void Msp430TimerCapComP_7_Timer_overflow( void ) ;

static void Msp430TimerCapComP_8_Capture_captured(uint16_t time) ;

static void Msp430TimerCapComP_8_Compare_fired( void ) ;

typedef msp430_compare_control_t Msp430TimerCapComP_8_cc_t;

inline static Msp430TimerCapComP_8_cc_t Msp430TimerCapComP_8_int2CC(uint16_t x) ;

inline static Msp430TimerCapComP_8_cc_t Msp430TimerCapComP_8_Control_getControl( void ) ;

inline static uint16_t Msp430TimerCapComP_8_Capture_getEvent( void ) ;

inline static void Msp430TimerCapComP_8_Event_fired( void ) ;

inline static void Msp430TimerCapComP_8_Capture_default_captured(uint16_t n) ;

inline static void Msp430TimerCapComP_8_Compare_default_fired( void ) ;

inline static void Msp430TimerCapComP_8_Timer_overflow( void ) ;

static void Msp430TimerCapComP_9_Capture_captured(uint16_t time) ;

static void Msp430TimerCapComP_9_Compare_fired( void ) ;

typedef msp430_compare_control_t Msp430TimerCapComP_9_cc_t;

inline static Msp430TimerCapComP_9_cc_t Msp430TimerCapComP_9_int2CC(uint16_t x) ;

inline static Msp430TimerCapComP_9_cc_t Msp430TimerCapComP_9_Control_getControl( void ) ;

inline static uint16_t Msp430TimerCapComP_9_Capture_getEvent( void ) ;

inline static void Msp430TimerCapComP_9_Event_fired( void ) ;

inline static void Msp430TimerCapComP_9_Capture_default_captured(uint16_t n) ;

inline static void Msp430TimerCapComP_9_Compare_default_fired( void ) ;

inline static void Msp430TimerCapComP_9_Timer_overflow( void ) ;

static void Msp430TimerCommonP_VectorTimerB0_fired( void ) ;

void sig_TIMERB0_VECTOR( void ) ;

static error_t RealMainP_SoftwareInit_init( void ) ;

static void RealMainP_Boot_booted( void ) ;

static error_t RealMainP_PlatformInit_init( void ) ;

static void RealMainP_Scheduler_init( void ) ;

static void RealMainP_Scheduler_taskLoop( void ) ;

static bool RealMainP_Scheduler_runNextTask( void ) ;

int main( void ) ;

static void SchedulerBasicP_TaskBasic_runTask(uint8_t arg_0x147bb38) ;

static void SchedulerBasicP_McuSleep_sleep( void ) ;

enum SchedulerBasicP___nesc_unnamed4278 { SchedulerBasicP_NUM_TASKS = 8U, SchedulerBasicP_NO_TASK = 255 };

uint8_t SchedulerBasicP_m_head;

uint8_t SchedulerBasicP_m_tail;

uint8_t (SchedulerBasicP_m_next[SchedulerBasicP_NUM_TASKS]);

__inline static uint8_t SchedulerBasicP_popTask( void ) ;

inline static bool SchedulerBasicP_isWaiting(uint8_t id) ;

inline static bool SchedulerBasicP_pushTask(uint8_t id) ;

inline static void SchedulerBasicP_Scheduler_init( void ) ;

static bool SchedulerBasicP_Scheduler_runNextTask( void ) ;

inline static void SchedulerBasicP_Scheduler_taskLoop( void ) ;

static error_t SchedulerBasicP_TaskBasic_postTask(uint8_t id) ;

static void SchedulerBasicP_TaskBasic_default_runTask(uint8_t id) ;

static mcu_power_t McuSleepC_McuPowerOverride_lowestState( void ) ;

bool McuSleepC_dirty = TRUE;

mcu_power_t McuSleepC_powerState = MSP430_POWER_ACTIVE;

const uint16_t (McuSleepC_msp430PowerBits[MSP430_POWER_LPM4 + 1]) = {0, 0x0010, 0x0040 + 0x0010, 0x0080 + 0x0010, (0x0080 + 0x0040) + 0x0010, ((0x0080 + 0x0040) + 0x0020) + 0x0010};

inline static mcu_power_t McuSleepC_getPowerState( void ) ;

inline static void McuSleepC_computePowerState( void ) ;

inline static void McuSleepC_McuSleep_sleep( void ) ;

inline static mcu_power_t McuSleepC_McuPowerOverride_default_lowestState( void ) ;

static void LedsP_Led0_makeOutput( void ) ;

static void LedsP_Led0_set( void ) ;

static void LedsP_Led0_clr( void ) ;

static void LedsP_Led1_makeOutput( void ) ;

static void LedsP_Led1_set( void ) ;

static void LedsP_Led1_clr( void ) ;

static void LedsP_Led2_makeOutput( void ) ;

static void LedsP_Led2_set( void ) ;

static void LedsP_Led2_clr( void ) ;

inline static error_t LedsP_Init_init( void ) ;

inline static void LedsP_Leds_led0On( void ) ;

inline static void LedsP_Leds_led0Off( void ) ;

inline static void LedsP_Leds_led1On( void ) ;

inline static void LedsP_Leds_led1Off( void ) ;

inline static void LedsP_Leds_led2On( void ) ;

inline static void LedsP_Leds_led2Off( void ) ;

static void HplMsp430GeneralIOP_36_IO_set( void ) ;

inline static void HplMsp430GeneralIOP_36_IO_clr( void ) ;

inline static void HplMsp430GeneralIOP_36_IO_makeOutput( void ) ;

static void HplMsp430GeneralIOP_37_IO_set( void ) ;

inline static void HplMsp430GeneralIOP_37_IO_clr( void ) ;

inline static void HplMsp430GeneralIOP_37_IO_makeOutput( void ) ;

static void HplMsp430GeneralIOP_38_IO_set( void ) ;

inline static void HplMsp430GeneralIOP_38_IO_clr( void ) ;

inline static void HplMsp430GeneralIOP_38_IO_makeOutput( void ) ;

inline static void HplMsp430GeneralIOP_40_IO_makeInput( void ) ;

inline static void HplMsp430GeneralIOP_40_IO_selectModuleFunc( void ) ;

inline static void HplMsp430GeneralIOP_40_IO_selectIOFunc( void ) ;

inline static void HplMsp430GeneralIOP_41_IO_makeInput( void ) ;

inline static void HplMsp430GeneralIOP_41_IO_selectModuleFunc( void ) ;

inline static void HplMsp430GeneralIOP_41_IO_selectIOFunc( void ) ;

inline static void HplMsp430GeneralIOP_42_IO_makeInput( void ) ;

inline static void HplMsp430GeneralIOP_42_IO_selectModuleFunc( void ) ;

inline static void HplMsp430GeneralIOP_42_IO_selectIOFunc( void ) ;

inline static void HplMsp430GeneralIOP_43_IO_makeInput( void ) ;

inline static void HplMsp430GeneralIOP_43_IO_selectModuleFunc( void ) ;

inline static void HplMsp430GeneralIOP_43_IO_selectIOFunc( void ) ;

inline static void HplMsp430GeneralIOP_44_IO_makeInput( void ) ;

inline static void HplMsp430GeneralIOP_44_IO_selectModuleFunc( void ) ;

inline static void HplMsp430GeneralIOP_44_IO_selectIOFunc( void ) ;

inline static void HplMsp430GeneralIOP_45_IO_makeInput( void ) ;

inline static void HplMsp430GeneralIOP_45_IO_selectModuleFunc( void ) ;

inline static void HplMsp430GeneralIOP_45_IO_selectIOFunc( void ) ;

inline static void HplMsp430GeneralIOP_46_IO_makeInput( void ) ;

inline static void HplMsp430GeneralIOP_46_IO_selectModuleFunc( void ) ;

inline static void HplMsp430GeneralIOP_46_IO_selectIOFunc( void ) ;

inline static void HplMsp430GeneralIOP_47_IO_makeInput( void ) ;

inline static void HplMsp430GeneralIOP_47_IO_selectModuleFunc( void ) ;

inline static void HplMsp430GeneralIOP_47_IO_selectIOFunc( void ) ;

static void Msp430GpioC_0_HplGeneralIO_makeOutput( void ) ;

static void Msp430GpioC_0_HplGeneralIO_set( void ) ;

static void Msp430GpioC_0_HplGeneralIO_clr( void ) ;

inline static void Msp430GpioC_0_GeneralIO_set( void ) ;

inline static void Msp430GpioC_0_GeneralIO_clr( void ) ;

inline static void Msp430GpioC_0_GeneralIO_makeOutput( void ) ;

static void Msp430GpioC_1_HplGeneralIO_makeOutput( void ) ;

static void Msp430GpioC_1_HplGeneralIO_set( void ) ;

static void Msp430GpioC_1_HplGeneralIO_clr( void ) ;

inline static void Msp430GpioC_1_GeneralIO_set( void ) ;

inline static void Msp430GpioC_1_GeneralIO_clr( void ) ;

inline static void Msp430GpioC_1_GeneralIO_makeOutput( void ) ;

static void Msp430GpioC_2_HplGeneralIO_makeOutput( void ) ;

static void Msp430GpioC_2_HplGeneralIO_set( void ) ;

static void Msp430GpioC_2_HplGeneralIO_clr( void ) ;

inline static void Msp430GpioC_2_GeneralIO_set( void ) ;

inline static void Msp430GpioC_2_GeneralIO_clr( void ) ;

inline static void Msp430GpioC_2_GeneralIO_makeOutput( void ) ;

static void Msp430AlarmC_0_Msp430Compare_setEvent(uint16_t time) ;

static void Msp430AlarmC_0_Msp430Compare_setEventFromNow(uint16_t delta) ;

static uint16_t Msp430AlarmC_0_Msp430Timer_get( void ) ;

static void Msp430AlarmC_0_Alarm_fired( void ) ;

static void Msp430AlarmC_0_Msp430TimerControl_enableEvents( void ) ;

static void Msp430AlarmC_0_Msp430TimerControl_setControlAsCompare( void ) ;

static void Msp430AlarmC_0_Msp430TimerControl_disableEvents( void ) ;

static void Msp430AlarmC_0_Msp430TimerControl_clearPendingInterrupt( void ) ;

inline static error_t Msp430AlarmC_0_Init_init( void ) ;

inline static void Msp430AlarmC_0_Alarm_stop( void ) ;

inline static void Msp430AlarmC_0_Msp430Compare_fired( void ) ;

inline static void Msp430AlarmC_0_Alarm_startAt(uint16_t t0, uint16_t dt) ;

inline static void Msp430AlarmC_0_Msp430Timer_overflow( void ) ;

static uint16_t Msp430CounterC_0_Msp430Timer_get( void ) ;

static bool Msp430CounterC_0_Msp430Timer_isOverflowPending( void ) ;

static void Msp430CounterC_0_Counter_overflow( void ) ;

inline static uint16_t Msp430CounterC_0_Counter_get( void ) ;

inline static bool Msp430CounterC_0_Counter_isOverflowPending( void ) ;

inline static void Msp430CounterC_0_Msp430Timer_overflow( void ) ;

static TransformCounterC_0_CounterFrom_size_type TransformCounterC_0_CounterFrom_get( void ) ;

static bool TransformCounterC_0_CounterFrom_isOverflowPending( void ) ;

static void TransformCounterC_0_Counter_overflow( void ) ;

TransformCounterC_0_upper_count_type TransformCounterC_0_m_upper;

enum TransformCounterC_0___nesc_unnamed4279 { TransformCounterC_0_LOW_SHIFT_RIGHT = 5, TransformCounterC_0_HIGH_SHIFT_LEFT = (8 * sizeof( TransformCounterC_0_from_size_type )) - TransformCounterC_0_LOW_SHIFT_RIGHT, TransformCounterC_0_NUM_UPPER_BITS = ((8 * sizeof( TransformCounterC_0_to_size_type )) - (8 * sizeof( TransformCounterC_0_from_size_type ))) + 5, TransformCounterC_0_OVERFLOW_MASK = TransformCounterC_0_NUM_UPPER_BITS? (( TransformCounterC_0_upper_count_type )(2) << (TransformCounterC_0_NUM_UPPER_BITS - 1)) - 1 : 0 };

static TransformCounterC_0_to_size_type TransformCounterC_0_Counter_get( void ) ;

inline static void TransformCounterC_0_CounterFrom_overflow( void ) ;

static void TransformAlarmC_0_Alarm_fired( void ) ;

static void TransformAlarmC_0_AlarmFrom_startAt(TransformAlarmC_0_AlarmFrom_size_type t0, TransformAlarmC_0_AlarmFrom_size_type dt) ;

static void TransformAlarmC_0_AlarmFrom_stop( void ) ;

static TransformAlarmC_0_Counter_size_type TransformAlarmC_0_Counter_get( void ) ;

TransformAlarmC_0_to_size_type TransformAlarmC_0_m_t0;

TransformAlarmC_0_to_size_type TransformAlarmC_0_m_dt;

enum TransformAlarmC_0___nesc_unnamed4280 { TransformAlarmC_0_MAX_DELAY_LOG2 = ((8 * sizeof( TransformAlarmC_0_from_size_type )) - 1) - 5, TransformAlarmC_0_MAX_DELAY = ( TransformAlarmC_0_to_size_type )(1) << TransformAlarmC_0_MAX_DELAY_LOG2 };

inline static TransformAlarmC_0_to_size_type TransformAlarmC_0_Alarm_getNow( void ) ;

inline static TransformAlarmC_0_to_size_type TransformAlarmC_0_Alarm_getAlarm( void ) ;

inline static void TransformAlarmC_0_Alarm_stop( void ) ;

static void TransformAlarmC_0_set_alarm( void ) ;

static void TransformAlarmC_0_Alarm_startAt(TransformAlarmC_0_to_size_type t0, TransformAlarmC_0_to_size_type dt) ;

inline static void TransformAlarmC_0_AlarmFrom_fired( void ) ;

inline static void TransformAlarmC_0_Counter_overflow( void ) ;

static error_t AlarmToTimerC_0_fired_postTask( void ) ;

static AlarmToTimerC_0_Alarm_size_type AlarmToTimerC_0_Alarm_getNow( void ) ;

static void AlarmToTimerC_0_Alarm_startAt(AlarmToTimerC_0_Alarm_size_type t0, AlarmToTimerC_0_Alarm_size_type dt) ;

static AlarmToTimerC_0_Alarm_size_type AlarmToTimerC_0_Alarm_getAlarm( void ) ;

static void AlarmToTimerC_0_Alarm_stop( void ) ;

static void AlarmToTimerC_0_Timer_fired( void ) ;

enum AlarmToTimerC_0___nesc_unnamed4281 { AlarmToTimerC_0_fired = 0U };

typedef int (AlarmToTimerC_0___nesc_sillytask_fired[AlarmToTimerC_0_fired]);

uint32_t AlarmToTimerC_0_m_dt;

bool AlarmToTimerC_0_m_oneshot;

inline static void AlarmToTimerC_0_start(uint32_t t0, uint32_t dt, bool oneshot) ;

inline static void AlarmToTimerC_0_Timer_stop( void ) ;

inline static void AlarmToTimerC_0_fired_runTask( void ) ;

inline static void AlarmToTimerC_0_Alarm_fired( void ) ;

inline static void AlarmToTimerC_0_Timer_startOneShotAt(uint32_t t0, uint32_t dt) ;

inline static uint32_t AlarmToTimerC_0_Timer_getNow( void ) ;

static error_t VirtualizeTimerC_0_updateFromTimer_postTask( void ) ;

static uint32_t VirtualizeTimerC_0_TimerFrom_getNow( void ) ;

static void VirtualizeTimerC_0_TimerFrom_startOneShotAt(uint32_t t0, uint32_t dt) ;

static void VirtualizeTimerC_0_TimerFrom_stop( void ) ;

static void VirtualizeTimerC_0_Timer_fired(uint8_t arg_0x18923c8) ;

enum VirtualizeTimerC_0___nesc_unnamed4282 { VirtualizeTimerC_0_updateFromTimer = 1U };

typedef int (VirtualizeTimerC_0___nesc_sillytask_updateFromTimer[VirtualizeTimerC_0_updateFromTimer]);

enum VirtualizeTimerC_0___nesc_unnamed4283 { VirtualizeTimerC_0_NUM_TIMERS = 3U, VirtualizeTimerC_0_END_OF_LIST = 255 };

typedef struct VirtualizeTimerC_0___nesc_unnamed4284 {
  uint32_t t0;
  uint32_t dt;
  bool isoneshot :1;
  bool isrunning :1;
  bool _reserved :6;
} VirtualizeTimerC_0_Timer_t;

VirtualizeTimerC_0_Timer_t (VirtualizeTimerC_0_m_timers[VirtualizeTimerC_0_NUM_TIMERS]);

static void VirtualizeTimerC_0_fireTimers(uint32_t now) ;

inline static void VirtualizeTimerC_0_updateFromTimer_runTask( void ) ;

inline static void VirtualizeTimerC_0_TimerFrom_fired( void ) ;

static void VirtualizeTimerC_0_startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot) ;

inline static void VirtualizeTimerC_0_Timer_startPeriodic(uint8_t num, uint32_t dt) ;

inline static void VirtualizeTimerC_0_Timer_startOneShot(uint8_t num, uint32_t dt) ;

inline static void VirtualizeTimerC_0_Timer_stop(uint8_t num) ;

inline static void VirtualizeTimerC_0_Timer_default_fired(uint8_t num) ;

inline static void CounterToLocalTimeC_0_Counter_overflow( void ) ;

static void AdcP_Read_readDone(uint8_t arg_0x1903338, error_t result, AdcP_Read_val_t val) ;

static void AdcP_ReadNow_readDone(uint8_t arg_0x1901350, error_t result, AdcP_ReadNow_val_t val) ;

static void AdcP_ResourceReadNow_granted(uint8_t arg_0x18ff330) ;

static AdcP_Config_adc_config_t AdcP_Config_getConfiguration(uint8_t arg_0x18fc648) ;

static error_t AdcP_SingleChannel_getData(uint8_t arg_0x191ee60) ;

static error_t AdcP_SingleChannel_configureSingle(uint8_t arg_0x191ee60, const msp430adc12_channel_config_t *config) ;

static error_t AdcP_ResourceRead_release(uint8_t arg_0x18ffe18) ;

static error_t AdcP_ResourceRead_request(uint8_t arg_0x18ffe18) ;

static error_t AdcP_readDone_postTask( void ) ;

enum AdcP___nesc_unnamed4285 { AdcP_readDone = 2U };

typedef int (AdcP___nesc_sillytask_readDone[AdcP_readDone]);

enum AdcP___nesc_unnamed4286 { AdcP_STATE_READ, AdcP_STATE_READNOW, AdcP_STATE_READNOW_INVALID_CONFIG };

uint8_t AdcP_state;

uint8_t AdcP_owner;

uint16_t AdcP_value;

static error_t AdcP_configure(uint8_t client) ;

inline static error_t AdcP_Read_read(uint8_t client) ;

static void AdcP_ResourceRead_granted(uint8_t client) ;

static void AdcP_SubResourceReadNow_granted(uint8_t nowClient) ;

inline static void AdcP_readDone_runTask( void ) ;

static error_t AdcP_SingleChannel_singleDataReady(uint8_t client, uint16_t data) ;

inline static uint16_t *AdcP_SingleChannel_multipleDataReady(uint8_t client, uint16_t *buf, uint16_t numSamples) ;

inline static error_t AdcP_ResourceRead_default_request(uint8_t client) ;

inline static error_t AdcP_ResourceRead_default_release(uint8_t client) ;

inline static void AdcP_Read_default_readDone(uint8_t client, error_t result, uint16_t val) ;

inline static void AdcP_ResourceReadNow_default_granted(uint8_t nowClient) ;

inline static void AdcP_ReadNow_default_readDone(uint8_t client, error_t result, uint16_t val) ;

inline static error_t AdcP_SingleChannel_default_getData(uint8_t client) ;

const msp430adc12_channel_config_t AdcP_defaultConfig = {INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0};

const inline static msp430adc12_channel_config_t *AdcP_Config_default_getConfiguration(uint8_t client) ;

inline static error_t AdcP_SingleChannel_default_configureSingle(uint8_t client, const msp430adc12_channel_config_t *config) ;

static void Msp430Adc12ImplP_MultiChannel_dataReady(uint8_t arg_0x1972108, uint16_t *buffer, uint16_t numSamples) ;

static adc12ctl0_t Msp430Adc12ImplP_HplAdc12_getCtl0( void ) ;

static adc12memctl_t Msp430Adc12ImplP_HplAdc12_getMCtl(uint8_t idx) ;

static void Msp430Adc12ImplP_HplAdc12_resetIFGs( void ) ;

static void Msp430Adc12ImplP_HplAdc12_setMCtl(uint8_t idx, adc12memctl_t memControl) ;

static void Msp430Adc12ImplP_HplAdc12_startConversion( void ) ;

static void Msp430Adc12ImplP_HplAdc12_setCtl0(adc12ctl0_t control0) ;

static uint16_t Msp430Adc12ImplP_HplAdc12_getMem(uint8_t idx) ;

static void Msp430Adc12ImplP_HplAdc12_setIEFlags(uint16_t mask) ;

static void Msp430Adc12ImplP_HplAdc12_stopConversion( void ) ;

static void Msp430Adc12ImplP_HplAdc12_setCtl1(adc12ctl1_t control1) ;

static void Msp430Adc12ImplP_Port64_makeInput( void ) ;

static void Msp430Adc12ImplP_Port64_selectIOFunc( void ) ;

static void Msp430Adc12ImplP_Port64_selectModuleFunc( void ) ;

static void Msp430Adc12ImplP_CompareA1_setEvent(uint16_t time) ;

static void Msp430Adc12ImplP_ControlA0_setControl(msp430_compare_control_t control) ;

static void Msp430Adc12ImplP_Port62_makeInput( void ) ;

static void Msp430Adc12ImplP_Port62_selectIOFunc( void ) ;

static void Msp430Adc12ImplP_Port62_selectModuleFunc( void ) ;

static void Msp430Adc12ImplP_Overflow_memOverflow(uint8_t arg_0x19729f8) ;

static void Msp430Adc12ImplP_Overflow_conversionTimeOverflow(uint8_t arg_0x19729f8) ;

static void Msp430Adc12ImplP_Port67_makeInput( void ) ;

static void Msp430Adc12ImplP_Port67_selectIOFunc( void ) ;

static void Msp430Adc12ImplP_Port67_selectModuleFunc( void ) ;

static void Msp430Adc12ImplP_Port60_makeInput( void ) ;

static void Msp430Adc12ImplP_Port60_selectIOFunc( void ) ;

static void Msp430Adc12ImplP_Port60_selectModuleFunc( void ) ;

static void Msp430Adc12ImplP_Port65_makeInput( void ) ;

static void Msp430Adc12ImplP_Port65_selectIOFunc( void ) ;

static void Msp430Adc12ImplP_Port65_selectModuleFunc( void ) ;

static void Msp430Adc12ImplP_TimerA_clear( void ) ;

static void Msp430Adc12ImplP_TimerA_setClockSource(uint16_t clockSource) ;

static void Msp430Adc12ImplP_TimerA_disableEvents( void ) ;

static void Msp430Adc12ImplP_TimerA_setMode(int mode) ;

static void Msp430Adc12ImplP_TimerA_setInputDivider(uint16_t inputDivider) ;

static uint8_t Msp430Adc12ImplP_ADCArbiterInfo_userId( void ) ;

static void Msp430Adc12ImplP_ControlA1_setControl(msp430_compare_control_t control) ;

static uint16_t *Msp430Adc12ImplP_SingleChannel_multipleDataReady(uint8_t arg_0x19743f0, uint16_t *buffer, uint16_t numSamples) ;

static error_t Msp430Adc12ImplP_SingleChannel_singleDataReady(uint8_t arg_0x19743f0, uint16_t data) ;

static void Msp430Adc12ImplP_Port63_makeInput( void ) ;

static void Msp430Adc12ImplP_Port63_selectIOFunc( void ) ;

static void Msp430Adc12ImplP_Port63_selectModuleFunc( void ) ;

static void Msp430Adc12ImplP_CompareA0_setEvent(uint16_t time) ;

static void Msp430Adc12ImplP_Port61_makeInput( void ) ;

static void Msp430Adc12ImplP_Port61_selectIOFunc( void ) ;

static void Msp430Adc12ImplP_Port61_selectModuleFunc( void ) ;

static void Msp430Adc12ImplP_Port66_makeInput( void ) ;

static void Msp430Adc12ImplP_Port66_selectIOFunc( void ) ;

static void Msp430Adc12ImplP_Port66_selectModuleFunc( void ) ;

enum Msp430Adc12ImplP___nesc_unnamed4287 { Msp430Adc12ImplP_SINGLE_DATA = 1, Msp430Adc12ImplP_SINGLE_DATA_REPEAT = 2, Msp430Adc12ImplP_MULTIPLE_DATA = 4, Msp430Adc12ImplP_MULTIPLE_DATA_REPEAT = 8, Msp430Adc12ImplP_MULTI_CHANNEL = 16, Msp430Adc12ImplP_CONVERSION_MODE_MASK = 0x1F, Msp430Adc12ImplP_ADC_BUSY = 32, Msp430Adc12ImplP_USE_TIMERA = 64, Msp430Adc12ImplP_ADC_OVERFLOW = 128 };

uint8_t Msp430Adc12ImplP_state;

uint16_t Msp430Adc12ImplP_resultBufferLength;

uint16_t *Msp430Adc12ImplP_resultBufferStart;

uint16_t Msp430Adc12ImplP_resultBufferIndex;

uint8_t Msp430Adc12ImplP_numChannels;

uint8_t Msp430Adc12ImplP_clientID;

inline static error_t Msp430Adc12ImplP_Init_init( void ) ;

inline static void Msp430Adc12ImplP_prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON) ;

inline static void Msp430Adc12ImplP_startTimerA( void ) ;

inline static void Msp430Adc12ImplP_configureAdcPin(uint8_t inch) ;

static void Msp430Adc12ImplP_resetAdcPin(uint8_t inch) ;

static error_t Msp430Adc12ImplP_SingleChannel_configureSingle(uint8_t id, const msp430adc12_channel_config_t *config) ;

inline static error_t Msp430Adc12ImplP_SingleChannel_configureMultiple(uint8_t id, const msp430adc12_channel_config_t *config, uint16_t *buf, uint16_t length, uint16_t jiffies) ;

static error_t Msp430Adc12ImplP_SingleChannel_getData(uint8_t id) ;

static void Msp430Adc12ImplP_stopConversion( void ) ;

inline static void Msp430Adc12ImplP_HplAdc12_conversionDone(uint16_t iv) ;

inline static error_t Msp430Adc12ImplP_SingleChannel_default_singleDataReady(uint8_t id, uint16_t data) ;

inline static uint16_t *Msp430Adc12ImplP_SingleChannel_default_multipleDataReady(uint8_t id, uint16_t *buf, uint16_t numSamples) ;

inline static void Msp430Adc12ImplP_MultiChannel_default_dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples) ;

inline static void Msp430Adc12ImplP_Overflow_default_memOverflow(uint8_t id) ;

inline static void Msp430Adc12ImplP_Overflow_default_conversionTimeOverflow(uint8_t id) ;

static void HplAdc12P_HplAdc12_conversionDone(uint16_t iv) ;

inline static void HplAdc12P_HplAdc12_setCtl0(adc12ctl0_t control0) ;

inline static void HplAdc12P_HplAdc12_setCtl1(adc12ctl1_t control1) ;

inline static adc12ctl0_t HplAdc12P_HplAdc12_getCtl0( void ) ;

inline static void HplAdc12P_HplAdc12_setMCtl(uint8_t i, adc12memctl_t memControl) ;

static adc12memctl_t HplAdc12P_HplAdc12_getMCtl(uint8_t i) ;

inline static uint16_t HplAdc12P_HplAdc12_getMem(uint8_t i) ;

inline static void HplAdc12P_HplAdc12_setIEFlags(uint16_t mask) ;

inline static void HplAdc12P_HplAdc12_resetIFGs( void ) ;

inline static void HplAdc12P_HplAdc12_startConversion( void ) ;

static void HplAdc12P_HplAdc12_stopConversion( void ) ;

inline static bool HplAdc12P_HplAdc12_isBusy( void ) ;

void sig_ADC_VECTOR( void ) ;

enum RoundRobinResourceQueueC_0___nesc_unnamed4288 { RoundRobinResourceQueueC_0_NO_ENTRY = 0xFF, RoundRobinResourceQueueC_0_SIZE = 3U? ((3U - 1) / 8) + 1 : 0 };

uint8_t (RoundRobinResourceQueueC_0_resQ[RoundRobinResourceQueueC_0_SIZE]);

uint8_t RoundRobinResourceQueueC_0_last = 0;

inline static void RoundRobinResourceQueueC_0_clearEntry(uint8_t id) ;

inline static error_t RoundRobinResourceQueueC_0_Init_init( void ) ;

inline static bool RoundRobinResourceQueueC_0_RoundRobinQueue_isEmpty( void ) ;

static bool RoundRobinResourceQueueC_0_RoundRobinQueue_isEnqueued(resource_client_id_t id) ;

inline static resource_client_id_t RoundRobinResourceQueueC_0_RoundRobinQueue_dequeue( void ) ;

inline static error_t RoundRobinResourceQueueC_0_RoundRobinQueue_enqueue(resource_client_id_t id) ;

static void SimpleArbiterP_0_ResourceRequested_requested(uint8_t arg_0x1a813f0) ;

static void SimpleArbiterP_0_ResourceConfigure_unconfigure(uint8_t arg_0x1a7f030) ;

static void SimpleArbiterP_0_ResourceConfigure_configure(uint8_t arg_0x1a7f030) ;

static error_t SimpleArbiterP_0_Queue_enqueue(resource_client_id_t id) ;

static bool SimpleArbiterP_0_Queue_isEmpty( void ) ;

static resource_client_id_t SimpleArbiterP_0_Queue_dequeue( void ) ;

static void SimpleArbiterP_0_Resource_granted(uint8_t arg_0x1a82930) ;

static error_t SimpleArbiterP_0_grantedTask_postTask( void ) ;

enum SimpleArbiterP_0___nesc_unnamed4289 { SimpleArbiterP_0_grantedTask = 3U };

typedef int (SimpleArbiterP_0___nesc_sillytask_grantedTask[SimpleArbiterP_0_grantedTask]);

enum SimpleArbiterP_0___nesc_unnamed4290 { SimpleArbiterP_0_RES_IDLE = 0, SimpleArbiterP_0_RES_GRANTING = 1, SimpleArbiterP_0_RES_BUSY = 2 };

enum SimpleArbiterP_0___nesc_unnamed4291 { SimpleArbiterP_0_NO_RES = 0xFF };

uint8_t SimpleArbiterP_0_state = SimpleArbiterP_0_RES_IDLE;

uint8_t SimpleArbiterP_0_resId = SimpleArbiterP_0_NO_RES;

uint8_t SimpleArbiterP_0_reqResId;

static error_t SimpleArbiterP_0_Resource_request(uint8_t id) ;

static error_t SimpleArbiterP_0_Resource_release(uint8_t id) ;

static uint8_t SimpleArbiterP_0_ArbiterInfo_userId( void ) ;

inline static void SimpleArbiterP_0_grantedTask_runTask( void ) ;

inline static void SimpleArbiterP_0_Resource_default_granted(uint8_t id) ;

inline static void SimpleArbiterP_0_ResourceRequested_default_requested(uint8_t id) ;

inline static void SimpleArbiterP_0_ResourceConfigure_default_configure(uint8_t id) ;

inline static void SimpleArbiterP_0_ResourceConfigure_default_unconfigure(uint8_t id) ;

static adc12ctl0_t Msp430RefVoltGeneratorP_HplAdc12_getCtl0( void ) ;

static bool Msp430RefVoltGeneratorP_HplAdc12_isBusy( void ) ;

static void Msp430RefVoltGeneratorP_HplAdc12_setCtl0(adc12ctl0_t control0) ;

static void Msp430RefVoltGeneratorP_SwitchOffTimer_startOneShot(uint32_t dt) ;

static void Msp430RefVoltGeneratorP_SwitchOffTimer_stop( void ) ;

static void Msp430RefVoltGeneratorP_RefVolt_2_5V_startDone(error_t error) ;

static void Msp430RefVoltGeneratorP_RefVolt_2_5V_stopDone(error_t error) ;

static void Msp430RefVoltGeneratorP_RefVolt_1_5V_startDone(error_t error) ;

static void Msp430RefVoltGeneratorP_RefVolt_1_5V_stopDone(error_t error) ;

static void Msp430RefVoltGeneratorP_SwitchOnTimer_startOneShot(uint32_t dt) ;

static void Msp430RefVoltGeneratorP_SwitchOnTimer_stop( void ) ;

enum Msp430RefVoltGeneratorP___nesc_unnamed4292 { Msp430RefVoltGeneratorP_GENERATOR_OFF, Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING, Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING, Msp430RefVoltGeneratorP_REFERENCE_1_5V_STABLE, Msp430RefVoltGeneratorP_REFERENCE_2_5V_STABLE };

uint8_t Msp430RefVoltGeneratorP_state;

static error_t Msp430RefVoltGeneratorP_switchOn(uint8_t level) ;

static error_t Msp430RefVoltGeneratorP_switchOff( void ) ;

inline static error_t Msp430RefVoltGeneratorP_RefVolt_1_5V_start( void ) ;

inline static error_t Msp430RefVoltGeneratorP_RefVolt_1_5V_stop( void ) ;

inline static error_t Msp430RefVoltGeneratorP_RefVolt_2_5V_start( void ) ;

inline static void Msp430RefVoltGeneratorP_SwitchOnTimer_fired( void ) ;

inline static void Msp430RefVoltGeneratorP_SwitchOffTimer_fired( void ) ;

inline static void Msp430RefVoltGeneratorP_HplAdc12_conversionDone(uint16_t iv) ;

static Msp430RefVoltArbiterImplP_Config_adc_config_t Msp430RefVoltArbiterImplP_Config_getConfiguration(uint8_t arg_0x1af6638) ;

static error_t Msp430RefVoltArbiterImplP_RefVolt_2_5V_start( void ) ;

static error_t Msp430RefVoltArbiterImplP_AdcResource_release(uint8_t arg_0x1af9ac8) ;

static error_t Msp430RefVoltArbiterImplP_AdcResource_request(uint8_t arg_0x1af9ac8) ;

static void Msp430RefVoltArbiterImplP_ClientResource_granted(uint8_t arg_0x1af9010) ;

static error_t Msp430RefVoltArbiterImplP_switchOff_postTask( void ) ;

static error_t Msp430RefVoltArbiterImplP_RefVolt_1_5V_start( void ) ;

static error_t Msp430RefVoltArbiterImplP_RefVolt_1_5V_stop( void ) ;

enum Msp430RefVoltArbiterImplP___nesc_unnamed4293 { Msp430RefVoltArbiterImplP_switchOff = 4U };

typedef int (Msp430RefVoltArbiterImplP___nesc_sillytask_switchOff[Msp430RefVoltArbiterImplP_switchOff]);

enum Msp430RefVoltArbiterImplP___nesc_unnamed4294 { Msp430RefVoltArbiterImplP_NO_OWNER = 0xFF };

uint8_t Msp430RefVoltArbiterImplP_syncOwner = Msp430RefVoltArbiterImplP_NO_OWNER;

inline static error_t Msp430RefVoltArbiterImplP_ClientResource_request(uint8_t client) ;

static void Msp430RefVoltArbiterImplP_AdcResource_granted(uint8_t client) ;

inline static void Msp430RefVoltArbiterImplP_RefVolt_1_5V_startDone(error_t error) ;

static void Msp430RefVoltArbiterImplP_RefVolt_2_5V_startDone(error_t error) ;

static error_t Msp430RefVoltArbiterImplP_ClientResource_release(uint8_t client) ;

inline static void Msp430RefVoltArbiterImplP_switchOff_runTask( void ) ;

inline static void Msp430RefVoltArbiterImplP_RefVolt_1_5V_stopDone(error_t error) ;

inline static void Msp430RefVoltArbiterImplP_RefVolt_2_5V_stopDone(error_t error) ;

inline static void Msp430RefVoltArbiterImplP_ClientResource_default_granted(uint8_t client) ;

inline static error_t Msp430RefVoltArbiterImplP_AdcResource_default_request(uint8_t client) ;

inline static error_t Msp430RefVoltArbiterImplP_AdcResource_default_release(uint8_t client) ;

const msp430adc12_channel_config_t Msp430RefVoltArbiterImplP_defaultConfig = {INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0};

const inline static msp430adc12_channel_config_t *Msp430RefVoltArbiterImplP_Config_default_getConfiguration(uint8_t client) ;

static Msp430Adc12ConfAlertC_0_ConfUp_adc_config_t Msp430Adc12ConfAlertC_0_ConfUp_getConfiguration( void ) ;

const inline static msp430adc12_channel_config_t *Msp430Adc12ConfAlertC_0_ConfSub_getConfiguration( void ) ;

static error_t AdcStreamP_bufferDone_postTask( void ) ;

static error_t AdcStreamP_readStreamDone_postTask( void ) ;

static error_t AdcStreamP_readStreamFail_postTask( void ) ;

static AdcStreamP_Alarm_size_type AdcStreamP_Alarm_getNow( void ) ;

static void AdcStreamP_Alarm_startAt(AdcStreamP_Alarm_size_type t0, AdcStreamP_Alarm_size_type dt) ;

static AdcStreamP_AdcConfigure_adc_config_t AdcStreamP_AdcConfigure_getConfiguration(uint8_t arg_0x1b5cde0) ;

static error_t AdcStreamP_SingleChannel_getData(uint8_t arg_0x1b5c010) ;

static error_t AdcStreamP_SingleChannel_configureSingle(uint8_t arg_0x1b5c010, const msp430adc12_channel_config_t *config) ;

static error_t AdcStreamP_SingleChannel_configureMultiple(uint8_t arg_0x1b5c010, const msp430adc12_channel_config_t *config, uint16_t *buffer, uint16_t numSamples, uint16_t jiffies) ;

static void AdcStreamP_ReadStream_bufferDone(uint8_t arg_0x1b5f340, error_t result, AdcStreamP_ReadStream_val_t *buf, uint16_t count) ;

static void AdcStreamP_ReadStream_readDone(uint8_t arg_0x1b5f340, error_t result, uint32_t usActualPeriod) ;

enum AdcStreamP___nesc_unnamed4295 { AdcStreamP_readStreamDone = 5U };

typedef int (AdcStreamP___nesc_sillytask_readStreamDone[AdcStreamP_readStreamDone]);

enum AdcStreamP___nesc_unnamed4296 { AdcStreamP_readStreamFail = 6U };

typedef int (AdcStreamP___nesc_sillytask_readStreamFail[AdcStreamP_readStreamFail]);

enum AdcStreamP___nesc_unnamed4297 { AdcStreamP_bufferDone = 7U };

typedef int (AdcStreamP___nesc_sillytask_bufferDone[AdcStreamP_bufferDone]);

enum AdcStreamP___nesc_unnamed4298 { AdcStreamP_NSTREAM = 1U };

uint8_t AdcStreamP_client = AdcStreamP_NSTREAM;

struct AdcStreamP_list_entry_t {
  uint16_t count;
  struct AdcStreamP_list_entry_t  *next;
};

struct AdcStreamP_list_entry_t  (*AdcStreamP_bufferQueue[AdcStreamP_NSTREAM]);

struct AdcStreamP_list_entry_t  (**AdcStreamP_bufferQueueEnd[AdcStreamP_NSTREAM]);

uint16_t *AdcStreamP_lastBuffer;

uint16_t AdcStreamP_lastCount;

uint16_t AdcStreamP_count;

uint16_t *AdcStreamP_buffer;

uint16_t *AdcStreamP_pos;

uint32_t AdcStreamP_now;

uint32_t AdcStreamP_period;

bool AdcStreamP_periodModified;

inline static error_t AdcStreamP_Init_init( void ) ;

inline static void AdcStreamP_sampleSingle( void ) ;

inline static error_t AdcStreamP_postBuffer(uint8_t c, uint16_t *buf, uint16_t n) ;

inline static void AdcStreamP_readStreamDone_runTask( void ) ;

inline static void AdcStreamP_readStreamFail_runTask( void ) ;

inline static void AdcStreamP_bufferDone_runTask( void ) ;

inline static void AdcStreamP_nextAlarm( void ) ;

inline static void AdcStreamP_Alarm_fired( void ) ;

static error_t AdcStreamP_nextBuffer(bool startNextAlarm) ;

static void AdcStreamP_nextMultiple(uint8_t c) ;

static error_t AdcStreamP_ReadStream_read(uint8_t c, uint32_t usPeriod) ;

static error_t AdcStreamP_SingleChannel_singleDataReady(uint8_t streamClient, uint16_t data) ;

static uint16_t *AdcStreamP_SingleChannel_multipleDataReady(uint8_t streamClient, uint16_t *buf, uint16_t length) ;

const msp430adc12_channel_config_t AdcStreamP_defaultConfig = {.inch = SUPPLY_VOLTAGE_HALF_CHANNEL, .sref = REFERENCE_VREFplus_AVss, .ref2_5v = REFVOLT_LEVEL_1_5, .adc12ssel = SHT_SOURCE_ACLK, .adc12div = SHT_CLOCK_DIV_1, .sht = SAMPLE_HOLD_4_CYCLES, .sampcon_ssel = SAMPCON_SOURCE_SMCLK, .sampcon_id = SAMPCON_CLOCK_DIV_1};

const inline static msp430adc12_channel_config_t *AdcStreamP_AdcConfigure_default_getConfiguration(uint8_t c) ;

inline static error_t AdcStreamP_SingleChannel_default_configureMultiple(uint8_t c, const msp430adc12_channel_config_t *config, uint16_t (b), uint16_t numSamples, uint16_t jiffies) ;

inline static error_t AdcStreamP_SingleChannel_default_getData(uint8_t c) ;

inline static error_t AdcStreamP_SingleChannel_default_configureSingle(uint8_t c, const msp430adc12_channel_config_t *config) ;

static void Msp430AlarmC_1_Msp430Compare_setEvent(uint16_t time) ;

static void Msp430AlarmC_1_Msp430Compare_setEventFromNow(uint16_t delta) ;

static uint16_t Msp430AlarmC_1_Msp430Timer_get( void ) ;

static void Msp430AlarmC_1_Alarm_fired( void ) ;

static void Msp430AlarmC_1_Msp430TimerControl_enableEvents( void ) ;

static void Msp430AlarmC_1_Msp430TimerControl_disableEvents( void ) ;

static void Msp430AlarmC_1_Msp430TimerControl_clearPendingInterrupt( void ) ;

inline static void Msp430AlarmC_1_Msp430Compare_fired( void ) ;

inline static void Msp430AlarmC_1_Alarm_startAt(uint16_t t0, uint16_t dt) ;

inline static void Msp430AlarmC_1_Msp430Timer_overflow( void ) ;

static void TransformAlarmC_1_Alarm_fired( void ) ;

static void TransformAlarmC_1_AlarmFrom_startAt(TransformAlarmC_1_AlarmFrom_size_type t0, TransformAlarmC_1_AlarmFrom_size_type dt) ;

static TransformAlarmC_1_Counter_size_type TransformAlarmC_1_Counter_get( void ) ;

TransformAlarmC_1_to_size_type TransformAlarmC_1_m_t0;

TransformAlarmC_1_to_size_type TransformAlarmC_1_m_dt;

enum TransformAlarmC_1___nesc_unnamed4299 { TransformAlarmC_1_MAX_DELAY_LOG2 = ((8 * sizeof( TransformAlarmC_1_from_size_type )) - 1) - 5, TransformAlarmC_1_MAX_DELAY = ( TransformAlarmC_1_to_size_type )(1) << TransformAlarmC_1_MAX_DELAY_LOG2 };

inline static TransformAlarmC_1_to_size_type TransformAlarmC_1_Alarm_getNow( void ) ;

static void TransformAlarmC_1_set_alarm( void ) ;

inline static void TransformAlarmC_1_Alarm_startAt(TransformAlarmC_1_to_size_type t0, TransformAlarmC_1_to_size_type dt) ;

inline static void TransformAlarmC_1_AlarmFrom_fired( void ) ;

inline static void TransformAlarmC_1_Counter_overflow( void ) ;

static error_t ArbitratedReadStreamC_0_Service_read(uint8_t arg_0x1bc37c8, uint32_t usPeriod) ;

static void ArbitratedReadStreamC_0_ReadStream_bufferDone(uint8_t arg_0x1bc7bb0, error_t result, ArbitratedReadStreamC_0_ReadStream_val_t *buf, uint16_t count) ;

static void ArbitratedReadStreamC_0_ReadStream_readDone(uint8_t arg_0x1bc7bb0, error_t result, uint32_t usActualPeriod) ;

static error_t ArbitratedReadStreamC_0_Resource_release(uint8_t arg_0x1bbf278) ;

uint32_t (ArbitratedReadStreamC_0_period[1U]);

inline static void ArbitratedReadStreamC_0_Service_bufferDone(uint8_t client, error_t result, ArbitratedReadStreamC_0_val_t *buf, uint16_t count) ;

inline static void ArbitratedReadStreamC_0_Service_readDone(uint8_t client, error_t result, uint32_t actualPeriod) ;

inline static void ArbitratedReadStreamC_0_Resource_granted(uint8_t client) ;

inline static error_t ArbitratedReadStreamC_0_Resource_default_release(uint8_t client) ;

inline static void ArbitratedReadStreamC_0_ReadStream_default_bufferDone(uint8_t client, error_t result, ArbitratedReadStreamC_0_val_t *buf, uint16_t count) ;

inline static void ArbitratedReadStreamC_0_ReadStream_default_readDone(uint8_t client, error_t result, uint32_t actualPeriod) ;

static Msp430Adc12ConfAlertC_1_ConfUp_adc_config_t Msp430Adc12ConfAlertC_1_ConfUp_getConfiguration( void ) ;

const inline static msp430adc12_channel_config_t *Msp430Adc12ConfAlertC_1_ConfSub_getConfiguration( void ) ;

const msp430adc12_channel_config_t Msp430InternalVoltageP_config = {.inch = SUPPLY_VOLTAGE_HALF_CHANNEL, .sref = REFERENCE_VREFplus_AVss, .ref2_5v = REFVOLT_LEVEL_1_5, .adc12ssel = SHT_SOURCE_ACLK, .adc12div = SHT_CLOCK_DIV_1, .sht = SAMPLE_HOLD_4_CYCLES, .sampcon_ssel = SAMPCON_SOURCE_SMCLK, .sampcon_id = SAMPCON_CLOCK_DIV_1};

const inline static msp430adc12_channel_config_t *Msp430InternalVoltageP_AdcConfigure_getConfiguration( void ) ;

static Msp430Adc12ConfAlertC_2_ConfUp_adc_config_t Msp430Adc12ConfAlertC_2_ConfUp_getConfiguration( void ) ;

const inline static msp430adc12_channel_config_t *Msp430Adc12ConfAlertC_2_ConfSub_getConfiguration( void ) ;

inline static void __nesc_enable_interrupt( void ) 
{
  _R2 |= 0x0008;
}

inline static void Msp430TimerP_1_VectorTimerX0_fired( void ) 
{
  Msp430TimerP_1_Event_fired(0);
}

static inline void Msp430TimerCommonP_VectorTimerB0_fired( void ) 
{
  Msp430TimerP_1_VectorTimerX0_fired();
}

inline static void Msp430TimerCapComP_9_Timer_overflow( void ) 
{
}

inline static void Msp430TimerCapComP_8_Timer_overflow( void ) 
{
}

inline static void Msp430TimerCapComP_7_Timer_overflow( void ) 
{
}

inline static void Msp430TimerCapComP_6_Timer_overflow( void ) 
{
}

inline static void Msp430TimerCapComP_5_Timer_overflow( void ) 
{
}

inline static void Msp430TimerCapComP_4_Timer_overflow( void ) 
{
}

inline static void Msp430TimerCapComP_3_Timer_overflow( void ) 
{
}

inline static void Msp430AlarmC_1_Msp430Timer_overflow( void ) 
{
}

inline static void Msp430AlarmC_0_Msp430Timer_overflow( void ) 
{
}

inline static void CounterToLocalTimeC_0_Counter_overflow( void ) 
{
}

inline static void TransformAlarmC_0_Counter_overflow( void ) 
{
}

inline static void TransformAlarmC_1_Counter_overflow( void ) 
{
}

static inline void TransformCounterC_0_Counter_overflow( void ) 
{
  TransformAlarmC_1_Counter_overflow();
  TransformAlarmC_0_Counter_overflow();
  CounterToLocalTimeC_0_Counter_overflow();
}

inline static void TransformCounterC_0_CounterFrom_overflow( void ) 
{
{
  TransformCounterC_0_m_upper++;
  if((TransformCounterC_0_m_upper & TransformCounterC_0_OVERFLOW_MASK) == 0)
  {
    TransformCounterC_0_Counter_overflow();
  }
}
}

static inline void Msp430CounterC_0_Counter_overflow( void ) 
{
  TransformCounterC_0_CounterFrom_overflow();
}

inline static void Msp430CounterC_0_Msp430Timer_overflow( void ) 
{
  Msp430CounterC_0_Counter_overflow();
}

static inline void Msp430TimerP_1_Timer_overflow( void ) 
{
  Msp430CounterC_0_Msp430Timer_overflow();
  Msp430AlarmC_0_Msp430Timer_overflow();
  Msp430AlarmC_1_Msp430Timer_overflow();
  Msp430TimerCapComP_3_Timer_overflow();
  Msp430TimerCapComP_4_Timer_overflow();
  Msp430TimerCapComP_5_Timer_overflow();
  Msp430TimerCapComP_6_Timer_overflow();
  Msp430TimerCapComP_7_Timer_overflow();
  Msp430TimerCapComP_8_Timer_overflow();
  Msp430TimerCapComP_9_Timer_overflow();
}

inline static void Msp430TimerP_1_Overflow_fired( void ) 
{
  Msp430TimerP_1_Timer_overflow();
}

static inline error_t AlarmToTimerC_0_fired_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(AlarmToTimerC_0_fired);
  return result;
}

inline static void AlarmToTimerC_0_Alarm_fired( void ) 
{
  AlarmToTimerC_0_fired_postTask();
}

static inline void TransformAlarmC_0_Alarm_fired( void ) 
{
  AlarmToTimerC_0_Alarm_fired();
}

inline static void TransformAlarmC_0_AlarmFrom_fired( void ) 
{
{
  /* IRQ INSTR 0 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  if(TransformAlarmC_0_m_dt == 0)
  {
    TransformAlarmC_0_Alarm_fired();
  }
  else 
  {
    TransformAlarmC_0_set_alarm();
  }
}
}

static inline void Msp430AlarmC_0_Alarm_fired( void ) 
{
  TransformAlarmC_0_AlarmFrom_fired();
}

inline static void Msp430TimerCapComP_3_Control_disableEvents( void ) 
{
  _TBCCTL0 &= ~0x0010;
}

static inline void Msp430AlarmC_0_Msp430TimerControl_disableEvents( void ) 
{
  Msp430TimerCapComP_3_Control_disableEvents();
}

inline static void Msp430AlarmC_0_Msp430Compare_fired( void ) 
{
  Msp430AlarmC_0_Msp430TimerControl_disableEvents();
  Msp430AlarmC_0_Alarm_fired();
}

static inline void Msp430TimerCapComP_3_Compare_fired( void ) 
{
  Msp430AlarmC_0_Msp430Compare_fired();
}

inline static uint16_t Msp430TimerCapComP_3_Capture_getEvent( void ) 
{
  return _TBCCR0;
}

inline static void Msp430TimerCapComP_3_Capture_default_captured(uint16_t n) 
{
}

static inline void Msp430TimerCapComP_3_Capture_captured(uint16_t time) 
{
  Msp430TimerCapComP_3_Capture_default_captured(time);
}

inline static Msp430TimerCapComP_3_cc_t Msp430TimerCapComP_3_int2CC(uint16_t x) 
{
  union Msp430TimerCapComP_3___nesc_unnamed4303  {
  uint16_t f;
  Msp430TimerCapComP_3_cc_t t;
} c;
  c.f = x;
  return c.t;
}

inline static Msp430TimerCapComP_3_cc_t Msp430TimerCapComP_3_Control_getControl( void ) 
{
  return Msp430TimerCapComP_3_int2CC(_TBCCTL0);
}

inline static void Msp430TimerCapComP_3_Event_fired( void ) 
{
  if(Msp430TimerCapComP_3_Control_getControl().cap)
  {
    Msp430TimerCapComP_3_Capture_captured(Msp430TimerCapComP_3_Capture_getEvent());
  }
  else 
  {
    Msp430TimerCapComP_3_Compare_fired();
  }
}

inline static bool SchedulerBasicP_isWaiting(uint8_t id) 
{
  /* IRQ INSTR 1 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 2 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  return (SchedulerBasicP_m_next[id] != SchedulerBasicP_NO_TASK) || (SchedulerBasicP_m_tail == id);
}

inline static bool SchedulerBasicP_pushTask(uint8_t id) 
{
  if(!SchedulerBasicP_isWaiting(id))
  {
  /* IRQ INSTR 3 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 4 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    if(SchedulerBasicP_m_head == SchedulerBasicP_NO_TASK)
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

static inline uint16_t Msp430CounterC_0_Msp430Timer_get( void ) 
{
  int unsigned result;
  result = Msp430TimerP_1_Timer_get();
  return result;
}

inline static uint16_t Msp430CounterC_0_Counter_get( void ) 
{
  return Msp430CounterC_0_Msp430Timer_get();
}

static inline TransformCounterC_0_CounterFrom_size_type TransformCounterC_0_CounterFrom_get( void ) 
{
  int unsigned result;
  result = Msp430CounterC_0_Counter_get();
  return result;
}

inline static bool Msp430TimerP_1_Timer_isOverflowPending( void ) 
{
  return _TBCTL & 1U;
}

static inline bool Msp430CounterC_0_Msp430Timer_isOverflowPending( void ) 
{
  char unsigned result;
  result = Msp430TimerP_1_Timer_isOverflowPending();
  return result;
}

inline static bool Msp430CounterC_0_Counter_isOverflowPending( void ) 
{
  return Msp430CounterC_0_Msp430Timer_isOverflowPending();
}

static inline bool TransformCounterC_0_CounterFrom_isOverflowPending( void ) 
{
  char unsigned result;
  result = Msp430CounterC_0_Counter_isOverflowPending();
  return result;
}

inline static void Msp430TimerCapComP_3_Control_enableEvents( void ) 
{
  _TBCCTL0 |= 0x0010;
}

static inline void Msp430AlarmC_0_Msp430TimerControl_enableEvents( void ) 
{
  Msp430TimerCapComP_3_Control_enableEvents();
}

inline static void Msp430TimerCapComP_3_Control_clearPendingInterrupt( void ) 
{
  _TBCCTL0 &= ~0x0001;
}

static inline void Msp430AlarmC_0_Msp430TimerControl_clearPendingInterrupt( void ) 
{
  Msp430TimerCapComP_3_Control_clearPendingInterrupt();
}

inline static void Msp430TimerCapComP_3_Compare_setEvent(uint16_t x) 
{
  _TBCCR0 = x;
}

static inline void Msp430AlarmC_0_Msp430Compare_setEvent(uint16_t time) 
{
  Msp430TimerCapComP_3_Compare_setEvent(time);
}

static inline uint16_t Msp430TimerCapComP_3_Timer_get( void ) 
{
  int unsigned result;
  result = Msp430TimerP_1_Timer_get();
  return result;
}

inline static void Msp430TimerCapComP_3_Compare_setEventFromNow(uint16_t x) 
{
  _TBCCR0 = Msp430TimerCapComP_3_Timer_get() + x;
}

static inline void Msp430AlarmC_0_Msp430Compare_setEventFromNow(uint16_t delta) 
{
  Msp430TimerCapComP_3_Compare_setEventFromNow(delta);
}

static inline uint16_t Msp430AlarmC_0_Msp430Timer_get( void ) 
{
  int unsigned result;
  result = Msp430TimerP_1_Timer_get();
  return result;
}

inline static void Msp430AlarmC_0_Alarm_startAt(uint16_t t0, uint16_t dt) 
{
{
  uint16_t now = Msp430AlarmC_0_Msp430Timer_get();
  uint16_t elapsed = now - t0;
  if(elapsed >= dt)
  {
    Msp430AlarmC_0_Msp430Compare_setEventFromNow(2);
  }
  else 
  {
    uint16_t remaining = dt - elapsed;
    if(remaining <= 2)
    {
      Msp430AlarmC_0_Msp430Compare_setEventFromNow(2);
    }
    else 
    {
      Msp430AlarmC_0_Msp430Compare_setEvent(now + remaining);
    }
  }
  Msp430AlarmC_0_Msp430TimerControl_clearPendingInterrupt();
  Msp430AlarmC_0_Msp430TimerControl_enableEvents();
}
}

static inline void TransformAlarmC_0_AlarmFrom_startAt(TransformAlarmC_0_AlarmFrom_size_type t0, TransformAlarmC_0_AlarmFrom_size_type dt) 
{
  Msp430AlarmC_0_Alarm_startAt(t0, dt);
}

inline static void Msp430TimerCapComP_4_Compare_default_fired( void ) 
{
}

static inline void Msp430TimerCapComP_4_Compare_fired( void ) 
{
  Msp430TimerCapComP_4_Compare_default_fired();
}

inline static uint16_t Msp430TimerCapComP_4_Capture_getEvent( void ) 
{
  return _TBCCR1;
}

inline static void Msp430TimerCapComP_4_Capture_default_captured(uint16_t n) 
{
}

static inline void Msp430TimerCapComP_4_Capture_captured(uint16_t time) 
{
  Msp430TimerCapComP_4_Capture_default_captured(time);
}

inline static Msp430TimerCapComP_4_cc_t Msp430TimerCapComP_4_int2CC(uint16_t x) 
{
  union Msp430TimerCapComP_4___nesc_unnamed4304  {
  uint16_t f;
  Msp430TimerCapComP_4_cc_t t;
} c;
  c.f = x;
  return c.t;
}

inline static Msp430TimerCapComP_4_cc_t Msp430TimerCapComP_4_Control_getControl( void ) 
{
  return Msp430TimerCapComP_4_int2CC(_TBCCTL1);
}

inline static void Msp430TimerCapComP_4_Event_fired( void ) 
{
  if(Msp430TimerCapComP_4_Control_getControl().cap)
  {
    Msp430TimerCapComP_4_Capture_captured(Msp430TimerCapComP_4_Capture_getEvent());
  }
  else 
  {
    Msp430TimerCapComP_4_Compare_fired();
  }
}

inline static error_t AdcStreamP_SingleChannel_default_getData(uint8_t c) 
{
  return FAIL;
}

static inline error_t AdcStreamP_SingleChannel_getData(uint8_t arg_0x1b5c010) 
{
  char unsigned result;
  switch (arg_0x1b5c010) 
  {
    case AdcReadStreamClientC_0_RSCLIENT:
      result = Msp430Adc12ImplP_SingleChannel_getData(Msp430Adc12ClientAutoRVGC_1_ID);
      break;
    default :
      result = AdcStreamP_SingleChannel_default_getData(arg_0x1b5c010);
      break;
  }

  return result;
}

inline static void AdcStreamP_sampleSingle( void ) 
{
  /* IRQ INSTR 5 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  AdcStreamP_SingleChannel_getData(AdcStreamP_client);
}

inline static void AdcStreamP_Alarm_fired( void ) 
{
  AdcStreamP_sampleSingle();
}

static inline void TransformAlarmC_1_Alarm_fired( void ) 
{
  AdcStreamP_Alarm_fired();
}

inline static void TransformAlarmC_1_AlarmFrom_fired( void ) 
{
{
  /* IRQ INSTR 6 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 7 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  if(TransformAlarmC_1_m_dt == 0)
  {
    TransformAlarmC_1_Alarm_fired();
  }
  else 
  {
    TransformAlarmC_1_set_alarm();
  }
}
}

static inline void Msp430AlarmC_1_Alarm_fired( void ) 
{
  TransformAlarmC_1_AlarmFrom_fired();
}

inline static void Msp430TimerCapComP_5_Control_disableEvents( void ) 
{
  _TBCCTL2 &= ~0x0010;
}

static inline void Msp430AlarmC_1_Msp430TimerControl_disableEvents( void ) 
{
  Msp430TimerCapComP_5_Control_disableEvents();
}

inline static void Msp430AlarmC_1_Msp430Compare_fired( void ) 
{
  Msp430AlarmC_1_Msp430TimerControl_disableEvents();
  Msp430AlarmC_1_Alarm_fired();
}

static inline void Msp430TimerCapComP_5_Compare_fired( void ) 
{
  Msp430AlarmC_1_Msp430Compare_fired();
}

inline static uint16_t Msp430TimerCapComP_5_Capture_getEvent( void ) 
{
  return _TBCCR2;
}

inline static void Msp430TimerCapComP_5_Capture_default_captured(uint16_t n) 
{
}

static inline void Msp430TimerCapComP_5_Capture_captured(uint16_t time) 
{
  Msp430TimerCapComP_5_Capture_default_captured(time);
}

inline static Msp430TimerCapComP_5_cc_t Msp430TimerCapComP_5_int2CC(uint16_t x) 
{
  union Msp430TimerCapComP_5___nesc_unnamed4305  {
  uint16_t f;
  Msp430TimerCapComP_5_cc_t t;
} c;
  c.f = x;
  return c.t;
}

inline static Msp430TimerCapComP_5_cc_t Msp430TimerCapComP_5_Control_getControl( void ) 
{
  return Msp430TimerCapComP_5_int2CC(_TBCCTL2);
}

inline static void Msp430TimerCapComP_5_Event_fired( void ) 
{
  if(Msp430TimerCapComP_5_Control_getControl().cap)
  {
    Msp430TimerCapComP_5_Capture_captured(Msp430TimerCapComP_5_Capture_getEvent());
  }
  else 
  {
    Msp430TimerCapComP_5_Compare_fired();
  }
}

inline static void HplMsp430GeneralIOP_47_IO_makeInput( void ) 
{
  _P6DIR &= ~0x01 << 7;
}

static inline void Msp430Adc12ImplP_Port67_makeInput( void ) 
{
  HplMsp430GeneralIOP_47_IO_makeInput();
}

inline static void HplMsp430GeneralIOP_47_IO_selectModuleFunc( void ) 
{
  _P6SEL |= 0x01 << 7;
}

static inline void Msp430Adc12ImplP_Port67_selectModuleFunc( void ) 
{
  HplMsp430GeneralIOP_47_IO_selectModuleFunc();
}

inline static void HplMsp430GeneralIOP_46_IO_makeInput( void ) 
{
  _P6DIR &= ~0x01 << 6;
}

static inline void Msp430Adc12ImplP_Port66_makeInput( void ) 
{
  HplMsp430GeneralIOP_46_IO_makeInput();
}

inline static void HplMsp430GeneralIOP_46_IO_selectModuleFunc( void ) 
{
  _P6SEL |= 0x01 << 6;
}

static inline void Msp430Adc12ImplP_Port66_selectModuleFunc( void ) 
{
  HplMsp430GeneralIOP_46_IO_selectModuleFunc();
}

inline static void HplMsp430GeneralIOP_45_IO_makeInput( void ) 
{
  _P6DIR &= ~0x01 << 5;
}

static inline void Msp430Adc12ImplP_Port65_makeInput( void ) 
{
  HplMsp430GeneralIOP_45_IO_makeInput();
}

inline static void HplMsp430GeneralIOP_45_IO_selectModuleFunc( void ) 
{
  _P6SEL |= 0x01 << 5;
}

static inline void Msp430Adc12ImplP_Port65_selectModuleFunc( void ) 
{
  HplMsp430GeneralIOP_45_IO_selectModuleFunc();
}

inline static void HplMsp430GeneralIOP_44_IO_makeInput( void ) 
{
  _P6DIR &= ~0x01 << 4;
}

static inline void Msp430Adc12ImplP_Port64_makeInput( void ) 
{
  HplMsp430GeneralIOP_44_IO_makeInput();
}

inline static void HplMsp430GeneralIOP_44_IO_selectModuleFunc( void ) 
{
  _P6SEL |= 0x01 << 4;
}

static inline void Msp430Adc12ImplP_Port64_selectModuleFunc( void ) 
{
  HplMsp430GeneralIOP_44_IO_selectModuleFunc();
}

inline static void HplMsp430GeneralIOP_43_IO_makeInput( void ) 
{
  _P6DIR &= ~0x01 << 3;
}

static inline void Msp430Adc12ImplP_Port63_makeInput( void ) 
{
  HplMsp430GeneralIOP_43_IO_makeInput();
}

inline static void HplMsp430GeneralIOP_43_IO_selectModuleFunc( void ) 
{
  _P6SEL |= 0x01 << 3;
}

static inline void Msp430Adc12ImplP_Port63_selectModuleFunc( void ) 
{
  HplMsp430GeneralIOP_43_IO_selectModuleFunc();
}

inline static void HplMsp430GeneralIOP_42_IO_makeInput( void ) 
{
  _P6DIR &= ~0x01 << 2;
}

static inline void Msp430Adc12ImplP_Port62_makeInput( void ) 
{
  HplMsp430GeneralIOP_42_IO_makeInput();
}

inline static void HplMsp430GeneralIOP_42_IO_selectModuleFunc( void ) 
{
  _P6SEL |= 0x01 << 2;
}

static inline void Msp430Adc12ImplP_Port62_selectModuleFunc( void ) 
{
  HplMsp430GeneralIOP_42_IO_selectModuleFunc();
}

inline static void HplMsp430GeneralIOP_41_IO_makeInput( void ) 
{
  _P6DIR &= ~0x01 << 1;
}

static inline void Msp430Adc12ImplP_Port61_makeInput( void ) 
{
  HplMsp430GeneralIOP_41_IO_makeInput();
}

inline static void HplMsp430GeneralIOP_41_IO_selectModuleFunc( void ) 
{
  _P6SEL |= 0x01 << 1;
}

static inline void Msp430Adc12ImplP_Port61_selectModuleFunc( void ) 
{
  HplMsp430GeneralIOP_41_IO_selectModuleFunc();
}

inline static void HplMsp430GeneralIOP_40_IO_makeInput( void ) 
{
  _P6DIR &= ~0x01 << 0;
}

static inline void Msp430Adc12ImplP_Port60_makeInput( void ) 
{
  HplMsp430GeneralIOP_40_IO_makeInput();
}

inline static void HplMsp430GeneralIOP_40_IO_selectModuleFunc( void ) 
{
  _P6SEL |= 0x01 << 0;
}

static inline void Msp430Adc12ImplP_Port60_selectModuleFunc( void ) 
{
  HplMsp430GeneralIOP_40_IO_selectModuleFunc();
}

inline static void Msp430Adc12ImplP_configureAdcPin(uint8_t inch) 
{
  switch (inch) 
  {
    case 0:
      Msp430Adc12ImplP_Port60_selectModuleFunc();
    Msp430Adc12ImplP_Port60_makeInput();
      break;
    case 1:
      Msp430Adc12ImplP_Port61_selectModuleFunc();
    Msp430Adc12ImplP_Port61_makeInput();
      break;
    case 2:
      Msp430Adc12ImplP_Port62_selectModuleFunc();
    Msp430Adc12ImplP_Port62_makeInput();
      break;
    case 3:
      Msp430Adc12ImplP_Port63_selectModuleFunc();
    Msp430Adc12ImplP_Port63_makeInput();
      break;
    case 4:
      Msp430Adc12ImplP_Port64_selectModuleFunc();
    Msp430Adc12ImplP_Port64_makeInput();
      break;
    case 5:
      Msp430Adc12ImplP_Port65_selectModuleFunc();
    Msp430Adc12ImplP_Port65_makeInput();
      break;
    case 6:
      Msp430Adc12ImplP_Port66_selectModuleFunc();
    Msp430Adc12ImplP_Port66_makeInput();
      break;
    case 7:
      Msp430Adc12ImplP_Port67_selectModuleFunc();
    Msp430Adc12ImplP_Port67_makeInput();
      break;
  }

}

inline static void HplAdc12P_HplAdc12_startConversion( void ) 
{
  _ADC12CTL0 |= 0x0010;
  _ADC12CTL0 |= 0x0001 + 0x0002;
}

static inline void Msp430Adc12ImplP_HplAdc12_startConversion( void ) 
{
  HplAdc12P_HplAdc12_startConversion();
}

static inline void Msp430Adc12ImplP_TimerA_setMode(int mode) 
{
  Msp430TimerP_0_Timer_setMode(mode);
}

inline static uint16_t Msp430TimerCapComP_1_CC2int(Msp430TimerCapComP_1_cc_t x) 
{
  union Msp430TimerCapComP_1___nesc_unnamed4306  {
  Msp430TimerCapComP_1_cc_t f;
  uint16_t t;
} c;
  c.f = x;
  return c.t;
}

inline static void Msp430TimerCapComP_1_Control_setControl(Msp430TimerCapComP_1_cc_t x) 
{
  _TACCTL1 = Msp430TimerCapComP_1_CC2int(x);
}

static inline void Msp430Adc12ImplP_ControlA1_setControl(msp430_compare_control_t control) 
{
  Msp430TimerCapComP_1_Control_setControl(control);
}

inline static void Msp430Adc12ImplP_startTimerA( void ) 
{
  msp430_compare_control_t ccSetSHI = {.ccifg = 0, .cov = 0, .out = 1, .cci = 0, .ccie = 0, .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0};
  msp430_compare_control_t ccResetSHI = {.ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0};
  msp430_compare_control_t ccRSOutmod = {.ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, .outmod = 7, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0};
  Msp430Adc12ImplP_ControlA1_setControl(ccResetSHI);
  Msp430Adc12ImplP_ControlA1_setControl(ccSetSHI);
  Msp430Adc12ImplP_ControlA1_setControl(ccRSOutmod);
  Msp430Adc12ImplP_TimerA_setMode(MSP430TIMER_UP_MODE);
}

inline static void Msp430TimerCapComP_5_Control_enableEvents( void ) 
{
  _TBCCTL2 |= 0x0010;
}

static inline void Msp430AlarmC_1_Msp430TimerControl_enableEvents( void ) 
{
  Msp430TimerCapComP_5_Control_enableEvents();
}

inline static void Msp430TimerCapComP_5_Control_clearPendingInterrupt( void ) 
{
  _TBCCTL2 &= ~0x0001;
}

static inline void Msp430AlarmC_1_Msp430TimerControl_clearPendingInterrupt( void ) 
{
  Msp430TimerCapComP_5_Control_clearPendingInterrupt();
}

inline static void Msp430TimerCapComP_5_Compare_setEvent(uint16_t x) 
{
  _TBCCR2 = x;
}

static inline void Msp430AlarmC_1_Msp430Compare_setEvent(uint16_t time) 
{
  Msp430TimerCapComP_5_Compare_setEvent(time);
}

static inline uint16_t Msp430TimerCapComP_5_Timer_get( void ) 
{
  int unsigned result;
  result = Msp430TimerP_1_Timer_get();
  return result;
}

inline static void Msp430TimerCapComP_5_Compare_setEventFromNow(uint16_t x) 
{
  _TBCCR2 = Msp430TimerCapComP_5_Timer_get() + x;
}

static inline void Msp430AlarmC_1_Msp430Compare_setEventFromNow(uint16_t delta) 
{
  Msp430TimerCapComP_5_Compare_setEventFromNow(delta);
}

static inline uint16_t Msp430AlarmC_1_Msp430Timer_get( void ) 
{
  int unsigned result;
  result = Msp430TimerP_1_Timer_get();
  return result;
}

inline static void Msp430AlarmC_1_Alarm_startAt(uint16_t t0, uint16_t dt) 
{
{
  uint16_t now = Msp430AlarmC_1_Msp430Timer_get();
  uint16_t elapsed = now - t0;
  if(elapsed >= dt)
  {
    Msp430AlarmC_1_Msp430Compare_setEventFromNow(2);
  }
  else 
  {
    uint16_t remaining = dt - elapsed;
    if(remaining <= 2)
    {
      Msp430AlarmC_1_Msp430Compare_setEventFromNow(2);
    }
    else 
    {
      Msp430AlarmC_1_Msp430Compare_setEvent(now + remaining);
    }
  }
  Msp430AlarmC_1_Msp430TimerControl_clearPendingInterrupt();
  Msp430AlarmC_1_Msp430TimerControl_enableEvents();
}
}

static inline void TransformAlarmC_1_AlarmFrom_startAt(TransformAlarmC_1_AlarmFrom_size_type t0, TransformAlarmC_1_AlarmFrom_size_type dt) 
{
  Msp430AlarmC_1_Alarm_startAt(t0, dt);
}

inline static void Msp430TimerCapComP_6_Compare_default_fired( void ) 
{
}

static inline void Msp430TimerCapComP_6_Compare_fired( void ) 
{
  Msp430TimerCapComP_6_Compare_default_fired();
}

inline static uint16_t Msp430TimerCapComP_6_Capture_getEvent( void ) 
{
  return _TBCCR3;
}

inline static void Msp430TimerCapComP_6_Capture_default_captured(uint16_t n) 
{
}

static inline void Msp430TimerCapComP_6_Capture_captured(uint16_t time) 
{
  Msp430TimerCapComP_6_Capture_default_captured(time);
}

inline static Msp430TimerCapComP_6_cc_t Msp430TimerCapComP_6_int2CC(uint16_t x) 
{
  union Msp430TimerCapComP_6___nesc_unnamed4307  {
  uint16_t f;
  Msp430TimerCapComP_6_cc_t t;
} c;
  c.f = x;
  return c.t;
}

inline static Msp430TimerCapComP_6_cc_t Msp430TimerCapComP_6_Control_getControl( void ) 
{
  return Msp430TimerCapComP_6_int2CC(_TBCCTL3);
}

inline static void Msp430TimerCapComP_6_Event_fired( void ) 
{
  if(Msp430TimerCapComP_6_Control_getControl().cap)
  {
    Msp430TimerCapComP_6_Capture_captured(Msp430TimerCapComP_6_Capture_getEvent());
  }
  else 
  {
    Msp430TimerCapComP_6_Compare_fired();
  }
}

inline static void Msp430TimerCapComP_7_Compare_default_fired( void ) 
{
}

static inline void Msp430TimerCapComP_7_Compare_fired( void ) 
{
  Msp430TimerCapComP_7_Compare_default_fired();
}

inline static uint16_t Msp430TimerCapComP_7_Capture_getEvent( void ) 
{
  return _TBCCR4;
}

inline static void Msp430TimerCapComP_7_Capture_default_captured(uint16_t n) 
{
}

static inline void Msp430TimerCapComP_7_Capture_captured(uint16_t time) 
{
  Msp430TimerCapComP_7_Capture_default_captured(time);
}

inline static Msp430TimerCapComP_7_cc_t Msp430TimerCapComP_7_int2CC(uint16_t x) 
{
  union Msp430TimerCapComP_7___nesc_unnamed4308  {
  uint16_t f;
  Msp430TimerCapComP_7_cc_t t;
} c;
  c.f = x;
  return c.t;
}

inline static Msp430TimerCapComP_7_cc_t Msp430TimerCapComP_7_Control_getControl( void ) 
{
  return Msp430TimerCapComP_7_int2CC(_TBCCTL4);
}

inline static void Msp430TimerCapComP_7_Event_fired( void ) 
{
  if(Msp430TimerCapComP_7_Control_getControl().cap)
  {
    Msp430TimerCapComP_7_Capture_captured(Msp430TimerCapComP_7_Capture_getEvent());
  }
  else 
  {
    Msp430TimerCapComP_7_Compare_fired();
  }
}

inline static void Msp430TimerCapComP_8_Compare_default_fired( void ) 
{
}

static inline void Msp430TimerCapComP_8_Compare_fired( void ) 
{
  Msp430TimerCapComP_8_Compare_default_fired();
}

inline static uint16_t Msp430TimerCapComP_8_Capture_getEvent( void ) 
{
  return _TBCCR5;
}

inline static void Msp430TimerCapComP_8_Capture_default_captured(uint16_t n) 
{
}

static inline void Msp430TimerCapComP_8_Capture_captured(uint16_t time) 
{
  Msp430TimerCapComP_8_Capture_default_captured(time);
}

inline static Msp430TimerCapComP_8_cc_t Msp430TimerCapComP_8_int2CC(uint16_t x) 
{
  union Msp430TimerCapComP_8___nesc_unnamed4309  {
  uint16_t f;
  Msp430TimerCapComP_8_cc_t t;
} c;
  c.f = x;
  return c.t;
}

inline static Msp430TimerCapComP_8_cc_t Msp430TimerCapComP_8_Control_getControl( void ) 
{
  return Msp430TimerCapComP_8_int2CC(_TBCCTL5);
}

inline static void Msp430TimerCapComP_8_Event_fired( void ) 
{
  if(Msp430TimerCapComP_8_Control_getControl().cap)
  {
    Msp430TimerCapComP_8_Capture_captured(Msp430TimerCapComP_8_Capture_getEvent());
  }
  else 
  {
    Msp430TimerCapComP_8_Compare_fired();
  }
}

inline static void Msp430TimerCapComP_9_Compare_default_fired( void ) 
{
}

static inline void Msp430TimerCapComP_9_Compare_fired( void ) 
{
  Msp430TimerCapComP_9_Compare_default_fired();
}

inline static uint16_t Msp430TimerCapComP_9_Capture_getEvent( void ) 
{
  return _TBCCR6;
}

inline static void Msp430TimerCapComP_9_Capture_default_captured(uint16_t n) 
{
}

static inline void Msp430TimerCapComP_9_Capture_captured(uint16_t time) 
{
  Msp430TimerCapComP_9_Capture_default_captured(time);
}

inline static Msp430TimerCapComP_9_cc_t Msp430TimerCapComP_9_int2CC(uint16_t x) 
{
  union Msp430TimerCapComP_9___nesc_unnamed4310  {
  uint16_t f;
  Msp430TimerCapComP_9_cc_t t;
} c;
  c.f = x;
  return c.t;
}

inline static Msp430TimerCapComP_9_cc_t Msp430TimerCapComP_9_Control_getControl( void ) 
{
  return Msp430TimerCapComP_9_int2CC(_TBCCTL6);
}

inline static void Msp430TimerCapComP_9_Event_fired( void ) 
{
  if(Msp430TimerCapComP_9_Control_getControl().cap)
  {
    Msp430TimerCapComP_9_Capture_captured(Msp430TimerCapComP_9_Capture_getEvent());
  }
  else 
  {
    Msp430TimerCapComP_9_Compare_fired();
  }
}

inline static void SchedulerBasicP_Scheduler_init( void ) 
{
{
  /* IRQ INSTR 8 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 9 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  memset(( void * )(SchedulerBasicP_m_next), SchedulerBasicP_NO_TASK, sizeof(SchedulerBasicP_m_next));
  SchedulerBasicP_m_head = SchedulerBasicP_NO_TASK;
  SchedulerBasicP_m_tail = SchedulerBasicP_NO_TASK;
}
}

static inline void RealMainP_Scheduler_init( void ) 
{
  SchedulerBasicP_Scheduler_init();
}

static inline void Msp430GpioC_2_HplGeneralIO_set( void ) 
{
  HplMsp430GeneralIOP_38_IO_set();
}

inline static void Msp430GpioC_2_GeneralIO_set( void ) 
{
  Msp430GpioC_2_HplGeneralIO_set();
}

static inline void LedsP_Led2_set( void ) 
{
  Msp430GpioC_2_GeneralIO_set();
}

static inline void Msp430GpioC_1_HplGeneralIO_set( void ) 
{
  HplMsp430GeneralIOP_37_IO_set();
}

inline static void Msp430GpioC_1_GeneralIO_set( void ) 
{
  Msp430GpioC_1_HplGeneralIO_set();
}

static inline void LedsP_Led1_set( void ) 
{
  Msp430GpioC_1_GeneralIO_set();
}

static inline void Msp430GpioC_0_HplGeneralIO_set( void ) 
{
  HplMsp430GeneralIOP_36_IO_set();
}

inline static void Msp430GpioC_0_GeneralIO_set( void ) 
{
  Msp430GpioC_0_HplGeneralIO_set();
}

static inline void LedsP_Led0_set( void ) 
{
  Msp430GpioC_0_GeneralIO_set();
}

inline static void HplMsp430GeneralIOP_38_IO_makeOutput( void ) 
{
  _P5DIR |= 0x01 << 6;
}

static inline void Msp430GpioC_2_HplGeneralIO_makeOutput( void ) 
{
  HplMsp430GeneralIOP_38_IO_makeOutput();
}

inline static void Msp430GpioC_2_GeneralIO_makeOutput( void ) 
{
  Msp430GpioC_2_HplGeneralIO_makeOutput();
}

static inline void LedsP_Led2_makeOutput( void ) 
{
  Msp430GpioC_2_GeneralIO_makeOutput();
}

inline static void HplMsp430GeneralIOP_37_IO_makeOutput( void ) 
{
  _P5DIR |= 0x01 << 5;
}

static inline void Msp430GpioC_1_HplGeneralIO_makeOutput( void ) 
{
  HplMsp430GeneralIOP_37_IO_makeOutput();
}

inline static void Msp430GpioC_1_GeneralIO_makeOutput( void ) 
{
  Msp430GpioC_1_HplGeneralIO_makeOutput();
}

static inline void LedsP_Led1_makeOutput( void ) 
{
  Msp430GpioC_1_GeneralIO_makeOutput();
}

inline static void HplMsp430GeneralIOP_36_IO_makeOutput( void ) 
{
  _P5DIR |= 0x01 << 4;
}

static inline void Msp430GpioC_0_HplGeneralIO_makeOutput( void ) 
{
  HplMsp430GeneralIOP_36_IO_makeOutput();
}

inline static void Msp430GpioC_0_GeneralIO_makeOutput( void ) 
{
  Msp430GpioC_0_HplGeneralIO_makeOutput();
}

static inline void LedsP_Led0_makeOutput( void ) 
{
  Msp430GpioC_0_GeneralIO_makeOutput();
}

inline static error_t LedsP_Init_init( void ) 
{
{
  LedsP_Led0_makeOutput();
  LedsP_Led1_makeOutput();
  LedsP_Led2_makeOutput();
  LedsP_Led0_set();
  LedsP_Led1_set();
  LedsP_Led2_set();
}
  return SUCCESS;
}

static inline error_t PlatformP_LedsInit_init( void ) 
{
  char unsigned result;
  result = LedsP_Init_init();
  return result;
}

inline static void TOSH_SET_SIMO0_PIN() 
{

  _P3OUT |= 1 << 1;
}

inline static void TOSH_SET_UCLK0_PIN() 
{

  _P3OUT |= 1 << 3;
}

inline static void TOSH_SET_FLASH_CS_PIN() 
{

  _P3OUT |= 1 << 4;
}

inline static void TOSH_CLR_UCLK0_PIN() 
{

  _P4OUT &= ~1 << 3;
}

inline static void TOSH_CLR_FLASH_CS_PIN() 
{

  _P3OUT &= ~1 << 4;
}

__inline static void MotePlatformC_TOSH_wait( void ) 
{
}

inline static void TOSH_SET_FLASH_HOLD_PIN() 
{

  _P4OUT |= 1 << 7;
}

inline static void TOSH_MAKE_FLASH_CS_OUTPUT() 
{

  _P4OUT |= 1 << 4;
}

inline static void TOSH_MAKE_FLASH_HOLD_OUTPUT() 
{

  _P4DIR |= 1 << 7;
}

inline static void TOSH_MAKE_UCLK0_OUTPUT() 
{

  _P4DIR |= 1 << 3;
}

inline static void TOSH_MAKE_SIMO0_OUTPUT() 
{

  _P3DIR |= 1 << 1;
}

inline static void MotePlatformC_TOSH_FLASH_M25P_DP( void ) 
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

inline static error_t MotePlatformC_Init_init( void ) 
{
{
  _P1SEL = 0;
  _P2SEL = 0;
  _P3SEL = 0;
  _P4SEL = 0;
  _P5SEL = 0;
  _P6SEL = 0;
  _P1OUT = 0x00;
  _P1DIR = 0xe0;
  _P2OUT = 0x30;
  _P2DIR = 0x7b;
  _P3OUT = 0x00;
  _P3DIR = 0xf1;
  _P4OUT = 0xdd;
  _P4DIR = 0xfd;
  _P5OUT = 0xff;
  _P5DIR = 0xff;
  _P6OUT = 0x00;
  _P6DIR = 0xff;
  _P1IE = 0;
  _P2IE = 0;
  MotePlatformC_TOSH_FLASH_M25P_DP();
}
  return SUCCESS;
}

static inline error_t PlatformP_MoteInit_init( void ) 
{
  char unsigned result;
  result = MotePlatformC_Init_init();
  return result;
}

inline static void Msp430ClockP_startTimerB( void ) 
{
  _TBCTL = 0x0020 | (_TBCTL & ~0x0020 | 0x0010);
}

inline static void Msp430ClockP_startTimerA( void ) 
{
  _TACTL = 0x0020 | (_TACTL & ~0x0020 | 0x0010);
}

inline static void Msp430ClockP_Msp430ClockInit_defaultInitTimerB( void ) 
{
  _TBR = 0;
  _TBCTL = 0x0100 | 0x0002;
}

inline static void Msp430ClockP_Msp430ClockInit_default_initTimerB( void ) 
{
  Msp430ClockP_Msp430ClockInit_defaultInitTimerB();
}

static inline void Msp430ClockP_Msp430ClockInit_initTimerB( void ) 
{
  Msp430ClockP_Msp430ClockInit_default_initTimerB();
}

inline static void Msp430ClockP_Msp430ClockInit_defaultInitTimerA( void ) 
{
  _TAR = 0;
  _TACTL = 0x0200 | 0x0002;
}

inline static void Msp430ClockP_Msp430ClockInit_default_initTimerA( void ) 
{
  Msp430ClockP_Msp430ClockInit_defaultInitTimerA();
}

static inline void Msp430ClockP_Msp430ClockInit_initTimerA( void ) 
{
  Msp430ClockP_Msp430ClockInit_default_initTimerA();
}

inline static void Msp430ClockP_Msp430ClockInit_defaultInitClocks( void ) 
{
  _BCSCTL1 = 0x80 | (_BCSCTL1 & ((0x04 | 0x02) | 0x01));
  _BCSCTL2 = 0x04;
  _IE1 &= ~1 << 1;
}

inline static void Msp430ClockP_Msp430ClockInit_default_initClocks( void ) 
{
  Msp430ClockP_Msp430ClockInit_defaultInitClocks();
}

static inline void Msp430ClockP_Msp430ClockInit_initClocks( void ) 
{
  Msp430ClockP_Msp430ClockInit_default_initClocks();
}

inline static error_t Msp430ClockP_Init_init( void ) 
{
  _TACTL = 0x0004;
  _TAIV = 0;
  _TBCTL = 0x0004;
  _TBIV = 0;
{
  Msp430ClockP_Msp430ClockInit_initClocks();
  Msp430ClockP_Msp430ClockInit_initTimerA();
  Msp430ClockP_Msp430ClockInit_initTimerB();
  Msp430ClockP_startTimerA();
  Msp430ClockP_startTimerB();
}
  return SUCCESS;
}

static inline error_t PlatformP_MoteClockInit_init( void ) 
{
  char unsigned result;
  result = Msp430ClockP_Init_init();
  return result;
}

inline static error_t PlatformP_Init_init( void ) 
{
  PlatformP_MoteClockInit_init();
  PlatformP_MoteInit_init();
  PlatformP_LedsInit_init();
  return SUCCESS;
}

static inline error_t RealMainP_PlatformInit_init( void ) 
{
  char unsigned result;
  result = PlatformP_Init_init();
  return result;
}

inline static void TOSH_CLR_SIMO0_PIN() 
{

  _P3DIR &= ~1 << 1;
}

static inline bool RealMainP_Scheduler_runNextTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_Scheduler_runNextTask();
  return result;
}

inline static void ArbitratedReadStreamC_0_ReadStream_default_bufferDone(uint8_t client, error_t result, ArbitratedReadStreamC_0_val_t *buf, uint16_t count) 
{
}

static inline void ArbitratedReadStreamC_0_ReadStream_bufferDone(uint8_t arg_0x1bc7bb0, error_t result, ArbitratedReadStreamC_0_ReadStream_val_t *buf, uint16_t count) 
{
  ArbitratedReadStreamC_0_ReadStream_default_bufferDone(arg_0x1bc7bb0, result, buf, count);
}

inline static void ArbitratedReadStreamC_0_Service_bufferDone(uint8_t client, error_t result, ArbitratedReadStreamC_0_val_t *buf, uint16_t count) 
{
  ArbitratedReadStreamC_0_ReadStream_bufferDone(client, result, buf, count);
}

static inline void AdcStreamP_ReadStream_bufferDone(uint8_t arg_0x1b5f340, error_t result, AdcStreamP_ReadStream_val_t *buf, uint16_t count) 
{
  ArbitratedReadStreamC_0_Service_bufferDone(arg_0x1b5f340, result, buf, count);
}

inline static void AdcStreamP_bufferDone_runTask( void ) 
{
  uint16_t *b;
  uint16_t c;
{
  /* IRQ INSTR 12 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 13 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  /* IRQ INSTR 10 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  b = AdcStreamP_lastBuffer;
  /* IRQ INSTR 11 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  c = AdcStreamP_lastCount;
  AdcStreamP_lastBuffer = _IE1;
}
  __nesc_atomic_end(__nesc_atomic);
}
  /* IRQ INSTR 14 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  AdcStreamP_ReadStream_bufferDone(AdcStreamP_client, SUCCESS, b, c);
}

inline static void ArbitratedReadStreamC_0_ReadStream_default_readDone(uint8_t client, error_t result, uint32_t actualPeriod) 
{
}

static inline void ArbitratedReadStreamC_0_ReadStream_readDone(uint8_t arg_0x1bc7bb0, error_t result, uint32_t usActualPeriod) 
{
  ArbitratedReadStreamC_0_ReadStream_default_readDone(arg_0x1bc7bb0, result, usActualPeriod);
}

inline static error_t ArbitratedReadStreamC_0_Resource_default_release(uint8_t client) 
{
  return FAIL;
}

static inline error_t ArbitratedReadStreamC_0_Resource_release(uint8_t arg_0x1bbf278) 
{
  char unsigned result;
  switch (arg_0x1bbf278) 
  {
    case AdcReadStreamClientC_0_RSCLIENT:
      result = Msp430RefVoltArbiterImplP_ClientResource_release(Msp430Adc12ClientAutoRVGC_1_ID);
      break;
    default :
      result = ArbitratedReadStreamC_0_Resource_default_release(arg_0x1bbf278);
      break;
  }

  return result;
}

inline static void ArbitratedReadStreamC_0_Service_readDone(uint8_t client, error_t result, uint32_t actualPeriod) 
{
  ArbitratedReadStreamC_0_Resource_release(client);
  ArbitratedReadStreamC_0_ReadStream_readDone(client, result, actualPeriod);
}

static inline void AdcStreamP_ReadStream_readDone(uint8_t arg_0x1b5f340, error_t result, uint32_t usActualPeriod) 
{
  ArbitratedReadStreamC_0_Service_readDone(arg_0x1b5f340, result, usActualPeriod);
}

inline static void AdcStreamP_readStreamFail_runTask( void ) 
{
  struct AdcStreamP_list_entry_t  *entry;
  /* IRQ INSTR 20 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  uint8_t c = AdcStreamP_client;
{
  /* IRQ INSTR 16 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 17 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  /* IRQ INSTR 15 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  entry = AdcStreamP_bufferQueue[c];
  __nesc_atomic_end(__nesc_atomic);
}
  for(; entry; entry = entry->next)
  {
    uint16_t tmp_count = entry->count;
    AdcStreamP_ReadStream_bufferDone(c, FAIL, ( uint16_t * )(entry), entry->count);
  }

{
  /* IRQ INSTR 18 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 19 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  AdcStreamP_bufferQueue[c] = _IE1;
  AdcStreamP_bufferQueueEnd[c] = &AdcStreamP_bufferQueue[c];
}
  __nesc_atomic_end(__nesc_atomic);
}
  AdcStreamP_client = AdcStreamP_NSTREAM;
  AdcStreamP_ReadStream_readDone(c, FAIL, 0);
}

inline static error_t Msp430RefVoltArbiterImplP_AdcResource_default_release(uint8_t client) 
{
  return FAIL;
}

static inline error_t Msp430RefVoltArbiterImplP_AdcResource_release(uint8_t arg_0x1af9ac8) 
{
  char unsigned result;
  switch (arg_0x1af9ac8) 
  {
    case Msp430Adc12ClientAutoRVGC_0_ID:
      result = SimpleArbiterP_0_Resource_release(Msp430Adc12ClientAutoRVGC_0_ID);
      break;
    case Msp430Adc12ClientAutoRVGC_1_ID:
      result = SimpleArbiterP_0_Resource_release(Msp430Adc12ClientAutoRVGC_1_ID);
      break;
    case Msp430Adc12ClientAutoRVGC_2_ID:
      result = SimpleArbiterP_0_Resource_release(Msp430Adc12ClientAutoRVGC_2_ID);
      break;
    default :
      result = Msp430RefVoltArbiterImplP_AdcResource_default_release(arg_0x1af9ac8);
      break;
  }

  return result;
}

inline static bool RoundRobinResourceQueueC_0_RoundRobinQueue_isEmpty( void ) 
{
  int i;
{
  for(i = 0; i < sizeof(RoundRobinResourceQueueC_0_resQ); i++)
    if(RoundRobinResourceQueueC_0_resQ[i] > 0)
    {
      char unsigned __nesc_temp = FALSE;
      return __nesc_temp;
    }

{
  char unsigned __nesc_temp = TRUE;
  return __nesc_temp;
}
}
}

static inline bool SimpleArbiterP_0_Queue_isEmpty( void ) 
{
  char unsigned result;
  result = RoundRobinResourceQueueC_0_RoundRobinQueue_isEmpty();
  return result;
}

inline static void RoundRobinResourceQueueC_0_clearEntry(uint8_t id) 
{
  RoundRobinResourceQueueC_0_resQ[id / 8] &= ~1 << (id % 8);
}

inline static resource_client_id_t RoundRobinResourceQueueC_0_RoundRobinQueue_dequeue( void ) 
{
  int i;
{
  for(i = RoundRobinResourceQueueC_0_last + 1; ; i++)
  {
    if(i == 3U)
    {
      i = 0;
    }
    if(RoundRobinResourceQueueC_0_RoundRobinQueue_isEnqueued(i))
    {
      RoundRobinResourceQueueC_0_clearEntry(i);
      RoundRobinResourceQueueC_0_last = i;
    {
      char unsigned __nesc_temp = i;
      return __nesc_temp;
    }
    }
    if(i == RoundRobinResourceQueueC_0_last)
    {
        break;
    }
  }

{
  char unsigned __nesc_temp = RoundRobinResourceQueueC_0_NO_ENTRY;
  return __nesc_temp;
}
}
}

static inline resource_client_id_t SimpleArbiterP_0_Queue_dequeue( void ) 
{
  char unsigned result;
  result = RoundRobinResourceQueueC_0_RoundRobinQueue_dequeue();
  return result;
}

static inline error_t SimpleArbiterP_0_grantedTask_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(SimpleArbiterP_0_grantedTask);
  return result;
}

inline static void SimpleArbiterP_0_ResourceConfigure_default_unconfigure(uint8_t id) 
{
}

static inline void SimpleArbiterP_0_ResourceConfigure_unconfigure(uint8_t arg_0x1a7f030) 
{
  SimpleArbiterP_0_ResourceConfigure_default_unconfigure(arg_0x1a7f030);
}

inline static void AdcStreamP_readStreamDone_runTask( void ) 
{
  /* IRQ INSTR 23 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  uint8_t c = AdcStreamP_client;
  uint32_t actualPeriod = AdcStreamP_period;
  if(AdcStreamP_periodModified)
  {
    actualPeriod = AdcStreamP_period - (AdcStreamP_period % 1000);
  }
{
  /* IRQ INSTR 21 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 22 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  AdcStreamP_bufferQueue[c] = _IE1;
  AdcStreamP_bufferQueueEnd[c] = &AdcStreamP_bufferQueue[c];
}
  __nesc_atomic_end(__nesc_atomic);
}
  AdcStreamP_client = AdcStreamP_NSTREAM;
  AdcStreamP_ReadStream_readDone(c, SUCCESS, actualPeriod);
}

static inline error_t Msp430RefVoltArbiterImplP_switchOff_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(Msp430RefVoltArbiterImplP_switchOff);
  return result;
}

static inline TransformAlarmC_0_Counter_size_type TransformAlarmC_0_Counter_get( void ) 
{
  long unsigned result;
  result = TransformCounterC_0_Counter_get();
  return result;
}

inline static TransformAlarmC_0_to_size_type TransformAlarmC_0_Alarm_getNow( void ) 
{
  return TransformAlarmC_0_Counter_get();
}

static inline AlarmToTimerC_0_Alarm_size_type AlarmToTimerC_0_Alarm_getNow( void ) 
{
  long unsigned result;
  result = TransformAlarmC_0_Alarm_getNow();
  return result;
}

inline static uint32_t AlarmToTimerC_0_Timer_getNow( void ) 
{
  return AlarmToTimerC_0_Alarm_getNow();
}

static inline uint32_t VirtualizeTimerC_0_TimerFrom_getNow( void ) 
{
  long unsigned result;
  result = AlarmToTimerC_0_Timer_getNow();
  return result;
}

inline static void VirtualizeTimerC_0_Timer_startOneShot(uint8_t num, uint32_t dt) 
{
  VirtualizeTimerC_0_startTimer(num, VirtualizeTimerC_0_TimerFrom_getNow(), dt, TRUE);
}

static inline void Msp430RefVoltGeneratorP_SwitchOffTimer_startOneShot(uint32_t dt) 
{
  VirtualizeTimerC_0_Timer_startOneShot(2U, dt);
}

inline static void Msp430RefVoltArbiterImplP_RefVolt_2_5V_stopDone(error_t error) 
{
}

static inline void Msp430RefVoltGeneratorP_RefVolt_2_5V_stopDone(error_t error) 
{
  Msp430RefVoltArbiterImplP_RefVolt_2_5V_stopDone(error);
}

inline static void Msp430RefVoltArbiterImplP_RefVolt_1_5V_stopDone(error_t error) 
{
}

static inline void Msp430RefVoltGeneratorP_RefVolt_1_5V_stopDone(error_t error) 
{
  Msp430RefVoltArbiterImplP_RefVolt_1_5V_stopDone(error);
}

inline static void VirtualizeTimerC_0_Timer_stop(uint8_t num) 
{
  VirtualizeTimerC_0_m_timers[num].isrunning = FALSE;
}

static inline void Msp430RefVoltGeneratorP_SwitchOnTimer_stop( void ) 
{
  VirtualizeTimerC_0_Timer_stop(1U);
}

inline static error_t Msp430RefVoltGeneratorP_RefVolt_1_5V_stop( void ) 
{
  switch (Msp430RefVoltGeneratorP_state) 
  {
    case Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING:
      case Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING:
        if(Msp430RefVoltGeneratorP_switchOff() == SUCCESS)
        {
          Msp430RefVoltGeneratorP_SwitchOnTimer_stop();
          Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_GENERATOR_OFF;
          if(Msp430RefVoltGeneratorP_state == Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING)
          {
            Msp430RefVoltGeneratorP_RefVolt_1_5V_stopDone(SUCCESS);
          }
          else 
          {
            Msp430RefVoltGeneratorP_RefVolt_2_5V_stopDone(SUCCESS);
          }
          return SUCCESS;
        }
        else 
        {
          return FAIL;
        }
    case Msp430RefVoltGeneratorP_REFERENCE_1_5V_STABLE:
      case Msp430RefVoltGeneratorP_REFERENCE_2_5V_STABLE:
        Msp430RefVoltGeneratorP_SwitchOffTimer_startOneShot(20);
    return SUCCESS;
    case Msp430RefVoltGeneratorP_GENERATOR_OFF:
      default :
        return FAIL;
  }

}

static inline error_t Msp430RefVoltArbiterImplP_RefVolt_1_5V_stop( void ) 
{
  char unsigned result;
  result = Msp430RefVoltGeneratorP_RefVolt_1_5V_stop();
  return result;
}

inline static void Msp430RefVoltArbiterImplP_switchOff_runTask( void ) 
{
  if(Msp430RefVoltArbiterImplP_syncOwner != Msp430RefVoltArbiterImplP_NO_OWNER)
  {
    if(Msp430RefVoltArbiterImplP_RefVolt_1_5V_stop() == SUCCESS)
    {
      Msp430RefVoltArbiterImplP_syncOwner = Msp430RefVoltArbiterImplP_NO_OWNER;
    }
    else 
    {
      Msp430RefVoltArbiterImplP_switchOff_postTask();
    }
  }
}

inline static bool HplAdc12P_HplAdc12_isBusy( void ) 
{
  return _ADC12CTL1 & 0x0001;
}

static inline bool Msp430RefVoltGeneratorP_HplAdc12_isBusy( void ) 
{
  char unsigned result;
  result = HplAdc12P_HplAdc12_isBusy();
  return result;
}

inline static adc12ctl0_t HplAdc12P_HplAdc12_getCtl0( void ) 
{
  return *( adc12ctl0_t * )(&_ADC12CTL0);
}

static inline adc12ctl0_t Msp430RefVoltGeneratorP_HplAdc12_getCtl0( void ) 
{
  struct __nesc_unnamed4254  result;
  result = HplAdc12P_HplAdc12_getCtl0();
  return result;
}

inline static void HplAdc12P_HplAdc12_setCtl0(adc12ctl0_t control0) 
{
  _ADC12CTL0 = *( uint16_t * )(&control0);
}

static inline void Msp430RefVoltGeneratorP_HplAdc12_setCtl0(adc12ctl0_t control0) 
{
  HplAdc12P_HplAdc12_setCtl0(control0);
}

static inline error_t VirtualizeTimerC_0_updateFromTimer_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(VirtualizeTimerC_0_updateFromTimer);
  return result;
}

inline static void SimpleArbiterP_0_Resource_default_granted(uint8_t id) 
{
}

static inline void SimpleArbiterP_0_Resource_granted(uint8_t arg_0x1a82930) 
{
  switch (arg_0x1a82930) 
  {
    case Msp430Adc12ClientAutoRVGC_0_ID:
      Msp430RefVoltArbiterImplP_AdcResource_granted(Msp430Adc12ClientAutoRVGC_0_ID);
      break;
    case Msp430Adc12ClientAutoRVGC_1_ID:
      Msp430RefVoltArbiterImplP_AdcResource_granted(Msp430Adc12ClientAutoRVGC_1_ID);
      break;
    case Msp430Adc12ClientAutoRVGC_2_ID:
      Msp430RefVoltArbiterImplP_AdcResource_granted(Msp430Adc12ClientAutoRVGC_2_ID);
      break;
    default :
      SimpleArbiterP_0_Resource_default_granted(arg_0x1a82930);
      break;
  }

}

inline static void SimpleArbiterP_0_ResourceConfigure_default_configure(uint8_t id) 
{
}

static inline void SimpleArbiterP_0_ResourceConfigure_configure(uint8_t arg_0x1a7f030) 
{
  SimpleArbiterP_0_ResourceConfigure_default_configure(arg_0x1a7f030);
}

inline static void SimpleArbiterP_0_grantedTask_runTask( void ) 
{
{
  /* IRQ INSTR 24 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 25 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  SimpleArbiterP_0_resId = SimpleArbiterP_0_reqResId;
  SimpleArbiterP_0_state = SimpleArbiterP_0_RES_BUSY;
}
  __nesc_atomic_end(__nesc_atomic);
}
  SimpleArbiterP_0_ResourceConfigure_configure(SimpleArbiterP_0_resId);
  SimpleArbiterP_0_Resource_granted(SimpleArbiterP_0_resId);
}

const inline static msp430adc12_channel_config_t *Msp430InternalVoltageP_AdcConfigure_getConfiguration( void ) 
{
  return &Msp430InternalVoltageP_config;
}

static inline Msp430Adc12ConfAlertC_0_ConfUp_adc_config_t Msp430Adc12ConfAlertC_0_ConfUp_getConfiguration( void ) 
{
  struct __nesc_unnamed4267 *result;
  result = Msp430InternalVoltageP_AdcConfigure_getConfiguration();
  return result;
}

const inline static msp430adc12_channel_config_t *Msp430Adc12ConfAlertC_0_ConfSub_getConfiguration( void ) 
{
  return Msp430Adc12ConfAlertC_0_ConfUp_getConfiguration();
}

static inline Msp430Adc12ConfAlertC_1_ConfUp_adc_config_t Msp430Adc12ConfAlertC_1_ConfUp_getConfiguration( void ) 
{
  struct __nesc_unnamed4267  *result;
  result = Msp430InternalVoltageP_AdcConfigure_getConfiguration();
  return result;
}

const inline static msp430adc12_channel_config_t *Msp430Adc12ConfAlertC_1_ConfSub_getConfiguration( void ) 
{
  return Msp430Adc12ConfAlertC_1_ConfUp_getConfiguration();
}

static inline Msp430Adc12ConfAlertC_2_ConfUp_adc_config_t Msp430Adc12ConfAlertC_2_ConfUp_getConfiguration( void ) 
{
  struct __nesc_unnamed4267  *result;
  result = Msp430InternalVoltageP_AdcConfigure_getConfiguration();
  return result;
}

const inline static msp430adc12_channel_config_t *Msp430Adc12ConfAlertC_2_ConfSub_getConfiguration( void ) 
{
  return Msp430Adc12ConfAlertC_2_ConfUp_getConfiguration();
}

const inline static msp430adc12_channel_config_t *Msp430RefVoltArbiterImplP_Config_default_getConfiguration(uint8_t client) 
{
  return &Msp430RefVoltArbiterImplP_defaultConfig;
}

static inline Msp430RefVoltArbiterImplP_Config_adc_config_t Msp430RefVoltArbiterImplP_Config_getConfiguration(uint8_t arg_0x1af6638) 
{
  struct __nesc_unnamed4267  *result;
  switch (arg_0x1af6638) 
  {
    case Msp430Adc12ClientAutoRVGC_0_ID:
      result = Msp430Adc12ConfAlertC_0_ConfSub_getConfiguration();
      break;
    case Msp430Adc12ClientAutoRVGC_1_ID:
      result = Msp430Adc12ConfAlertC_1_ConfSub_getConfiguration();
      break;
    case Msp430Adc12ClientAutoRVGC_2_ID:
      result = Msp430Adc12ConfAlertC_2_ConfSub_getConfiguration();
      break;
    default :
      result = Msp430RefVoltArbiterImplP_Config_default_getConfiguration(arg_0x1af6638);
      break;
  }

  return result;
}

inline static void SimpleArbiterP_0_ResourceRequested_default_requested(uint8_t id) 
{
}

static inline void SimpleArbiterP_0_ResourceRequested_requested(uint8_t arg_0x1a813f0) 
{
  SimpleArbiterP_0_ResourceRequested_default_requested(arg_0x1a813f0);
}

inline static error_t RoundRobinResourceQueueC_0_RoundRobinQueue_enqueue(resource_client_id_t id) 
{
{
  if(!RoundRobinResourceQueueC_0_RoundRobinQueue_isEnqueued(id))
  {
    RoundRobinResourceQueueC_0_resQ[id / 8] |= 1 << (id % 8);
  {
    char unsigned __nesc_temp = SUCCESS;
    return __nesc_temp;
  }
  }
{
  char unsigned __nesc_temp = EBUSY;
  return __nesc_temp;
}
}
}

static inline error_t SimpleArbiterP_0_Queue_enqueue(resource_client_id_t id) 
{
  char unsigned result;
  result = RoundRobinResourceQueueC_0_RoundRobinQueue_enqueue(id);
  return result;
}

static inline error_t ArbitratedReadStreamC_0_Service_read(uint8_t arg_0x1bc37c8, uint32_t usPeriod) 
{
  char unsigned result;
  result = AdcStreamP_ReadStream_read(arg_0x1bc37c8, usPeriod);
  return result;
}

inline static void ArbitratedReadStreamC_0_Resource_granted(uint8_t client) 
{
  ArbitratedReadStreamC_0_Service_read(client, ArbitratedReadStreamC_0_period[client]);
}

inline static void Msp430RefVoltArbiterImplP_ClientResource_default_granted(uint8_t client) 
{
}

static inline void Msp430RefVoltArbiterImplP_ClientResource_granted(uint8_t arg_0x1af9010) 
{
  switch (arg_0x1af9010) 
  {
    case Msp430Adc12ClientAutoRVGC_0_ID:
      AdcP_ResourceRead_granted(AdcReadClientC_0_CLIENT);
      break;
    case Msp430Adc12ClientAutoRVGC_1_ID:
      ArbitratedReadStreamC_0_Resource_granted(AdcReadStreamClientC_0_RSCLIENT);
      break;
    case Msp430Adc12ClientAutoRVGC_2_ID:
      AdcP_SubResourceReadNow_granted(AdcReadNowClientC_0_CLIENT);
      break;
    default :
      Msp430RefVoltArbiterImplP_ClientResource_default_granted(arg_0x1af9010);
      break;
  }

}

inline static void Msp430RefVoltArbiterImplP_RefVolt_1_5V_startDone(error_t error) 
{
  if(Msp430RefVoltArbiterImplP_syncOwner != Msp430RefVoltArbiterImplP_NO_OWNER)
  {
    Msp430RefVoltArbiterImplP_ClientResource_granted(Msp430RefVoltArbiterImplP_syncOwner);
  }
}

static inline void Msp430RefVoltGeneratorP_RefVolt_1_5V_startDone(error_t error) 
{
  Msp430RefVoltArbiterImplP_RefVolt_1_5V_startDone(error);
}

static inline void Msp430RefVoltGeneratorP_SwitchOffTimer_stop( void ) 
{
  VirtualizeTimerC_0_Timer_stop(2U);
}

inline static error_t Msp430RefVoltGeneratorP_RefVolt_1_5V_start( void ) 
{
  switch (Msp430RefVoltGeneratorP_state) 
  {
    case Msp430RefVoltGeneratorP_REFERENCE_1_5V_STABLE:
      Msp430RefVoltGeneratorP_SwitchOffTimer_stop();
    Msp430RefVoltGeneratorP_RefVolt_1_5V_startDone(SUCCESS);
    return SUCCESS;
    case Msp430RefVoltGeneratorP_GENERATOR_OFF:
      if(Msp430RefVoltGeneratorP_switchOn(Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING) == SUCCESS)
      {
        Msp430RefVoltGeneratorP_SwitchOnTimer_startOneShot(17);
        Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING;
        return SUCCESS;
      }
      else 
      {
        return FAIL;
      }
    case Msp430RefVoltGeneratorP_REFERENCE_2_5V_STABLE:
      if(Msp430RefVoltGeneratorP_switchOn(Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING) == SUCCESS)
      {
        Msp430RefVoltGeneratorP_SwitchOffTimer_stop();
        Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_REFERENCE_1_5V_STABLE;
        Msp430RefVoltGeneratorP_RefVolt_1_5V_startDone(SUCCESS);
        return SUCCESS;
      }
      else 
      {
        return FAIL;
      }
    case Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING:
      case Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING:
        default :
          return FAIL;
  }

}

static inline error_t Msp430RefVoltArbiterImplP_RefVolt_1_5V_start( void ) 
{
  char unsigned result;
  result = Msp430RefVoltGeneratorP_RefVolt_1_5V_start();
  return result;
}

const inline static msp430adc12_channel_config_t *AdcP_Config_default_getConfiguration(uint8_t client) 
{
  return &AdcP_defaultConfig;
}

static inline AdcP_Config_adc_config_t AdcP_Config_getConfiguration(uint8_t arg_0x18fc648) 
{
  struct __nesc_unnamed4267  *result;
  switch (arg_0x18fc648) 
  {
    case AdcReadClientC_0_CLIENT:
      result = Msp430InternalVoltageP_AdcConfigure_getConfiguration();
      break;
    case AdcReadNowClientC_0_CLIENT:
      result = Msp430InternalVoltageP_AdcConfigure_getConfiguration();
      break;
    default :
      result = AdcP_Config_default_getConfiguration(arg_0x18fc648);
      break;
  }

  return result;
}

inline static error_t AdcP_SingleChannel_default_configureSingle(uint8_t client, const msp430adc12_channel_config_t *config) 
{
  return FAIL;
}

static inline error_t AdcP_SingleChannel_configureSingle(uint8_t arg_0x191ee60, const msp430adc12_channel_config_t *config) 
{
  char unsigned result;
  switch (arg_0x191ee60) 
  {
    case AdcReadClientC_0_CLIENT:
      result = Msp430Adc12ImplP_SingleChannel_configureSingle(Msp430Adc12ClientAutoRVGC_0_ID, config);
      break;
    case AdcReadNowClientC_0_CLIENT:
      result = Msp430Adc12ImplP_SingleChannel_configureSingle(Msp430Adc12ClientAutoRVGC_2_ID, config);
      break;
    default :
      result = AdcP_SingleChannel_default_configureSingle(arg_0x191ee60, config);
      break;
  }

  return result;
}

inline static void AdcP_ResourceReadNow_default_granted(uint8_t nowClient) 
{
}

static inline void AdcP_ResourceReadNow_granted(uint8_t arg_0x18ff330) 
{
  AdcP_ResourceReadNow_default_granted(arg_0x18ff330);
}

static inline TransformAlarmC_1_Counter_size_type TransformAlarmC_1_Counter_get( void ) 
{
  long unsigned result;
  result = TransformCounterC_0_Counter_get();
  return result;
}

inline static TransformAlarmC_1_to_size_type TransformAlarmC_1_Alarm_getNow( void ) 
{
  return TransformAlarmC_1_Counter_get();
}

static inline AdcStreamP_Alarm_size_type AdcStreamP_Alarm_getNow( void ) 
{
  long unsigned result;
  result = TransformAlarmC_1_Alarm_getNow();
  return result;
}

inline static error_t AdcStreamP_SingleChannel_default_configureSingle(uint8_t c, const msp430adc12_channel_config_t *config) 
{
  return FAIL;
}

static inline error_t AdcStreamP_SingleChannel_configureSingle(uint8_t arg_0x1b5c010, const msp430adc12_channel_config_t *config) 
{
  char unsigned result;
  switch (arg_0x1b5c010) 
  {
    case AdcReadStreamClientC_0_RSCLIENT:
      result = Msp430Adc12ImplP_SingleChannel_configureSingle(Msp430Adc12ClientAutoRVGC_1_ID, config);
      break;
    default :
      result = AdcStreamP_SingleChannel_default_configureSingle(arg_0x1b5c010, config);
      break;
  }

  return result;
}

const inline static msp430adc12_channel_config_t *AdcStreamP_AdcConfigure_default_getConfiguration(uint8_t c) 
{
  return &AdcStreamP_defaultConfig;
}

static inline AdcStreamP_AdcConfigure_adc_config_t AdcStreamP_AdcConfigure_getConfiguration(uint8_t arg_0x1b5cde0) 
{
  struct __nesc_unnamed4267  *result;
  switch (arg_0x1b5cde0) 
  {
    case AdcReadStreamClientC_0_RSCLIENT:
      result = Msp430InternalVoltageP_AdcConfigure_getConfiguration();
      break;
    default :
      result = AdcStreamP_AdcConfigure_default_getConfiguration(arg_0x1b5cde0);
      break;
  }

  return result;
}

static inline error_t AdcStreamP_readStreamDone_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(AdcStreamP_readStreamDone);
  return result;
}

inline static void TransformAlarmC_1_Alarm_startAt(TransformAlarmC_1_to_size_type t0, TransformAlarmC_1_to_size_type dt) 
{
{
  TransformAlarmC_1_m_t0 = t0;
  TransformAlarmC_1_m_dt = dt;
  TransformAlarmC_1_set_alarm();
}
}

static inline void AdcStreamP_Alarm_startAt(AdcStreamP_Alarm_size_type t0, AdcStreamP_Alarm_size_type dt) 
{
  TransformAlarmC_1_Alarm_startAt(t0, dt);
}

inline static void AdcStreamP_nextAlarm( void ) 
{
  AdcStreamP_Alarm_startAt(AdcStreamP_now, AdcStreamP_period);
  AdcStreamP_now += AdcStreamP_period;
}

inline static void Msp430TimerCapComP_1_Compare_setEvent(uint16_t x) 
{
  _TACCR1 = x;
}

static inline void Msp430Adc12ImplP_CompareA1_setEvent(uint16_t time) 
{
  Msp430TimerCapComP_1_Compare_setEvent(time);
}

inline static void Msp430TimerCapComP_0_Compare_setEvent(uint16_t x) 
{
  _TACCR0 = x;
}

static inline void Msp430Adc12ImplP_CompareA0_setEvent(uint16_t time) 
{
  Msp430TimerCapComP_0_Compare_setEvent(time);
}

inline static uint16_t Msp430TimerCapComP_0_CC2int(Msp430TimerCapComP_0_cc_t x) 
{
  union Msp430TimerCapComP_0___nesc_unnamed4311  {
  Msp430TimerCapComP_0_cc_t f;
  uint16_t t;
} c;
  c.f = x;
  return c.t;
}

inline static void Msp430TimerCapComP_0_Control_setControl(Msp430TimerCapComP_0_cc_t x) 
{
  _TACCTL0 = Msp430TimerCapComP_0_CC2int(x);
}

static inline void Msp430Adc12ImplP_ControlA0_setControl(msp430_compare_control_t control) 
{
  Msp430TimerCapComP_0_Control_setControl(control);
}

inline static void Msp430TimerP_0_Timer_setInputDivider(uint16_t inputDivider) 
{
  _TACTL = (_TACTL & ~0x0040 | 0x0080) | ((inputDivider << 6) & (0x0040 | 0x0080));
}

static inline void Msp430Adc12ImplP_TimerA_setInputDivider(uint16_t inputDivider) 
{
  Msp430TimerP_0_Timer_setInputDivider(inputDivider);
}

inline static void Msp430TimerP_0_Timer_setClockSource(uint16_t clockSource) 
{
  _TACTL = (_TACTL & ~256U | 512U) | ((clockSource << 8) & (256U | 512U));
}

static inline void Msp430Adc12ImplP_TimerA_setClockSource(uint16_t clockSource) 
{
  Msp430TimerP_0_Timer_setClockSource(clockSource);
}

inline static void Msp430TimerP_0_Timer_disableEvents( void ) 
{
  _TACTL &= ~2U;
}

static inline void Msp430Adc12ImplP_TimerA_disableEvents( void ) 
{
  Msp430TimerP_0_Timer_disableEvents();
}

inline static void Msp430TimerP_0_Timer_clear( void ) 
{
  _TACTL |= 4U;
}

static inline void Msp430Adc12ImplP_TimerA_clear( void ) 
{
  Msp430TimerP_0_Timer_clear();
}

inline static void Msp430Adc12ImplP_prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON) 
{
  msp430_compare_control_t ccResetSHI = {.ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0};
  Msp430Adc12ImplP_TimerA_setMode(MSP430TIMER_STOP_MODE);
  Msp430Adc12ImplP_TimerA_clear();
  Msp430Adc12ImplP_TimerA_disableEvents();
  Msp430Adc12ImplP_TimerA_setClockSource(csSAMPCON);
  Msp430Adc12ImplP_TimerA_setInputDivider(cdSAMPCON);
  Msp430Adc12ImplP_ControlA0_setControl(ccResetSHI);
  Msp430Adc12ImplP_CompareA0_setEvent(interval - 1);
  Msp430Adc12ImplP_CompareA1_setEvent((interval - 1) / 2);
}

inline static void HplAdc12P_HplAdc12_setIEFlags(uint16_t mask) 
{
  _ADC12IE = mask;
}

static inline void Msp430Adc12ImplP_HplAdc12_setIEFlags(uint16_t mask) 
{
  HplAdc12P_HplAdc12_setIEFlags(mask);
}

inline static void HplAdc12P_HplAdc12_setMCtl(uint8_t i, adc12memctl_t memControl) 
{
  uint8_t *memCtlPtr = ( uint8_t * )(_ADC12MCTL0);
  memCtlPtr += i;
  *memCtlPtr = *( uint8_t * )(&memControl);
}

static inline void Msp430Adc12ImplP_HplAdc12_setMCtl(uint8_t idx, adc12memctl_t memControl) 
{
  HplAdc12P_HplAdc12_setMCtl(idx, memControl);
}

inline static void HplAdc12P_HplAdc12_setCtl1(adc12ctl1_t control1) 
{
  _ADC12CTL1 = *( uint16_t * )(&control1);
}

static inline void Msp430Adc12ImplP_HplAdc12_setCtl1(adc12ctl1_t control1) 
{
  HplAdc12P_HplAdc12_setCtl1(control1);
}

static inline void Msp430Adc12ImplP_HplAdc12_setCtl0(adc12ctl0_t control0) 
{
  HplAdc12P_HplAdc12_setCtl0(control0);
}

static inline adc12ctl0_t Msp430Adc12ImplP_HplAdc12_getCtl0( void ) 
{
  struct __nesc_unnamed4254  result;
  result = HplAdc12P_HplAdc12_getCtl0();
  return result;
}

static inline uint8_t Msp430Adc12ImplP_ADCArbiterInfo_userId( void ) 
{
  char unsigned result;
  result = SimpleArbiterP_0_ArbiterInfo_userId();
  return result;
}

inline static error_t Msp430Adc12ImplP_SingleChannel_configureMultiple(uint8_t id, const msp430adc12_channel_config_t *config, uint16_t *buf, uint16_t length, uint16_t jiffies) 
{
  error_t result = ERESERVE;
  if((((!config || !buf) || !length) || (jiffies == 1)) || (jiffies == 2))
  {
    return EINVAL;
  }
{
  /* IRQ INSTR 26 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 27 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(Msp430Adc12ImplP_state & Msp430Adc12ImplP_ADC_BUSY)
  {
    char unsigned __nesc_temp = EBUSY;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  if(Msp430Adc12ImplP_ADCArbiterInfo_userId() == id)
  {
    adc12ctl1_t ctl1 = {.adc12busy = 0, .conseq = length > 16? 3 : 1, .adc12ssel = config->adc12ssel, .adc12div = config->adc12div, .issh = 0, .shp = 1, .shs = jiffies == 0? 0 : 1, .cstartadd = 0};
    adc12memctl_t memctl = {.inch = config->inch, .sref = config->sref, .eos = 0};
    uint16_t i;
    uint16_t mask = 1;
    adc12ctl0_t ctl0 = Msp430Adc12ImplP_HplAdc12_getCtl0();
    ctl0.msc = jiffies == 0? 1 : 0;
    ctl0.sht0 = config->sht;
    ctl0.sht1 = config->sht;
    Msp430Adc12ImplP_state = Msp430Adc12ImplP_MULTIPLE_DATA;
    Msp430Adc12ImplP_resultBufferStart = _IE1;
    Msp430Adc12ImplP_resultBufferLength = length;
    Msp430Adc12ImplP_resultBufferStart = buf;
    Msp430Adc12ImplP_resultBufferIndex = 0;
    Msp430Adc12ImplP_HplAdc12_setCtl0(ctl0);
    Msp430Adc12ImplP_HplAdc12_setCtl1(ctl1);
    for(i = 0; (i < (length - 1)) && (i < 15); i++)
      Msp430Adc12ImplP_HplAdc12_setMCtl(i, memctl);

    memctl.eos = 1;
    Msp430Adc12ImplP_HplAdc12_setMCtl(i, memctl);
    Msp430Adc12ImplP_HplAdc12_setIEFlags(mask << i);
    if(jiffies)
    {
      Msp430Adc12ImplP_state |= Msp430Adc12ImplP_USE_TIMERA;
      Msp430Adc12ImplP_prepareTimerA(jiffies, config->sampcon_ssel, config->sampcon_id);
    }
    result = SUCCESS;
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
  return result;
}

inline static error_t AdcStreamP_SingleChannel_default_configureMultiple(uint8_t c, const msp430adc12_channel_config_t *config, uint16_t (b), uint16_t numSamples, uint16_t jiffies) 
{
  return FAIL;
}

static inline error_t AdcStreamP_SingleChannel_configureMultiple(uint8_t arg_0x1b5c010, const msp430adc12_channel_config_t *config, uint16_t *buffer, uint16_t numSamples, uint16_t jiffies) 
{
  char unsigned result;
  switch (arg_0x1b5c010) 
  {
    case AdcReadStreamClientC_0_RSCLIENT:
      result = Msp430Adc12ImplP_SingleChannel_configureMultiple(Msp430Adc12ClientAutoRVGC_1_ID, config, buffer, numSamples, jiffies);
      break;
    default :
      result = AdcStreamP_SingleChannel_default_configureMultiple(arg_0x1b5c010, config, buffer, numSamples, jiffies);
      break;
  }

  return result;
}

inline static error_t AdcStreamP_postBuffer(uint8_t c, uint16_t *buf, uint16_t n) 
{
  if(n < sizeof( struct AdcStreamP_list_entry_t  ))
  {
    return ESIZE;
  }
{
  /* IRQ INSTR 28 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 29 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  struct AdcStreamP_list_entry_t  *newEntry = ( struct AdcStreamP_list_entry_t * )(buf);
  if(!AdcStreamP_bufferQueueEnd[c])
  {
    char unsigned __nesc_temp = FAIL;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  newEntry->count = n;
  newEntry->next = _IE1;
  *AdcStreamP_bufferQueueEnd[c] = newEntry;
  AdcStreamP_bufferQueueEnd[c] = &newEntry->next;
}
  __nesc_atomic_end(__nesc_atomic);
}
  return SUCCESS;
}

static inline error_t AdcStreamP_readStreamFail_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(AdcStreamP_readStreamFail);
  return result;
}

inline static error_t AdcP_SingleChannel_default_getData(uint8_t client) 
{
  return EINVAL;
}

static inline error_t AdcP_SingleChannel_getData(uint8_t arg_0x191ee60) 
{
  char unsigned result;
  switch (arg_0x191ee60) 
  {
    case AdcReadClientC_0_CLIENT:
      result = Msp430Adc12ImplP_SingleChannel_getData(Msp430Adc12ClientAutoRVGC_0_ID);
      break;
    case AdcReadNowClientC_0_CLIENT:
      result = Msp430Adc12ImplP_SingleChannel_getData(Msp430Adc12ClientAutoRVGC_2_ID);
      break;
    default :
      result = AdcP_SingleChannel_default_getData(arg_0x191ee60);
      break;
  }

  return result;
}

inline static void HplMsp430GeneralIOP_38_IO_clr( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P5OUT &= ~0x01 << 6;
  __nesc_atomic_end(__nesc_atomic);
}
}

static inline void Msp430GpioC_2_HplGeneralIO_clr( void ) 
{
  HplMsp430GeneralIOP_38_IO_clr();
}

inline static void Msp430GpioC_2_GeneralIO_clr( void ) 
{
  Msp430GpioC_2_HplGeneralIO_clr();
}

static inline void LedsP_Led2_clr( void ) 
{
  Msp430GpioC_2_GeneralIO_clr();
}

inline static void LedsP_Leds_led2On( void ) 
{
  LedsP_Led2_clr();
}

static inline void SenseC_Leds_led2On( void ) 
{
  LedsP_Leds_led2On();
}

inline static void LedsP_Leds_led2Off( void ) 
{
  LedsP_Led2_set();
}

static inline void SenseC_Leds_led2Off( void ) 
{
  LedsP_Leds_led2Off();
}

inline static void HplMsp430GeneralIOP_37_IO_clr( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P5OUT &= ~0x01 << 5;
  __nesc_atomic_end(__nesc_atomic);
}
}

static inline void Msp430GpioC_1_HplGeneralIO_clr( void ) 
{
  HplMsp430GeneralIOP_37_IO_clr();
}

inline static void Msp430GpioC_1_GeneralIO_clr( void ) 
{
  Msp430GpioC_1_HplGeneralIO_clr();
}

static inline void LedsP_Led1_clr( void ) 
{
  Msp430GpioC_1_GeneralIO_clr();
}

inline static void LedsP_Leds_led1On( void ) 
{
  LedsP_Led1_clr();
}

static inline void SenseC_Leds_led1On( void ) 
{
  LedsP_Leds_led1On();
}

inline static void LedsP_Leds_led1Off( void ) 
{
  LedsP_Led1_set();
}

static inline void SenseC_Leds_led1Off( void ) 
{
  LedsP_Leds_led1Off();
}

inline static void HplMsp430GeneralIOP_36_IO_clr( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P5OUT &= ~0x01 << 4;
  __nesc_atomic_end(__nesc_atomic);
}
}

static inline void Msp430GpioC_0_HplGeneralIO_clr( void ) 
{
  HplMsp430GeneralIOP_36_IO_clr();
}

inline static void Msp430GpioC_0_GeneralIO_clr( void ) 
{
  Msp430GpioC_0_HplGeneralIO_clr();
}

static inline void LedsP_Led0_clr( void ) 
{
  Msp430GpioC_0_GeneralIO_clr();
}

inline static void LedsP_Leds_led0On( void ) 
{
  LedsP_Led0_clr();
}

static inline void SenseC_Leds_led0On( void ) 
{
  LedsP_Leds_led0On();
}

inline static void LedsP_Leds_led0Off( void ) 
{
  LedsP_Led0_set();
}

static inline void SenseC_Leds_led0Off( void ) 
{
  LedsP_Leds_led0Off();
}

static inline void Msp430RefVoltGeneratorP_RefVolt_2_5V_startDone(error_t error) 
{
  Msp430RefVoltArbiterImplP_RefVolt_2_5V_startDone(error);
}

inline static error_t Msp430RefVoltGeneratorP_RefVolt_2_5V_start( void ) 
{
  switch (Msp430RefVoltGeneratorP_state) 
  {
    case Msp430RefVoltGeneratorP_REFERENCE_2_5V_STABLE:
      Msp430RefVoltGeneratorP_SwitchOffTimer_stop();
    Msp430RefVoltGeneratorP_RefVolt_2_5V_startDone(SUCCESS);
    return SUCCESS;
    case Msp430RefVoltGeneratorP_GENERATOR_OFF:
      if(Msp430RefVoltGeneratorP_switchOn(Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING) == SUCCESS)
      {
        Msp430RefVoltGeneratorP_SwitchOnTimer_startOneShot(17);
        Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING;
        return SUCCESS;
      }
      else 
      {
        return FAIL;
      }
    case Msp430RefVoltGeneratorP_REFERENCE_1_5V_STABLE:
      if(Msp430RefVoltGeneratorP_switchOn(Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING) == SUCCESS)
      {
        Msp430RefVoltGeneratorP_SwitchOffTimer_stop();
        Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_REFERENCE_2_5V_STABLE;
        Msp430RefVoltGeneratorP_RefVolt_2_5V_startDone(SUCCESS);
        return SUCCESS;
      }
      else 
      {
        return FAIL;
      }
    case Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING:
      case Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING:
        default :
          return FAIL;
  }

}

static inline error_t Msp430RefVoltArbiterImplP_RefVolt_2_5V_start( void ) 
{
  char unsigned result;
  result = Msp430RefVoltGeneratorP_RefVolt_2_5V_start();
  return result;
}

inline static void AdcP_Read_default_readDone(uint8_t client, error_t result, uint16_t val) 
{
}

static inline void AdcP_Read_readDone(uint8_t arg_0x1903338, error_t result, AdcP_Read_val_t val) 
{
  switch (arg_0x1903338) 
  {
    case AdcReadClientC_0_CLIENT:
      SenseC_Read_readDone(result, val);
      break;
    default :
      AdcP_Read_default_readDone(arg_0x1903338, result, val);
      break;
  }

}

inline static error_t AdcP_ResourceRead_default_release(uint8_t client) 
{
  return FAIL;
}

static inline error_t AdcP_ResourceRead_release(uint8_t arg_0x18ffe18) 
{
  char unsigned result;
  switch (arg_0x18ffe18) 
  {
    case AdcReadClientC_0_CLIENT:
      result = Msp430RefVoltArbiterImplP_ClientResource_release(Msp430Adc12ClientAutoRVGC_0_ID);
      break;
    default :
      result = AdcP_ResourceRead_default_release(arg_0x18ffe18);
      break;
  }

  return result;
}

inline static void AdcP_readDone_runTask( void ) 
{
  AdcP_ResourceRead_release(AdcP_owner);
  AdcP_Read_readDone(AdcP_owner, SUCCESS, AdcP_value);
}

static inline void AlarmToTimerC_0_Alarm_startAt(AlarmToTimerC_0_Alarm_size_type t0, AlarmToTimerC_0_Alarm_size_type dt) 
{
  TransformAlarmC_0_Alarm_startAt(t0, dt);
}

inline static void AlarmToTimerC_0_start(uint32_t t0, uint32_t dt, bool oneshot) 
{
  AlarmToTimerC_0_m_dt = dt;
  AlarmToTimerC_0_m_oneshot = oneshot;
  AlarmToTimerC_0_Alarm_startAt(t0, dt);
}

inline static void AlarmToTimerC_0_Timer_startOneShotAt(uint32_t t0, uint32_t dt) 
{
  AlarmToTimerC_0_start(t0, dt, TRUE);
}

static inline void VirtualizeTimerC_0_TimerFrom_startOneShotAt(uint32_t t0, uint32_t dt) 
{
  AlarmToTimerC_0_Timer_startOneShotAt(t0, dt);
}

inline static void Msp430AlarmC_0_Alarm_stop( void ) 
{
  Msp430AlarmC_0_Msp430TimerControl_disableEvents();
}

static inline void TransformAlarmC_0_AlarmFrom_stop( void ) 
{
  Msp430AlarmC_0_Alarm_stop();
}

inline static void TransformAlarmC_0_Alarm_stop( void ) 
{
  TransformAlarmC_0_AlarmFrom_stop();
}

static inline void AlarmToTimerC_0_Alarm_stop( void ) 
{
  TransformAlarmC_0_Alarm_stop();
}

inline static void AlarmToTimerC_0_Timer_stop( void ) 
{
  AlarmToTimerC_0_Alarm_stop();
}

static inline void VirtualizeTimerC_0_TimerFrom_stop( void ) 
{
  AlarmToTimerC_0_Timer_stop();
}

inline static void VirtualizeTimerC_0_updateFromTimer_runTask( void ) 
{
  uint32_t now = VirtualizeTimerC_0_TimerFrom_getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;
  VirtualizeTimerC_0_TimerFrom_stop();
  for(num = 0; num < VirtualizeTimerC_0_NUM_TIMERS; num++)
  {
    VirtualizeTimerC_0_Timer_t *timer = &VirtualizeTimerC_0_m_timers[num];
    if(timer->isrunning)
    {
      uint32_t elapsed = now - timer->t0;
      int32_t remaining = timer->dt - elapsed;
      if(remaining < min_remaining)
      {
        min_remaining = remaining;
        min_remaining_isset = TRUE;
      }
    }
  }

  if(min_remaining_isset)
  {
    if(min_remaining <= 0)
    {
      VirtualizeTimerC_0_fireTimers(now);
    }
    else 
    {
      VirtualizeTimerC_0_TimerFrom_startOneShotAt(now, min_remaining);
    }
  }
}

inline static error_t Msp430RefVoltArbiterImplP_AdcResource_default_request(uint8_t client) 
{
  return FAIL;
}

static inline error_t Msp430RefVoltArbiterImplP_AdcResource_request(uint8_t arg_0x1af9ac8) 
{
  char unsigned result;
  switch (arg_0x1af9ac8) 
  {
    case Msp430Adc12ClientAutoRVGC_0_ID:
      result = SimpleArbiterP_0_Resource_request(Msp430Adc12ClientAutoRVGC_0_ID);
      break;
    case Msp430Adc12ClientAutoRVGC_1_ID:
      result = SimpleArbiterP_0_Resource_request(Msp430Adc12ClientAutoRVGC_1_ID);
      break;
    case Msp430Adc12ClientAutoRVGC_2_ID:
      result = SimpleArbiterP_0_Resource_request(Msp430Adc12ClientAutoRVGC_2_ID);
      break;
    default :
      result = Msp430RefVoltArbiterImplP_AdcResource_default_request(arg_0x1af9ac8);
      break;
  }

  return result;
}

inline static error_t Msp430RefVoltArbiterImplP_ClientResource_request(uint8_t client) 
{
  return Msp430RefVoltArbiterImplP_AdcResource_request(client);
}

inline static error_t AdcP_ResourceRead_default_request(uint8_t client) 
{
  return FAIL;
}

static inline error_t AdcP_ResourceRead_request(uint8_t arg_0x18ffe18) 
{
  char unsigned result;
  switch (arg_0x18ffe18) 
  {
    case AdcReadClientC_0_CLIENT:
      result = Msp430RefVoltArbiterImplP_ClientResource_request(Msp430Adc12ClientAutoRVGC_0_ID);
      break;
    default :
      result = AdcP_ResourceRead_default_request(arg_0x18ffe18);
      break;
  }

  return result;
}

inline static error_t AdcP_Read_read(uint8_t client) 
{
  return AdcP_ResourceRead_request(client);
}

static inline error_t SenseC_Read_read( void ) 
{
  char unsigned result;
  result = AdcP_Read_read(AdcReadClientC_0_CLIENT);
  return result;
}

inline static void SenseC_Timer_fired( void ) 
{
  SenseC_Read_read();
}

inline static void Msp430RefVoltGeneratorP_SwitchOnTimer_fired( void ) 
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
          default :
            return;
  }

}

inline static void Msp430RefVoltGeneratorP_SwitchOffTimer_fired( void ) 
{
  switch (Msp430RefVoltGeneratorP_state) 
  {
    case Msp430RefVoltGeneratorP_REFERENCE_1_5V_STABLE:
      if(Msp430RefVoltGeneratorP_switchOff() == SUCCESS)
      {
        Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_GENERATOR_OFF;
        Msp430RefVoltGeneratorP_RefVolt_1_5V_stopDone(SUCCESS);
      }
      else 
      {
        Msp430RefVoltGeneratorP_SwitchOffTimer_startOneShot(20);
      }
      break;
    case Msp430RefVoltGeneratorP_REFERENCE_2_5V_STABLE:
      if(Msp430RefVoltGeneratorP_switchOff() == SUCCESS)
      {
        Msp430RefVoltGeneratorP_state = Msp430RefVoltGeneratorP_GENERATOR_OFF;
        Msp430RefVoltGeneratorP_RefVolt_2_5V_stopDone(SUCCESS);
      }
      else 
      {
        Msp430RefVoltGeneratorP_SwitchOffTimer_startOneShot(20);
      }
      break;
    case Msp430RefVoltGeneratorP_GENERATOR_OFF:
      case Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING:
        case Msp430RefVoltGeneratorP_REFERENCE_2_5V_PENDING:
          default :
            return;
  }

}

inline static void VirtualizeTimerC_0_Timer_default_fired(uint8_t num) 
{
}

static inline void VirtualizeTimerC_0_Timer_fired(uint8_t arg_0x18923c8) 
{
  switch (arg_0x18923c8) 
  {
    case 0U:
      SenseC_Timer_fired();
      break;
    case 1U:
      Msp430RefVoltGeneratorP_SwitchOnTimer_fired();
      break;
    case 2U:
      Msp430RefVoltGeneratorP_SwitchOffTimer_fired();
      break;
    default :
      VirtualizeTimerC_0_Timer_default_fired(arg_0x18923c8);
      break;
  }

}

inline static void VirtualizeTimerC_0_TimerFrom_fired( void ) 
{
  VirtualizeTimerC_0_fireTimers(VirtualizeTimerC_0_TimerFrom_getNow());
}

static inline void AlarmToTimerC_0_Timer_fired( void ) 
{
  VirtualizeTimerC_0_TimerFrom_fired();
}

inline static TransformAlarmC_0_to_size_type TransformAlarmC_0_Alarm_getAlarm( void ) 
{
{
  /* IRQ INSTR 31 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 32 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  /* IRQ INSTR 30 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  TransformAlarmC_0_to_size_type __nesc_temp = TransformAlarmC_0_m_t0 + TransformAlarmC_0_m_dt;
{
  __nesc_atomic_end(__nesc_atomic);
  return __nesc_temp;
}
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static inline AlarmToTimerC_0_Alarm_size_type AlarmToTimerC_0_Alarm_getAlarm( void ) 
{
  long unsigned result;
  result = TransformAlarmC_0_Alarm_getAlarm();
  return result;
}

inline static void AlarmToTimerC_0_fired_runTask( void ) 
{
  if(AlarmToTimerC_0_m_oneshot == FALSE)
  {
    AlarmToTimerC_0_start(AlarmToTimerC_0_Alarm_getAlarm(), AlarmToTimerC_0_m_dt, FALSE);
  }
  AlarmToTimerC_0_Timer_fired();
}

inline static error_t ecombine(error_t r1, error_t r2) 
{
  return r1 == r2? r1 : FAIL;
}

inline static uint16_t Msp430TimerCapComP_3_CC2int(Msp430TimerCapComP_3_cc_t x) 
{
  union Msp430TimerCapComP_3___nesc_unnamed4312  {
  Msp430TimerCapComP_3_cc_t f;
  uint16_t t;
} c;
  c.f = x;
  return c.t;
}

inline static uint16_t Msp430TimerCapComP_3_compareControl( void ) 
{
  Msp430TimerCapComP_3_cc_t x = {.cm = 1, .ccis = 0, .clld = 0, .cap = 0, .ccie = 0};
  return Msp430TimerCapComP_3_CC2int(x);
}

inline static void Msp430TimerCapComP_3_Control_setControlAsCompare( void ) 
{
  _TBCCTL0 = Msp430TimerCapComP_3_compareControl();
}

static inline void Msp430AlarmC_0_Msp430TimerControl_setControlAsCompare( void ) 
{
  Msp430TimerCapComP_3_Control_setControlAsCompare();
}

inline static error_t Msp430AlarmC_0_Init_init( void ) 
{
  Msp430AlarmC_0_Msp430TimerControl_disableEvents();
  Msp430AlarmC_0_Msp430TimerControl_setControlAsCompare();
  return SUCCESS;
}

static inline void Msp430Adc12ImplP_HplAdc12_stopConversion( void ) 
{
  HplAdc12P_HplAdc12_stopConversion();
}

inline static error_t Msp430Adc12ImplP_Init_init( void ) 
{
  adc12ctl0_t ctl0;
  Msp430Adc12ImplP_HplAdc12_stopConversion();
  ctl0 = Msp430Adc12ImplP_HplAdc12_getCtl0();
  ctl0.adc12tovie = 1;
  ctl0.adc12ovie = 1;
  Msp430Adc12ImplP_HplAdc12_setCtl0(ctl0);
  return SUCCESS;
}

inline static error_t RoundRobinResourceQueueC_0_Init_init( void ) 
{
  memset(RoundRobinResourceQueueC_0_resQ, 0, sizeof(RoundRobinResourceQueueC_0_resQ));
  return SUCCESS;
}

inline static error_t AdcStreamP_Init_init( void ) 
{
  uint8_t i;
  for(i = 0; i != AdcStreamP_NSTREAM; i++)
    AdcStreamP_bufferQueueEnd[i] = &AdcStreamP_bufferQueue[i];

  return SUCCESS;
}

static inline error_t RealMainP_SoftwareInit_init( void ) 
{
  char unsigned result;
  result = AdcStreamP_Init_init();
  result = ecombine(result, RoundRobinResourceQueueC_0_Init_init());
  result = ecombine(result, Msp430Adc12ImplP_Init_init());
  result = ecombine(result, Msp430AlarmC_0_Init_init());
  return result;
}

inline static void VirtualizeTimerC_0_Timer_startPeriodic(uint8_t num, uint32_t dt) 
{
  VirtualizeTimerC_0_startTimer(num, VirtualizeTimerC_0_TimerFrom_getNow(), dt, FALSE);
}

static inline void SenseC_Timer_startPeriodic(uint32_t dt) 
{
  VirtualizeTimerC_0_Timer_startPeriodic(0U, dt);
}

inline static void SenseC_Boot_booted( void ) 
{
  SenseC_Timer_startPeriodic(100);
}

static inline void RealMainP_Boot_booted( void ) 
{
  SenseC_Boot_booted();
}

inline static void __nesc_disable_interrupt( void ) 
{
  _R2 &= 0xfff7;
}

inline static mcu_power_t McuSleepC_McuPowerOverride_default_lowestState( void ) 
{
  return MSP430_POWER_LPM4;
}

static inline mcu_power_t McuSleepC_McuPowerOverride_lowestState( void ) 
{
  char unsigned result;
  result = McuSleepC_McuPowerOverride_default_lowestState();
  return result;
}

inline static mcu_power_t McuSleepC_getPowerState( void ) 
{
  mcu_power_t pState = MSP430_POWER_LPM3;
  if(((((((_TACCTL0 & 0x0010) || (_TACCTL1 & 0x0010)) || (_TACCTL2 & 0x0010)) && ((_TACTL & (3 << 8)) == (2 << 8))) || ((_ME1 & ((1 << 7) | (1 << 6))) && (_U0TCTL & 0x20))) || ((_ME2 & ((1 << 5) | (1 << 4))) && (_U1TCTL & 0x20))) || (((((_U0CTL & 0x01) && (_U0TCTL & 0x20)) && (_U0RCTL & 0x20)) && (_U0CTL & 0x04)) && (_U0CTL & 0x20)))
  {
    pState = MSP430_POWER_LPM1;
  }
  if(_ADC12CTL0 & 0x0010)
  {
    if(_ADC12CTL1 & (2 << 3))
    {
      if(_ADC12CTL1 & (1 << 3))
      {
        pState = MSP430_POWER_LPM1;
      }
      else 
      {
        pState = MSP430_POWER_ACTIVE;
      }
    }
    else 
    {
      if((_ADC12CTL1 & 0x0400) && ((_TACTL & (3 << 8)) == (2 << 8)))
      {
        pState = MSP430_POWER_LPM1;
      }
    }
  }
  return pState;
}

inline static mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2) 
{
  return m1 < m2? m1 : m2;
}

inline static void McuSleepC_computePowerState( void ) 
{
  McuSleepC_powerState = mcombine(McuSleepC_getPowerState(), McuSleepC_McuPowerOverride_lowestState());
}

inline static void McuSleepC_McuSleep_sleep( void ) 
{
  uint16_t temp;
  if(McuSleepC_dirty)
  {
    McuSleepC_computePowerState();
  }
  temp = McuSleepC_msp430PowerBits[McuSleepC_powerState] | 0x0008;
  _R2 |= temp;
  __nesc_disable_interrupt();
}

static inline void SchedulerBasicP_McuSleep_sleep( void ) 
{
  McuSleepC_McuSleep_sleep();
}

__inline static uint8_t SchedulerBasicP_popTask( void ) 
{
  /* IRQ INSTR 39 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 40 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  if(SchedulerBasicP_m_head != SchedulerBasicP_NO_TASK)
  {
  /* IRQ INSTR 37 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 38 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    uint8_t id = SchedulerBasicP_m_head;
  /* IRQ INSTR 33 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 34 */
    if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    SchedulerBasicP_m_head = SchedulerBasicP_m_next[SchedulerBasicP_m_head];
  /* IRQ INSTR 35 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 36 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    if(SchedulerBasicP_m_head == SchedulerBasicP_NO_TASK)
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

inline static void SchedulerBasicP_Scheduler_taskLoop( void ) 
{
  for(; ; )
  {
    uint8_t nextTask;
  {
  /* IRQ INSTR 41 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 42 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  {
    while((nextTask = SchedulerBasicP_popTask()) == (SchedulerBasicP_NO_TASK))
    {
      SchedulerBasicP_McuSleep_sleep();
    }
  }
    __nesc_atomic_end(__nesc_atomic);
  }
    SchedulerBasicP_TaskBasic_runTask(nextTask);
  }

}

static inline void RealMainP_Scheduler_taskLoop( void ) 
{
  SchedulerBasicP_Scheduler_taskLoop();
}

inline static uint16_t *AdcP_SingleChannel_multipleDataReady(uint8_t client, uint16_t *buf, uint16_t numSamples) 
{
  return 0;
}

inline static uint16_t *Msp430Adc12ImplP_SingleChannel_default_multipleDataReady(uint8_t id, uint16_t *buf, uint16_t numSamples) 
{
  return 0;
}

static inline uint16_t *Msp430Adc12ImplP_SingleChannel_multipleDataReady(uint8_t arg_0x19743f0, uint16_t *buffer, uint16_t numSamples) 
{
  int unsigned *result;
  switch (arg_0x19743f0) 
  {
    case Msp430Adc12ClientAutoRVGC_0_ID:
      result = AdcP_SingleChannel_multipleDataReady(AdcReadClientC_0_CLIENT, buffer, numSamples);
      break;
    case Msp430Adc12ClientAutoRVGC_1_ID:
      result = AdcStreamP_SingleChannel_multipleDataReady(AdcReadStreamClientC_0_RSCLIENT, buffer, numSamples);
      break;
    case Msp430Adc12ClientAutoRVGC_2_ID:
      result = AdcP_SingleChannel_multipleDataReady(AdcReadNowClientC_0_CLIENT, buffer, numSamples);
      break;
    default :
      result = Msp430Adc12ImplP_SingleChannel_default_multipleDataReady(arg_0x19743f0, buffer, numSamples);
      break;
  }

  return result;
}

inline static uint16_t HplAdc12P_HplAdc12_getMem(uint8_t i) 
{
  return *( uint16_t * )(_ADC12MEM0) + i;
}

static inline uint16_t Msp430Adc12ImplP_HplAdc12_getMem(uint8_t idx) 
{
  int unsigned result;
  result = HplAdc12P_HplAdc12_getMem(idx);
  return result;
}

static inline adc12memctl_t Msp430Adc12ImplP_HplAdc12_getMCtl(uint8_t idx) 
{
  struct __nesc_unnamed4268  result;
  result = HplAdc12P_HplAdc12_getMCtl(idx);
  return result;
}

inline static void Msp430Adc12ImplP_MultiChannel_default_dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples) 
{
}

static inline void Msp430Adc12ImplP_MultiChannel_dataReady(uint8_t arg_0x1972108, uint16_t *buffer, uint16_t numSamples) 
{
  Msp430Adc12ImplP_MultiChannel_default_dataReady(arg_0x1972108, buffer, numSamples);
}

inline static error_t Msp430Adc12ImplP_SingleChannel_default_singleDataReady(uint8_t id, uint16_t data) 
{
  return FAIL;
}

static inline error_t Msp430Adc12ImplP_SingleChannel_singleDataReady(uint8_t arg_0x19743f0, uint16_t data) 
{
  char unsigned result;
  switch (arg_0x19743f0) 
  {
    case Msp430Adc12ClientAutoRVGC_0_ID:
      result = AdcP_SingleChannel_singleDataReady(AdcReadClientC_0_CLIENT, data);
      break;
    case Msp430Adc12ClientAutoRVGC_1_ID:
      result = AdcStreamP_SingleChannel_singleDataReady(AdcReadStreamClientC_0_RSCLIENT, data);
      break;
    case Msp430Adc12ClientAutoRVGC_2_ID:
      result = AdcP_SingleChannel_singleDataReady(AdcReadNowClientC_0_CLIENT, data);
      break;
    default :
      result = Msp430Adc12ImplP_SingleChannel_default_singleDataReady(arg_0x19743f0, data);
      break;
  }

  return result;
}

inline static void Msp430Adc12ImplP_Overflow_default_conversionTimeOverflow(uint8_t id) 
{
}

static inline void Msp430Adc12ImplP_Overflow_conversionTimeOverflow(uint8_t arg_0x19729f8) 
{
  Msp430Adc12ImplP_Overflow_default_conversionTimeOverflow(arg_0x19729f8);
}

inline static void Msp430Adc12ImplP_Overflow_default_memOverflow(uint8_t id) 
{
}

static inline void Msp430Adc12ImplP_Overflow_memOverflow(uint8_t arg_0x19729f8) 
{
  Msp430Adc12ImplP_Overflow_default_memOverflow(arg_0x19729f8);
}

inline static void Msp430Adc12ImplP_HplAdc12_conversionDone(uint16_t iv) 
{
  bool overflow = FALSE;
  uint16_t *resultBuffer;
  if(iv <= 4)
  {
    if(iv == 2)
    {
      Msp430Adc12ImplP_Overflow_memOverflow(Msp430Adc12ImplP_clientID);
    }
    else 
    {
      Msp430Adc12ImplP_Overflow_conversionTimeOverflow(Msp430Adc12ImplP_clientID);
    }
    if(!Msp430Adc12ImplP_HplAdc12_getCtl0().msc)
    {
      overflow = TRUE;
    }
  }
  switch (Msp430Adc12ImplP_state & Msp430Adc12ImplP_CONVERSION_MODE_MASK) 
  {
    case Msp430Adc12ImplP_SINGLE_DATA:
      Msp430Adc12ImplP_stopConversion();
    Msp430Adc12ImplP_SingleChannel_singleDataReady(Msp430Adc12ImplP_clientID, Msp430Adc12ImplP_HplAdc12_getMem(0));
      break;
    case Msp430Adc12ImplP_SINGLE_DATA_REPEAT:
    {
      error_t repeatContinue;
      repeatContinue = Msp430Adc12ImplP_SingleChannel_singleDataReady(Msp430Adc12ImplP_clientID, Msp430Adc12ImplP_HplAdc12_getMem(0));
      if(repeatContinue != SUCCESS)
      {
        Msp430Adc12ImplP_stopConversion();
      }
        break;
    }
    case Msp430Adc12ImplP_MULTI_CHANNEL:
    {
      uint16_t i = 0;
      uint16_t k;
      resultBuffer = Msp430Adc12ImplP_resultBufferStart + Msp430Adc12ImplP_resultBufferIndex;
      do
      {
        *resultBuffer++ = Msp430Adc12ImplP_HplAdc12_getMem(i);
      }
      while(++i < Msp430Adc12ImplP_numChannels);
      Msp430Adc12ImplP_resultBufferIndex += Msp430Adc12ImplP_numChannels;
      if(overflow || (Msp430Adc12ImplP_resultBufferLength == Msp430Adc12ImplP_resultBufferIndex))
      {
        Msp430Adc12ImplP_stopConversion();
        resultBuffer -= Msp430Adc12ImplP_resultBufferIndex;
        k = Msp430Adc12ImplP_resultBufferIndex - Msp430Adc12ImplP_numChannels;
        Msp430Adc12ImplP_resultBufferIndex = 0;
        Msp430Adc12ImplP_MultiChannel_dataReady(Msp430Adc12ImplP_clientID, resultBuffer, overflow? k : Msp430Adc12ImplP_resultBufferLength);
      }
    }
      break;
    case Msp430Adc12ImplP_MULTIPLE_DATA:
    {
      uint16_t i = 0;
      uint16_t length;
      uint16_t k;
      resultBuffer = Msp430Adc12ImplP_resultBufferStart + Msp430Adc12ImplP_resultBufferIndex;
      if((Msp430Adc12ImplP_resultBufferLength - Msp430Adc12ImplP_resultBufferIndex) > 16)
      {
        length = 16;
      }
      else 
      {
        length = Msp430Adc12ImplP_resultBufferLength - Msp430Adc12ImplP_resultBufferIndex;
      }
      do
      {
        *resultBuffer++ = Msp430Adc12ImplP_HplAdc12_getMem(i);
      }
      while(++i < length);
      Msp430Adc12ImplP_resultBufferIndex += length;
      if(overflow || (Msp430Adc12ImplP_resultBufferLength == Msp430Adc12ImplP_resultBufferIndex))
      {
        Msp430Adc12ImplP_stopConversion();
        resultBuffer -= Msp430Adc12ImplP_resultBufferIndex;
        k = Msp430Adc12ImplP_resultBufferIndex - length;
        Msp430Adc12ImplP_resultBufferIndex = 0;
        Msp430Adc12ImplP_SingleChannel_multipleDataReady(Msp430Adc12ImplP_clientID, resultBuffer, overflow? k : Msp430Adc12ImplP_resultBufferLength);
      }
      else 
      {
        if((Msp430Adc12ImplP_resultBufferLength - Msp430Adc12ImplP_resultBufferIndex) > 15)
        {
          return;
        }
        else 
        {
          adc12memctl_t memctl = Msp430Adc12ImplP_HplAdc12_getMCtl(0);
          memctl.eos = 1;
          Msp430Adc12ImplP_HplAdc12_setMCtl(Msp430Adc12ImplP_resultBufferLength - Msp430Adc12ImplP_resultBufferIndex, memctl);
        }
      }
    }
      break;
    case Msp430Adc12ImplP_MULTIPLE_DATA_REPEAT:
    {
      uint8_t i = 0;
      resultBuffer = Msp430Adc12ImplP_resultBufferStart;
      do
      {
        *resultBuffer++ = Msp430Adc12ImplP_HplAdc12_getMem(i);
      }
      while(++i < Msp430Adc12ImplP_resultBufferLength);
      Msp430Adc12ImplP_resultBufferStart = Msp430Adc12ImplP_SingleChannel_multipleDataReady(Msp430Adc12ImplP_clientID, resultBuffer - Msp430Adc12ImplP_resultBufferLength, overflow? 0 : Msp430Adc12ImplP_resultBufferLength);
      if(!Msp430Adc12ImplP_resultBufferStart)
      {
        Msp430Adc12ImplP_stopConversion();
      }
        break;
    }
  }

}

inline static void Msp430RefVoltGeneratorP_HplAdc12_conversionDone(uint16_t iv) 
{
}

static inline void HplAdc12P_HplAdc12_conversionDone(uint16_t iv) 
{
  Msp430RefVoltGeneratorP_HplAdc12_conversionDone(iv);
  Msp430Adc12ImplP_HplAdc12_conversionDone(iv);
}

inline static void HplMsp430GeneralIOP_40_IO_selectIOFunc( void ) 
{
  _P6SEL &= ~0x01 << 0;
}

static inline void Msp430Adc12ImplP_Port60_selectIOFunc( void ) 
{
  HplMsp430GeneralIOP_40_IO_selectIOFunc();
}

inline static void HplMsp430GeneralIOP_41_IO_selectIOFunc( void ) 
{
  _P6SEL &= ~0x01 << 1;
}

static inline void Msp430Adc12ImplP_Port61_selectIOFunc( void ) 
{
  HplMsp430GeneralIOP_41_IO_selectIOFunc();
}

inline static void HplMsp430GeneralIOP_42_IO_selectIOFunc( void ) 
{
  _P6SEL &= ~0x01 << 2;
}

static inline void Msp430Adc12ImplP_Port62_selectIOFunc( void ) 
{
  HplMsp430GeneralIOP_42_IO_selectIOFunc();
}

inline static void HplMsp430GeneralIOP_43_IO_selectIOFunc( void ) 
{
  _P6SEL &= ~0x01 << 3;
}

static inline void Msp430Adc12ImplP_Port63_selectIOFunc( void ) 
{
  HplMsp430GeneralIOP_43_IO_selectIOFunc();
}

inline static void HplMsp430GeneralIOP_44_IO_selectIOFunc( void ) 
{
  _P6SEL &= ~0x01 << 4;
}

static inline void Msp430Adc12ImplP_Port64_selectIOFunc( void ) 
{
  HplMsp430GeneralIOP_44_IO_selectIOFunc();
}

inline static void HplMsp430GeneralIOP_45_IO_selectIOFunc( void ) 
{
  _P6SEL &= ~0x01 << 5;
}

static inline void Msp430Adc12ImplP_Port65_selectIOFunc( void ) 
{
  HplMsp430GeneralIOP_45_IO_selectIOFunc();
}

inline static void HplMsp430GeneralIOP_46_IO_selectIOFunc( void ) 
{
  _P6SEL &= ~0x01 << 6;
}

static inline void Msp430Adc12ImplP_Port66_selectIOFunc( void ) 
{
  HplMsp430GeneralIOP_46_IO_selectIOFunc();
}

inline static void HplMsp430GeneralIOP_47_IO_selectIOFunc( void ) 
{
  _P6SEL &= ~0x01 << 7;
}

static inline void Msp430Adc12ImplP_Port67_selectIOFunc( void ) 
{
  HplMsp430GeneralIOP_47_IO_selectIOFunc();
}

inline static void HplAdc12P_HplAdc12_resetIFGs( void ) 
{
  _ADC12IV = 0;
  _ADC12IFG = 0;
}

static inline void Msp430Adc12ImplP_HplAdc12_resetIFGs( void ) 
{
  HplAdc12P_HplAdc12_resetIFGs();
}

static inline error_t AdcP_readDone_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(AdcP_readDone);
  return result;
}

inline static void AdcP_ReadNow_default_readDone(uint8_t client, error_t result, uint16_t val) 
{
}

static inline void AdcP_ReadNow_readDone(uint8_t arg_0x1901350, error_t result, AdcP_ReadNow_val_t val) 
{
  AdcP_ReadNow_default_readDone(arg_0x1901350, result, val);
}

static inline error_t AdcStreamP_bufferDone_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(AdcStreamP_bufferDone);
  return result;
}

__nesc_atomic_t __nesc_atomic_start( void ) 
{
  __nesc_atomic_t result = (_R2 & 0x0008) != 0;
  __nesc_disable_interrupt();
  return result;
}

void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts) 
{
  if(reenable_interrupts)
  {
    __nesc_enable_interrupt();
  }
}

void sig_TIMERB0_VECTOR( void ) 
{
  assert(0);
  Msp430TimerCommonP_VectorTimerB0_fired();
}

static void Msp430TimerP_1_Event_default_fired(uint8_t n) 
{
}

static void Msp430TimerP_1_Event_fired(uint8_t arg_0x15734f0) 
{
  switch (arg_0x15734f0) 
  {
    case 0:
      Msp430TimerCapComP_3_Event_fired();
      break;
    case 1:
      Msp430TimerCapComP_4_Event_fired();
      break;
    case 2:
      Msp430TimerCapComP_5_Event_fired();
      break;
    case 3:
      Msp430TimerCapComP_6_Event_fired();
      break;
    case 4:
      Msp430TimerCapComP_7_Event_fired();
      break;
    case 5:
      Msp430TimerCapComP_8_Event_fired();
      break;
    case 6:
      Msp430TimerCapComP_9_Event_fired();
      break;
    case 7:
      Msp430TimerP_1_Overflow_fired();
      break;
    default :
      Msp430TimerP_1_Event_default_fired(arg_0x15734f0);
      break;
  }

}

static error_t SchedulerBasicP_TaskBasic_postTask(uint8_t id) 
{
{
  /* IRQ INSTR 43 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 44 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
{
  char unsigned __nesc_temp = SchedulerBasicP_pushTask(id)? SUCCESS : EBUSY;
{
  __nesc_atomic_end(__nesc_atomic);
  return __nesc_temp;
}
}
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static void TransformAlarmC_0_set_alarm( void ) 
{
  TransformAlarmC_0_to_size_type now = TransformAlarmC_0_Counter_get();
  TransformAlarmC_0_to_size_type expires;
  TransformAlarmC_0_to_size_type remaining;
  /* IRQ INSTR 45 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  expires = TransformAlarmC_0_m_t0 + TransformAlarmC_0_m_dt;
  remaining = ( TransformAlarmC_0_to_size_type )(expires - now);
  /* IRQ INSTR 48 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  if(TransformAlarmC_0_m_t0 <= now)
  {
  /* IRQ INSTR 46 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    if((expires >= TransformAlarmC_0_m_t0) && (expires <= now))
    {
      remaining = 0;
    }
  }
  else 
  {
  /* IRQ INSTR 47 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    if((expires >= TransformAlarmC_0_m_t0) || (expires <= now))
    {
      remaining = 0;
    }
  }
  if(remaining > TransformAlarmC_0_MAX_DELAY)
  {
    TransformAlarmC_0_m_t0 = now + TransformAlarmC_0_MAX_DELAY;
    TransformAlarmC_0_m_dt = remaining - TransformAlarmC_0_MAX_DELAY;
    remaining = TransformAlarmC_0_MAX_DELAY;
  }
  else 
  {
  /* IRQ INSTR 49 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    TransformAlarmC_0_m_t0 += TransformAlarmC_0_m_dt;
    TransformAlarmC_0_m_dt = 0;
  }
  TransformAlarmC_0_AlarmFrom_startAt(( TransformAlarmC_0_from_size_type )(now) << 5, ( TransformAlarmC_0_from_size_type )(remaining) << 5);
}

static TransformCounterC_0_to_size_type TransformCounterC_0_Counter_get( void ) 
{
  TransformCounterC_0_to_size_type rv = 0;
{
  /* IRQ INSTR 50 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 51 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  TransformCounterC_0_upper_count_type high = TransformCounterC_0_m_upper;
  TransformCounterC_0_from_size_type low = TransformCounterC_0_CounterFrom_get();
  if(TransformCounterC_0_CounterFrom_isOverflowPending())
  {
    high++;
    low = TransformCounterC_0_CounterFrom_get();
  }
{
  TransformCounterC_0_to_size_type high_to = high;
  TransformCounterC_0_to_size_type low_to = low >> TransformCounterC_0_LOW_SHIFT_RIGHT;
  rv = (high_to << TransformCounterC_0_HIGH_SHIFT_LEFT) | low_to;
}
}
  __nesc_atomic_end(__nesc_atomic);
}
  return rv;
}

static uint16_t Msp430TimerP_1_Timer_get( void ) 
{
/*
  if(1)
  {
  {
    uint16_t t0;
    uint16_t t1 = _TBR;
    do
    {
      t0 = t1;
      t1 = _TBR;
    }
    while(t0 != t1);
  {
    int unsigned __nesc_temp = t1;
    return __nesc_temp;
  }
  }
  }
  else 
  {*/
    _TBR = nondet_bv();
    return _TBR;
/*  } */
}

static error_t Msp430Adc12ImplP_SingleChannel_getData(uint8_t id) 
{
{
  /* IRQ INSTR 52 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 53 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(Msp430Adc12ImplP_ADCArbiterInfo_userId() == id)
  {
    if((Msp430Adc12ImplP_state & Msp430Adc12ImplP_MULTIPLE_DATA_REPEAT) && !Msp430Adc12ImplP_resultBufferStart)
    {
      char unsigned __nesc_temp = EINVAL;
    {
      __nesc_atomic_end(__nesc_atomic);
      return __nesc_temp;
    }
    }
    if(Msp430Adc12ImplP_state & Msp430Adc12ImplP_ADC_BUSY)
    {
      char unsigned __nesc_temp = EBUSY;
    {
      __nesc_atomic_end(__nesc_atomic);
      return __nesc_temp;
    }
    }
    Msp430Adc12ImplP_state |= Msp430Adc12ImplP_ADC_BUSY;
    Msp430Adc12ImplP_clientID = id;
    Msp430Adc12ImplP_configureAdcPin(Msp430Adc12ImplP_HplAdc12_getMCtl(0).inch);
    Msp430Adc12ImplP_HplAdc12_startConversion();
    if(Msp430Adc12ImplP_state & Msp430Adc12ImplP_USE_TIMERA)
    {
      Msp430Adc12ImplP_startTimerA();
    }
  {
    char unsigned __nesc_temp = SUCCESS;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
  return FAIL;
}

static uint8_t SimpleArbiterP_0_ArbiterInfo_userId( void ) 
{
{
  if(SimpleArbiterP_0_state != SimpleArbiterP_0_RES_BUSY)
  {
    char unsigned __nesc_temp = SimpleArbiterP_0_NO_RES;
    return __nesc_temp;
  }
{
  char unsigned __nesc_temp = SimpleArbiterP_0_resId;
  return __nesc_temp;
}
}
}

static adc12memctl_t HplAdc12P_HplAdc12_getMCtl(uint8_t i) 
{
  adc12memctl_t x = {.inch = 0, .sref = 0, .eos = 0};
  uint8_t *memCtlPtr = ( uint8_t * )(_ADC12MCTL0);
  memCtlPtr += i;
  x = *( adc12memctl_t * )(memCtlPtr);
  return x;
}

static void Msp430TimerP_0_Timer_setMode(int mode) 
{
  _TACTL = (_TACTL & ~0x0020 | 0x0010) | ((mode << 4) & (0x0020 | 0x0010));
}

static void TransformAlarmC_1_set_alarm( void ) 
{
  TransformAlarmC_1_to_size_type now = TransformAlarmC_1_Counter_get();
  TransformAlarmC_1_to_size_type expires;
  TransformAlarmC_1_to_size_type remaining;
  /* IRQ INSTR 54 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 55 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  expires = TransformAlarmC_1_m_t0 + TransformAlarmC_1_m_dt;
  remaining = ( TransformAlarmC_1_to_size_type )(expires - now);
  /* IRQ INSTR 60 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 61 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  if(TransformAlarmC_1_m_t0 <= now)
  {
  /* IRQ INSTR 56 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 57 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    if((expires >= TransformAlarmC_1_m_t0) && (expires <= now))
    {
      remaining = 0;
    }
  }
  else 
  {
  /* IRQ INSTR 58 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 59 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    if((expires >= TransformAlarmC_1_m_t0) || (expires <= now))
    {
      remaining = 0;
    }
  }
  if(remaining > TransformAlarmC_1_MAX_DELAY)
  {
    TransformAlarmC_1_m_t0 = now + TransformAlarmC_1_MAX_DELAY;
    TransformAlarmC_1_m_dt = remaining - TransformAlarmC_1_MAX_DELAY;
    remaining = TransformAlarmC_1_MAX_DELAY;
  }
  else 
  {
  /* IRQ INSTR 62 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 63 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    TransformAlarmC_1_m_t0 += TransformAlarmC_1_m_dt;
    TransformAlarmC_1_m_dt = 0;
  }
  TransformAlarmC_1_AlarmFrom_startAt(( TransformAlarmC_1_from_size_type )(now) << 5, ( TransformAlarmC_1_from_size_type )(remaining) << 5);
}

int main( void ) 
{
{
  /* IRQ INSTR 64 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 65 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
{
}
  RealMainP_Scheduler_init();
  RealMainP_PlatformInit_init();
  while(RealMainP_Scheduler_runNextTask())
  ;
  RealMainP_SoftwareInit_init();
  while(RealMainP_Scheduler_runNextTask())
  ;
}
  __nesc_atomic_end(__nesc_atomic);
}
  __nesc_enable_interrupt();
  RealMainP_Boot_booted();
  RealMainP_Scheduler_taskLoop();
  return -1;
}

static void MotePlatformC_TOSH_FLASH_M25P_DP_bit(bool set) 
{
  if(set)
  {
    TOSH_SET_SIMO0_PIN();
  }
  else 
  {
    TOSH_CLR_SIMO0_PIN();
  }
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

static void HplMsp430GeneralIOP_36_IO_set( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P5OUT |= 0x01 << 4;
  __nesc_atomic_end(__nesc_atomic);
}
}

static void HplMsp430GeneralIOP_37_IO_set( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P5OUT |= 0x01 << 5;
  __nesc_atomic_end(__nesc_atomic);
}
}

static void HplMsp430GeneralIOP_38_IO_set( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P5OUT |= 0x01 << 6;
  __nesc_atomic_end(__nesc_atomic);
}
}

static bool SchedulerBasicP_Scheduler_runNextTask( void ) 
{
  uint8_t nextTask;
{
  nextTask = SchedulerBasicP_popTask();
  if(nextTask == SchedulerBasicP_NO_TASK)
  {
  {
    char unsigned __nesc_temp = FALSE;
    return __nesc_temp;
  }
  }
}
  SchedulerBasicP_TaskBasic_runTask(nextTask);
  return TRUE;
}

static void SchedulerBasicP_TaskBasic_default_runTask(uint8_t id) 
{
}

static void SchedulerBasicP_TaskBasic_runTask(uint8_t arg_0x147bb38) 
{
  switch (arg_0x147bb38) 
  {
    case AlarmToTimerC_0_fired:
      AlarmToTimerC_0_fired_runTask();
      break;
    case VirtualizeTimerC_0_updateFromTimer:
      VirtualizeTimerC_0_updateFromTimer_runTask();
      break;
    case AdcP_readDone:
      AdcP_readDone_runTask();
      break;
    case SimpleArbiterP_0_grantedTask:
      SimpleArbiterP_0_grantedTask_runTask();
      break;
    case Msp430RefVoltArbiterImplP_switchOff:
      Msp430RefVoltArbiterImplP_switchOff_runTask();
      break;
    case AdcStreamP_readStreamDone:
      AdcStreamP_readStreamDone_runTask();
      break;
    case AdcStreamP_readStreamFail:
      AdcStreamP_readStreamFail_runTask();
      break;
    case AdcStreamP_bufferDone:
      AdcStreamP_bufferDone_runTask();
      break;
    default :
      SchedulerBasicP_TaskBasic_default_runTask(arg_0x147bb38);
      break;
  }

}

static error_t Msp430RefVoltArbiterImplP_ClientResource_release(uint8_t client) 
{
  error_t error;
  if(Msp430RefVoltArbiterImplP_syncOwner == client)
  {
    Msp430RefVoltArbiterImplP_switchOff_postTask();
  }
  error = Msp430RefVoltArbiterImplP_AdcResource_release(client);
  return error;
}

static error_t SimpleArbiterP_0_Resource_release(uint8_t id) 
{
  bool released = FALSE;
{
  /* IRQ INSTR 66 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 67 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if((SimpleArbiterP_0_state == SimpleArbiterP_0_RES_BUSY) && (SimpleArbiterP_0_resId == id))
  {
    if(SimpleArbiterP_0_Queue_isEmpty() == FALSE)
    {
      SimpleArbiterP_0_resId = SimpleArbiterP_0_NO_RES;
      SimpleArbiterP_0_reqResId = SimpleArbiterP_0_Queue_dequeue();
      SimpleArbiterP_0_state = SimpleArbiterP_0_RES_GRANTING;
      SimpleArbiterP_0_grantedTask_postTask();
    }
    else 
    {
      SimpleArbiterP_0_resId = SimpleArbiterP_0_NO_RES;
      SimpleArbiterP_0_state = SimpleArbiterP_0_RES_IDLE;
    }
    released = TRUE;
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
  if(released == TRUE)
  {
    SimpleArbiterP_0_ResourceConfigure_unconfigure(id);
    return SUCCESS;
  }
  return FAIL;
}

static bool RoundRobinResourceQueueC_0_RoundRobinQueue_isEnqueued(resource_client_id_t id) 
{
  return RoundRobinResourceQueueC_0_resQ[id / 8] & (1 << (id % 8));
}

static error_t Msp430RefVoltGeneratorP_switchOff( void ) 
{
{
  /* IRQ INSTR 68 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 69 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(Msp430RefVoltGeneratorP_HplAdc12_isBusy())
  {
    char unsigned __nesc_temp = FAIL;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  else 
  {
    adc12ctl0_t ctl0 = Msp430RefVoltGeneratorP_HplAdc12_getCtl0();
    ctl0.enc = 0;
    Msp430RefVoltGeneratorP_HplAdc12_setCtl0(ctl0);
    ctl0.refon = 0;
    Msp430RefVoltGeneratorP_HplAdc12_setCtl0(ctl0);
  {
    char unsigned __nesc_temp = SUCCESS;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static void VirtualizeTimerC_0_startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot) 
{
  VirtualizeTimerC_0_Timer_t *timer = &VirtualizeTimerC_0_m_timers[num];
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  VirtualizeTimerC_0_updateFromTimer_postTask();
}

static void Msp430RefVoltArbiterImplP_AdcResource_granted(uint8_t client) 
{
  const msp430adc12_channel_config_t *settings = Msp430RefVoltArbiterImplP_Config_getConfiguration(client);
  if((settings->sref == REFERENCE_VREFplus_AVss) || (settings->sref == REFERENCE_VREFplus_VREFnegterm))
  {
    error_t started;
    if(Msp430RefVoltArbiterImplP_syncOwner != Msp430RefVoltArbiterImplP_NO_OWNER)
    {
      Msp430RefVoltArbiterImplP_AdcResource_release(client);
      Msp430RefVoltArbiterImplP_AdcResource_request(client);
      return;
    }
    Msp430RefVoltArbiterImplP_syncOwner = client;
    if(settings->ref2_5v == REFVOLT_LEVEL_1_5)
    {
      started = Msp430RefVoltArbiterImplP_RefVolt_1_5V_start();
    }
    else 
    {
      started = Msp430RefVoltArbiterImplP_RefVolt_2_5V_start();
    }
    if(started != SUCCESS)
    {
      Msp430RefVoltArbiterImplP_syncOwner = Msp430RefVoltArbiterImplP_NO_OWNER;
      Msp430RefVoltArbiterImplP_AdcResource_release(client);
      Msp430RefVoltArbiterImplP_AdcResource_request(client);
    }
  }
  else 
  {
    Msp430RefVoltArbiterImplP_ClientResource_granted(client);
  }
}

static error_t SimpleArbiterP_0_Resource_request(uint8_t id) 
{
  SimpleArbiterP_0_ResourceRequested_requested(SimpleArbiterP_0_resId);
{
  /* IRQ INSTR 70 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 71 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(SimpleArbiterP_0_state == SimpleArbiterP_0_RES_IDLE)
  {
    SimpleArbiterP_0_state = SimpleArbiterP_0_RES_GRANTING;
    SimpleArbiterP_0_reqResId = id;
    SimpleArbiterP_0_grantedTask_postTask();
  {
    char unsigned __nesc_temp = SUCCESS;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
{
  char unsigned __nesc_temp = SimpleArbiterP_0_Queue_enqueue(id);
{
  __nesc_atomic_end(__nesc_atomic);
  return __nesc_temp;
}
}
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static void AdcP_SubResourceReadNow_granted(uint8_t nowClient) 
{
  if(AdcP_configure(nowClient) == SUCCESS)
  {
    AdcP_state = AdcP_STATE_READNOW;
  }
  else 
  {
    AdcP_state = AdcP_STATE_READNOW_INVALID_CONFIG;
  }
  AdcP_ResourceReadNow_granted(nowClient);
}

static error_t AdcP_configure(uint8_t client) 
{
  error_t result = EINVAL;
  const msp430adc12_channel_config_t *config;
  config = AdcP_Config_getConfiguration(client);
  if(config->inch != INPUT_CHANNEL_NONE)
  {
    result = AdcP_SingleChannel_configureSingle(client, config);
  }
  return result;
}

static error_t Msp430Adc12ImplP_SingleChannel_configureSingle(uint8_t id, const msp430adc12_channel_config_t *config) 
{
  error_t result = ERESERVE;
  if(!config)
  {
    return EINVAL;
  }
{
  /* IRQ INSTR 72 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 73 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(Msp430Adc12ImplP_state & Msp430Adc12ImplP_ADC_BUSY)
  {
    char unsigned __nesc_temp = EBUSY;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  if(Msp430Adc12ImplP_ADCArbiterInfo_userId() == id)
  {
    adc12ctl1_t ctl1 = {.adc12busy = 0, .conseq = 0, .adc12ssel = config->adc12ssel, .adc12div = config->adc12div, .issh = 0, .shp = 1, .shs = 0, .cstartadd = 0};
    adc12memctl_t memctl = {.inch = config->inch, .sref = config->sref, .eos = 1};
    adc12ctl0_t ctl0 = Msp430Adc12ImplP_HplAdc12_getCtl0();
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
  __nesc_atomic_end(__nesc_atomic);
}
  return result;
}

static error_t AdcStreamP_ReadStream_read(uint8_t c, uint32_t usPeriod) 
{
  if(usPeriod & 0xFFFF0000)
  {
    AdcStreamP_period = usPeriod / 1000;
    AdcStreamP_periodModified = TRUE;
    AdcStreamP_client = c;
    AdcStreamP_now = AdcStreamP_Alarm_getNow();
    AdcStreamP_SingleChannel_configureSingle(c, AdcStreamP_AdcConfigure_getConfiguration(c));
    if(AdcStreamP_nextBuffer(FALSE) == SUCCESS)
    {
      AdcStreamP_sampleSingle();
    }
  }
  else 
  {
    AdcStreamP_period = usPeriod;
    AdcStreamP_periodModified = FALSE;
    AdcStreamP_client = c;
    AdcStreamP_nextMultiple(c);
  }
  return SUCCESS;
}

static error_t AdcStreamP_nextBuffer(bool startNextAlarm) 
{
{
  /* IRQ INSTR 76 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 77 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  /* IRQ INSTR 75 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  struct AdcStreamP_list_entry_t  *entry = AdcStreamP_bufferQueue[AdcStreamP_client];
  if(!entry)
  {
    AdcStreamP_bufferQueueEnd[AdcStreamP_client] = _IE1;
    AdcStreamP_readStreamDone_postTask();
  {
    char unsigned __nesc_temp = FAIL;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
  else 
  {
    uint16_t tmp_count;
    AdcStreamP_bufferQueue[AdcStreamP_client] = entry->next;
    if(!AdcStreamP_bufferQueue[AdcStreamP_client])
    {
      AdcStreamP_bufferQueueEnd[AdcStreamP_client] = &AdcStreamP_bufferQueue[AdcStreamP_client];
    }
    AdcStreamP_pos = AdcStreamP_buffer = _IE1;
    AdcStreamP_count = entry->count;
  /* IRQ INSTR 74 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
    tmp_count = AdcStreamP_count;
    AdcStreamP_pos = AdcStreamP_buffer = ( uint16_t * )(entry);
    if(startNextAlarm)
    {
      AdcStreamP_nextAlarm();
    }
  {
    char unsigned __nesc_temp = SUCCESS;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static void AdcStreamP_nextMultiple(uint8_t c) 
{
  if(AdcStreamP_nextBuffer(FALSE) == SUCCESS)
  {
    msp430adc12_channel_config_t config = *AdcStreamP_AdcConfigure_getConfiguration(c);
    config.sampcon_ssel = SAMPCON_SOURCE_SMCLK;
    config.sampcon_id = SAMPCON_CLOCK_DIV_1;
  /* IRQ INSTR 79 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
    if(AdcStreamP_SingleChannel_configureMultiple(c, &config, AdcStreamP_pos, AdcStreamP_count, AdcStreamP_period) == SUCCESS)
    {
      AdcStreamP_SingleChannel_getData(c);
    }
    else 
    {
  /* IRQ INSTR 78 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
      AdcStreamP_postBuffer(c, AdcStreamP_pos, AdcStreamP_count);
      AdcStreamP_readStreamFail_postTask();
    }
  }
}

static void AdcP_ResourceRead_granted(uint8_t client) 
{
  error_t result = AdcP_configure(client);
  if(result == SUCCESS)
  {
    AdcP_state = AdcP_STATE_READ;
    result = AdcP_SingleChannel_getData(client);
  }
  else 
  {
    AdcP_ResourceRead_release(client);
    AdcP_Read_readDone(client, result, 0);
  }
}

static void SenseC_Read_readDone(error_t result, uint16_t data) 
{
  if(result == SUCCESS)
  {
    if(data & 0x0004)
    {
      SenseC_Leds_led2On();
    }
    else 
    {
      SenseC_Leds_led2Off();
    }
    if(data & 0x0002)
    {
      SenseC_Leds_led1On();
    }
    else 
    {
      SenseC_Leds_led1Off();
    }
    if(data & 0x0001)
    {
      SenseC_Leds_led0On();
    }
    else 
    {
      SenseC_Leds_led0Off();
    }
  }
}

static error_t Msp430RefVoltGeneratorP_switchOn(uint8_t level) 
{
{
  /* IRQ INSTR 80 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 81 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(Msp430RefVoltGeneratorP_HplAdc12_isBusy())
  {
    char unsigned __nesc_temp = FAIL;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  else 
  {
    adc12ctl0_t ctl0 = Msp430RefVoltGeneratorP_HplAdc12_getCtl0();
    ctl0.enc = 0;
    Msp430RefVoltGeneratorP_HplAdc12_setCtl0(ctl0);
    ctl0.refon = 1;
    if(level == Msp430RefVoltGeneratorP_REFERENCE_1_5V_PENDING)
    {
      ctl0.r2_5v = 0;
    }
    else 
    {
      ctl0.r2_5v = 1;
    }
    Msp430RefVoltGeneratorP_HplAdc12_setCtl0(ctl0);
  {
    char unsigned __nesc_temp = SUCCESS;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static void Msp430RefVoltGeneratorP_SwitchOnTimer_startOneShot(uint32_t dt) 
{
  VirtualizeTimerC_0_Timer_startOneShot(1U, dt);
}

static void Msp430RefVoltArbiterImplP_RefVolt_2_5V_startDone(error_t error) 
{
  if(Msp430RefVoltArbiterImplP_syncOwner != Msp430RefVoltArbiterImplP_NO_OWNER)
  {
    Msp430RefVoltArbiterImplP_ClientResource_granted(Msp430RefVoltArbiterImplP_syncOwner);
  }
}

static void VirtualizeTimerC_0_fireTimers(uint32_t now) 
{
  uint8_t num;
  for(num = 0; num < VirtualizeTimerC_0_NUM_TIMERS; num++)
  {
    VirtualizeTimerC_0_Timer_t *timer = &VirtualizeTimerC_0_m_timers[num];
    if(timer->isrunning)
    {
      uint32_t elapsed = now - timer->t0;
      if(elapsed >= timer->dt)
      {
        if(timer->isoneshot)
        {
          timer->isrunning = FALSE;
        }
        else 
        {
          timer->t0 += timer->dt;
        }
        VirtualizeTimerC_0_Timer_fired(num);
          break;
      }
    }
  }

  VirtualizeTimerC_0_updateFromTimer_postTask();
}

static void TransformAlarmC_0_Alarm_startAt(TransformAlarmC_0_to_size_type t0, TransformAlarmC_0_to_size_type dt) 
{
{
  /* IRQ INSTR 82 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 83 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  TransformAlarmC_0_m_t0 = t0;
  TransformAlarmC_0_m_dt = dt;
  TransformAlarmC_0_set_alarm();
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static void HplAdc12P_HplAdc12_stopConversion( void ) 
{
  uint16_t ctl1 = _ADC12CTL1;
  _ADC12CTL1 &= ~0x0002 | 0x0004;
  _ADC12CTL0 &= ~0x0001 + 0x0002;
  _ADC12CTL0 &= ~0x0010;
  _ADC12CTL1 |= ctl1 & (0x0002 | 0x0004);
}

void sig_ADC_VECTOR( void ) 
{
  assert(0);
  HplAdc12P_HplAdc12_conversionDone(_ADC12IV);
}

static void Msp430Adc12ImplP_stopConversion( void ) 
{
  uint8_t i;
  if(Msp430Adc12ImplP_state & Msp430Adc12ImplP_USE_TIMERA)
  {
    Msp430Adc12ImplP_TimerA_setMode(MSP430TIMER_STOP_MODE);
  }
  Msp430Adc12ImplP_resetAdcPin(Msp430Adc12ImplP_HplAdc12_getMCtl(0).inch);
  if(Msp430Adc12ImplP_state & Msp430Adc12ImplP_MULTI_CHANNEL)
  {
    for(i = 1; i < Msp430Adc12ImplP_numChannels; i++)
      Msp430Adc12ImplP_resetAdcPin(Msp430Adc12ImplP_HplAdc12_getMCtl(i).inch);

  }
{
  Msp430Adc12ImplP_HplAdc12_stopConversion();
  Msp430Adc12ImplP_HplAdc12_resetIFGs();
  Msp430Adc12ImplP_state &= ~Msp430Adc12ImplP_ADC_BUSY;
}
}

static void Msp430Adc12ImplP_resetAdcPin(uint8_t inch) 
{
  switch (inch) 
  {
    case 0:
      Msp430Adc12ImplP_Port60_selectIOFunc();
      break;
    case 1:
      Msp430Adc12ImplP_Port61_selectIOFunc();
      break;
    case 2:
      Msp430Adc12ImplP_Port62_selectIOFunc();
      break;
    case 3:
      Msp430Adc12ImplP_Port63_selectIOFunc();
      break;
    case 4:
      Msp430Adc12ImplP_Port64_selectIOFunc();
      break;
    case 5:
      Msp430Adc12ImplP_Port65_selectIOFunc();
      break;
    case 6:
      Msp430Adc12ImplP_Port66_selectIOFunc();
      break;
    case 7:
      Msp430Adc12ImplP_Port67_selectIOFunc();
      break;
  }

}

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
    default :
        break;
  }

  return SUCCESS;
}

static error_t AdcStreamP_SingleChannel_singleDataReady(uint8_t streamClient, uint16_t data) 
{
  /* IRQ INSTR 84 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  if(AdcStreamP_client == AdcStreamP_NSTREAM)
  {
    return FAIL;
  }
  /* IRQ INSTR 88 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  if(AdcStreamP_count == 0)
  {
    AdcStreamP_now = AdcStreamP_Alarm_getNow();
    AdcStreamP_nextBuffer(TRUE);
  }
  else 
  {
    *AdcStreamP_pos++ = data;
  /* IRQ INSTR 87 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
    if(AdcStreamP_pos == (AdcStreamP_buffer + AdcStreamP_count))
    {
    {
  /* IRQ INSTR 86 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
      if(AdcStreamP_lastBuffer)
      {
        AdcStreamP_bufferQueueEnd[AdcStreamP_client] = _IE1;
        AdcStreamP_readStreamFail_postTask();
      {
        char unsigned __nesc_temp = FAIL;
        return __nesc_temp;
      }
      }
      else 
      {
  /* IRQ INSTR 85 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
        AdcStreamP_lastCount = AdcStreamP_count;
        AdcStreamP_lastBuffer = AdcStreamP_buffer;
      }
    }
      AdcStreamP_bufferDone_postTask();
      AdcStreamP_nextBuffer(TRUE);
    }
    else 
    {
      AdcStreamP_nextAlarm();
    }
  }
  return FAIL;
}

static uint16_t *AdcStreamP_SingleChannel_multipleDataReady(uint8_t streamClient, uint16_t *buf, uint16_t length) 
{
{
  /* IRQ INSTR 89 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_ADC_VECTOR(); _R2 |= 0x0008; }
  if(AdcStreamP_lastBuffer)
  {
    AdcStreamP_bufferQueueEnd[AdcStreamP_client] = _IE1;
    AdcStreamP_readStreamFail_postTask();
  {
    int unsigned *__nesc_temp = 0;
    return __nesc_temp;
  }
  }
  else 
  {
    AdcStreamP_lastBuffer = AdcStreamP_buffer;
    AdcStreamP_lastCount = AdcStreamP_pos - AdcStreamP_buffer;
  }
}
  AdcStreamP_bufferDone_postTask();
  AdcStreamP_nextMultiple(streamClient);
  return 0;
}