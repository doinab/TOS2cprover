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

__inline static uint8_t __nesc_ntoh_uint8(const void *source) ;

__inline static uint8_t __nesc_hton_uint8(void *target, uint8_t value) ;

__inline static uint8_t __nesc_ntoh_leuint8(const void *source) ;

__inline static uint8_t __nesc_hton_leuint8(void *target, uint8_t value) ;

__inline static int8_t __nesc_ntoh_int8(const void *source) ;

__inline static int8_t __nesc_hton_int8(void *target, int8_t value) ;

__inline static uint16_t __nesc_ntoh_uint16(const void *source) ;

__inline static uint16_t __nesc_hton_uint16(void *target, uint16_t value) ;

__inline static uint16_t __nesc_ntoh_leuint16(const void *source) ;

__inline static uint16_t __nesc_hton_leuint16(void *target, uint16_t value) ;

__inline static uint32_t __nesc_ntoh_uint32(const void *source) ;

__inline static uint32_t __nesc_hton_uint32(void *target, uint32_t value) ;

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

extern void *memcpy(void *arg_0x5fa670, const void *arg_0x5fa808, size_t arg_0x5fa9a0) ;

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

uint16_t TOS_NODE_ID = 1;

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

typedef uint8_t cc2420_status_t;

typedef struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;
  nxle_uint8_t network;
  nxle_uint8_t type;
} __attribute((packed)) cc2420_header_t;

typedef struct cc2420_footer_t {} cc2420_footer_t;

typedef struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute((packed)) cc2420_metadata_t;

typedef struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t (data);
} __attribute((packed)) cc2420_packet_t;

enum __nesc_unnamed4267 { MAC_HEADER_SIZE = sizeof( cc2420_header_t ) - 1, MAC_FOOTER_SIZE = sizeof( uint16_t ), MAC_PACKET_SIZE = (MAC_HEADER_SIZE + 28) + MAC_FOOTER_SIZE, CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE };

enum cc2420_enums { CC2420_TIME_ACK_TURNAROUND = 7, CC2420_TIME_VREN = 20, CC2420_TIME_SYMBOL = 2, CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, CC2420_ACK_WAIT_DELAY = 256 };

enum cc2420_status_enums { CC2420_STATUS_RSSI_VALID = 1 << 1, CC2420_STATUS_LOCK = 1 << 2, CC2420_STATUS_TX_ACTIVE = 1 << 3, CC2420_STATUS_ENC_BUSY = 1 << 4, CC2420_STATUS_TX_UNDERFLOW = 1 << 5, CC2420_STATUS_XOSC16M_STABLE = 1 << 6 };

enum cc2420_config_reg_enums { CC2420_SNOP = 0x00, CC2420_SXOSCON = 0x01, CC2420_STXCAL = 0x02, CC2420_SRXON = 0x03, CC2420_STXON = 0x04, CC2420_STXONCCA = 0x05, CC2420_SRFOFF = 0x06, CC2420_SXOSCOFF = 0x07, CC2420_SFLUSHRX = 0x08, CC2420_SFLUSHTX = 0x09, CC2420_SACK = 0x0a, CC2420_SACKPEND = 0x0b, CC2420_SRXDEC = 0x0c, CC2420_STXENC = 0x0d, CC2420_SAES = 0x0e, CC2420_MAIN = 0x10, CC2420_MDMCTRL0 = 0x11, CC2420_MDMCTRL1 = 0x12, CC2420_RSSI = 0x13, CC2420_SYNCWORD = 0x14, CC2420_TXCTRL = 0x15, CC2420_RXCTRL0 = 0x16, CC2420_RXCTRL1 = 0x17, CC2420_FSCTRL = 0x18, CC2420_SECCTRL0 = 0x19, CC2420_SECCTRL1 = 0x1a, CC2420_BATTMON = 0x1b, CC2420_IOCFG0 = 0x1c, CC2420_IOCFG1 = 0x1d, CC2420_MANFIDL = 0x1e, CC2420_MANFIDH = 0x1f, CC2420_FSMTC = 0x20, CC2420_MANAND = 0x21, CC2420_MANOR = 0x22, CC2420_AGCCTRL = 0x23, CC2420_AGCTST0 = 0x24, CC2420_AGCTST1 = 0x25, CC2420_AGCTST2 = 0x26, CC2420_FSTST0 = 0x27, CC2420_FSTST1 = 0x28, CC2420_FSTST2 = 0x29, CC2420_FSTST3 = 0x2a, CC2420_RXBPFTST = 0x2b, CC2420_FMSTATE = 0x2c, CC2420_ADCTST = 0x2d, CC2420_DACTST = 0x2e, CC2420_TOPTST = 0x2f, CC2420_TXFIFO = 0x3e, CC2420_RXFIFO = 0x3f };

enum cc2420_ram_addr_enums { CC2420_RAM_TXFIFO = 0x000, CC2420_RAM_RXFIFO = 0x080, CC2420_RAM_KEY0 = 0x100, CC2420_RAM_RXNONCE = 0x110, CC2420_RAM_SABUF = 0x120, CC2420_RAM_KEY1 = 0x130, CC2420_RAM_TXNONCE = 0x140, CC2420_RAM_CBCSTATE = 0x150, CC2420_RAM_IEEEADR = 0x160, CC2420_RAM_PANID = 0x168, CC2420_RAM_SHORTADR = 0x16a };

enum cc2420_nonce_enums { CC2420_NONCE_BLOCK_COUNTER = 0, CC2420_NONCE_KEY_SEQ_COUNTER = 2, CC2420_NONCE_FRAME_COUNTER = 3, CC2420_NONCE_SOURCE_ADDRESS = 7, CC2420_NONCE_FLAGS = 15 };

enum cc2420_main_enums { CC2420_MAIN_RESETn = 15, CC2420_MAIN_ENC_RESETn = 14, CC2420_MAIN_DEMOD_RESETn = 13, CC2420_MAIN_MOD_RESETn = 12, CC2420_MAIN_FS_RESETn = 11, CC2420_MAIN_XOSC16M_BYPASS = 0 };

enum cc2420_mdmctrl0_enums { CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, CC2420_MDMCTRL0_PAN_COORDINATOR = 12, CC2420_MDMCTRL0_ADR_DECODE = 11, CC2420_MDMCTRL0_CCA_HYST = 8, CC2420_MDMCTRL0_CCA_MOD = 6, CC2420_MDMCTRL0_AUTOCRC = 5, CC2420_MDMCTRL0_AUTOACK = 4, CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0 };

enum cc2420_mdmctrl1_enums { CC2420_MDMCTRL1_CORR_THR = 6, CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, CC2420_MDMCTRL1_MODULATION_MODE = 4, CC2420_MDMCTRL1_TX_MODE = 2, CC2420_MDMCTRL1_RX_MODE = 0 };

enum cc2420_rssi_enums { CC2420_RSSI_CCA_THR = 8, CC2420_RSSI_RSSI_VAL = 0 };

enum cc2420_syncword_enums { CC2420_SYNCWORD_SYNCWORD = 0 };

enum cc2420_txctrl_enums { CC2420_TXCTRL_TXMIXBUF_CUR = 14, CC2420_TXCTRL_TX_TURNAROUND = 13, CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, CC2420_TXCTRL_TXMIX_CURRENT = 9, CC2420_TXCTRL_PA_CURRENT = 6, CC2420_TXCTRL_RESERVED = 5, CC2420_TXCTRL_PA_LEVEL = 0 };

enum cc2420_rxctrl0_enums { CC2420_RXCTRL0_RXMIXBUF_CUR = 12, CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, CC2420_RXCTRL0_MED_LNA_GAIN = 8, CC2420_RXCTRL0_LOW_LNA_GAIN = 6, CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, CC2420_RXCTRL0_MED_LNA_CURRENT = 2, CC2420_RXCTRL0_LOW_LNA_CURRENT = 0 };

enum cc2420_rxctrl1_enums { CC2420_RXCTRL1_RXBPF_LOCUR = 13, CC2420_RXCTRL1_RXBPF_MIDCUR = 12, CC2420_RXCTRL1_LOW_LOWGAIN = 11, CC2420_RXCTRL1_MED_LOWGAIN = 10, CC2420_RXCTRL1_HIGH_HGM = 9, CC2420_RXCTRL1_MED_HGM = 8, CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, CC2420_RXCTRL1_RXMIX_TAIL = 4, CC2420_RXCTRL1_RXMIX_VCM = 2, CC2420_RXCTRL1_RXMIX_CURRENT = 0 };

enum cc2420_rsctrl_enums { CC2420_FSCTRL_LOCK_THR = 14, CC2420_FSCTRL_CAL_DONE = 13, CC2420_FSCTRL_CAL_RUNNING = 12, CC2420_FSCTRL_LOCK_LENGTH = 11, CC2420_FSCTRL_LOCK_STATUS = 10, CC2420_FSCTRL_FREQ = 0 };

enum cc2420_secctrl0_enums { CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, CC2420_SECCTRL0_SEC_CBC_HEAD = 8, CC2420_SECCTRL0_SEC_SAKEYSEL = 7, CC2420_SECCTRL0_SEC_TXKEYSEL = 6, CC2420_SECCTRL0_SEC_RXKEYSEL = 5, CC2420_SECCTRL0_SEC_M = 2, CC2420_SECCTRL0_SEC_MODE = 0 };

enum cc2420_secctrl1_enums { CC2420_SECCTRL1_SEC_TXL = 8, CC2420_SECCTRL1_SEC_RXL = 0 };

enum cc2420_battmon_enums { CC2420_BATTMON_BATT_OK = 6, CC2420_BATTMON_BATTMON_EN = 5, CC2420_BATTMON_BATTMON_VOLTAGE = 0 };

enum cc2420_iocfg0_enums { CC2420_IOCFG0_BCN_ACCEPT = 11, CC2420_IOCFG0_FIFO_POLARITY = 10, CC2420_IOCFG0_FIFOP_POLARITY = 9, CC2420_IOCFG0_SFD_POLARITY = 8, CC2420_IOCFG0_CCA_POLARITY = 7, CC2420_IOCFG0_FIFOP_THR = 0 };

enum cc2420_iocfg1_enums { CC2420_IOCFG1_HSSD_SRC = 10, CC2420_IOCFG1_SFDMUX = 5, CC2420_IOCFG1_CCAMUX = 0 };

enum cc2420_manfidl_enums { CC2420_MANFIDL_PARTNUM = 12, CC2420_MANFIDL_MANFID = 0 };

enum cc2420_manfidh_enums { CC2420_MANFIDH_VERSION = 12, CC2420_MANFIDH_PARTNUM = 0 };

enum cc2420_fsmtc_enums { CC2420_FSMTC_TC_RXCHAIN2RX = 13, CC2420_FSMTC_TC_SWITCH2TX = 10, CC2420_FSMTC_TC_PAON2TX = 6, CC2420_FSMTC_TC_TXEND2SWITCH = 3, CC2420_FSMTC_TC_TXEND2PAOFF = 0 };

enum cc2420_sfdmux_enums { CC2420_SFDMUX_SFD = 0, CC2420_SFDMUX_XOSC16M_STABLE = 24 };

enum __nesc_unnamed4268 { CC2420_INVALID_TIMESTAMP = 0x80000000L };

typedef nx_uint8_t nx_am_id_t;

typedef nx_uint8_t nx_am_group_t;

typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;

typedef uint8_t am_group_t;

typedef uint16_t am_addr_t;

enum __nesc_unnamed4269 { AM_BROADCAST_ADDR = 0xffff };

enum __nesc_unnamed4270 { TOS_AM_GROUP = 0x22, TOS_AM_ADDRESS = 1 };

typedef uint8_t uart_id_t;

enum __nesc_unnamed4271 { HDLC_FLAG_BYTE = 0x7e, HDLC_CTLESC_BYTE = 0x7d };

enum __nesc_unnamed4272 { TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, TOS_SERIAL_CC1000_ID = 1, TOS_SERIAL_802_15_4_ID = 2, TOS_SERIAL_UNKNOWN_ID = 255 };

enum __nesc_unnamed4273 { SERIAL_PROTO_ACK = 67, SERIAL_PROTO_PACKET_ACK = 68, SERIAL_PROTO_PACKET_NOACK = 69, SERIAL_PROTO_PACKET_UNKNOWN = 255 };

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

typedef struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute((packed)) serial_header_t;

typedef struct serial_packet {
  serial_header_t header;
  nx_uint8_t (data);
} __attribute((packed)) serial_packet_t;

typedef struct serial_metadata {
  nx_uint8_t ack;
} __attribute((packed)) serial_metadata_t;

typedef union message_header  {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;

typedef union TOSRadioFooter  {
  cc2420_footer_t cc2420;
} message_footer_t;

typedef union TOSRadioMetadata  {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;

typedef struct message_t {
  nx_uint8_t (header[sizeof( message_header_t )]);
  nx_uint8_t (data[28]);
  nx_uint8_t (footer[sizeof( message_footer_t )]);
  nx_uint8_t (metadata[sizeof( message_metadata_t )]);
} __attribute((packed)) message_t;

enum ieee154_fcf_enums { IEEE154_FCF_FRAME_TYPE = 0, IEEE154_FCF_SECURITY_ENABLED = 3, IEEE154_FCF_FRAME_PENDING = 4, IEEE154_FCF_ACK_REQ = 5, IEEE154_FCF_INTRAPAN = 6, IEEE154_FCF_DEST_ADDR_MODE = 10, IEEE154_FCF_SRC_ADDR_MODE = 14 };

enum ieee154_fcf_type_enums { IEEE154_TYPE_BEACON = 0, IEEE154_TYPE_DATA = 1, IEEE154_TYPE_ACK = 2, IEEE154_TYPE_MAC_CMD = 3 };

enum iee154_fcf_addr_mode_enums { IEEE154_ADDR_NONE = 0, IEEE154_ADDR_SHORT = 2, IEEE154_ADDR_EXT = 3 };

typedef enum __nesc_unnamed4274 { USART_NONE = 0, USART_UART = 1, USART_UART_TX = 2, USART_UART_RX = 3, USART_SPI = 4, USART_I2C = 5 } msp430_usartmode_t;

typedef struct __nesc_unnamed4275 {
  int unsigned swrst :1;
  int unsigned mm :1;
  int unsigned sync :1;
  int unsigned listen :1;
  int unsigned clen :1;
  int unsigned spb :1;
  int unsigned pev :1;
  int unsigned pena :1;
} __attribute((packed)) msp430_uctl_t;

typedef struct __nesc_unnamed4276 {
  int unsigned txept :1;
  int unsigned stc :1;
  int unsigned txwake :1;
  int unsigned urxse :1;
  int unsigned ssel :2;
  int unsigned ckpl :1;
  int unsigned ckph :1;
} __attribute((packed)) msp430_utctl_t;

typedef struct __nesc_unnamed4277 {
  int unsigned rxerr :1;
  int unsigned rxwake :1;
  int unsigned urxwie :1;
  int unsigned urxeie :1;
  int unsigned brk :1;
  int unsigned oe :1;
  int unsigned pe :1;
  int unsigned fe :1;
} __attribute((packed)) msp430_urctl_t;

typedef struct __nesc_unnamed4278 {
  int unsigned ubr :16;
  int unsigned mm :1;
  int unsigned listen :1;
  int unsigned clen :1;
  int unsigned stc :1;
  int unsigned ssel :2;
  int unsigned ckpl :1;
  int unsigned ckph :1;
} msp430_spi_config_t;

typedef struct __nesc_unnamed4279 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;

typedef union __nesc_unnamed4280  {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;

msp430_spi_union_config_t 
  msp430_spi_default_config = 
  {0x0002, 1, 0, 1, 1, 0x02, 0, 1};

typedef enum __nesc_unnamed4281 { UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55 } msp430_uart_rate_t;

typedef struct __nesc_unnamed4282 {
  int unsigned ubr :16;
  int unsigned umctl :8;
  int unsigned mm :1;
  int unsigned listen :1;
  int unsigned clen :1;
  int unsigned spb :1;
  int unsigned pev :1;
  int unsigned pena :1;
  int unsigned urxse :1;
  int unsigned ssel :2;
  int unsigned ckpl :1;
  int unsigned urxwie :1;
  int unsigned urxeie :1;
  int unsigned utxe :1;
  int unsigned urxe :1;
} msp430_uart_config_t;

typedef struct __nesc_unnamed4283 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;

typedef union __nesc_unnamed4284  {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;

typedef struct __nesc_unnamed4285 {
  int unsigned i2cstt :1;
  int unsigned i2cstp :1;
  int unsigned i2cstb :1;
  int unsigned i2cctrx :1;
  int unsigned i2cssel :2;
  int unsigned i2ccrm :1;
  int unsigned i2cword :1;
} __attribute((packed)) msp430_i2ctctl_t;

typedef struct __nesc_unnamed4286 {
  int unsigned mst :1;
  int unsigned listen :1;
  int unsigned xa :1;
  int unsigned txdmaen :1;
  int unsigned rxdmaen :1;
  int unsigned i2cssel :2;
  int unsigned i2crm :1;
  int unsigned i2cword :1;
  int unsigned i2cpsc :8;
  int unsigned i2csclh :8;
  int unsigned i2cscll :8;
  int unsigned i2coa :10;
} msp430_i2c_config_t;

typedef struct __nesc_unnamed4287 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;

typedef union __nesc_unnamed4288  {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;

typedef uint8_t resource_client_id_t;

typedef nx_uint32_t timesync_radio_t;

enum __nesc_unnamed4289 { AM_DISSEMINATION_MESSAGE = 0x60, AM_DISSEMINATION_PROBE_MESSAGE = 0x61, DISSEMINATION_SEQNO_UNKNOWN = 0 };

typedef struct dissemination_message {
  nx_uint16_t key;
  nx_uint32_t seqno;
  nx_uint8_t (data[0]);
} __attribute((packed)) dissemination_message_t;

typedef struct dissemination_probe_message {
  nx_uint16_t key;
} __attribute((packed)) dissemination_probe_message_t;

typedef uint32_t TestDisseminationC_Update32_t;

typedef uint32_t TestDisseminationC_Value32_t;

typedef uint16_t TestDisseminationC_Value16_t;

typedef uint16_t TestDisseminationC_Update16_t;

typedef TMilli TestDisseminationC_Timer_precision_tag;

typedef T32khz CC2420ControlP_StartupTimer_precision_tag;

typedef uint32_t CC2420ControlP_StartupTimer_size_type;

typedef uint16_t CC2420ControlP_ReadRssi_val_t;

enum Msp430Timer32khzC_0___nesc_unnamed4290 { Msp430Timer32khzC_0_ALARM_ID = 0U };

typedef T32khz Msp430AlarmC_0_frequency_tag;

typedef Msp430AlarmC_0_frequency_tag Msp430AlarmC_0_Alarm_precision_tag;

typedef uint16_t Msp430AlarmC_0_Alarm_size_type;

typedef T32khz Msp430CounterC_0_frequency_tag;

typedef Msp430CounterC_0_frequency_tag Msp430CounterC_0_Counter_precision_tag;

typedef uint16_t Msp430CounterC_0_Counter_size_type;

typedef T32khz TransformCounterC_0_to_precision_tag;

typedef uint32_t TransformCounterC_0_to_size_type;

typedef T32khz TransformCounterC_0_from_precision_tag;

typedef uint16_t TransformCounterC_0_from_size_type;

typedef uint16_t TransformCounterC_0_upper_count_type;

typedef TransformCounterC_0_from_precision_tag TransformCounterC_0_CounterFrom_precision_tag;

typedef TransformCounterC_0_from_size_type TransformCounterC_0_CounterFrom_size_type;

typedef TransformCounterC_0_to_precision_tag TransformCounterC_0_Counter_precision_tag;

typedef TransformCounterC_0_to_size_type TransformCounterC_0_Counter_size_type;

typedef T32khz TransformAlarmC_0_to_precision_tag;

typedef uint32_t TransformAlarmC_0_to_size_type;

typedef T32khz TransformAlarmC_0_from_precision_tag;

typedef uint16_t TransformAlarmC_0_from_size_type;

typedef TransformAlarmC_0_to_precision_tag TransformAlarmC_0_Alarm_precision_tag;

typedef TransformAlarmC_0_to_size_type TransformAlarmC_0_Alarm_size_type;

typedef TransformAlarmC_0_from_precision_tag TransformAlarmC_0_AlarmFrom_precision_tag;

typedef TransformAlarmC_0_from_size_type TransformAlarmC_0_AlarmFrom_size_type;

typedef TransformAlarmC_0_to_precision_tag TransformAlarmC_0_Counter_precision_tag;

typedef TransformAlarmC_0_to_size_type TransformAlarmC_0_Counter_size_type;

enum CC2420SpiC_0___nesc_unnamed4291 { CC2420SpiC_0_CLIENT_ID = 0U };

enum Msp430Spi0C_0___nesc_unnamed4292 { Msp430Spi0C_0_CLIENT_ID = 0U };

enum Msp430Usart0C_0___nesc_unnamed4293 { Msp430Usart0C_0_CLIENT_ID = 0U };

enum CC2420SpiC_1___nesc_unnamed4294 { CC2420SpiC_1_CLIENT_ID = 1U };

enum CC2420SpiC_2___nesc_unnamed4295 { CC2420SpiC_2_CLIENT_ID = 2U };

typedef T32khz CC2420TransmitP_PacketTimeStamp_precision_tag;

typedef uint32_t CC2420TransmitP_PacketTimeStamp_size_type;

typedef T32khz CC2420TransmitP_BackoffTimer_precision_tag;

typedef uint32_t CC2420TransmitP_BackoffTimer_size_type;

enum CC2420SpiC_3___nesc_unnamed4296 { CC2420SpiC_3_CLIENT_ID = 3U };

typedef T32khz CC2420ReceiveP_PacketTimeStamp_precision_tag;

typedef uint32_t CC2420ReceiveP_PacketTimeStamp_size_type;

typedef T32khz CC2420PacketP_PacketTimeStamp32khz_precision_tag;

typedef uint32_t CC2420PacketP_PacketTimeStamp32khz_size_type;

typedef T32khz CC2420PacketP_LocalTime32khz_precision_tag;

typedef TMilli CC2420PacketP_LocalTimeMilli_precision_tag;

typedef TMilli CC2420PacketP_PacketTimeStampMilli_precision_tag;

typedef uint32_t CC2420PacketP_PacketTimeStampMilli_size_type;

typedef T32khz CounterToLocalTimeC_0_precision_tag;

typedef CounterToLocalTimeC_0_precision_tag CounterToLocalTimeC_0_LocalTime_precision_tag;

typedef CounterToLocalTimeC_0_precision_tag CounterToLocalTimeC_0_Counter_precision_tag;

typedef uint32_t CounterToLocalTimeC_0_Counter_size_type;

enum Msp430Timer32khzC_1___nesc_unnamed4297 { Msp430Timer32khzC_1_ALARM_ID = 1U };

typedef T32khz Msp430AlarmC_1_frequency_tag;

typedef Msp430AlarmC_1_frequency_tag Msp430AlarmC_1_Alarm_precision_tag;

typedef uint16_t Msp430AlarmC_1_Alarm_size_type;

typedef TMilli TransformCounterC_1_to_precision_tag;

typedef uint32_t TransformCounterC_1_to_size_type;

typedef T32khz TransformCounterC_1_from_precision_tag;

typedef uint16_t TransformCounterC_1_from_size_type;

typedef uint32_t TransformCounterC_1_upper_count_type;

typedef TransformCounterC_1_from_precision_tag TransformCounterC_1_CounterFrom_precision_tag;

typedef TransformCounterC_1_from_size_type TransformCounterC_1_CounterFrom_size_type;

typedef TransformCounterC_1_to_precision_tag TransformCounterC_1_Counter_precision_tag;

typedef TransformCounterC_1_to_size_type TransformCounterC_1_Counter_size_type;

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

typedef TMilli AlarmToTimerC_0_precision_tag;

typedef AlarmToTimerC_0_precision_tag AlarmToTimerC_0_Alarm_precision_tag;

typedef uint32_t AlarmToTimerC_0_Alarm_size_type;

typedef AlarmToTimerC_0_precision_tag AlarmToTimerC_0_Timer_precision_tag;

typedef TMilli VirtualizeTimerC_0_precision_tag;

typedef VirtualizeTimerC_0_precision_tag VirtualizeTimerC_0_TimerFrom_precision_tag;

typedef VirtualizeTimerC_0_precision_tag VirtualizeTimerC_0_Timer_precision_tag;

typedef TMilli CounterToLocalTimeC_1_precision_tag;

typedef CounterToLocalTimeC_1_precision_tag CounterToLocalTimeC_1_LocalTime_precision_tag;

typedef CounterToLocalTimeC_1_precision_tag CounterToLocalTimeC_1_Counter_precision_tag;

typedef uint32_t CounterToLocalTimeC_1_Counter_size_type;

enum CC2420SpiC_4___nesc_unnamed4298 { CC2420SpiC_4_CLIENT_ID = 4U };

typedef uint16_t RandomMlcgC_SeedInit_parameter;

enum AMQueueP___nesc_unnamed4299 { AMQueueP_NUM_CLIENTS = 2U };

typedef uint32_t DisseminatorC_0_t;

enum DisseminatorC_0___nesc_unnamed4300 { DisseminatorC_0_TIMER_ID = 0U };

typedef DisseminatorC_0_t DisseminatorP_0_t;

typedef DisseminatorP_0_t DisseminatorP_0_DisseminationUpdate_t;

typedef DisseminatorP_0_t DisseminatorP_0_DisseminationValue_t;

typedef TMilli TrickleTimerImplP_0_Timer_precision_tag;

typedef uint16_t DisseminatorC_1_t;

enum DisseminatorC_1___nesc_unnamed4301 { DisseminatorC_1_TIMER_ID = 1U };

typedef DisseminatorC_1_t DisseminatorP_1_t;

typedef DisseminatorP_1_t DisseminatorP_1_DisseminationUpdate_t;

typedef DisseminatorP_1_t DisseminatorP_1_DisseminationValue_t;

static void TestDisseminationC_Boot_booted( void ) ;

static void TestDisseminationC_RadioControl_startDone(error_t error) ;

static void TestDisseminationC_RadioControl_stopDone(error_t error) ;

static void TestDisseminationC_Value32_changed( void ) ;

static void TestDisseminationC_Value16_changed( void ) ;

static void TestDisseminationC_Timer_fired( void ) ;

static error_t PlatformP_Init_init( void ) ;

static error_t MotePlatformC_Init_init( void ) ;

static void Msp430ClockP_Msp430ClockInit_defaultInitClocks( void ) ;

static void Msp430ClockP_Msp430ClockInit_default_initTimerB( void ) ;

static void Msp430ClockP_Msp430ClockInit_defaultInitTimerA( void ) ;

static void Msp430ClockP_Msp430ClockInit_default_initTimerA( void ) ;

static void Msp430ClockP_Msp430ClockInit_defaultInitTimerB( void ) ;

static void Msp430ClockP_Msp430ClockInit_defaultSetupDcoCalibrate( void ) ;

static void Msp430ClockP_Msp430ClockInit_default_setupDcoCalibrate( void ) ;

static void Msp430ClockP_Msp430ClockInit_default_initClocks( void ) ;

static error_t Msp430ClockP_Init_init( void ) ;

static void Msp430TimerP_1_VectorTimerX0_fired( void ) ;

static void Msp430TimerP_1_Overflow_fired( void ) ;

static void Msp430TimerP_1_Event_default_fired(uint8_t arg_0x1593680) ;

static uint16_t Msp430TimerP_1_Timer_get( void ) ;

static bool Msp430TimerP_1_Timer_isOverflowPending( void ) ;

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

static void Msp430TimerCapComP_4_Capture_clearOverflow( void ) ;

static void Msp430TimerCapComP_4_Control_setControlAsCapture(uint8_t cm) ;

static msp430_compare_control_t Msp430TimerCapComP_4_Control_getControl( void ) ;

static void Msp430TimerCapComP_4_Control_enableEvents( void ) ;

static void Msp430TimerCapComP_4_Control_disableEvents( void ) ;

static void Msp430TimerCapComP_4_Control_clearPendingInterrupt( void ) ;

static void Msp430TimerCapComP_4_Event_fired( void ) ;

static void Msp430TimerCapComP_4_Compare_default_fired( void ) ;

static void Msp430TimerCapComP_4_Timer_overflow( void ) ;

static uint16_t Msp430TimerCapComP_5_Capture_getEvent( void ) ;

static void Msp430TimerCapComP_5_Capture_default_captured(uint16_t time) ;

static msp430_compare_control_t Msp430TimerCapComP_5_Control_getControl( void ) ;

static void Msp430TimerCapComP_5_Control_enableEvents( void ) ;

static void Msp430TimerCapComP_5_Control_setControlAsCompare( void ) ;

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

static message_t *CC2420ActiveMessageP_SubReceive_receive(message_t *msg, void *payload, uint8_t len) ;

static void CC2420ActiveMessageP_SubSend_sendDone(message_t *msg, error_t error) ;

static void CC2420ActiveMessageP_CC2420Config_syncDone(error_t error) ;

static void CC2420ActiveMessageP_RadioBackoff_default_requestCca(am_id_t arg_0x16fd2e8, message_t *msg) ;

static void CC2420ActiveMessageP_RadioBackoff_default_requestInitialBackoff(am_id_t arg_0x16fd2e8, message_t *msg) ;

static void CC2420ActiveMessageP_RadioBackoff_default_requestCongestionBackoff(am_id_t arg_0x16fd2e8, message_t *msg) ;

static void CC2420ActiveMessageP_SendNotifier_default_aboutToSend(am_id_t arg_0x16fec58, am_addr_t dest, message_t *msg) ;

static void CC2420ActiveMessageP_SubBackoff_requestCca(message_t *msg) ;

static void CC2420ActiveMessageP_SubBackoff_requestInitialBackoff(message_t *msg) ;

static void CC2420ActiveMessageP_SubBackoff_requestCongestionBackoff(message_t *msg) ;

static uint8_t CC2420ActiveMessageP_Packet_payloadLength(message_t *msg) ;

static void *CC2420ActiveMessageP_Packet_getPayload(message_t *msg, uint8_t len) ;

static uint8_t CC2420ActiveMessageP_Packet_maxPayloadLength( void ) ;

static void CC2420ActiveMessageP_Packet_setPayloadLength(message_t *msg, uint8_t len) ;

static error_t CC2420ActiveMessageP_AMSend_send(am_id_t arg_0x16e11b8, am_addr_t addr, message_t *msg, uint8_t len) ;

static void *CC2420ActiveMessageP_AMSend_getPayload(am_id_t arg_0x16e11b8, message_t *msg, uint8_t len) ;

static uint8_t CC2420ActiveMessageP_AMSend_maxPayloadLength(am_id_t arg_0x16e11b8) ;

static message_t *CC2420ActiveMessageP_Snoop_default_receive(am_id_t arg_0x16df220, message_t *msg, void *payload, uint8_t len) ;

static message_t *CC2420ActiveMessageP_Receive_default_receive(am_id_t arg_0x16e1b78, message_t *msg, void *payload, uint8_t len) ;

static am_addr_t CC2420ActiveMessageP_AMPacket_address( void ) ;

static am_addr_t CC2420ActiveMessageP_AMPacket_destination(message_t *amsg) ;

static void CC2420ActiveMessageP_AMPacket_setDestination(message_t *amsg, am_addr_t addr) ;

static am_id_t CC2420ActiveMessageP_AMPacket_type(message_t *amsg) ;

static void CC2420ActiveMessageP_AMPacket_setType(message_t *amsg, am_id_t t) ;

static bool CC2420ActiveMessageP_AMPacket_isForMe(message_t *amsg) ;

static error_t CC2420CsmaP_SplitControl_start( void ) ;

static void CC2420CsmaP_SubBackoff_requestInitialBackoff(message_t *msg) ;

static void CC2420CsmaP_SubBackoff_requestCongestionBackoff(message_t *msg) ;

static void CC2420CsmaP_CC2420Transmit_sendDone(message_t *p_msg, error_t error) ;

static error_t CC2420CsmaP_Send_send(message_t *msg, uint8_t len) ;

static void *CC2420CsmaP_Send_getPayload(message_t *msg, uint8_t len) ;

static uint8_t CC2420CsmaP_Send_maxPayloadLength( void ) ;

static void CC2420CsmaP_CC2420Power_startOscillatorDone( void ) ;

static void CC2420CsmaP_CC2420Power_startVRegDone( void ) ;

static void CC2420CsmaP_Resource_granted( void ) ;

static void CC2420CsmaP_sendDone_task_runTask( void ) ;

static void CC2420CsmaP_stopDone_task_runTask( void ) ;

static void CC2420CsmaP_startDone_task_runTask( void ) ;

static bool CC2420ControlP_CC2420Config_isAddressRecognitionEnabled( void ) ;

static bool CC2420ControlP_CC2420Config_isAutoAckEnabled( void ) ;

static bool CC2420ControlP_CC2420Config_isHwAutoAckDefault( void ) ;

static uint16_t CC2420ControlP_CC2420Config_getShortAddr( void ) ;

static error_t CC2420ControlP_CC2420Config_sync( void ) ;

static uint16_t CC2420ControlP_CC2420Config_getPanAddr( void ) ;

static void CC2420ControlP_StartupTimer_fired( void ) ;

static void CC2420ControlP_ReadRssi_default_readDone(error_t result, CC2420ControlP_ReadRssi_val_t val) ;

static void CC2420ControlP_syncDone_runTask( void ) ;

static error_t CC2420ControlP_Init_init( void ) ;

static void CC2420ControlP_SpiResource_granted( void ) ;

static void CC2420ControlP_SyncResource_granted( void ) ;

static error_t CC2420ControlP_CC2420Power_startOscillator( void ) ;

static error_t CC2420ControlP_CC2420Power_rxOn( void ) ;

static error_t CC2420ControlP_CC2420Power_startVReg( void ) ;

static error_t CC2420ControlP_CC2420Power_stopVReg( void ) ;

static void CC2420ControlP_sync_runTask( void ) ;

static error_t CC2420ControlP_Resource_release( void ) ;

static error_t CC2420ControlP_Resource_request( void ) ;

static void CC2420ControlP_InterruptCCA_fired( void ) ;

static void CC2420ControlP_RssiResource_granted( void ) ;

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

static void TransformAlarmC_0_Alarm_start(TransformAlarmC_0_Alarm_size_type dt) ;

static void TransformAlarmC_0_Alarm_stop( void ) ;

static void TransformAlarmC_0_AlarmFrom_fired( void ) ;

static void TransformAlarmC_0_Counter_overflow( void ) ;

static bool HplMsp430GeneralIOP_0_IO_get( void ) ;

static uint8_t HplMsp430GeneralIOP_0_IO_getRaw( void ) ;

static bool HplMsp430GeneralIOP_3_IO_get( void ) ;

static uint8_t HplMsp430GeneralIOP_3_IO_getRaw( void ) ;

static void HplMsp430GeneralIOP_4_IO_makeInput( void ) ;

static bool HplMsp430GeneralIOP_4_IO_get( void ) ;

static uint8_t HplMsp430GeneralIOP_4_IO_getRaw( void ) ;

static void HplMsp430GeneralIOP_17_IO_selectIOFunc( void ) ;

static void HplMsp430GeneralIOP_17_IO_selectModuleFunc( void ) ;

static void HplMsp430GeneralIOP_18_IO_selectIOFunc( void ) ;

static void HplMsp430GeneralIOP_18_IO_selectModuleFunc( void ) ;

static void HplMsp430GeneralIOP_19_IO_selectIOFunc( void ) ;

static void HplMsp430GeneralIOP_19_IO_selectModuleFunc( void ) ;

static void HplMsp430GeneralIOP_20_IO_selectIOFunc( void ) ;

static void HplMsp430GeneralIOP_21_IO_selectIOFunc( void ) ;

static void HplMsp430GeneralIOP_25_IO_makeInput( void ) ;

static bool HplMsp430GeneralIOP_25_IO_get( void ) ;

static void HplMsp430GeneralIOP_25_IO_selectIOFunc( void ) ;

static uint8_t HplMsp430GeneralIOP_25_IO_getRaw( void ) ;

static void HplMsp430GeneralIOP_25_IO_selectModuleFunc( void ) ;

static void HplMsp430GeneralIOP_26_IO_makeOutput( void ) ;

static void HplMsp430GeneralIOP_26_IO_set( void ) ;

static void HplMsp430GeneralIOP_26_IO_clr( void ) ;

static void HplMsp430GeneralIOP_29_IO_makeOutput( void ) ;

static void HplMsp430GeneralIOP_29_IO_set( void ) ;

static void HplMsp430GeneralIOP_29_IO_clr( void ) ;

static void HplMsp430GeneralIOP_30_IO_makeOutput( void ) ;

static void HplMsp430GeneralIOP_30_IO_set( void ) ;

static void HplMsp430GeneralIOP_30_IO_clr( void ) ;

static void HplMsp430GeneralIOP_36_IO_toggle( void ) ;

static void HplMsp430GeneralIOP_36_IO_makeOutput( void ) ;

static void HplMsp430GeneralIOP_36_IO_set( void ) ;

static void HplMsp430GeneralIOP_37_IO_toggle( void ) ;

static void HplMsp430GeneralIOP_37_IO_makeOutput( void ) ;

static void HplMsp430GeneralIOP_37_IO_set( void ) ;

static void HplMsp430GeneralIOP_38_IO_toggle( void ) ;

static void HplMsp430GeneralIOP_38_IO_makeOutput( void ) ;

static void HplMsp430GeneralIOP_38_IO_set( void ) ;

static void Msp430GpioC_0_GeneralIO_makeInput( void ) ;

static bool Msp430GpioC_0_GeneralIO_get( void ) ;

static void Msp430GpioC_1_GeneralIO_makeOutput( void ) ;

static void Msp430GpioC_1_GeneralIO_set( void ) ;

static void Msp430GpioC_1_GeneralIO_clr( void ) ;

static bool Msp430GpioC_2_GeneralIO_get( void ) ;

static bool Msp430GpioC_3_GeneralIO_get( void ) ;

static void Msp430GpioC_4_GeneralIO_makeOutput( void ) ;

static void Msp430GpioC_4_GeneralIO_set( void ) ;

static void Msp430GpioC_4_GeneralIO_clr( void ) ;

static void Msp430GpioC_5_GeneralIO_makeInput( void ) ;

static bool Msp430GpioC_5_GeneralIO_get( void ) ;

static void Msp430GpioC_6_GeneralIO_makeOutput( void ) ;

static void Msp430GpioC_6_GeneralIO_set( void ) ;

static void Msp430GpioC_6_GeneralIO_clr( void ) ;

static void GpioCaptureC_0_Msp430Capture_captured(uint16_t time) ;

static error_t GpioCaptureC_0_Capture_captureFallingEdge( void ) ;

static void GpioCaptureC_0_Capture_disable( void ) ;

static error_t GpioCaptureC_0_Capture_captureRisingEdge( void ) ;

static void HplMsp430InterruptP_Port14_clear( void ) ;

static void HplMsp430InterruptP_Port14_disable( void ) ;

static void HplMsp430InterruptP_Port14_edge(bool low_to_high) ;

static void HplMsp430InterruptP_Port14_enable( void ) ;

static void HplMsp430InterruptP_Port26_clear( void ) ;

static void HplMsp430InterruptP_Port26_default_fired( void ) ;

static void HplMsp430InterruptP_Port17_clear( void ) ;

static void HplMsp430InterruptP_Port17_default_fired( void ) ;

static void HplMsp430InterruptP_Port21_clear( void ) ;

static void HplMsp430InterruptP_Port21_default_fired( void ) ;

static void HplMsp430InterruptP_Port12_clear( void ) ;

static void HplMsp430InterruptP_Port12_default_fired( void ) ;

static void HplMsp430InterruptP_Port24_clear( void ) ;

static void HplMsp430InterruptP_Port24_default_fired( void ) ;

static void HplMsp430InterruptP_Port15_clear( void ) ;

static void HplMsp430InterruptP_Port15_default_fired( void ) ;

static void HplMsp430InterruptP_Port27_clear( void ) ;

static void HplMsp430InterruptP_Port27_default_fired( void ) ;

static void HplMsp430InterruptP_Port10_clear( void ) ;

static void HplMsp430InterruptP_Port10_disable( void ) ;

static void HplMsp430InterruptP_Port10_edge(bool low_to_high) ;

static void HplMsp430InterruptP_Port10_enable( void ) ;

static void HplMsp430InterruptP_Port22_clear( void ) ;

static void HplMsp430InterruptP_Port22_default_fired( void ) ;

static void HplMsp430InterruptP_Port13_clear( void ) ;

static void HplMsp430InterruptP_Port13_default_fired( void ) ;

static void HplMsp430InterruptP_Port25_clear( void ) ;

static void HplMsp430InterruptP_Port25_default_fired( void ) ;

static void HplMsp430InterruptP_Port16_clear( void ) ;

static void HplMsp430InterruptP_Port16_default_fired( void ) ;

static void HplMsp430InterruptP_Port20_clear( void ) ;

static void HplMsp430InterruptP_Port20_default_fired( void ) ;

static void HplMsp430InterruptP_Port11_clear( void ) ;

static void HplMsp430InterruptP_Port11_default_fired( void ) ;

static void HplMsp430InterruptP_Port23_clear( void ) ;

static void HplMsp430InterruptP_Port23_default_fired( void ) ;

static void Msp430InterruptC_0_HplInterrupt_fired( void ) ;

static error_t Msp430InterruptC_0_Interrupt_disable( void ) ;

static error_t Msp430InterruptC_0_Interrupt_enableRisingEdge( void ) ;

static void Msp430InterruptC_1_HplInterrupt_fired( void ) ;

static error_t Msp430InterruptC_1_Interrupt_disable( void ) ;

static error_t Msp430InterruptC_1_Interrupt_enableFallingEdge( void ) ;

static void CC2420SpiP_SpiPacket_sendDone(uint8_t *txBuf, uint8_t *rxBuf, uint16_t len, error_t error) ;

static error_t CC2420SpiP_Fifo_continueRead(uint8_t arg_0x1b79698, uint8_t *data, uint8_t length) ;

static void CC2420SpiP_Fifo_default_writeDone(uint8_t arg_0x1b79698, uint8_t *data, uint8_t length, error_t error) ;

static cc2420_status_t CC2420SpiP_Fifo_write(uint8_t arg_0x1b79698, uint8_t *data, uint8_t length) ;

static cc2420_status_t CC2420SpiP_Fifo_beginRead(uint8_t arg_0x1b79698, uint8_t *data, uint8_t length) ;

static void CC2420SpiP_Fifo_default_readDone(uint8_t arg_0x1b79698, uint8_t *data, uint8_t length, error_t error) ;

static void CC2420SpiP_ChipSpiResource_abortRelease( void ) ;

static error_t CC2420SpiP_ChipSpiResource_attemptRelease( void ) ;

static void CC2420SpiP_SpiResource_granted( void ) ;

static cc2420_status_t CC2420SpiP_Ram_write(uint16_t arg_0x1b77118, uint8_t offset, uint8_t *data, uint8_t length) ;

static cc2420_status_t CC2420SpiP_Reg_read(uint8_t arg_0x1b778c0, uint16_t *data) ;

static cc2420_status_t CC2420SpiP_Reg_write(uint8_t arg_0x1b778c0, uint16_t data) ;

static error_t CC2420SpiP_Resource_release(uint8_t arg_0x1b7abd0) ;

static error_t CC2420SpiP_Resource_immediateRequest(uint8_t arg_0x1b7abd0) ;

static error_t CC2420SpiP_Resource_request(uint8_t arg_0x1b7abd0) ;

static void CC2420SpiP_Resource_default_granted(uint8_t arg_0x1b7abd0) ;

static bool CC2420SpiP_Resource_isOwner(uint8_t arg_0x1b7abd0) ;

static void CC2420SpiP_grant_runTask( void ) ;

static cc2420_status_t CC2420SpiP_Strobe_strobe(uint8_t arg_0x1b76088) ;

static error_t StateImplP_Init_init( void ) ;

static void StateImplP_State_toIdle(uint8_t arg_0x1bee428) ;

static bool StateImplP_State_isState(uint8_t arg_0x1bee428, uint8_t myState) ;

static bool StateImplP_State_isIdle(uint8_t arg_0x1bee428) ;

static error_t StateImplP_State_requestState(uint8_t arg_0x1bee428, uint8_t reqState) ;

static void StateImplP_State_forceState(uint8_t arg_0x1bee428, uint8_t reqState) ;

static void Msp430SpiNoDmaP_0_ResourceConfigure_unconfigure(uint8_t arg_0x1c69c00) ;

static void Msp430SpiNoDmaP_0_ResourceConfigure_configure(uint8_t arg_0x1c69c00) ;

static error_t Msp430SpiNoDmaP_0_SpiPacket_send(uint8_t arg_0x1c67740, uint8_t *txBuf, uint8_t *rxBuf, uint16_t len) ;

static void Msp430SpiNoDmaP_0_SpiPacket_default_sendDone(uint8_t arg_0x1c67740, uint8_t *txBuf, uint8_t *rxBuf, uint16_t len, error_t error) ;

static msp430_spi_union_config_t *Msp430SpiNoDmaP_0_Msp430SpiConfigure_default_getConfig(uint8_t arg_0x1c66a60) ;

static uint8_t Msp430SpiNoDmaP_0_SpiByte_write(uint8_t tx) ;

static error_t Msp430SpiNoDmaP_0_UsartResource_default_release(uint8_t arg_0x1c66010) ;

static error_t Msp430SpiNoDmaP_0_UsartResource_default_immediateRequest(uint8_t arg_0x1c66010) ;

static error_t Msp430SpiNoDmaP_0_UsartResource_default_request(uint8_t arg_0x1c66010) ;

static void Msp430SpiNoDmaP_0_UsartResource_granted(uint8_t arg_0x1c66010) ;

static bool Msp430SpiNoDmaP_0_UsartResource_default_isOwner(uint8_t arg_0x1c66010) ;

static error_t Msp430SpiNoDmaP_0_Resource_release(uint8_t arg_0x1c691b8) ;

static error_t Msp430SpiNoDmaP_0_Resource_immediateRequest(uint8_t arg_0x1c691b8) ;

static error_t Msp430SpiNoDmaP_0_Resource_request(uint8_t arg_0x1c691b8) ;

static void Msp430SpiNoDmaP_0_Resource_default_granted(uint8_t arg_0x1c691b8) ;

static bool Msp430SpiNoDmaP_0_Resource_isOwner(uint8_t arg_0x1c691b8) ;

static void Msp430SpiNoDmaP_0_UsartInterrupts_rxDone(uint8_t data) ;

static void Msp430SpiNoDmaP_0_UsartInterrupts_txDone( void ) ;

static void Msp430SpiNoDmaP_0_signalDone_task_runTask( void ) ;

static void HplMsp430Usart0P_Usart_enableRxIntr( void ) ;

static void HplMsp430Usart0P_Usart_clrRxIntr( void ) ;

static void HplMsp430Usart0P_Usart_resetUsart(bool reset) ;

static void HplMsp430Usart0P_Usart_disableIntr( void ) ;

static void HplMsp430Usart0P_Usart_setUmctl(uint8_t umctl) ;

static void HplMsp430Usart0P_Usart_disableRxIntr( void ) ;

static void HplMsp430Usart0P_Usart_clrIntr( void ) ;

static void HplMsp430Usart0P_Usart_setUbr(uint16_t ubr) ;

static void HplMsp430Usart0P_Usart_tx(uint8_t data) ;

static void HplMsp430Usart0P_Usart_disableUart( void ) ;

static void HplMsp430Usart0P_Usart_enableSpi( void ) ;

static void HplMsp430Usart0P_Usart_setModeSpi(msp430_spi_union_config_t *config) ;

static uint8_t HplMsp430Usart0P_Usart_rx( void ) ;

static bool HplMsp430Usart0P_Usart_isRxIntrPending( void ) ;

static void HplMsp430Usart0P_Usart_disableSpi( void ) ;

static error_t LedsP_Init_init( void ) ;

static void LedsP_Leds_led0Toggle( void ) ;

static void LedsP_Leds_led1Toggle( void ) ;

static void LedsP_Leds_led2Toggle( void ) ;

static void Msp430GpioC_7_GeneralIO_toggle( void ) ;

static void Msp430GpioC_7_GeneralIO_makeOutput( void ) ;

static void Msp430GpioC_7_GeneralIO_set( void ) ;

static void Msp430GpioC_8_GeneralIO_toggle( void ) ;

static void Msp430GpioC_8_GeneralIO_makeOutput( void ) ;

static void Msp430GpioC_8_GeneralIO_set( void ) ;

static void Msp430GpioC_9_GeneralIO_toggle( void ) ;

static void Msp430GpioC_9_GeneralIO_makeOutput( void ) ;

static void Msp430GpioC_9_GeneralIO_set( void ) ;

static void Msp430UsartShareP_0_Interrupts_default_rxDone(uint8_t arg_0x1dac1d8, uint8_t data) ;

static void Msp430UsartShareP_0_Interrupts_default_txDone(uint8_t arg_0x1dac1d8) ;

static void Msp430UsartShareP_0_RawI2CInterrupts_fired( void ) ;

static void Msp430UsartShareP_0_I2CInterrupts_default_fired(uint8_t arg_0x1daca58) ;

static void Msp430UsartShareP_0_RawInterrupts_rxDone(uint8_t data) ;

static void Msp430UsartShareP_0_RawInterrupts_txDone( void ) ;

static error_t FcfsResourceQueueC_0_Init_init( void ) ;

static error_t FcfsResourceQueueC_0_FcfsQueue_enqueue(resource_client_id_t id) ;

static bool FcfsResourceQueueC_0_FcfsQueue_isEmpty( void ) ;

static bool FcfsResourceQueueC_0_FcfsQueue_isEnqueued(resource_client_id_t id) ;

static resource_client_id_t FcfsResourceQueueC_0_FcfsQueue_dequeue( void ) ;

static void ArbiterP_0_ResourceRequested_default_requested(uint8_t arg_0x1db7c58) ;

static void ArbiterP_0_ResourceRequested_default_immediateRequested(uint8_t arg_0x1db7c58) ;

static void ArbiterP_0_ResourceConfigure_default_unconfigure(uint8_t arg_0x1db4030) ;

static void ArbiterP_0_ResourceConfigure_default_configure(uint8_t arg_0x1db4030) ;

static error_t ArbiterP_0_ResourceDefaultOwner_release( void ) ;

static void ArbiterP_0_ResourceDefaultOwner_default_requested( void ) ;

static void ArbiterP_0_ResourceDefaultOwner_default_granted( void ) ;

static void ArbiterP_0_ResourceDefaultOwner_default_immediateRequested( void ) ;

static error_t ArbiterP_0_Resource_release(uint8_t arg_0x1db7230) ;

static error_t ArbiterP_0_Resource_immediateRequest(uint8_t arg_0x1db7230) ;

static error_t ArbiterP_0_Resource_request(uint8_t arg_0x1db7230) ;

static void ArbiterP_0_Resource_default_granted(uint8_t arg_0x1db7230) ;

static bool ArbiterP_0_Resource_isOwner(uint8_t arg_0x1db7230) ;

static bool ArbiterP_0_ArbiterInfo_inUse( void ) ;

static uint8_t ArbiterP_0_ArbiterInfo_userId( void ) ;

static void ArbiterP_0_grantedTask_runTask( void ) ;

static void HplMsp430I2C0P_HplI2C_clearModeI2C( void ) ;

static bool HplMsp430I2C0P_HplI2C_isI2C( void ) ;

static am_addr_t ActiveMessageAddressC_amAddress( void ) ;

static am_addr_t ActiveMessageAddressC_ActiveMessageAddress_amAddress( void ) ;

static am_group_t ActiveMessageAddressC_ActiveMessageAddress_amGroup( void ) ;

static void CC2420TransmitP_RadioBackoff_setCongestionBackoff(uint16_t backoffTime) ;

static void CC2420TransmitP_RadioBackoff_setInitialBackoff(uint16_t backoffTime) ;

static void CC2420TransmitP_CaptureSFD_captured(uint16_t time) ;

static void CC2420TransmitP_BackoffTimer_fired( void ) ;

static void CC2420TransmitP_CC2420Receive_receive(uint8_t type, message_t *message) ;

static error_t CC2420TransmitP_Send_send(message_t *p_msg, bool useCca) ;

static void CC2420TransmitP_ChipSpiResource_releasing( void ) ;

static error_t CC2420TransmitP_Init_init( void ) ;

static void CC2420TransmitP_SpiResource_granted( void ) ;

static error_t CC2420TransmitP_StdControl_start( void ) ;

static error_t CC2420TransmitP_StdControl_stop( void ) ;

static void CC2420TransmitP_TXFIFO_writeDone(uint8_t *data, uint8_t length, error_t error) ;

static void CC2420TransmitP_TXFIFO_readDone(uint8_t *data, uint8_t length, error_t error) ;

static void CC2420ReceiveP_CC2420Config_syncDone(error_t error) ;

static void CC2420ReceiveP_receiveDone_task_runTask( void ) ;

static void CC2420ReceiveP_CC2420Receive_sfd_dropped( void ) ;

static void CC2420ReceiveP_CC2420Receive_sfd(uint32_t time) ;

static error_t CC2420ReceiveP_Init_init( void ) ;

static void CC2420ReceiveP_SpiResource_granted( void ) ;

static void CC2420ReceiveP_RXFIFO_writeDone(uint8_t *data, uint8_t length, error_t error) ;

static void CC2420ReceiveP_RXFIFO_readDone(uint8_t *data, uint8_t length, error_t error) ;

static void CC2420ReceiveP_InterruptFIFOP_fired( void ) ;

static error_t CC2420ReceiveP_StdControl_start( void ) ;

static error_t CC2420ReceiveP_StdControl_stop( void ) ;

static void CC2420PacketP_PacketTimeStamp32khz_clear(message_t *msg) ;

static void CC2420PacketP_PacketTimeStamp32khz_set(message_t *msg, CC2420PacketP_PacketTimeStamp32khz_size_type value) ;

static cc2420_header_t *CC2420PacketP_CC2420PacketBody_getHeader(message_t *msg) ;

static cc2420_metadata_t *CC2420PacketP_CC2420PacketBody_getMetadata(message_t *msg) ;

static uint8_t CC2420PacketP_PacketTimeSyncOffset_get(message_t *msg) ;

static bool CC2420PacketP_PacketTimeSyncOffset_isSet(message_t *msg) ;

static void CounterToLocalTimeC_0_Counter_overflow( void ) ;

static void Msp430AlarmC_1_Msp430Compare_fired( void ) ;

static void Msp430AlarmC_1_Msp430Timer_overflow( void ) ;

static void Msp430AlarmC_1_Alarm_startAt(Msp430AlarmC_1_Alarm_size_type t0, Msp430AlarmC_1_Alarm_size_type dt) ;

static void Msp430AlarmC_1_Alarm_stop( void ) ;

static error_t Msp430AlarmC_1_Init_init( void ) ;

static void TransformCounterC_1_CounterFrom_overflow( void ) ;

static TransformCounterC_1_Counter_size_type TransformCounterC_1_Counter_get( void ) ;

static TransformAlarmC_1_Alarm_size_type TransformAlarmC_1_Alarm_getNow( void ) ;

static void TransformAlarmC_1_Alarm_startAt(TransformAlarmC_1_Alarm_size_type t0, TransformAlarmC_1_Alarm_size_type dt) ;

static TransformAlarmC_1_Alarm_size_type TransformAlarmC_1_Alarm_getAlarm( void ) ;

static void TransformAlarmC_1_Alarm_stop( void ) ;

static void TransformAlarmC_1_AlarmFrom_fired( void ) ;

static void TransformAlarmC_1_Counter_overflow( void ) ;

static void AlarmToTimerC_0_fired_runTask( void ) ;

static void AlarmToTimerC_0_Alarm_fired( void ) ;

static uint32_t AlarmToTimerC_0_Timer_getNow( void ) ;

static void AlarmToTimerC_0_Timer_startOneShotAt(uint32_t t0, uint32_t dt) ;

static void AlarmToTimerC_0_Timer_stop( void ) ;

static void VirtualizeTimerC_0_updateFromTimer_runTask( void ) ;

static void VirtualizeTimerC_0_TimerFrom_fired( void ) ;

static uint32_t VirtualizeTimerC_0_Timer_getNow(uint8_t arg_0x20a3dd8) ;

static void VirtualizeTimerC_0_Timer_default_fired(uint8_t arg_0x20a3dd8) ;

static uint32_t VirtualizeTimerC_0_Timer_getdt(uint8_t arg_0x20a3dd8) ;

static uint32_t VirtualizeTimerC_0_Timer_gett0(uint8_t arg_0x20a3dd8) ;

static void VirtualizeTimerC_0_Timer_startPeriodic(uint8_t arg_0x20a3dd8, uint32_t dt) ;

static void VirtualizeTimerC_0_Timer_startOneShot(uint8_t arg_0x20a3dd8, uint32_t dt) ;

static void VirtualizeTimerC_0_Timer_stop(uint8_t arg_0x20a3dd8) ;

static void CounterToLocalTimeC_1_Counter_overflow( void ) ;

static uint16_t RandomMlcgC_Random_rand16( void ) ;

static uint32_t RandomMlcgC_Random_rand32( void ) ;

static error_t RandomMlcgC_Init_init( void ) ;

static void UniqueSendP_SubSend_sendDone(message_t *msg, error_t error) ;

static error_t UniqueSendP_Send_send(message_t *msg, uint8_t len) ;

static void *UniqueSendP_Send_getPayload(message_t *msg, uint8_t len) ;

static error_t UniqueSendP_Init_init( void ) ;

static message_t *UniqueReceiveP_SubReceive_receive(message_t *msg, void *payload, uint8_t len) ;

static error_t UniqueReceiveP_Init_init( void ) ;

static message_t *UniqueReceiveP_DuplicateReceive_default_receive(message_t *msg, void *payload, uint8_t len) ;

static void CC2420TinyosNetworkP_SubSend_sendDone(message_t *msg, error_t error) ;

static message_t *CC2420TinyosNetworkP_SubReceive_receive(message_t *msg, void *payload, uint8_t len) ;

static error_t CC2420TinyosNetworkP_Send_send(message_t *msg, uint8_t len) ;

static void *CC2420TinyosNetworkP_Send_getPayload(message_t *msg, uint8_t len) ;

static message_t *CC2420TinyosNetworkP_NonTinyosReceive_default_receive(uint8_t arg_0x218d9b8, message_t *msg, void *payload, uint8_t len) ;

static message_t *DisseminationEngineImplP_ProbeReceive_receive(message_t *msg, void *payload, uint8_t len) ;

static void DisseminationEngineImplP_ProbeAMSend_sendDone(message_t *msg, error_t error) ;

static void DisseminationEngineImplP_AMSend_sendDone(message_t *msg, error_t error) ;

static void DisseminationEngineImplP_TrickleTimer_fired(uint16_t arg_0x21c4c10) ;

static void DisseminationEngineImplP_TrickleTimer_default_incrementCounter(uint16_t arg_0x21c4c10) ;

static void DisseminationEngineImplP_TrickleTimer_default_reset(uint16_t arg_0x21c4c10) ;

static error_t DisseminationEngineImplP_TrickleTimer_default_start(uint16_t arg_0x21c4c10) ;

static void DisseminationEngineImplP_DisseminationCache_default_storeData(uint16_t arg_0x21c4120, void *data, uint8_t size, uint32_t seqno) ;

static void DisseminationEngineImplP_DisseminationCache_newData(uint16_t arg_0x21c4120) ;

static error_t DisseminationEngineImplP_DisseminationCache_start(uint16_t arg_0x21c4120) ;

static uint32_t DisseminationEngineImplP_DisseminationCache_default_requestSeqno(uint16_t arg_0x21c4120) ;

static void *DisseminationEngineImplP_DisseminationCache_default_requestData(uint16_t arg_0x21c4120, uint8_t *size) ;

static message_t *DisseminationEngineImplP_Receive_receive(message_t *msg, void *payload, uint8_t len) ;

static error_t DisseminationEngineImplP_StdControl_start( void ) ;

static error_t DisseminationEngineImplP_DisseminatorControl_default_start(uint16_t arg_0x21c2738) ;

static error_t AMQueueEntryP_0_AMSend_send(am_addr_t addr, message_t *msg, uint8_t len) ;

static void *AMQueueEntryP_0_AMSend_getPayload(message_t *msg, uint8_t len) ;

static uint8_t AMQueueEntryP_0_AMSend_maxPayloadLength( void ) ;

static void AMQueueEntryP_0_Send_sendDone(message_t *msg, error_t error) ;

static void AMQueueImplP_0_AMSend_sendDone(am_id_t arg_0x2246108, message_t *msg, error_t error) ;

static error_t AMQueueImplP_0_Send_send(uint8_t arg_0x22476f8, message_t *msg, uint8_t len) ;

static void *AMQueueImplP_0_Send_getPayload(uint8_t arg_0x22476f8, message_t *msg, uint8_t len) ;

static uint8_t AMQueueImplP_0_Send_maxPayloadLength(uint8_t arg_0x22476f8) ;

static void AMQueueImplP_0_Send_default_sendDone(uint8_t arg_0x22476f8, message_t *msg, error_t error) ;

static void AMQueueImplP_0_errorTask_runTask( void ) ;

static void AMQueueImplP_0_CancelTask_runTask( void ) ;

static void AMQueueEntryP_1_Send_sendDone(message_t *msg, error_t error) ;

static void DisseminatorP_0_changedTask_runTask( void ) ;

static void *DisseminatorP_0_DisseminationCache_requestData(uint8_t *size) ;

static void DisseminatorP_0_DisseminationCache_storeData(void *data, uint8_t size, uint32_t seqno) ;

static uint32_t DisseminatorP_0_DisseminationCache_requestSeqno( void ) ;

static void DisseminatorP_0_DisseminationUpdate_change(DisseminatorP_0_DisseminationUpdate_t *newVal) ;

const static DisseminatorP_0_DisseminationValue_t *DisseminatorP_0_DisseminationValue_get( void ) ;

static void DisseminatorP_0_DisseminationValue_set(const DisseminatorP_0_DisseminationValue_t *arg_0x14ad7a8) ;

static error_t DisseminatorP_0_StdControl_start( void ) ;

static void TrickleTimerImplP_0_TrickleTimer_default_fired(uint8_t arg_0x22e4c20) ;

static void TrickleTimerImplP_0_TrickleTimer_incrementCounter(uint8_t arg_0x22e4c20) ;

static void TrickleTimerImplP_0_TrickleTimer_reset(uint8_t arg_0x22e4c20) ;

static error_t TrickleTimerImplP_0_TrickleTimer_start(uint8_t arg_0x22e4c20) ;

static error_t TrickleTimerImplP_0_Init_init( void ) ;

static void TrickleTimerImplP_0_timerTask_runTask( void ) ;

static void TrickleTimerImplP_0_Timer_fired( void ) ;

static void BitVectorC_0_BitVector_clearAll( void ) ;

static void BitVectorC_0_BitVector_clear(uint16_t bitnum) ;

static bool BitVectorC_0_BitVector_get(uint16_t bitnum) ;

static void BitVectorC_0_BitVector_set(uint16_t bitnum) ;

static void BitVectorC_1_BitVector_clearAll( void ) ;

static void BitVectorC_1_BitVector_clear(uint16_t bitnum) ;

static bool BitVectorC_1_BitVector_get(uint16_t bitnum) ;

static void BitVectorC_1_BitVector_set(uint16_t bitnum) ;

static void DisseminatorP_1_changedTask_runTask( void ) ;

static void *DisseminatorP_1_DisseminationCache_requestData(uint8_t *size) ;

static void DisseminatorP_1_DisseminationCache_storeData(void *data, uint8_t size, uint32_t seqno) ;

static uint32_t DisseminatorP_1_DisseminationCache_requestSeqno( void ) ;

static void DisseminatorP_1_DisseminationUpdate_change(DisseminatorP_1_DisseminationUpdate_t *newVal) ;

const static DisseminatorP_1_DisseminationValue_t *DisseminatorP_1_DisseminationValue_get( void ) ;

static void DisseminatorP_1_DisseminationValue_set(const DisseminatorP_1_DisseminationValue_t *arg_0x14ad7a8) ;

static error_t DisseminatorP_1_StdControl_start( void ) ;

static error_t TestDisseminationC_RadioControl_start( void ) ;

static error_t TestDisseminationC_DisseminationControl_start( void ) ;

static void TestDisseminationC_Update32_change(TestDisseminationC_Update32_t *newVal) ;

const static TestDisseminationC_Value32_t *TestDisseminationC_Value32_get( void ) ;

static void TestDisseminationC_Value32_set(const TestDisseminationC_Value32_t *arg_0x14ad7a8) ;

const static TestDisseminationC_Value16_t *TestDisseminationC_Value16_get( void ) ;

static void TestDisseminationC_Value16_set(const TestDisseminationC_Value16_t *arg_0x14ad7a8) ;

static void TestDisseminationC_Update16_change(TestDisseminationC_Update16_t *newVal) ;

static void TestDisseminationC_Leds_led0Toggle( void ) ;

static void TestDisseminationC_Leds_led1Toggle( void ) ;

static void TestDisseminationC_Leds_led2Toggle( void ) ;

static void TestDisseminationC_Timer_startPeriodic(uint32_t dt) ;

inline static void TestDisseminationC_Boot_booted( void ) ;

inline static void TestDisseminationC_RadioControl_startDone(error_t result) ;

inline static void TestDisseminationC_RadioControl_stopDone(error_t result) ;

inline static void TestDisseminationC_Timer_fired( void ) ;

static void TestDisseminationC_Value32_changed( void ) ;

static void TestDisseminationC_Value16_changed( void ) ;

static error_t PlatformP_MoteInit_init( void ) ;

static error_t PlatformP_MoteClockInit_init( void ) ;

static error_t PlatformP_LedsInit_init( void ) ;

inline static error_t PlatformP_Init_init( void ) ;

__inline static void MotePlatformC_uwait(uint16_t u) ;

__inline static void MotePlatformC_TOSH_wait( void ) ;

static void MotePlatformC_TOSH_FLASH_M25P_DP_bit(bool set) ;

inline static void MotePlatformC_TOSH_FLASH_M25P_DP( void ) ;

inline static error_t MotePlatformC_Init_init( void ) ;

static void Msp430ClockP_Msp430ClockInit_initTimerB( void ) ;

static void Msp430ClockP_Msp430ClockInit_initTimerA( void ) ;

static void Msp430ClockP_Msp430ClockInit_setupDcoCalibrate( void ) ;

static void Msp430ClockP_Msp430ClockInit_initClocks( void ) ;

enum Msp430ClockP___nesc_unnamed4302 { Msp430ClockP_ACLK_CALIB_PERIOD = 8, Msp430ClockP_TARGET_DCO_DELTA = (4096 / 32) * Msp430ClockP_ACLK_CALIB_PERIOD };

inline static void Msp430ClockP_Msp430ClockInit_defaultSetupDcoCalibrate( void ) ;

inline static void Msp430ClockP_Msp430ClockInit_defaultInitClocks( void ) ;

inline static void Msp430ClockP_Msp430ClockInit_defaultInitTimerA( void ) ;

inline static void Msp430ClockP_Msp430ClockInit_defaultInitTimerB( void ) ;

inline static void Msp430ClockP_Msp430ClockInit_default_setupDcoCalibrate( void ) ;

inline static void Msp430ClockP_Msp430ClockInit_default_initClocks( void ) ;

inline static void Msp430ClockP_Msp430ClockInit_default_initTimerA( void ) ;

inline static void Msp430ClockP_Msp430ClockInit_default_initTimerB( void ) ;

inline static void Msp430ClockP_startTimerA( void ) ;

inline static void Msp430ClockP_startTimerB( void ) ;

static void Msp430ClockP_set_dco_calib(int calib) ;

inline static uint16_t Msp430ClockP_test_calib_busywait_delta(int calib) ;

inline static void Msp430ClockP_busyCalibrateDco( void ) ;

inline static error_t Msp430ClockP_Init_init( void ) ;

static void Msp430TimerP_1_Event_fired(uint8_t arg_0x1593680) ;

static void Msp430TimerP_1_Timer_overflow( void ) ;

static uint16_t Msp430TimerP_1_Timer_get( void ) ;

inline static bool Msp430TimerP_1_Timer_isOverflowPending( void ) ;

inline static void Msp430TimerP_1_VectorTimerX0_fired( void ) ;

inline static void Msp430TimerP_1_Overflow_fired( void ) ;

static void Msp430TimerP_1_Event_default_fired(uint8_t n) ;

typedef msp430_compare_control_t Msp430TimerCapComP_0_cc_t;

typedef msp430_compare_control_t Msp430TimerCapComP_1_cc_t;

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

inline static uint16_t Msp430TimerCapComP_4_CC2int(Msp430TimerCapComP_4_cc_t x) ;

inline static Msp430TimerCapComP_4_cc_t Msp430TimerCapComP_4_int2CC(uint16_t x) ;

inline static uint16_t Msp430TimerCapComP_4_captureControl(uint8_t l_cm) ;

inline static Msp430TimerCapComP_4_cc_t Msp430TimerCapComP_4_Control_getControl( void ) ;

inline static void Msp430TimerCapComP_4_Control_clearPendingInterrupt( void ) ;

inline static void Msp430TimerCapComP_4_Control_setControlAsCapture(uint8_t cm) ;

inline static void Msp430TimerCapComP_4_Control_enableEvents( void ) ;

inline static void Msp430TimerCapComP_4_Control_disableEvents( void ) ;

inline static uint16_t Msp430TimerCapComP_4_Capture_getEvent( void ) ;

inline static void Msp430TimerCapComP_4_Capture_clearOverflow( void ) ;

inline static void Msp430TimerCapComP_4_Event_fired( void ) ;

inline static void Msp430TimerCapComP_4_Compare_default_fired( void ) ;

inline static void Msp430TimerCapComP_4_Timer_overflow( void ) ;

static void Msp430TimerCapComP_5_Capture_captured(uint16_t time) ;

static void Msp430TimerCapComP_5_Compare_fired( void ) ;

static uint16_t Msp430TimerCapComP_5_Timer_get( void ) ;

typedef msp430_compare_control_t Msp430TimerCapComP_5_cc_t;

inline static uint16_t Msp430TimerCapComP_5_CC2int(Msp430TimerCapComP_5_cc_t x) ;

inline static Msp430TimerCapComP_5_cc_t Msp430TimerCapComP_5_int2CC(uint16_t x) ;

inline static uint16_t Msp430TimerCapComP_5_compareControl( void ) ;

inline static Msp430TimerCapComP_5_cc_t Msp430TimerCapComP_5_Control_getControl( void ) ;

inline static void Msp430TimerCapComP_5_Control_clearPendingInterrupt( void ) ;

inline static void Msp430TimerCapComP_5_Control_setControlAsCompare( void ) ;

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

enum SchedulerBasicP___nesc_unnamed4303 { SchedulerBasicP_NUM_TASKS = 16U, SchedulerBasicP_NO_TASK = 255 };

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

static error_t CC2420ActiveMessageP_SubSend_send(message_t *msg, uint8_t len) ;

static void *CC2420ActiveMessageP_SubSend_getPayload(message_t *msg, uint8_t len) ;

static uint16_t CC2420ActiveMessageP_CC2420Config_getPanAddr( void ) ;

static void CC2420ActiveMessageP_RadioBackoff_requestCca(am_id_t arg_0x16fd2e8, message_t *msg) ;

static void CC2420ActiveMessageP_RadioBackoff_requestInitialBackoff(am_id_t arg_0x16fd2e8, message_t *msg) ;

static void CC2420ActiveMessageP_RadioBackoff_requestCongestionBackoff(am_id_t arg_0x16fd2e8, message_t *msg) ;

static void CC2420ActiveMessageP_SendNotifier_aboutToSend(am_id_t arg_0x16fec58, am_addr_t dest, message_t *msg) ;

static void CC2420ActiveMessageP_AMSend_sendDone(am_id_t arg_0x16e11b8, message_t *msg, error_t error) ;

static message_t *CC2420ActiveMessageP_Snoop_receive(am_id_t arg_0x16df220, message_t *msg, void *payload, uint8_t len) ;

static am_addr_t CC2420ActiveMessageP_ActiveMessageAddress_amAddress( void ) ;

static cc2420_header_t *CC2420ActiveMessageP_CC2420PacketBody_getHeader(message_t *msg) ;

static cc2420_metadata_t *CC2420ActiveMessageP_CC2420PacketBody_getMetadata(message_t *msg) ;

static message_t *CC2420ActiveMessageP_Receive_receive(am_id_t arg_0x16e1b78, message_t *msg, void *payload, uint8_t len) ;

static error_t CC2420ActiveMessageP_AMSend_send(am_id_t id, am_addr_t addr, message_t *msg, uint8_t len) ;

inline static uint8_t CC2420ActiveMessageP_AMSend_maxPayloadLength(am_id_t id) ;

inline static void *CC2420ActiveMessageP_AMSend_getPayload(am_id_t id, message_t *m, uint8_t len) ;

inline static am_addr_t CC2420ActiveMessageP_AMPacket_address( void ) ;

static am_addr_t CC2420ActiveMessageP_AMPacket_destination(message_t *amsg) ;

inline static void CC2420ActiveMessageP_AMPacket_setDestination(message_t *amsg, am_addr_t addr) ;

inline static bool CC2420ActiveMessageP_AMPacket_isForMe(message_t *amsg) ;

static am_id_t CC2420ActiveMessageP_AMPacket_type(message_t *amsg) ;

inline static void CC2420ActiveMessageP_AMPacket_setType(message_t *amsg, am_id_t type) ;

inline static uint8_t CC2420ActiveMessageP_Packet_payloadLength(message_t *msg) ;

inline static void CC2420ActiveMessageP_Packet_setPayloadLength(message_t *msg, uint8_t len) ;

inline static uint8_t CC2420ActiveMessageP_Packet_maxPayloadLength( void ) ;

inline static void *CC2420ActiveMessageP_Packet_getPayload(message_t *msg, uint8_t len) ;

inline static void CC2420ActiveMessageP_SubSend_sendDone(message_t *msg, error_t result) ;

inline static message_t *CC2420ActiveMessageP_SubReceive_receive(message_t *msg, void *payload, uint8_t len) ;

inline static void CC2420ActiveMessageP_CC2420Config_syncDone(error_t error) ;

inline static void CC2420ActiveMessageP_SubBackoff_requestInitialBackoff(message_t *msg) ;

inline static void CC2420ActiveMessageP_SubBackoff_requestCongestionBackoff(message_t *msg) ;

inline static void CC2420ActiveMessageP_SubBackoff_requestCca(message_t *msg) ;

inline static message_t *CC2420ActiveMessageP_Receive_default_receive(am_id_t id, message_t *msg, void *payload, uint8_t len) ;

inline static message_t *CC2420ActiveMessageP_Snoop_default_receive(am_id_t id, message_t *msg, void *payload, uint8_t len) ;

inline static void CC2420ActiveMessageP_SendNotifier_default_aboutToSend(am_id_t amId, am_addr_t addr, message_t *msg) ;

inline static void CC2420ActiveMessageP_RadioBackoff_default_requestInitialBackoff(am_id_t id, message_t *msg) ;

inline static void CC2420ActiveMessageP_RadioBackoff_default_requestCongestionBackoff(am_id_t id, message_t *msg) ;

inline static void CC2420ActiveMessageP_RadioBackoff_default_requestCca(am_id_t id, message_t *msg) ;

static void CC2420CsmaP_SplitControl_startDone(error_t error) ;

static void CC2420CsmaP_SplitControl_stopDone(error_t error) ;

static void CC2420CsmaP_RadioBackoff_requestCca(message_t *msg) ;

static void CC2420CsmaP_RadioBackoff_requestInitialBackoff(message_t *msg) ;

static void CC2420CsmaP_RadioBackoff_requestCongestionBackoff(message_t *msg) ;

static void CC2420CsmaP_SubBackoff_setCongestionBackoff(uint16_t backoffTime) ;

static void CC2420CsmaP_SubBackoff_setInitialBackoff(uint16_t backoffTime) ;

static error_t CC2420CsmaP_CC2420Transmit_send(message_t *p_msg, bool useCca) ;

static void CC2420CsmaP_Send_sendDone(message_t *msg, error_t error) ;

static uint16_t CC2420CsmaP_Random_rand16( void ) ;

static error_t CC2420CsmaP_SubControl_start( void ) ;

static error_t CC2420CsmaP_SubControl_stop( void ) ;

static cc2420_header_t *CC2420CsmaP_CC2420PacketBody_getHeader(message_t *msg) ;

static cc2420_metadata_t *CC2420CsmaP_CC2420PacketBody_getMetadata(message_t *msg) ;

static error_t CC2420CsmaP_CC2420Power_startOscillator( void ) ;

static error_t CC2420CsmaP_CC2420Power_rxOn( void ) ;

static error_t CC2420CsmaP_CC2420Power_startVReg( void ) ;

static error_t CC2420CsmaP_CC2420Power_stopVReg( void ) ;

static error_t CC2420CsmaP_Resource_release( void ) ;

static error_t CC2420CsmaP_Resource_request( void ) ;

static bool CC2420CsmaP_SplitControlState_isState(uint8_t myState) ;

static error_t CC2420CsmaP_SplitControlState_requestState(uint8_t reqState) ;

static void CC2420CsmaP_SplitControlState_forceState(uint8_t reqState) ;

static error_t CC2420CsmaP_sendDone_task_postTask( void ) ;

static error_t CC2420CsmaP_stopDone_task_postTask( void ) ;

static error_t CC2420CsmaP_startDone_task_postTask( void ) ;

enum CC2420CsmaP___nesc_unnamed4304 { CC2420CsmaP_startDone_task = 0U };

typedef int (CC2420CsmaP___nesc_sillytask_startDone_task[CC2420CsmaP_startDone_task]);

enum CC2420CsmaP___nesc_unnamed4305 { CC2420CsmaP_stopDone_task = 1U };

typedef int (CC2420CsmaP___nesc_sillytask_stopDone_task[CC2420CsmaP_stopDone_task]);

enum CC2420CsmaP___nesc_unnamed4306 { CC2420CsmaP_sendDone_task = 2U };

typedef int (CC2420CsmaP___nesc_sillytask_sendDone_task[CC2420CsmaP_sendDone_task]);

enum CC2420CsmaP___nesc_unnamed4307 { CC2420CsmaP_S_STOPPED, CC2420CsmaP_S_STARTING, CC2420CsmaP_S_STARTED, CC2420CsmaP_S_STOPPING, CC2420CsmaP_S_TRANSMITTING };

message_t *CC2420CsmaP_m_msg;

error_t CC2420CsmaP_sendErr = SUCCESS;

bool CC2420CsmaP_ccaOn;

inline static void CC2420CsmaP_shutdown( void ) ;

static error_t CC2420CsmaP_SplitControl_start( void ) ;

inline static error_t CC2420CsmaP_Send_send(message_t *p_msg, uint8_t len) ;

static void *CC2420CsmaP_Send_getPayload(message_t *m, uint8_t len) ;

inline static uint8_t CC2420CsmaP_Send_maxPayloadLength( void ) ;

inline static void CC2420CsmaP_CC2420Transmit_sendDone(message_t *p_msg, error_t err) ;

inline static void CC2420CsmaP_CC2420Power_startVRegDone( void ) ;

inline static void CC2420CsmaP_Resource_granted( void ) ;

inline static void CC2420CsmaP_CC2420Power_startOscillatorDone( void ) ;

inline static void CC2420CsmaP_SubBackoff_requestInitialBackoff(message_t *msg) ;

inline static void CC2420CsmaP_SubBackoff_requestCongestionBackoff(message_t *msg) ;

inline static void CC2420CsmaP_sendDone_task_runTask( void ) ;

inline static void CC2420CsmaP_startDone_task_runTask( void ) ;

inline static void CC2420CsmaP_stopDone_task_runTask( void ) ;

inline static void CC2420CsmaP_shutdown( void ) ;

static void CC2420ControlP_CC2420Config_syncDone(error_t error) ;

static cc2420_status_t CC2420ControlP_RXCTRL1_write(uint16_t data) ;

static void CC2420ControlP_StartupTimer_start(CC2420ControlP_StartupTimer_size_type dt) ;

static cc2420_status_t CC2420ControlP_MDMCTRL0_write(uint16_t data) ;

static void CC2420ControlP_RSTN_makeOutput( void ) ;

static void CC2420ControlP_RSTN_set( void ) ;

static void CC2420ControlP_RSTN_clr( void ) ;

static void CC2420ControlP_ReadRssi_readDone(error_t result, CC2420ControlP_ReadRssi_val_t val) ;

static error_t CC2420ControlP_syncDone_postTask( void ) ;

static cc2420_status_t CC2420ControlP_RSSI_read(uint16_t *data) ;

static cc2420_status_t CC2420ControlP_IOCFG0_write(uint16_t data) ;

static am_addr_t CC2420ControlP_ActiveMessageAddress_amAddress( void ) ;

static am_group_t CC2420ControlP_ActiveMessageAddress_amGroup( void ) ;

static void CC2420ControlP_CSN_makeOutput( void ) ;

static void CC2420ControlP_CSN_set( void ) ;

static void CC2420ControlP_CSN_clr( void ) ;

static void CC2420ControlP_VREN_makeOutput( void ) ;

static void CC2420ControlP_VREN_set( void ) ;

static void CC2420ControlP_VREN_clr( void ) ;

static cc2420_status_t CC2420ControlP_SXOSCON_strobe( void ) ;

static error_t CC2420ControlP_SpiResource_release( void ) ;

static error_t CC2420ControlP_SpiResource_request( void ) ;

static error_t CC2420ControlP_SyncResource_release( void ) ;

static error_t CC2420ControlP_SyncResource_request( void ) ;

static void CC2420ControlP_CC2420Power_startOscillatorDone( void ) ;

static void CC2420ControlP_CC2420Power_startVRegDone( void ) ;

static cc2420_status_t CC2420ControlP_IOCFG1_write(uint16_t data) ;

static cc2420_status_t CC2420ControlP_FSCTRL_write(uint16_t data) ;

static cc2420_status_t CC2420ControlP_SRXON_strobe( void ) ;

static void CC2420ControlP_Resource_granted( void ) ;

static cc2420_status_t CC2420ControlP_PANID_write(uint8_t offset, uint8_t *data, uint8_t length) ;

static error_t CC2420ControlP_InterruptCCA_disable( void ) ;

static error_t CC2420ControlP_InterruptCCA_enableRisingEdge( void ) ;

static error_t CC2420ControlP_RssiResource_release( void ) ;

static cc2420_status_t CC2420ControlP_SRFOFF_strobe( void ) ;

enum CC2420ControlP___nesc_unnamed4308 { CC2420ControlP_sync = 3U };

typedef int (CC2420ControlP___nesc_sillytask_sync[CC2420ControlP_sync]);

enum CC2420ControlP___nesc_unnamed4309 { CC2420ControlP_syncDone = 4U };

typedef int (CC2420ControlP___nesc_sillytask_syncDone[CC2420ControlP_syncDone]);

typedef enum CC2420ControlP___nesc_unnamed4310 { CC2420ControlP_S_VREG_STOPPED, CC2420ControlP_S_VREG_STARTING, CC2420ControlP_S_VREG_STARTED, CC2420ControlP_S_XOSC_STARTING, CC2420ControlP_S_XOSC_STARTED } CC2420ControlP_cc2420_control_state_t;

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

static void CC2420ControlP_writeFsctrl( void ) ;

static void CC2420ControlP_writeMdmctrl0( void ) ;

static void CC2420ControlP_writeId( void ) ;

inline static error_t CC2420ControlP_Init_init( void ) ;

inline static error_t CC2420ControlP_Resource_request( void ) ;

inline static error_t CC2420ControlP_Resource_release( void ) ;

inline static error_t CC2420ControlP_CC2420Power_startVReg( void ) ;

inline static error_t CC2420ControlP_CC2420Power_stopVReg( void ) ;

inline static error_t CC2420ControlP_CC2420Power_startOscillator( void ) ;

inline static error_t CC2420ControlP_CC2420Power_rxOn( void ) ;

static uint16_t CC2420ControlP_CC2420Config_getShortAddr( void ) ;

inline static uint16_t CC2420ControlP_CC2420Config_getPanAddr( void ) ;

inline static error_t CC2420ControlP_CC2420Config_sync( void ) ;

inline static bool CC2420ControlP_CC2420Config_isAddressRecognitionEnabled( void ) ;

inline static bool CC2420ControlP_CC2420Config_isHwAutoAckDefault( void ) ;

inline static bool CC2420ControlP_CC2420Config_isAutoAckEnabled( void ) ;

inline static void CC2420ControlP_SyncResource_granted( void ) ;

inline static void CC2420ControlP_SpiResource_granted( void ) ;

inline static void CC2420ControlP_RssiResource_granted( void ) ;

inline static void CC2420ControlP_StartupTimer_fired( void ) ;

inline static void CC2420ControlP_InterruptCCA_fired( void ) ;

inline static void CC2420ControlP_sync_runTask( void ) ;

inline static void CC2420ControlP_syncDone_runTask( void ) ;

static void CC2420ControlP_writeFsctrl( void ) ;

static void CC2420ControlP_writeMdmctrl0( void ) ;

static void CC2420ControlP_writeId( void ) ;

inline static void CC2420ControlP_ReadRssi_default_readDone(error_t error, uint16_t data) ;

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

enum TransformCounterC_0___nesc_unnamed4311 { TransformCounterC_0_LOW_SHIFT_RIGHT = 0, TransformCounterC_0_HIGH_SHIFT_LEFT = (8 * sizeof( TransformCounterC_0_from_size_type )) - TransformCounterC_0_LOW_SHIFT_RIGHT, TransformCounterC_0_NUM_UPPER_BITS = ((8 * sizeof( TransformCounterC_0_to_size_type )) - (8 * sizeof( TransformCounterC_0_from_size_type ))) + 0, TransformCounterC_0_OVERFLOW_MASK = TransformCounterC_0_NUM_UPPER_BITS? (( TransformCounterC_0_upper_count_type )(2) << (TransformCounterC_0_NUM_UPPER_BITS - 1)) - 1 : 0 };

static TransformCounterC_0_to_size_type TransformCounterC_0_Counter_get( void ) ;

inline static void TransformCounterC_0_CounterFrom_overflow( void ) ;

static void TransformAlarmC_0_Alarm_fired( void ) ;

static void TransformAlarmC_0_AlarmFrom_startAt(TransformAlarmC_0_AlarmFrom_size_type t0, TransformAlarmC_0_AlarmFrom_size_type dt) ;

static void TransformAlarmC_0_AlarmFrom_stop( void ) ;

static TransformAlarmC_0_Counter_size_type TransformAlarmC_0_Counter_get( void ) ;

TransformAlarmC_0_to_size_type TransformAlarmC_0_m_t0;

TransformAlarmC_0_to_size_type TransformAlarmC_0_m_dt;

enum TransformAlarmC_0___nesc_unnamed4312 { TransformAlarmC_0_MAX_DELAY_LOG2 = ((8 * sizeof( TransformAlarmC_0_from_size_type )) - 1) - 0, TransformAlarmC_0_MAX_DELAY = ( TransformAlarmC_0_to_size_type )(1) << TransformAlarmC_0_MAX_DELAY_LOG2 };

inline static TransformAlarmC_0_to_size_type TransformAlarmC_0_Alarm_getNow( void ) ;

inline static void TransformAlarmC_0_Alarm_stop( void ) ;

static void TransformAlarmC_0_set_alarm( void ) ;

static void TransformAlarmC_0_Alarm_startAt(TransformAlarmC_0_to_size_type t0, TransformAlarmC_0_to_size_type dt) ;

inline static void TransformAlarmC_0_Alarm_start(TransformAlarmC_0_to_size_type dt) ;

inline static void TransformAlarmC_0_AlarmFrom_fired( void ) ;

inline static void TransformAlarmC_0_Counter_overflow( void ) ;

inline static uint8_t HplMsp430GeneralIOP_0_IO_getRaw( void ) ;

inline static bool HplMsp430GeneralIOP_0_IO_get( void ) ;

inline static uint8_t HplMsp430GeneralIOP_3_IO_getRaw( void ) ;

inline static bool HplMsp430GeneralIOP_3_IO_get( void ) ;

inline static uint8_t HplMsp430GeneralIOP_4_IO_getRaw( void ) ;

inline static bool HplMsp430GeneralIOP_4_IO_get( void ) ;

inline static void HplMsp430GeneralIOP_4_IO_makeInput( void ) ;

inline static void HplMsp430GeneralIOP_17_IO_selectModuleFunc( void ) ;

inline static void HplMsp430GeneralIOP_17_IO_selectIOFunc( void ) ;

inline static void HplMsp430GeneralIOP_18_IO_selectModuleFunc( void ) ;

inline static void HplMsp430GeneralIOP_18_IO_selectIOFunc( void ) ;

inline static void HplMsp430GeneralIOP_19_IO_selectModuleFunc( void ) ;

inline static void HplMsp430GeneralIOP_19_IO_selectIOFunc( void ) ;

inline static void HplMsp430GeneralIOP_20_IO_selectIOFunc( void ) ;

inline static void HplMsp430GeneralIOP_21_IO_selectIOFunc( void ) ;

inline static uint8_t HplMsp430GeneralIOP_25_IO_getRaw( void ) ;

inline static bool HplMsp430GeneralIOP_25_IO_get( void ) ;

inline static void HplMsp430GeneralIOP_25_IO_makeInput( void ) ;

inline static void HplMsp430GeneralIOP_25_IO_selectModuleFunc( void ) ;

inline static void HplMsp430GeneralIOP_25_IO_selectIOFunc( void ) ;

static void HplMsp430GeneralIOP_26_IO_set( void ) ;

static void HplMsp430GeneralIOP_26_IO_clr( void ) ;

inline static void HplMsp430GeneralIOP_26_IO_makeOutput( void ) ;

inline static void HplMsp430GeneralIOP_29_IO_set( void ) ;

inline static void HplMsp430GeneralIOP_29_IO_clr( void ) ;

inline static void HplMsp430GeneralIOP_29_IO_makeOutput( void ) ;

static void HplMsp430GeneralIOP_30_IO_set( void ) ;

static void HplMsp430GeneralIOP_30_IO_clr( void ) ;

inline static void HplMsp430GeneralIOP_30_IO_makeOutput( void ) ;

inline static void HplMsp430GeneralIOP_36_IO_set( void ) ;

static void HplMsp430GeneralIOP_36_IO_toggle( void ) ;

inline static void HplMsp430GeneralIOP_36_IO_makeOutput( void ) ;

inline static void HplMsp430GeneralIOP_37_IO_set( void ) ;

static void HplMsp430GeneralIOP_37_IO_toggle( void ) ;

inline static void HplMsp430GeneralIOP_37_IO_makeOutput( void ) ;

inline static void HplMsp430GeneralIOP_38_IO_set( void ) ;

inline static void HplMsp430GeneralIOP_38_IO_toggle( void ) ;

inline static void HplMsp430GeneralIOP_38_IO_makeOutput( void ) ;

static void Msp430GpioC_0_HplGeneralIO_makeInput( void ) ;

static bool Msp430GpioC_0_HplGeneralIO_get( void ) ;

inline static bool Msp430GpioC_0_GeneralIO_get( void ) ;

inline static void Msp430GpioC_0_GeneralIO_makeInput( void ) ;

static void Msp430GpioC_1_HplGeneralIO_makeOutput( void ) ;

static void Msp430GpioC_1_HplGeneralIO_set( void ) ;

static void Msp430GpioC_1_HplGeneralIO_clr( void ) ;

inline static void Msp430GpioC_1_GeneralIO_set( void ) ;

inline static void Msp430GpioC_1_GeneralIO_clr( void ) ;

inline static void Msp430GpioC_1_GeneralIO_makeOutput( void ) ;

static bool Msp430GpioC_2_HplGeneralIO_get( void ) ;

inline static bool Msp430GpioC_2_GeneralIO_get( void ) ;

static bool Msp430GpioC_3_HplGeneralIO_get( void ) ;

inline static bool Msp430GpioC_3_GeneralIO_get( void ) ;

static void Msp430GpioC_4_HplGeneralIO_makeOutput( void ) ;

static void Msp430GpioC_4_HplGeneralIO_set( void ) ;

static void Msp430GpioC_4_HplGeneralIO_clr( void ) ;

inline static void Msp430GpioC_4_GeneralIO_set( void ) ;

inline static void Msp430GpioC_4_GeneralIO_clr( void ) ;

inline static void Msp430GpioC_4_GeneralIO_makeOutput( void ) ;

static void Msp430GpioC_5_HplGeneralIO_makeInput( void ) ;

static bool Msp430GpioC_5_HplGeneralIO_get( void ) ;

inline static bool Msp430GpioC_5_GeneralIO_get( void ) ;

inline static void Msp430GpioC_5_GeneralIO_makeInput( void ) ;

static void Msp430GpioC_6_HplGeneralIO_makeOutput( void ) ;

static void Msp430GpioC_6_HplGeneralIO_set( void ) ;

static void Msp430GpioC_6_HplGeneralIO_clr( void ) ;

inline static void Msp430GpioC_6_GeneralIO_set( void ) ;

inline static void Msp430GpioC_6_GeneralIO_clr( void ) ;

inline static void Msp430GpioC_6_GeneralIO_makeOutput( void ) ;

static void GpioCaptureC_0_Msp430Capture_clearOverflow( void ) ;

static void GpioCaptureC_0_Capture_captured(uint16_t time) ;

static void GpioCaptureC_0_Msp430TimerControl_setControlAsCapture(uint8_t cm) ;

static void GpioCaptureC_0_Msp430TimerControl_enableEvents( void ) ;

static void GpioCaptureC_0_Msp430TimerControl_disableEvents( void ) ;

static void GpioCaptureC_0_Msp430TimerControl_clearPendingInterrupt( void ) ;

static void GpioCaptureC_0_GeneralIO_selectIOFunc( void ) ;

static void GpioCaptureC_0_GeneralIO_selectModuleFunc( void ) ;

static error_t GpioCaptureC_0_enableCapture(uint8_t mode) ;

inline static error_t GpioCaptureC_0_Capture_captureRisingEdge( void ) ;

inline static error_t GpioCaptureC_0_Capture_captureFallingEdge( void ) ;

inline static void GpioCaptureC_0_Capture_disable( void ) ;

inline static void GpioCaptureC_0_Msp430Capture_captured(uint16_t time) ;

static void HplMsp430InterruptP_Port14_fired( void ) ;

static void HplMsp430InterruptP_Port26_fired( void ) ;

static void HplMsp430InterruptP_Port17_fired( void ) ;

static void HplMsp430InterruptP_Port21_fired( void ) ;

static void HplMsp430InterruptP_Port12_fired( void ) ;

static void HplMsp430InterruptP_Port24_fired( void ) ;

static void HplMsp430InterruptP_Port15_fired( void ) ;

static void HplMsp430InterruptP_Port27_fired( void ) ;

static void HplMsp430InterruptP_Port10_fired( void ) ;

static void HplMsp430InterruptP_Port22_fired( void ) ;

static void HplMsp430InterruptP_Port13_fired( void ) ;

static void HplMsp430InterruptP_Port25_fired( void ) ;

static void HplMsp430InterruptP_Port16_fired( void ) ;

static void HplMsp430InterruptP_Port20_fired( void ) ;

static void HplMsp430InterruptP_Port11_fired( void ) ;

static void HplMsp430InterruptP_Port23_fired( void ) ;

void sig_PORT1_VECTOR( void ) ;

inline static void HplMsp430InterruptP_Port11_default_fired( void ) ;

inline static void HplMsp430InterruptP_Port12_default_fired( void ) ;

inline static void HplMsp430InterruptP_Port13_default_fired( void ) ;

inline static void HplMsp430InterruptP_Port15_default_fired( void ) ;

inline static void HplMsp430InterruptP_Port16_default_fired( void ) ;

inline static void HplMsp430InterruptP_Port17_default_fired( void ) ;

inline static void HplMsp430InterruptP_Port10_enable( void ) ;

inline static void HplMsp430InterruptP_Port14_enable( void ) ;

inline static void HplMsp430InterruptP_Port10_disable( void ) ;

inline static void HplMsp430InterruptP_Port14_disable( void ) ;

inline static void HplMsp430InterruptP_Port10_clear( void ) ;

inline static void HplMsp430InterruptP_Port11_clear( void ) ;

inline static void HplMsp430InterruptP_Port12_clear( void ) ;

inline static void HplMsp430InterruptP_Port13_clear( void ) ;

inline static void HplMsp430InterruptP_Port14_clear( void ) ;

inline static void HplMsp430InterruptP_Port15_clear( void ) ;

inline static void HplMsp430InterruptP_Port16_clear( void ) ;

inline static void HplMsp430InterruptP_Port17_clear( void ) ;

inline static void HplMsp430InterruptP_Port10_edge(bool l2h) ;

inline static void HplMsp430InterruptP_Port14_edge(bool l2h) ;

void sig_PORT2_VECTOR( void ) ;

inline static void HplMsp430InterruptP_Port20_default_fired( void ) ;

inline static void HplMsp430InterruptP_Port21_default_fired( void ) ;

inline static void HplMsp430InterruptP_Port22_default_fired( void ) ;

inline static void HplMsp430InterruptP_Port23_default_fired( void ) ;

inline static void HplMsp430InterruptP_Port24_default_fired( void ) ;

inline static void HplMsp430InterruptP_Port25_default_fired( void ) ;

inline static void HplMsp430InterruptP_Port26_default_fired( void ) ;

inline static void HplMsp430InterruptP_Port27_default_fired( void ) ;

inline static void HplMsp430InterruptP_Port20_clear( void ) ;

inline static void HplMsp430InterruptP_Port21_clear( void ) ;

inline static void HplMsp430InterruptP_Port22_clear( void ) ;

inline static void HplMsp430InterruptP_Port23_clear( void ) ;

inline static void HplMsp430InterruptP_Port24_clear( void ) ;

inline static void HplMsp430InterruptP_Port25_clear( void ) ;

inline static void HplMsp430InterruptP_Port26_clear( void ) ;

inline static void HplMsp430InterruptP_Port27_clear( void ) ;

static void Msp430InterruptC_0_HplInterrupt_clear( void ) ;

static void Msp430InterruptC_0_HplInterrupt_disable( void ) ;

static void Msp430InterruptC_0_HplInterrupt_edge(bool low_to_high) ;

static void Msp430InterruptC_0_HplInterrupt_enable( void ) ;

static void Msp430InterruptC_0_Interrupt_fired( void ) ;

inline static error_t Msp430InterruptC_0_enable(bool rising) ;

inline static error_t Msp430InterruptC_0_Interrupt_enableRisingEdge( void ) ;

inline static error_t Msp430InterruptC_0_Interrupt_disable( void ) ;

inline static void Msp430InterruptC_0_HplInterrupt_fired( void ) ;

static void Msp430InterruptC_1_HplInterrupt_clear( void ) ;

static void Msp430InterruptC_1_HplInterrupt_disable( void ) ;

static void Msp430InterruptC_1_HplInterrupt_edge(bool low_to_high) ;

static void Msp430InterruptC_1_HplInterrupt_enable( void ) ;

static void Msp430InterruptC_1_Interrupt_fired( void ) ;

inline static error_t Msp430InterruptC_1_enable(bool rising) ;

inline static error_t Msp430InterruptC_1_Interrupt_enableFallingEdge( void ) ;

inline static error_t Msp430InterruptC_1_Interrupt_disable( void ) ;

inline static void Msp430InterruptC_1_HplInterrupt_fired( void ) ;

static error_t CC2420SpiP_SpiPacket_send(uint8_t *txBuf, uint8_t *rxBuf, uint16_t len) ;

static void CC2420SpiP_Fifo_writeDone(uint8_t arg_0x1b79698, uint8_t *data, uint8_t length, error_t error) ;

static void CC2420SpiP_Fifo_readDone(uint8_t arg_0x1b79698, uint8_t *data, uint8_t length, error_t error) ;

static void CC2420SpiP_ChipSpiResource_releasing( void ) ;

static uint8_t CC2420SpiP_SpiByte_write(uint8_t tx) ;

static void CC2420SpiP_WorkingState_toIdle( void ) ;

static bool CC2420SpiP_WorkingState_isIdle( void ) ;

static error_t CC2420SpiP_WorkingState_requestState(uint8_t reqState) ;

static error_t CC2420SpiP_SpiResource_release( void ) ;

static error_t CC2420SpiP_SpiResource_immediateRequest( void ) ;

static error_t CC2420SpiP_SpiResource_request( void ) ;

static bool CC2420SpiP_SpiResource_isOwner( void ) ;

static void CC2420SpiP_Resource_granted(uint8_t arg_0x1b7abd0) ;

static error_t CC2420SpiP_grant_postTask( void ) ;

enum CC2420SpiP___nesc_unnamed4313 { CC2420SpiP_grant = 5U };

typedef int (CC2420SpiP___nesc_sillytask_grant[CC2420SpiP_grant]);

enum CC2420SpiP___nesc_unnamed4314 { CC2420SpiP_RESOURCE_COUNT = 5U, CC2420SpiP_NO_HOLDER = 0xFF };

enum CC2420SpiP___nesc_unnamed4315 { CC2420SpiP_S_IDLE, CC2420SpiP_S_BUSY };

uint16_t CC2420SpiP_m_addr;

uint8_t CC2420SpiP_m_requests = 0;

uint8_t CC2420SpiP_m_holder = CC2420SpiP_NO_HOLDER;

bool CC2420SpiP_release;

static error_t CC2420SpiP_attemptRelease( void ) ;

inline static void CC2420SpiP_ChipSpiResource_abortRelease( void ) ;

inline static error_t CC2420SpiP_ChipSpiResource_attemptRelease( void ) ;

static error_t CC2420SpiP_Resource_request(uint8_t id) ;

static error_t CC2420SpiP_Resource_immediateRequest(uint8_t id) ;

static error_t CC2420SpiP_Resource_release(uint8_t id) ;

inline static uint8_t CC2420SpiP_Resource_isOwner(uint8_t id) ;

inline static void CC2420SpiP_SpiResource_granted( void ) ;

static cc2420_status_t CC2420SpiP_Fifo_beginRead(uint8_t addr, uint8_t *data, uint8_t len) ;

inline static error_t CC2420SpiP_Fifo_continueRead(uint8_t addr, uint8_t *data, uint8_t len) ;

inline static cc2420_status_t CC2420SpiP_Fifo_write(uint8_t addr, uint8_t *data, uint8_t len) ;

static cc2420_status_t CC2420SpiP_Ram_write(uint16_t addr, uint8_t offset, uint8_t *data, uint8_t len) ;

inline static cc2420_status_t CC2420SpiP_Reg_read(uint8_t addr, uint16_t *data) ;

static cc2420_status_t CC2420SpiP_Reg_write(uint8_t addr, uint16_t data) ;

static cc2420_status_t CC2420SpiP_Strobe_strobe(uint8_t addr) ;

static void CC2420SpiP_SpiPacket_sendDone(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error) ;

static error_t CC2420SpiP_attemptRelease( void ) ;

inline static void CC2420SpiP_grant_runTask( void ) ;

inline static void CC2420SpiP_Resource_default_granted(uint8_t id) ;

inline static void CC2420SpiP_Fifo_default_readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error) ;

inline static void CC2420SpiP_Fifo_default_writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error) ;

uint8_t (StateImplP_state[4U]);

enum StateImplP___nesc_unnamed4316 { StateImplP_S_IDLE = 0 };

inline static error_t StateImplP_Init_init( void ) ;

static error_t StateImplP_State_requestState(uint8_t id, uint8_t reqState) ;

inline static void StateImplP_State_forceState(uint8_t id, uint8_t reqState) ;

inline static void StateImplP_State_toIdle(uint8_t id) ;

inline static bool StateImplP_State_isIdle(uint8_t id) ;

static bool StateImplP_State_isState(uint8_t id, uint8_t myState) ;

static void Msp430SpiNoDmaP_0_SpiPacket_sendDone(uint8_t arg_0x1c67740, uint8_t *txBuf, uint8_t *rxBuf, uint16_t len, error_t error) ;

static msp430_spi_union_config_t *Msp430SpiNoDmaP_0_Msp430SpiConfigure_getConfig(uint8_t arg_0x1c66a60) ;

static void Msp430SpiNoDmaP_0_Usart_enableRxIntr( void ) ;

static void Msp430SpiNoDmaP_0_Usart_clrRxIntr( void ) ;

static void Msp430SpiNoDmaP_0_Usart_resetUsart(bool reset) ;

static void Msp430SpiNoDmaP_0_Usart_disableRxIntr( void ) ;

static void Msp430SpiNoDmaP_0_Usart_tx(uint8_t data) ;

static void Msp430SpiNoDmaP_0_Usart_setModeSpi(msp430_spi_union_config_t *config) ;

static uint8_t Msp430SpiNoDmaP_0_Usart_rx( void ) ;

static bool Msp430SpiNoDmaP_0_Usart_isRxIntrPending( void ) ;

static void Msp430SpiNoDmaP_0_Usart_disableSpi( void ) ;

static error_t Msp430SpiNoDmaP_0_UsartResource_release(uint8_t arg_0x1c66010) ;

static error_t Msp430SpiNoDmaP_0_UsartResource_immediateRequest(uint8_t arg_0x1c66010) ;

static error_t Msp430SpiNoDmaP_0_UsartResource_request(uint8_t arg_0x1c66010) ;

static bool Msp430SpiNoDmaP_0_UsartResource_isOwner(uint8_t arg_0x1c66010) ;

static void Msp430SpiNoDmaP_0_Resource_granted(uint8_t arg_0x1c691b8) ;

static error_t Msp430SpiNoDmaP_0_signalDone_task_postTask( void ) ;

enum Msp430SpiNoDmaP_0___nesc_unnamed4317 { Msp430SpiNoDmaP_0_signalDone_task = 6U };

typedef int (Msp430SpiNoDmaP_0___nesc_sillytask_signalDone_task[Msp430SpiNoDmaP_0_signalDone_task]);

enum Msp430SpiNoDmaP_0___nesc_unnamed4318 { Msp430SpiNoDmaP_0_SPI_ATOMIC_SIZE = 2 };

uint16_t Msp430SpiNoDmaP_0_m_len;

uint8_t *Msp430SpiNoDmaP_0_m_tx_buf;

uint8_t *Msp430SpiNoDmaP_0_m_rx_buf;

uint16_t Msp430SpiNoDmaP_0_m_pos;

uint8_t Msp430SpiNoDmaP_0_m_client;

inline static void Msp430SpiNoDmaP_0_signalDone( void ) ;

inline static error_t Msp430SpiNoDmaP_0_Resource_immediateRequest(uint8_t id) ;

inline static error_t Msp430SpiNoDmaP_0_Resource_request(uint8_t id) ;

inline static uint8_t Msp430SpiNoDmaP_0_Resource_isOwner(uint8_t id) ;

inline static error_t Msp430SpiNoDmaP_0_Resource_release(uint8_t id) ;

inline static void Msp430SpiNoDmaP_0_ResourceConfigure_configure(uint8_t id) ;

inline static void Msp430SpiNoDmaP_0_ResourceConfigure_unconfigure(uint8_t id) ;

inline static void Msp430SpiNoDmaP_0_UsartResource_granted(uint8_t id) ;

static uint8_t Msp430SpiNoDmaP_0_SpiByte_write(uint8_t tx) ;

inline static error_t Msp430SpiNoDmaP_0_UsartResource_default_isOwner(uint8_t id) ;

inline static error_t Msp430SpiNoDmaP_0_UsartResource_default_request(uint8_t id) ;

inline static error_t Msp430SpiNoDmaP_0_UsartResource_default_immediateRequest(uint8_t id) ;

inline static error_t Msp430SpiNoDmaP_0_UsartResource_default_release(uint8_t id) ;

inline static msp430_spi_union_config_t *Msp430SpiNoDmaP_0_Msp430SpiConfigure_default_getConfig(uint8_t id) ;

inline static void Msp430SpiNoDmaP_0_Resource_default_granted(uint8_t id) ;

static void Msp430SpiNoDmaP_0_continueOp( void ) ;

static error_t Msp430SpiNoDmaP_0_SpiPacket_send(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) ;

inline static void Msp430SpiNoDmaP_0_signalDone_task_runTask( void ) ;

inline static void Msp430SpiNoDmaP_0_UsartInterrupts_rxDone(uint8_t data) ;

inline static void Msp430SpiNoDmaP_0_signalDone( void ) ;

inline static void Msp430SpiNoDmaP_0_UsartInterrupts_txDone( void ) ;

inline static void Msp430SpiNoDmaP_0_SpiPacket_default_sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error) ;

static void HplMsp430Usart0P_UCLK_selectIOFunc( void ) ;

static void HplMsp430Usart0P_UCLK_selectModuleFunc( void ) ;

static void HplMsp430Usart0P_Interrupts_rxDone(uint8_t data) ;

static void HplMsp430Usart0P_Interrupts_txDone( void ) ;

static void HplMsp430Usart0P_URXD_selectIOFunc( void ) ;

static void HplMsp430Usart0P_UTXD_selectIOFunc( void ) ;

static void HplMsp430Usart0P_HplI2C_clearModeI2C( void ) ;

static bool HplMsp430Usart0P_HplI2C_isI2C( void ) ;

static void HplMsp430Usart0P_SOMI_selectIOFunc( void ) ;

static void HplMsp430Usart0P_SOMI_selectModuleFunc( void ) ;

static void HplMsp430Usart0P_I2CInterrupts_fired( void ) ;

static void HplMsp430Usart0P_SIMO_selectIOFunc( void ) ;

static void HplMsp430Usart0P_SIMO_selectModuleFunc( void ) ;

void sig_UART0RX_VECTOR( void ) ;

void sig_UART0TX_VECTOR( void ) ;

inline static void HplMsp430Usart0P_Usart_setUbr(uint16_t control) ;

inline static void HplMsp430Usart0P_Usart_setUmctl(uint8_t control) ;

inline static void HplMsp430Usart0P_Usart_resetUsart(bool reset) ;

inline static void HplMsp430Usart0P_Usart_disableUart( void ) ;

inline static void HplMsp430Usart0P_Usart_enableSpi( void ) ;

static void HplMsp430Usart0P_Usart_disableSpi( void ) ;

inline static void HplMsp430Usart0P_configSpi(msp430_spi_union_config_t *config) ;

static void HplMsp430Usart0P_Usart_setModeSpi(msp430_spi_union_config_t *config) ;

inline static bool HplMsp430Usart0P_Usart_isRxIntrPending( void ) ;

inline static void HplMsp430Usart0P_Usart_clrRxIntr( void ) ;

inline static void HplMsp430Usart0P_Usart_clrIntr( void ) ;

inline static void HplMsp430Usart0P_Usart_disableRxIntr( void ) ;

inline static void HplMsp430Usart0P_Usart_disableIntr( void ) ;

inline static void HplMsp430Usart0P_Usart_enableRxIntr( void ) ;

inline static void HplMsp430Usart0P_Usart_tx(uint8_t data) ;

static uint8_t HplMsp430Usart0P_Usart_rx( void ) ;

static void LedsP_Led0_toggle( void ) ;

static void LedsP_Led0_makeOutput( void ) ;

static void LedsP_Led0_set( void ) ;

static void LedsP_Led1_toggle( void ) ;

static void LedsP_Led1_makeOutput( void ) ;

static void LedsP_Led1_set( void ) ;

static void LedsP_Led2_toggle( void ) ;

static void LedsP_Led2_makeOutput( void ) ;

static void LedsP_Led2_set( void ) ;

inline static error_t LedsP_Init_init( void ) ;

inline static void LedsP_Leds_led0Toggle( void ) ;

inline static void LedsP_Leds_led1Toggle( void ) ;

inline static void LedsP_Leds_led2Toggle( void ) ;

static void Msp430GpioC_7_HplGeneralIO_toggle( void ) ;

static void Msp430GpioC_7_HplGeneralIO_makeOutput( void ) ;

static void Msp430GpioC_7_HplGeneralIO_set( void ) ;

inline static void Msp430GpioC_7_GeneralIO_set( void ) ;

inline static void Msp430GpioC_7_GeneralIO_toggle( void ) ;

inline static void Msp430GpioC_7_GeneralIO_makeOutput( void ) ;

static void Msp430GpioC_8_HplGeneralIO_toggle( void ) ;

static void Msp430GpioC_8_HplGeneralIO_makeOutput( void ) ;

static void Msp430GpioC_8_HplGeneralIO_set( void ) ;

inline static void Msp430GpioC_8_GeneralIO_set( void ) ;

inline static void Msp430GpioC_8_GeneralIO_toggle( void ) ;

inline static void Msp430GpioC_8_GeneralIO_makeOutput( void ) ;

static void Msp430GpioC_9_HplGeneralIO_toggle( void ) ;

static void Msp430GpioC_9_HplGeneralIO_makeOutput( void ) ;

static void Msp430GpioC_9_HplGeneralIO_set( void ) ;

inline static void Msp430GpioC_9_GeneralIO_set( void ) ;

inline static void Msp430GpioC_9_GeneralIO_toggle( void ) ;

inline static void Msp430GpioC_9_GeneralIO_makeOutput( void ) ;

static bool Msp430UsartShareP_0_ArbiterInfo_inUse( void ) ;

static uint8_t Msp430UsartShareP_0_ArbiterInfo_userId( void ) ;

static void Msp430UsartShareP_0_Interrupts_rxDone(uint8_t arg_0x1dac1d8, uint8_t data) ;

static void Msp430UsartShareP_0_Interrupts_txDone(uint8_t arg_0x1dac1d8) ;

static void Msp430UsartShareP_0_I2CInterrupts_fired(uint8_t arg_0x1daca58) ;

inline static void Msp430UsartShareP_0_RawInterrupts_txDone( void ) ;

inline static void Msp430UsartShareP_0_RawInterrupts_rxDone(uint8_t data) ;

inline static void Msp430UsartShareP_0_RawI2CInterrupts_fired( void ) ;

inline static void Msp430UsartShareP_0_Interrupts_default_txDone(uint8_t id) ;

inline static void Msp430UsartShareP_0_Interrupts_default_rxDone(uint8_t id, uint8_t data) ;

inline static void Msp430UsartShareP_0_I2CInterrupts_default_fired(uint8_t id) ;

enum FcfsResourceQueueC_0___nesc_unnamed4319 { FcfsResourceQueueC_0_NO_ENTRY = 0xFF };

uint8_t (FcfsResourceQueueC_0_resQ[1U]);

uint8_t FcfsResourceQueueC_0_qHead = FcfsResourceQueueC_0_NO_ENTRY;

uint8_t FcfsResourceQueueC_0_qTail = FcfsResourceQueueC_0_NO_ENTRY;

inline static error_t FcfsResourceQueueC_0_Init_init( void ) ;

inline static bool FcfsResourceQueueC_0_FcfsQueue_isEmpty( void ) ;

inline static bool FcfsResourceQueueC_0_FcfsQueue_isEnqueued(resource_client_id_t id) ;

inline static resource_client_id_t FcfsResourceQueueC_0_FcfsQueue_dequeue( void ) ;

inline static error_t FcfsResourceQueueC_0_FcfsQueue_enqueue(resource_client_id_t id) ;

static void ArbiterP_0_ResourceRequested_requested(uint8_t arg_0x1db7c58) ;

static void ArbiterP_0_ResourceRequested_immediateRequested(uint8_t arg_0x1db7c58) ;

static void ArbiterP_0_ResourceConfigure_unconfigure(uint8_t arg_0x1db4030) ;

static void ArbiterP_0_ResourceConfigure_configure(uint8_t arg_0x1db4030) ;

static error_t ArbiterP_0_Queue_enqueue(resource_client_id_t id) ;

static bool ArbiterP_0_Queue_isEmpty( void ) ;

static resource_client_id_t ArbiterP_0_Queue_dequeue( void ) ;

static void ArbiterP_0_ResourceDefaultOwner_requested( void ) ;

static void ArbiterP_0_ResourceDefaultOwner_granted( void ) ;

static void ArbiterP_0_ResourceDefaultOwner_immediateRequested( void ) ;

static void ArbiterP_0_Resource_granted(uint8_t arg_0x1db7230) ;

static error_t ArbiterP_0_grantedTask_postTask( void ) ;

enum ArbiterP_0___nesc_unnamed4320 { ArbiterP_0_grantedTask = 7U };

typedef int (ArbiterP_0___nesc_sillytask_grantedTask[ArbiterP_0_grantedTask]);

enum ArbiterP_0___nesc_unnamed4321 { ArbiterP_0_RES_CONTROLLED, ArbiterP_0_RES_GRANTING, ArbiterP_0_RES_IMM_GRANTING, ArbiterP_0_RES_BUSY };

enum ArbiterP_0___nesc_unnamed4322 { ArbiterP_0_default_owner_id = 1U };

enum ArbiterP_0___nesc_unnamed4323 { ArbiterP_0_NO_RES = 0xFF };

uint8_t ArbiterP_0_state = ArbiterP_0_RES_CONTROLLED;

uint8_t ArbiterP_0_resId = ArbiterP_0_default_owner_id;

uint8_t ArbiterP_0_reqResId;

inline static error_t ArbiterP_0_Resource_request(uint8_t id) ;

inline static error_t ArbiterP_0_Resource_immediateRequest(uint8_t id) ;

inline static error_t ArbiterP_0_Resource_release(uint8_t id) ;

static error_t ArbiterP_0_ResourceDefaultOwner_release( void ) ;

static bool ArbiterP_0_ArbiterInfo_inUse( void ) ;

static uint8_t ArbiterP_0_ArbiterInfo_userId( void ) ;

static uint8_t ArbiterP_0_Resource_isOwner(uint8_t id) ;

inline static void ArbiterP_0_grantedTask_runTask( void ) ;

inline static void ArbiterP_0_Resource_default_granted(uint8_t id) ;

inline static void ArbiterP_0_ResourceRequested_default_requested(uint8_t id) ;

inline static void ArbiterP_0_ResourceRequested_default_immediateRequested(uint8_t id) ;

inline static void ArbiterP_0_ResourceDefaultOwner_default_granted( void ) ;

inline static void ArbiterP_0_ResourceDefaultOwner_default_requested( void ) ;

inline static void ArbiterP_0_ResourceDefaultOwner_default_immediateRequested( void ) ;

inline static void ArbiterP_0_ResourceConfigure_default_configure(uint8_t id) ;

inline static void ArbiterP_0_ResourceConfigure_default_unconfigure(uint8_t id) ;

static void HplMsp430I2C0P_HplUsart_resetUsart(bool reset) ;

inline static bool HplMsp430I2C0P_HplI2C_isI2C( void ) ;

inline static void HplMsp430I2C0P_HplI2C_clearModeI2C( void ) ;

am_addr_t ActiveMessageAddressC_addr = TOS_AM_ADDRESS;

am_group_t ActiveMessageAddressC_group = TOS_AM_GROUP;

inline static am_addr_t ActiveMessageAddressC_ActiveMessageAddress_amAddress( void ) ;

inline static am_group_t ActiveMessageAddressC_ActiveMessageAddress_amGroup( void ) ;

static am_addr_t ActiveMessageAddressC_amAddress( void ) ;

static void CC2420TransmitP_RadioBackoff_requestInitialBackoff(message_t *msg) ;

static void CC2420TransmitP_RadioBackoff_requestCongestionBackoff(message_t *msg) ;

static void CC2420TransmitP_PacketTimeStamp_clear(message_t *msg) ;

static void CC2420TransmitP_PacketTimeStamp_set(message_t *msg, CC2420TransmitP_PacketTimeStamp_size_type value) ;

static cc2420_status_t CC2420TransmitP_STXONCCA_strobe( void ) ;

static error_t CC2420TransmitP_CaptureSFD_captureFallingEdge( void ) ;

static void CC2420TransmitP_CaptureSFD_disable( void ) ;

static error_t CC2420TransmitP_CaptureSFD_captureRisingEdge( void ) ;

static CC2420TransmitP_BackoffTimer_size_type CC2420TransmitP_BackoffTimer_getNow( void ) ;

static void CC2420TransmitP_BackoffTimer_start(CC2420TransmitP_BackoffTimer_size_type dt) ;

static void CC2420TransmitP_BackoffTimer_stop( void ) ;

static cc2420_status_t CC2420TransmitP_TXFIFO_RAM_write(uint8_t offset, uint8_t *data, uint8_t length) ;

static cc2420_status_t CC2420TransmitP_TXCTRL_write(uint16_t data) ;

static void CC2420TransmitP_CC2420Receive_sfd_dropped( void ) ;

static void CC2420TransmitP_CC2420Receive_sfd(uint32_t time) ;

static void CC2420TransmitP_Send_sendDone(message_t *p_msg, error_t error) ;

static void CC2420TransmitP_ChipSpiResource_abortRelease( void ) ;

static error_t CC2420TransmitP_ChipSpiResource_attemptRelease( void ) ;

static cc2420_status_t CC2420TransmitP_SFLUSHTX_strobe( void ) ;

static void CC2420TransmitP_CSN_makeOutput( void ) ;

static void CC2420TransmitP_CSN_set( void ) ;

static void CC2420TransmitP_CSN_clr( void ) ;

static cc2420_header_t *CC2420TransmitP_CC2420PacketBody_getHeader(message_t *msg) ;

static cc2420_metadata_t *CC2420TransmitP_CC2420PacketBody_getMetadata(message_t *msg) ;

static uint8_t CC2420TransmitP_PacketTimeSyncOffset_get(message_t *msg) ;

static bool CC2420TransmitP_PacketTimeSyncOffset_isSet(message_t *msg) ;

static error_t CC2420TransmitP_SpiResource_release( void ) ;

static error_t CC2420TransmitP_SpiResource_immediateRequest( void ) ;

static error_t CC2420TransmitP_SpiResource_request( void ) ;

static void CC2420TransmitP_CCA_makeInput( void ) ;

static bool CC2420TransmitP_CCA_get( void ) ;

static cc2420_status_t CC2420TransmitP_SNOP_strobe( void ) ;

static void CC2420TransmitP_SFD_makeInput( void ) ;

static bool CC2420TransmitP_SFD_get( void ) ;

static cc2420_status_t CC2420TransmitP_TXFIFO_write(uint8_t *data, uint8_t length) ;

static cc2420_status_t CC2420TransmitP_STXON_strobe( void ) ;

typedef enum CC2420TransmitP___nesc_unnamed4324 { CC2420TransmitP_S_STOPPED, CC2420TransmitP_S_STARTED, CC2420TransmitP_S_LOAD, CC2420TransmitP_S_SAMPLE_CCA, CC2420TransmitP_S_BEGIN_TRANSMIT, CC2420TransmitP_S_SFD, CC2420TransmitP_S_EFD, CC2420TransmitP_S_ACK_WAIT, CC2420TransmitP_S_CANCEL } CC2420TransmitP_cc2420_transmit_state_t;

enum CC2420TransmitP___nesc_unnamed4325 { CC2420TransmitP_CC2420_ABORT_PERIOD = 320 };

message_t *CC2420TransmitP_m_msg;

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

inline static error_t CC2420TransmitP_send(message_t *p_msg, bool cca) ;

static void CC2420TransmitP_loadTXFIFO( void ) ;

static void CC2420TransmitP_attemptSend( void ) ;

static void CC2420TransmitP_congestionBackoff( void ) ;

static error_t CC2420TransmitP_acquireSpiResource( void ) ;

inline static error_t CC2420TransmitP_releaseSpiResource( void ) ;

static void CC2420TransmitP_signalDone(error_t err) ;

inline static error_t CC2420TransmitP_Init_init( void ) ;

inline static error_t CC2420TransmitP_StdControl_start( void ) ;

inline static error_t CC2420TransmitP_StdControl_stop( void ) ;

inline static error_t CC2420TransmitP_Send_send(message_t *p_msg, bool useCca) ;

inline static void CC2420TransmitP_RadioBackoff_setInitialBackoff(uint16_t backoffTime) ;

inline static void CC2420TransmitP_RadioBackoff_setCongestionBackoff(uint16_t backoffTime) ;

__inline static uint32_t CC2420TransmitP_getTime32(uint16_t time) ;

inline static void CC2420TransmitP_CaptureSFD_captured(uint16_t time) ;

inline static void CC2420TransmitP_ChipSpiResource_releasing( void ) ;

inline static void CC2420TransmitP_CC2420Receive_receive(uint8_t type, message_t *ack_msg) ;

inline static void CC2420TransmitP_SpiResource_granted( void ) ;

inline static void CC2420TransmitP_TXFIFO_writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error) ;

inline static void CC2420TransmitP_TXFIFO_readDone(uint8_t *tx_buf, uint8_t tx_len, error_t error) ;

inline static void CC2420TransmitP_BackoffTimer_fired( void ) ;

inline static error_t CC2420TransmitP_send(message_t *p_msg, bool cca) ;

static void CC2420TransmitP_attemptSend( void ) ;

static void CC2420TransmitP_congestionBackoff( void ) ;

static error_t CC2420TransmitP_acquireSpiResource( void ) ;

inline static error_t CC2420TransmitP_releaseSpiResource( void ) ;

static void CC2420TransmitP_loadTXFIFO( void ) ;

static void CC2420TransmitP_signalDone(error_t err) ;

static bool CC2420ReceiveP_FIFO_get( void ) ;

static bool CC2420ReceiveP_CC2420Config_isAddressRecognitionEnabled( void ) ;

static bool CC2420ReceiveP_CC2420Config_isAutoAckEnabled( void ) ;

static bool CC2420ReceiveP_CC2420Config_isHwAutoAckDefault( void ) ;

static uint16_t CC2420ReceiveP_CC2420Config_getShortAddr( void ) ;

static error_t CC2420ReceiveP_receiveDone_task_postTask( void ) ;

static bool CC2420ReceiveP_FIFOP_get( void ) ;

static void CC2420ReceiveP_PacketTimeStamp_clear(message_t *msg) ;

static void CC2420ReceiveP_PacketTimeStamp_set(message_t *msg, CC2420ReceiveP_PacketTimeStamp_size_type value) ;

static void CC2420ReceiveP_CC2420Receive_receive(uint8_t type, message_t *message) ;

static cc2420_status_t CC2420ReceiveP_SACK_strobe( void ) ;

static void CC2420ReceiveP_CSN_set( void ) ;

static void CC2420ReceiveP_CSN_clr( void ) ;

static cc2420_header_t *CC2420ReceiveP_CC2420PacketBody_getHeader(message_t *msg) ;

static cc2420_metadata_t *CC2420ReceiveP_CC2420PacketBody_getMetadata(message_t *msg) ;

static message_t *CC2420ReceiveP_Receive_receive(message_t *msg, void *payload, uint8_t len) ;

static error_t CC2420ReceiveP_SpiResource_release( void ) ;

static error_t CC2420ReceiveP_SpiResource_immediateRequest( void ) ;

static error_t CC2420ReceiveP_SpiResource_request( void ) ;

static bool CC2420ReceiveP_SpiResource_isOwner( void ) ;

static error_t CC2420ReceiveP_RXFIFO_continueRead(uint8_t *data, uint8_t length) ;

static cc2420_status_t CC2420ReceiveP_RXFIFO_beginRead(uint8_t *data, uint8_t length) ;

static error_t CC2420ReceiveP_InterruptFIFOP_disable( void ) ;

static error_t CC2420ReceiveP_InterruptFIFOP_enableFallingEdge( void ) ;

static cc2420_status_t CC2420ReceiveP_SFLUSHRX_strobe( void ) ;

enum CC2420ReceiveP___nesc_unnamed4326 { CC2420ReceiveP_receiveDone_task = 8U };

typedef int (CC2420ReceiveP___nesc_sillytask_receiveDone_task[CC2420ReceiveP_receiveDone_task]);

typedef enum CC2420ReceiveP___nesc_unnamed4327 { CC2420ReceiveP_S_STOPPED, CC2420ReceiveP_S_STARTED, CC2420ReceiveP_S_RX_LENGTH, CC2420ReceiveP_S_RX_FCF, CC2420ReceiveP_S_RX_PAYLOAD } CC2420ReceiveP_cc2420_receive_state_t;

enum CC2420ReceiveP___nesc_unnamed4328 { CC2420ReceiveP_RXFIFO_SIZE = 128, CC2420ReceiveP_TIMESTAMP_QUEUE_SIZE = 8, CC2420ReceiveP_SACK_HEADER_LENGTH = 7 };

uint32_t (CC2420ReceiveP_m_timestamp_queue[CC2420ReceiveP_TIMESTAMP_QUEUE_SIZE]);

uint8_t CC2420ReceiveP_m_timestamp_head;

uint8_t CC2420ReceiveP_m_timestamp_size;

uint8_t CC2420ReceiveP_m_missed_packets;

bool CC2420ReceiveP_receivingPacket;

uint8_t CC2420ReceiveP_rxFrameLength;

uint8_t CC2420ReceiveP_m_bytes_left;

message_t *CC2420ReceiveP_m_p_rx_buf;

message_t CC2420ReceiveP_m_rx_buf;

CC2420ReceiveP_cc2420_receive_state_t CC2420ReceiveP_m_state;

static void CC2420ReceiveP_reset_state( void ) ;

static void CC2420ReceiveP_beginReceive( void ) ;

static void CC2420ReceiveP_receive( void ) ;

static void CC2420ReceiveP_waitForNextPacket( void ) ;

static void CC2420ReceiveP_flush( void ) ;

inline static bool CC2420ReceiveP_passesAddressCheck(message_t *msg) ;

inline static error_t CC2420ReceiveP_Init_init( void ) ;

inline static error_t CC2420ReceiveP_StdControl_start( void ) ;

inline static error_t CC2420ReceiveP_StdControl_stop( void ) ;

inline static void CC2420ReceiveP_CC2420Receive_sfd(uint32_t time) ;

inline static void CC2420ReceiveP_CC2420Receive_sfd_dropped( void ) ;

inline static void CC2420ReceiveP_InterruptFIFOP_fired( void ) ;

inline static void CC2420ReceiveP_SpiResource_granted( void ) ;

inline static void CC2420ReceiveP_RXFIFO_readDone(uint8_t *rx_buf, uint8_t rx_len, error_t error) ;

inline static void CC2420ReceiveP_RXFIFO_writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error) ;

inline static void CC2420ReceiveP_receiveDone_task_runTask( void ) ;

inline static void CC2420ReceiveP_CC2420Config_syncDone(error_t error) ;

static void CC2420ReceiveP_beginReceive( void ) ;

static void CC2420ReceiveP_flush( void ) ;

static void CC2420ReceiveP_receive( void ) ;

static void CC2420ReceiveP_waitForNextPacket( void ) ;

static void CC2420ReceiveP_reset_state( void ) ;

inline static bool CC2420ReceiveP_passesAddressCheck(message_t *msg) ;

inline static cc2420_header_t *CC2420PacketP_CC2420PacketBody_getHeader(message_t *msg) ;

inline static cc2420_metadata_t *CC2420PacketP_CC2420PacketBody_getMetadata(message_t *msg) ;

static void CC2420PacketP_PacketTimeStamp32khz_clear(message_t *msg) ;

inline static void CC2420PacketP_PacketTimeStamp32khz_set(message_t *msg, uint32_t value) ;

inline static bool CC2420PacketP_PacketTimeSyncOffset_isSet(message_t *msg) ;

inline static uint8_t CC2420PacketP_PacketTimeSyncOffset_get(message_t *msg) ;

inline static void CounterToLocalTimeC_0_Counter_overflow( void ) ;

static void Msp430AlarmC_1_Msp430Compare_setEvent(uint16_t time) ;

static void Msp430AlarmC_1_Msp430Compare_setEventFromNow(uint16_t delta) ;

static uint16_t Msp430AlarmC_1_Msp430Timer_get( void ) ;

static void Msp430AlarmC_1_Alarm_fired( void ) ;

static void Msp430AlarmC_1_Msp430TimerControl_enableEvents( void ) ;

static void Msp430AlarmC_1_Msp430TimerControl_setControlAsCompare( void ) ;

static void Msp430AlarmC_1_Msp430TimerControl_disableEvents( void ) ;

static void Msp430AlarmC_1_Msp430TimerControl_clearPendingInterrupt( void ) ;

inline static error_t Msp430AlarmC_1_Init_init( void ) ;

inline static void Msp430AlarmC_1_Alarm_stop( void ) ;

inline static void Msp430AlarmC_1_Msp430Compare_fired( void ) ;

inline static void Msp430AlarmC_1_Alarm_startAt(uint16_t t0, uint16_t dt) ;

inline static void Msp430AlarmC_1_Msp430Timer_overflow( void ) ;

static TransformCounterC_1_CounterFrom_size_type TransformCounterC_1_CounterFrom_get( void ) ;

static bool TransformCounterC_1_CounterFrom_isOverflowPending( void ) ;

static void TransformCounterC_1_Counter_overflow( void ) ;

TransformCounterC_1_upper_count_type TransformCounterC_1_m_upper;

enum TransformCounterC_1___nesc_unnamed4329 { TransformCounterC_1_LOW_SHIFT_RIGHT = 5, TransformCounterC_1_HIGH_SHIFT_LEFT = (8 * sizeof( TransformCounterC_1_from_size_type )) - TransformCounterC_1_LOW_SHIFT_RIGHT, TransformCounterC_1_NUM_UPPER_BITS = ((8 * sizeof( TransformCounterC_1_to_size_type )) - (8 * sizeof( TransformCounterC_1_from_size_type ))) + 5, TransformCounterC_1_OVERFLOW_MASK = TransformCounterC_1_NUM_UPPER_BITS? (( TransformCounterC_1_upper_count_type )(2) << (TransformCounterC_1_NUM_UPPER_BITS - 1)) - 1 : 0 };

static TransformCounterC_1_to_size_type TransformCounterC_1_Counter_get( void ) ;

inline static void TransformCounterC_1_CounterFrom_overflow( void ) ;

static void TransformAlarmC_1_Alarm_fired( void ) ;

static void TransformAlarmC_1_AlarmFrom_startAt(TransformAlarmC_1_AlarmFrom_size_type t0, TransformAlarmC_1_AlarmFrom_size_type dt) ;

static void TransformAlarmC_1_AlarmFrom_stop( void ) ;

static TransformAlarmC_1_Counter_size_type TransformAlarmC_1_Counter_get( void ) ;

TransformAlarmC_1_to_size_type TransformAlarmC_1_m_t0;

TransformAlarmC_1_to_size_type TransformAlarmC_1_m_dt;

enum TransformAlarmC_1___nesc_unnamed4330 { TransformAlarmC_1_MAX_DELAY_LOG2 = ((8 * sizeof( TransformAlarmC_1_from_size_type )) - 1) - 5, TransformAlarmC_1_MAX_DELAY = ( TransformAlarmC_1_to_size_type )(1) << TransformAlarmC_1_MAX_DELAY_LOG2 };

inline static TransformAlarmC_1_to_size_type TransformAlarmC_1_Alarm_getNow( void ) ;

inline static TransformAlarmC_1_to_size_type TransformAlarmC_1_Alarm_getAlarm( void ) ;

inline static void TransformAlarmC_1_Alarm_stop( void ) ;

static void TransformAlarmC_1_set_alarm( void ) ;

static void TransformAlarmC_1_Alarm_startAt(TransformAlarmC_1_to_size_type t0, TransformAlarmC_1_to_size_type dt) ;

inline static void TransformAlarmC_1_AlarmFrom_fired( void ) ;

inline static void TransformAlarmC_1_Counter_overflow( void ) ;

static error_t AlarmToTimerC_0_fired_postTask( void ) ;

static AlarmToTimerC_0_Alarm_size_type AlarmToTimerC_0_Alarm_getNow( void ) ;

static void AlarmToTimerC_0_Alarm_startAt(AlarmToTimerC_0_Alarm_size_type t0, AlarmToTimerC_0_Alarm_size_type dt) ;

static AlarmToTimerC_0_Alarm_size_type AlarmToTimerC_0_Alarm_getAlarm( void ) ;

static void AlarmToTimerC_0_Alarm_stop( void ) ;

static void AlarmToTimerC_0_Timer_fired( void ) ;

enum AlarmToTimerC_0___nesc_unnamed4331 { AlarmToTimerC_0_fired = 9U };

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

static void VirtualizeTimerC_0_Timer_fired(uint8_t arg_0x20a3dd8) ;

enum VirtualizeTimerC_0___nesc_unnamed4332 { VirtualizeTimerC_0_updateFromTimer = 10U };

typedef int (VirtualizeTimerC_0___nesc_sillytask_updateFromTimer[VirtualizeTimerC_0_updateFromTimer]);

enum VirtualizeTimerC_0___nesc_unnamed4333 { VirtualizeTimerC_0_NUM_TIMERS = 3U, VirtualizeTimerC_0_END_OF_LIST = 255 };

typedef struct VirtualizeTimerC_0___nesc_unnamed4334 {
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

static void VirtualizeTimerC_0_Timer_startPeriodic(uint8_t num, uint32_t dt) ;

inline static void VirtualizeTimerC_0_Timer_startOneShot(uint8_t num, uint32_t dt) ;

inline static void VirtualizeTimerC_0_Timer_stop(uint8_t num) ;

inline static uint32_t VirtualizeTimerC_0_Timer_getNow(uint8_t num) ;

inline static uint32_t VirtualizeTimerC_0_Timer_gett0(uint8_t num) ;

inline static uint32_t VirtualizeTimerC_0_Timer_getdt(uint8_t num) ;

inline static void VirtualizeTimerC_0_Timer_default_fired(uint8_t num) ;

inline static void CounterToLocalTimeC_1_Counter_overflow( void ) ;

uint32_t RandomMlcgC_seed;

inline static error_t RandomMlcgC_Init_init( void ) ;

static uint32_t RandomMlcgC_Random_rand32( void ) ;

inline static uint16_t RandomMlcgC_Random_rand16( void ) ;

static error_t UniqueSendP_SubSend_send(message_t *msg, uint8_t len) ;

static void *UniqueSendP_SubSend_getPayload(message_t *msg, uint8_t len) ;

static void UniqueSendP_Send_sendDone(message_t *msg, error_t error) ;

static uint16_t UniqueSendP_Random_rand16( void ) ;

static cc2420_header_t *UniqueSendP_CC2420PacketBody_getHeader(message_t *msg) ;

static void UniqueSendP_State_toIdle( void ) ;

static error_t UniqueSendP_State_requestState(uint8_t reqState) ;

uint8_t UniqueSendP_localSendId;

enum UniqueSendP___nesc_unnamed4335 { UniqueSendP_S_IDLE, UniqueSendP_S_SENDING };

inline static error_t UniqueSendP_Init_init( void ) ;

inline static error_t UniqueSendP_Send_send(message_t *msg, uint8_t len) ;

inline static void *UniqueSendP_Send_getPayload(message_t *msg, uint8_t len) ;

inline static void UniqueSendP_SubSend_sendDone(message_t *msg, error_t error) ;

static message_t *UniqueReceiveP_Receive_receive(message_t *msg, void *payload, uint8_t len) ;

static cc2420_header_t *UniqueReceiveP_CC2420PacketBody_getHeader(message_t *msg) ;

static message_t *UniqueReceiveP_DuplicateReceive_receive(message_t *msg, void *payload, uint8_t len) ;

struct UniqueReceiveP___nesc_unnamed4336 {
  am_addr_t source;
  uint8_t dsn;
} (UniqueReceiveP_receivedMessages[4]);

uint8_t UniqueReceiveP_writeIndex = 0;

uint8_t UniqueReceiveP_recycleSourceElement;

enum UniqueReceiveP___nesc_unnamed4337 { UniqueReceiveP_INVALID_ELEMENT = 0xFF };

inline static error_t UniqueReceiveP_Init_init( void ) ;

inline static bool UniqueReceiveP_hasSeen(uint16_t msgSource, uint8_t msgDsn) ;

inline static void UniqueReceiveP_insert(uint16_t msgSource, uint8_t msgDsn) ;

inline static message_t *UniqueReceiveP_SubReceive_receive(message_t *msg, void *payload, uint8_t len) ;

inline static bool UniqueReceiveP_hasSeen(uint16_t msgSource, uint8_t msgDsn) ;

inline static void UniqueReceiveP_insert(uint16_t msgSource, uint8_t msgDsn) ;

inline static message_t *UniqueReceiveP_DuplicateReceive_default_receive(message_t *msg, void *payload, uint8_t len) ;

static error_t CC2420TinyosNetworkP_SubSend_send(message_t *msg, uint8_t len) ;

static void *CC2420TinyosNetworkP_SubSend_getPayload(message_t *msg, uint8_t len) ;

static void CC2420TinyosNetworkP_Send_sendDone(message_t *msg, error_t error) ;

static message_t *CC2420TinyosNetworkP_NonTinyosReceive_receive(uint8_t arg_0x218d9b8, message_t *msg, void *payload, uint8_t len) ;

static cc2420_header_t *CC2420TinyosNetworkP_CC2420PacketBody_getHeader(message_t *msg) ;

static message_t *CC2420TinyosNetworkP_Receive_receive(message_t *msg, void *payload, uint8_t len) ;

inline static error_t CC2420TinyosNetworkP_Send_send(message_t *msg, uint8_t len) ;

inline static void *CC2420TinyosNetworkP_Send_getPayload(message_t *msg, uint8_t len) ;

inline static void CC2420TinyosNetworkP_SubSend_sendDone(message_t *msg, error_t error) ;

inline static message_t *CC2420TinyosNetworkP_SubReceive_receive(message_t *msg, void *payload, uint8_t len) ;

inline static message_t *CC2420TinyosNetworkP_NonTinyosReceive_default_receive(uint8_t networkId, message_t *msg, void *payload, uint8_t len) ;

static error_t DisseminationEngineImplP_AMSend_send(am_addr_t addr, message_t *msg, uint8_t len) ;

static void *DisseminationEngineImplP_AMSend_getPayload(message_t *msg, uint8_t len) ;

static uint8_t DisseminationEngineImplP_AMSend_maxPayloadLength( void ) ;

static void DisseminationEngineImplP_TrickleTimer_incrementCounter(uint16_t arg_0x21c4c10) ;

static void DisseminationEngineImplP_TrickleTimer_reset(uint16_t arg_0x21c4c10) ;

static error_t DisseminationEngineImplP_TrickleTimer_start(uint16_t arg_0x21c4c10) ;

static void DisseminationEngineImplP_DisseminationCache_storeData(uint16_t arg_0x21c4120, void *data, uint8_t size, uint32_t seqno) ;

static uint32_t DisseminationEngineImplP_DisseminationCache_requestSeqno(uint16_t arg_0x21c4120) ;

static void *DisseminationEngineImplP_DisseminationCache_requestData(uint16_t arg_0x21c4120, uint8_t *size) ;

static error_t DisseminationEngineImplP_DisseminatorControl_start(uint16_t arg_0x21c2738) ;

enum DisseminationEngineImplP___nesc_unnamed4338 { DisseminationEngineImplP_NUM_DISSEMINATORS = 2U };

message_t DisseminationEngineImplP_m_buf;

bool DisseminationEngineImplP_m_running;

bool DisseminationEngineImplP_m_bufBusy;

static void DisseminationEngineImplP_sendObject(uint16_t key) ;

inline static error_t DisseminationEngineImplP_StdControl_start( void ) ;

static error_t DisseminationEngineImplP_DisseminationCache_start(uint16_t key) ;

static void DisseminationEngineImplP_DisseminationCache_newData(uint16_t key) ;

inline static void DisseminationEngineImplP_TrickleTimer_fired(uint16_t key) ;

static void DisseminationEngineImplP_sendObject(uint16_t key) ;

inline static void DisseminationEngineImplP_ProbeAMSend_sendDone(message_t *msg, error_t error) ;

inline static void DisseminationEngineImplP_AMSend_sendDone(message_t *msg, error_t error) ;

inline static message_t *DisseminationEngineImplP_Receive_receive(message_t *msg, void *payload, uint8_t len) ;

inline static message_t *DisseminationEngineImplP_ProbeReceive_receive(message_t *msg, void *payload, uint8_t len) ;

inline static void *DisseminationEngineImplP_DisseminationCache_default_requestData(uint16_t key, uint8_t *size) ;

inline static void DisseminationEngineImplP_DisseminationCache_default_storeData(uint16_t key, void *data, uint8_t size, uint32_t seqno) ;

inline static uint32_t DisseminationEngineImplP_DisseminationCache_default_requestSeqno(uint16_t key) ;

inline static error_t DisseminationEngineImplP_TrickleTimer_default_start(uint16_t key) ;

inline static void DisseminationEngineImplP_TrickleTimer_default_reset(uint16_t key) ;

inline static void DisseminationEngineImplP_TrickleTimer_default_incrementCounter(uint16_t key) ;

inline static error_t DisseminationEngineImplP_DisseminatorControl_default_start(uint16_t id) ;

static void AMQueueEntryP_0_AMSend_sendDone(message_t *msg, error_t error) ;

static error_t AMQueueEntryP_0_Send_send(message_t *msg, uint8_t len) ;

static void *AMQueueEntryP_0_Send_getPayload(message_t *msg, uint8_t len) ;

static uint8_t AMQueueEntryP_0_Send_maxPayloadLength( void ) ;

static void AMQueueEntryP_0_AMPacket_setDestination(message_t *amsg, am_addr_t addr) ;

static void AMQueueEntryP_0_AMPacket_setType(message_t *amsg, am_id_t t) ;

inline static error_t AMQueueEntryP_0_AMSend_send(am_addr_t dest, message_t *msg, uint8_t len) ;

inline static void AMQueueEntryP_0_Send_sendDone(message_t *m, error_t err) ;

inline static uint8_t AMQueueEntryP_0_AMSend_maxPayloadLength( void ) ;

inline static void *AMQueueEntryP_0_AMSend_getPayload(message_t *m, uint8_t len) ;

static error_t AMQueueImplP_0_AMSend_send(am_id_t arg_0x2246108, am_addr_t addr, message_t *msg, uint8_t len) ;

static void *AMQueueImplP_0_AMSend_getPayload(am_id_t arg_0x2246108, message_t *msg, uint8_t len) ;

static uint8_t AMQueueImplP_0_AMSend_maxPayloadLength(am_id_t arg_0x2246108) ;

static void AMQueueImplP_0_Send_sendDone(uint8_t arg_0x22476f8, message_t *msg, error_t error) ;

static uint8_t AMQueueImplP_0_Packet_payloadLength(message_t *msg) ;

static void AMQueueImplP_0_Packet_setPayloadLength(message_t *msg, uint8_t len) ;

static error_t AMQueueImplP_0_errorTask_postTask( void ) ;

static am_addr_t AMQueueImplP_0_AMPacket_destination(message_t *amsg) ;

static am_id_t AMQueueImplP_0_AMPacket_type(message_t *amsg) ;

enum AMQueueImplP_0___nesc_unnamed4339 { AMQueueImplP_0_CancelTask = 11U };

typedef int (AMQueueImplP_0___nesc_sillytask_CancelTask[AMQueueImplP_0_CancelTask]);

enum AMQueueImplP_0___nesc_unnamed4340 { AMQueueImplP_0_errorTask = 12U };

typedef int (AMQueueImplP_0___nesc_sillytask_errorTask[AMQueueImplP_0_errorTask]);

typedef struct AMQueueImplP_0___nesc_unnamed4341 {
  message_t *msg;
} AMQueueImplP_0_queue_entry_t;

uint8_t AMQueueImplP_0_current = 2;

AMQueueImplP_0_queue_entry_t (AMQueueImplP_0_queue[2]);

uint8_t (AMQueueImplP_0_cancelMask[(2 / 8) + 1]);

inline static void AMQueueImplP_0_tryToSend( void ) ;

inline static void AMQueueImplP_0_nextPacket( void ) ;

inline static error_t AMQueueImplP_0_Send_send(uint8_t clientId, message_t *msg, uint8_t len) ;

inline static void AMQueueImplP_0_CancelTask_runTask( void ) ;

static void AMQueueImplP_0_sendDone(uint8_t last, message_t *msg, error_t err) ;

inline static void AMQueueImplP_0_errorTask_runTask( void ) ;

inline static void AMQueueImplP_0_tryToSend( void ) ;

inline static void AMQueueImplP_0_AMSend_sendDone(am_id_t id, message_t *msg, error_t err) ;

inline static uint8_t AMQueueImplP_0_Send_maxPayloadLength(uint8_t id) ;

inline static void *AMQueueImplP_0_Send_getPayload(uint8_t id, message_t *m, uint8_t len) ;

inline static void AMQueueImplP_0_Send_default_sendDone(uint8_t id, message_t *msg, error_t err) ;

static void AMQueueEntryP_1_AMSend_sendDone(message_t *msg, error_t error) ;

inline static void AMQueueEntryP_1_Send_sendDone(message_t *m, error_t err) ;

static error_t DisseminatorP_0_changedTask_postTask( void ) ;

static void DisseminatorP_0_DisseminationCache_newData( void ) ;

static error_t DisseminatorP_0_DisseminationCache_start( void ) ;

static void DisseminatorP_0_DisseminationValue_changed( void ) ;

enum DisseminatorP_0___nesc_unnamed4342 { DisseminatorP_0_changedTask = 13U };

typedef int (DisseminatorP_0___nesc_sillytask_changedTask[DisseminatorP_0_changedTask]);

DisseminatorP_0_t DisseminatorP_0_valueCache;

bool DisseminatorP_0_m_running;

uint32_t DisseminatorP_0_seqno = DISSEMINATION_SEQNO_UNKNOWN;

inline static void DisseminatorP_0_changedTask_runTask( void ) ;

inline static error_t DisseminatorP_0_StdControl_start( void ) ;

const inline static DisseminatorP_0_t *DisseminatorP_0_DisseminationValue_get( void ) ;

inline static void DisseminatorP_0_DisseminationValue_set(const DisseminatorP_0_t *val) ;

inline static void DisseminatorP_0_DisseminationUpdate_change(DisseminatorP_0_t *newVal) ;

inline static void *DisseminatorP_0_DisseminationCache_requestData(uint8_t *size) ;

static void DisseminatorP_0_DisseminationCache_storeData(void *data, uint8_t size, uint32_t newSeqno) ;

inline static uint32_t DisseminatorP_0_DisseminationCache_requestSeqno( void ) ;

static void TrickleTimerImplP_0_Pending_clearAll( void ) ;

static void TrickleTimerImplP_0_Pending_clear(uint16_t bitnum) ;

static bool TrickleTimerImplP_0_Pending_get(uint16_t bitnum) ;

static void TrickleTimerImplP_0_Pending_set(uint16_t bitnum) ;

static void TrickleTimerImplP_0_Changed_clearAll( void ) ;

static void TrickleTimerImplP_0_Changed_clear(uint16_t bitnum) ;

static bool TrickleTimerImplP_0_Changed_get(uint16_t bitnum) ;

static void TrickleTimerImplP_0_Changed_set(uint16_t bitnum) ;

static uint16_t TrickleTimerImplP_0_Random_rand16( void ) ;

static void TrickleTimerImplP_0_TrickleTimer_fired(uint8_t arg_0x22e4c20) ;

static error_t TrickleTimerImplP_0_timerTask_postTask( void ) ;

static uint32_t TrickleTimerImplP_0_Timer_getNow( void ) ;

static uint32_t TrickleTimerImplP_0_Timer_getdt( void ) ;

static uint32_t TrickleTimerImplP_0_Timer_gett0( void ) ;

static void TrickleTimerImplP_0_Timer_startOneShot(uint32_t dt) ;

static void TrickleTimerImplP_0_Timer_stop( void ) ;

enum TrickleTimerImplP_0___nesc_unnamed4343 { TrickleTimerImplP_0_timerTask = 14U };

typedef int (TrickleTimerImplP_0___nesc_sillytask_timerTask[TrickleTimerImplP_0_timerTask]);

typedef struct TrickleTimerImplP_0___nesc_unnamed4344 {
  uint16_t period;
  uint32_t time;
  uint32_t remainder;
  uint8_t count;
} TrickleTimerImplP_0_trickle_t;

TrickleTimerImplP_0_trickle_t (TrickleTimerImplP_0_trickles[2U]);

static void TrickleTimerImplP_0_adjustTimer( void ) ;

static void TrickleTimerImplP_0_generateTime(uint8_t id) ;

inline static error_t TrickleTimerImplP_0_Init_init( void ) ;

static error_t TrickleTimerImplP_0_TrickleTimer_start(uint8_t id) ;

static void TrickleTimerImplP_0_TrickleTimer_reset(uint8_t id) ;

inline static void TrickleTimerImplP_0_TrickleTimer_incrementCounter(uint8_t id) ;

inline static void TrickleTimerImplP_0_timerTask_runTask( void ) ;

inline static void TrickleTimerImplP_0_Timer_fired( void ) ;

static void TrickleTimerImplP_0_adjustTimer( void ) ;

static void TrickleTimerImplP_0_generateTime(uint8_t id) ;

inline static void TrickleTimerImplP_0_TrickleTimer_default_fired(uint8_t id) ;

typedef uint8_t BitVectorC_0_int_type;

enum BitVectorC_0___nesc_unnamed4345 { BitVectorC_0_ELEMENT_SIZE = 8 * sizeof( BitVectorC_0_int_type ), BitVectorC_0_ARRAY_SIZE = ((2U + BitVectorC_0_ELEMENT_SIZE) - 1) / BitVectorC_0_ELEMENT_SIZE };

BitVectorC_0_int_type (BitVectorC_0_m_bits[BitVectorC_0_ARRAY_SIZE]);

inline static uint16_t BitVectorC_0_getIndex(uint16_t bitnum) ;

inline static uint16_t BitVectorC_0_getMask(uint16_t bitnum) ;

inline static void BitVectorC_0_BitVector_clearAll( void ) ;

inline static bool BitVectorC_0_BitVector_get(uint16_t bitnum) ;

inline static void BitVectorC_0_BitVector_set(uint16_t bitnum) ;

inline static void BitVectorC_0_BitVector_clear(uint16_t bitnum) ;

typedef uint8_t BitVectorC_1_int_type;

enum BitVectorC_1___nesc_unnamed4346 { BitVectorC_1_ELEMENT_SIZE = 8 * sizeof( BitVectorC_1_int_type ), BitVectorC_1_ARRAY_SIZE = ((2U + BitVectorC_1_ELEMENT_SIZE) - 1) / BitVectorC_1_ELEMENT_SIZE };

BitVectorC_1_int_type (BitVectorC_1_m_bits[BitVectorC_1_ARRAY_SIZE]);

inline static uint16_t BitVectorC_1_getIndex(uint16_t bitnum) ;

inline static uint16_t BitVectorC_1_getMask(uint16_t bitnum) ;

inline static void BitVectorC_1_BitVector_clearAll( void ) ;

inline static bool BitVectorC_1_BitVector_get(uint16_t bitnum) ;

inline static void BitVectorC_1_BitVector_set(uint16_t bitnum) ;

inline static void BitVectorC_1_BitVector_clear(uint16_t bitnum) ;

static error_t DisseminatorP_1_changedTask_postTask( void ) ;

static void DisseminatorP_1_DisseminationCache_newData( void ) ;

static error_t DisseminatorP_1_DisseminationCache_start( void ) ;

static void DisseminatorP_1_DisseminationValue_changed( void ) ;

enum DisseminatorP_1___nesc_unnamed4347 { DisseminatorP_1_changedTask = 15U };

typedef int (DisseminatorP_1___nesc_sillytask_changedTask[DisseminatorP_1_changedTask]);

DisseminatorP_1_t DisseminatorP_1_valueCache;

bool DisseminatorP_1_m_running;

uint32_t DisseminatorP_1_seqno = DISSEMINATION_SEQNO_UNKNOWN;

inline static void DisseminatorP_1_changedTask_runTask( void ) ;

inline static error_t DisseminatorP_1_StdControl_start( void ) ;

const inline static DisseminatorP_1_t *DisseminatorP_1_DisseminationValue_get( void ) ;

inline static void DisseminatorP_1_DisseminationValue_set(const DisseminatorP_1_t *val) ;

inline static void DisseminatorP_1_DisseminationUpdate_change(DisseminatorP_1_t *newVal) ;

inline static void *DisseminatorP_1_DisseminationCache_requestData(uint8_t *size) ;

static void DisseminatorP_1_DisseminationCache_storeData(void *data, uint8_t size, uint32_t newSeqno) ;

inline static uint32_t DisseminatorP_1_DisseminationCache_requestSeqno( void ) ;

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

inline static void TransformAlarmC_0_Counter_overflow( void ) 
{
}

inline static void CounterToLocalTimeC_0_Counter_overflow( void ) 
{
}

static inline void TransformCounterC_0_Counter_overflow( void ) 
{
  CounterToLocalTimeC_0_Counter_overflow();
  TransformAlarmC_0_Counter_overflow();
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

inline static void CounterToLocalTimeC_1_Counter_overflow( void ) 
{
}

inline static void TransformAlarmC_1_Counter_overflow( void ) 
{
}

static inline void TransformCounterC_1_Counter_overflow( void ) 
{
  TransformAlarmC_1_Counter_overflow();
  CounterToLocalTimeC_1_Counter_overflow();
}

inline static void TransformCounterC_1_CounterFrom_overflow( void ) 
{
{
  TransformCounterC_1_m_upper++;
  if((TransformCounterC_1_m_upper & TransformCounterC_1_OVERFLOW_MASK) == 0)
  {
    TransformCounterC_1_Counter_overflow();
  }
}
}

static inline void Msp430CounterC_0_Counter_overflow( void ) 
{
  TransformCounterC_1_CounterFrom_overflow();
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

static inline error_t CC2420ControlP_SpiResource_request( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Resource_request(CC2420SpiC_0_CLIENT_ID);
  return result;
}

inline static error_t CC2420ControlP_Resource_request( void ) 
{
  return CC2420ControlP_SpiResource_request();
}

static inline error_t CC2420CsmaP_Resource_request( void ) 
{
  char unsigned result;
  result = CC2420ControlP_Resource_request();
  return result;
}

inline static void CC2420CsmaP_CC2420Power_startVRegDone( void ) 
{
  CC2420CsmaP_Resource_request();
}

static inline void CC2420ControlP_CC2420Power_startVRegDone( void ) 
{
  CC2420CsmaP_CC2420Power_startVRegDone();
}

static inline void Msp430GpioC_4_HplGeneralIO_set( void ) 
{
  HplMsp430GeneralIOP_30_IO_set();
}

inline static void Msp430GpioC_4_GeneralIO_set( void ) 
{
  Msp430GpioC_4_HplGeneralIO_set();
}

static inline void CC2420ControlP_RSTN_set( void ) 
{
  Msp430GpioC_4_GeneralIO_set();
}

static inline void Msp430GpioC_4_HplGeneralIO_clr( void ) 
{
  HplMsp430GeneralIOP_30_IO_clr();
}

inline static void Msp430GpioC_4_GeneralIO_clr( void ) 
{
  Msp430GpioC_4_HplGeneralIO_clr();
}

static inline void CC2420ControlP_RSTN_clr( void ) 
{
  Msp430GpioC_4_GeneralIO_clr();
}

inline static void CC2420ControlP_StartupTimer_fired( void ) 
{
  if(CC2420ControlP_m_state == CC2420ControlP_S_VREG_STARTING)
  {
    CC2420ControlP_m_state = CC2420ControlP_S_VREG_STARTED;
    CC2420ControlP_RSTN_clr();
    CC2420ControlP_RSTN_set();
    CC2420ControlP_CC2420Power_startVRegDone();
  }
}

static inline error_t CC2420TransmitP_SpiResource_release( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Resource_release(CC2420SpiC_3_CLIENT_ID);
  return result;
}

inline static error_t CC2420TransmitP_releaseSpiResource( void ) 
{
  CC2420TransmitP_SpiResource_release();
  return SUCCESS;
}

inline static error_t GpioCaptureC_0_Capture_captureRisingEdge( void ) 
{
  return GpioCaptureC_0_enableCapture(MSP430TIMER_CM_RISING);
}

static inline error_t CC2420TransmitP_CaptureSFD_captureRisingEdge( void ) 
{
  char unsigned result;
  result = GpioCaptureC_0_Capture_captureRisingEdge();
  return result;
}

static inline cc2420_status_t CC2420TransmitP_SFLUSHTX_strobe( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Strobe_strobe(CC2420_SFLUSHTX);
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

inline static void TransformAlarmC_0_Alarm_start(TransformAlarmC_0_to_size_type dt) 
{
  TransformAlarmC_0_Alarm_startAt(TransformAlarmC_0_Alarm_getNow(), dt);
}

static inline void CC2420TransmitP_BackoffTimer_start(CC2420TransmitP_BackoffTimer_size_type dt) 
{
  TransformAlarmC_0_Alarm_start(dt);
}

inline static uint8_t HplMsp430GeneralIOP_4_IO_getRaw( void ) 
{
  return _P1IN & (0x01 << 4);
}

inline static bool HplMsp430GeneralIOP_4_IO_get( void ) 
{
  return HplMsp430GeneralIOP_4_IO_getRaw() != 0;
}

static inline bool Msp430GpioC_0_HplGeneralIO_get( void ) 
{
  char unsigned result;
  result = HplMsp430GeneralIOP_4_IO_get();
  return result;
}

inline static bool Msp430GpioC_0_GeneralIO_get( void ) 
{
  return Msp430GpioC_0_HplGeneralIO_get();
}

static inline bool CC2420TransmitP_CCA_get( void ) 
{
  char unsigned result;
  result = Msp430GpioC_0_GeneralIO_get();
  return result;
}

inline static void CC2420TransmitP_BackoffTimer_fired( void ) 
{
{
  /* IRQ INSTR 0 */
    if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 1 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  switch (CC2420TransmitP_m_state) 
  {
    case CC2420TransmitP_S_SAMPLE_CCA:
      if(CC2420TransmitP_CCA_get())
      {
        CC2420TransmitP_m_state = CC2420TransmitP_S_BEGIN_TRANSMIT;
        CC2420TransmitP_BackoffTimer_start(CC2420_TIME_ACK_TURNAROUND);
      }
      else 
      {
        CC2420TransmitP_congestionBackoff();
      }
      break;
    case CC2420TransmitP_S_BEGIN_TRANSMIT:
      case CC2420TransmitP_S_CANCEL:
        if(CC2420TransmitP_acquireSpiResource() == SUCCESS)
        {
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
    default :
        break;
  }

}
}

static inline void TransformAlarmC_0_Alarm_fired( void ) 
{
  CC2420TransmitP_BackoffTimer_fired();
  CC2420ControlP_StartupTimer_fired();
}

inline static void TransformAlarmC_0_AlarmFrom_fired( void ) 
{
{
  /* IRQ INSTR 2 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 3 */
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
  union Msp430TimerCapComP_3___nesc_unnamed4351  {
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

__inline static uint8_t __nesc_ntoh_leuint8(const void *source) 
{
  uint8_t *base = (uint8_t *)source;
  return base[0];
}

inline static void CC2420ActiveMessageP_RadioBackoff_default_requestCongestionBackoff(am_id_t id, message_t *msg) 
{
}

static inline void CC2420ActiveMessageP_RadioBackoff_requestCongestionBackoff(am_id_t arg_0x16fd2e8, message_t *msg) 
{
  CC2420ActiveMessageP_RadioBackoff_default_requestCongestionBackoff(arg_0x16fd2e8, msg);
}

inline static void CC2420ActiveMessageP_SubBackoff_requestCongestionBackoff(message_t *msg) 
{
  CC2420ActiveMessageP_RadioBackoff_requestCongestionBackoff(
    __nesc_ntoh_leuint8(((cc2420_header_t * )
                       ((uint8_t *)msg + 
                         (size_t )& ((message_t *)0)->data - 
                         sizeof(cc2420_header_t )))->type.data), msg);
}

static inline void CC2420CsmaP_RadioBackoff_requestCongestionBackoff(message_t *msg) 
{
  CC2420ActiveMessageP_SubBackoff_requestCongestionBackoff(msg);
}

inline static uint16_t RandomMlcgC_Random_rand16( void ) 
{
  return ( uint16_t )(RandomMlcgC_Random_rand32());
}

static inline uint16_t CC2420CsmaP_Random_rand16( void ) 
{
  int unsigned result;
  result = RandomMlcgC_Random_rand16();
  return result;
}

inline static void CC2420TransmitP_RadioBackoff_setCongestionBackoff(uint16_t backoffTime) 
{
  CC2420TransmitP_myCongestionBackoff = backoffTime + 1;
}

static inline void CC2420CsmaP_SubBackoff_setCongestionBackoff(uint16_t backoffTime) 
{
  CC2420TransmitP_RadioBackoff_setCongestionBackoff(backoffTime);
}

inline static void CC2420CsmaP_SubBackoff_requestCongestionBackoff(message_t *msg) 
{
  CC2420CsmaP_SubBackoff_setCongestionBackoff((CC2420CsmaP_Random_rand16() % (0x7 * CC2420_BACKOFF_PERIOD)) + CC2420_MIN_BACKOFF);
  CC2420CsmaP_RadioBackoff_requestCongestionBackoff(msg);
}

static inline void CC2420TransmitP_RadioBackoff_requestCongestionBackoff(message_t *msg) 
{
  CC2420CsmaP_SubBackoff_requestCongestionBackoff(msg);
}

static inline error_t CC2420TransmitP_SpiResource_immediateRequest( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Resource_immediateRequest(CC2420SpiC_3_CLIENT_ID);
  return result;
}

static inline error_t CC2420SpiP_WorkingState_requestState(uint8_t reqState) 
{
  char unsigned result;
  result = StateImplP_State_requestState(0U, reqState);
  return result;
}

inline static error_t Msp430SpiNoDmaP_0_UsartResource_default_isOwner(uint8_t id) 
{
  return FAIL;
}

static inline bool Msp430SpiNoDmaP_0_UsartResource_isOwner(uint8_t arg_0x1c66010) 
{
  char unsigned result;
  switch (arg_0x1c66010) 
  {
    case Msp430Spi0C_0_CLIENT_ID:
      result = ArbiterP_0_Resource_isOwner(Msp430Usart0C_0_CLIENT_ID);
      break;
    default :
      result = Msp430SpiNoDmaP_0_UsartResource_default_isOwner(arg_0x1c66010);
      break;
  }

  return result;
}

inline static uint8_t Msp430SpiNoDmaP_0_Resource_isOwner(uint8_t id) 
{
  return Msp430SpiNoDmaP_0_UsartResource_isOwner(id);
}

static inline bool CC2420SpiP_SpiResource_isOwner( void ) 
{
  char unsigned result;
  result = Msp430SpiNoDmaP_0_Resource_isOwner(Msp430Spi0C_0_CLIENT_ID);
  return result;
}

inline static msp430_spi_union_config_t *Msp430SpiNoDmaP_0_Msp430SpiConfigure_default_getConfig(uint8_t id) 
{
  return &msp430_spi_default_config;
}

static inline msp430_spi_union_config_t *Msp430SpiNoDmaP_0_Msp430SpiConfigure_getConfig(uint8_t arg_0x1c66a60) 
{
  union __nesc_unnamed4280  *result;
  result = Msp430SpiNoDmaP_0_Msp430SpiConfigure_default_getConfig(arg_0x1c66a60);
  return result;
}

static inline void Msp430SpiNoDmaP_0_Usart_setModeSpi(msp430_spi_union_config_t *config) 
{
  HplMsp430Usart0P_Usart_setModeSpi(config);
}

inline static void Msp430SpiNoDmaP_0_ResourceConfigure_configure(uint8_t id) 
{
  Msp430SpiNoDmaP_0_Usart_setModeSpi(Msp430SpiNoDmaP_0_Msp430SpiConfigure_getConfig(id));
}

inline static void ArbiterP_0_ResourceConfigure_default_configure(uint8_t id) 
{
}

static inline void ArbiterP_0_ResourceConfigure_configure(uint8_t arg_0x1db4030) 
{
  switch (arg_0x1db4030) 
  {
    case Msp430Usart0C_0_CLIENT_ID:
      Msp430SpiNoDmaP_0_ResourceConfigure_configure(Msp430Spi0C_0_CLIENT_ID);
      break;
    default :
      ArbiterP_0_ResourceConfigure_default_configure(arg_0x1db4030);
      break;
  }

}

inline static void ArbiterP_0_ResourceDefaultOwner_default_immediateRequested( void ) 
{
  ArbiterP_0_ResourceDefaultOwner_release();
}

static inline void ArbiterP_0_ResourceDefaultOwner_immediateRequested( void ) 
{
  ArbiterP_0_ResourceDefaultOwner_default_immediateRequested();
}

inline static void ArbiterP_0_ResourceRequested_default_immediateRequested(uint8_t id) 
{
}

static inline void ArbiterP_0_ResourceRequested_immediateRequested(uint8_t arg_0x1db7c58) 
{
  ArbiterP_0_ResourceRequested_default_immediateRequested(arg_0x1db7c58);
}

inline static error_t ArbiterP_0_Resource_immediateRequest(uint8_t id) 
{
  ArbiterP_0_ResourceRequested_immediateRequested(ArbiterP_0_resId);
{
  if(ArbiterP_0_state == ArbiterP_0_RES_CONTROLLED)
  {
    ArbiterP_0_state = ArbiterP_0_RES_IMM_GRANTING;
    ArbiterP_0_reqResId = id;
  }
  else 
  {
    char unsigned __nesc_temp = FAIL;
    return __nesc_temp;
  }
}
  ArbiterP_0_ResourceDefaultOwner_immediateRequested();
  if(ArbiterP_0_resId == id)
  {
    ArbiterP_0_ResourceConfigure_configure(ArbiterP_0_resId);
    return SUCCESS;
  }
  ArbiterP_0_state = ArbiterP_0_RES_CONTROLLED;
  return FAIL;
}

inline static error_t Msp430SpiNoDmaP_0_UsartResource_default_immediateRequest(uint8_t id) 
{
  return FAIL;
}

static inline error_t Msp430SpiNoDmaP_0_UsartResource_immediateRequest(uint8_t arg_0x1c66010) 
{
  char unsigned result;
  switch (arg_0x1c66010) 
  {
    case Msp430Spi0C_0_CLIENT_ID:
      result = ArbiterP_0_Resource_immediateRequest(Msp430Usart0C_0_CLIENT_ID);
      break;
    default :
      result = Msp430SpiNoDmaP_0_UsartResource_default_immediateRequest(arg_0x1c66010);
      break;
  }

  return result;
}

inline static error_t Msp430SpiNoDmaP_0_Resource_immediateRequest(uint8_t id) 
{
  return Msp430SpiNoDmaP_0_UsartResource_immediateRequest(id);
}

static inline error_t CC2420SpiP_SpiResource_immediateRequest( void ) 
{
  char unsigned result;
  result = Msp430SpiNoDmaP_0_Resource_immediateRequest(Msp430Spi0C_0_CLIENT_ID);
  return result;
}

inline static bool SchedulerBasicP_isWaiting(uint8_t id) 
{
  /* IRQ INSTR 4 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 5 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 6 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  return (SchedulerBasicP_m_next[id] != SchedulerBasicP_NO_TASK) || (SchedulerBasicP_m_tail == id);
}

inline static bool SchedulerBasicP_pushTask(uint8_t id) 
{
  if(!SchedulerBasicP_isWaiting(id))
  {
  /* IRQ INSTR 7 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 8 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 9 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
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

inline static void HplMsp430Usart0P_Usart_resetUsart(bool reset) 
{
  if(reset)
  {
    _U0CTL = 0x01;
  }
  else 
  {
    _U0CTL &= ~0x01;
  }
}

static inline void HplMsp430I2C0P_HplUsart_resetUsart(bool reset) 
{
  HplMsp430Usart0P_Usart_resetUsart(reset);
}

inline static void HplMsp430I2C0P_HplI2C_clearModeI2C( void ) 
{
{
  _U0CTL &= ~(0x20 | 0x04) | 0x01;
  HplMsp430I2C0P_HplUsart_resetUsart(TRUE);
}
}

static inline void HplMsp430Usart0P_HplI2C_clearModeI2C( void ) 
{
  HplMsp430I2C0P_HplI2C_clearModeI2C();
}

inline static void HplMsp430GeneralIOP_21_IO_selectIOFunc( void ) 
{
  _P3SEL &= ~0x01 << 5;
}

static inline void HplMsp430Usart0P_URXD_selectIOFunc( void ) 
{
  HplMsp430GeneralIOP_21_IO_selectIOFunc();
}

inline static void HplMsp430GeneralIOP_20_IO_selectIOFunc( void ) 
{
  _P3SEL &= ~0x01 << 4;
}

static inline void HplMsp430Usart0P_UTXD_selectIOFunc( void ) 
{
  HplMsp430GeneralIOP_20_IO_selectIOFunc();
}

inline static void HplMsp430Usart0P_Usart_disableUart( void ) 
{
{
  _ME1 &= ~(1 << 7) | (1 << 6);
  HplMsp430Usart0P_UTXD_selectIOFunc();
  HplMsp430Usart0P_URXD_selectIOFunc();
}
}

inline static void HplMsp430Usart0P_Usart_setUmctl(uint8_t control) 
{
  _U0MCTL = control;
}

inline static void HplMsp430Usart0P_Usart_setUbr(uint16_t control) 
{
{
  _U0BR0 = control & 0x00FF;
  _U0BR1 = (control >> 8) & 0x00FF;
}
}

inline static void HplMsp430Usart0P_configSpi(msp430_spi_union_config_t *config) 
{
  _U0CTL = (config->spiRegisters.uctl | 0x04) | 0x01;
  _U0TCTL = config->spiRegisters.utctl;
  HplMsp430Usart0P_Usart_setUbr(config->spiRegisters.ubr);
  HplMsp430Usart0P_Usart_setUmctl(0x00);
}

inline static void HplMsp430GeneralIOP_19_IO_selectModuleFunc( void ) 
{
  _P3SEL |= 0x01 << 3;
}

static inline void HplMsp430Usart0P_UCLK_selectModuleFunc( void ) 
{
  HplMsp430GeneralIOP_19_IO_selectModuleFunc();
}

inline static void HplMsp430GeneralIOP_18_IO_selectModuleFunc( void ) 
{
  _P3SEL |= 0x01 << 2;
}

static inline void HplMsp430Usart0P_SOMI_selectModuleFunc( void ) 
{
  HplMsp430GeneralIOP_18_IO_selectModuleFunc();
}

inline static void HplMsp430GeneralIOP_17_IO_selectModuleFunc( void ) 
{
  _P3SEL |= 0x01 << 1;
}

static inline void HplMsp430Usart0P_SIMO_selectModuleFunc( void ) 
{
  HplMsp430GeneralIOP_17_IO_selectModuleFunc();
}

inline static void HplMsp430Usart0P_Usart_enableSpi( void ) 
{
{
  HplMsp430Usart0P_SIMO_selectModuleFunc();
  HplMsp430Usart0P_SOMI_selectModuleFunc();
  HplMsp430Usart0P_UCLK_selectModuleFunc();
}
  _ME1 |= 1 << 6;
}

inline static void HplMsp430Usart0P_Usart_clrIntr( void ) 
{
  _IFG1 &= ~(1 << 7) | (1 << 6);
}

inline static void HplMsp430Usart0P_Usart_disableIntr( void ) 
{
  _IE1 &= ~(1 << 7) | (1 << 6);
}

inline static void StateImplP_State_toIdle(uint8_t id) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  StateImplP_state[id] = StateImplP_S_IDLE;
  __nesc_atomic_end(__nesc_atomic);
}
}

static inline void CC2420SpiP_WorkingState_toIdle( void ) 
{
  StateImplP_State_toIdle(0U);
}

static inline error_t CC2420TransmitP_SpiResource_request( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Resource_request(CC2420SpiC_3_CLIENT_ID);
  return result;
}

inline static void ArbiterP_0_ResourceDefaultOwner_default_requested( void ) 
{
  ArbiterP_0_ResourceDefaultOwner_release();
}

static inline void ArbiterP_0_ResourceDefaultOwner_requested( void ) 
{
  ArbiterP_0_ResourceDefaultOwner_default_requested();
}

inline static bool FcfsResourceQueueC_0_FcfsQueue_isEnqueued(resource_client_id_t id) 
{
  /* IRQ INSTR 10 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 11 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 12 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  return (FcfsResourceQueueC_0_resQ[id] != FcfsResourceQueueC_0_NO_ENTRY) || (FcfsResourceQueueC_0_qTail == id);
}

inline static error_t FcfsResourceQueueC_0_FcfsQueue_enqueue(resource_client_id_t id) 
{
{
  if(!FcfsResourceQueueC_0_FcfsQueue_isEnqueued(id))
  {
  /* IRQ INSTR 13 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 14 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 15 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
    if(FcfsResourceQueueC_0_qHead == FcfsResourceQueueC_0_NO_ENTRY)
    {
      FcfsResourceQueueC_0_qHead = id;
    }
    else 
    {
      FcfsResourceQueueC_0_resQ[FcfsResourceQueueC_0_qTail] = id;
    }
    FcfsResourceQueueC_0_qTail = id;
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

static inline error_t ArbiterP_0_Queue_enqueue(resource_client_id_t id) 
{
  char unsigned result;
  result = FcfsResourceQueueC_0_FcfsQueue_enqueue(id);
  return result;
}

inline static void ArbiterP_0_ResourceRequested_default_requested(uint8_t id) 
{
}

static inline void ArbiterP_0_ResourceRequested_requested(uint8_t arg_0x1db7c58) 
{
  ArbiterP_0_ResourceRequested_default_requested(arg_0x1db7c58);
}

inline static error_t ArbiterP_0_Resource_request(uint8_t id) 
{
  ArbiterP_0_ResourceRequested_requested(ArbiterP_0_resId);
{
  if(ArbiterP_0_state == ArbiterP_0_RES_CONTROLLED)
  {
    ArbiterP_0_state = ArbiterP_0_RES_GRANTING;
    ArbiterP_0_reqResId = id;
  }
  else 
  {
    char unsigned __nesc_temp = ArbiterP_0_Queue_enqueue(id);
    return __nesc_temp;
  }
}
  ArbiterP_0_ResourceDefaultOwner_requested();
  return SUCCESS;
}

inline static error_t Msp430SpiNoDmaP_0_UsartResource_default_request(uint8_t id) 
{
  return FAIL;
}

static inline error_t Msp430SpiNoDmaP_0_UsartResource_request(uint8_t arg_0x1c66010) 
{
  char unsigned result;
  switch (arg_0x1c66010) 
  {
    case Msp430Spi0C_0_CLIENT_ID:
      result = ArbiterP_0_Resource_request(Msp430Usart0C_0_CLIENT_ID);
      break;
    default :
      result = Msp430SpiNoDmaP_0_UsartResource_default_request(arg_0x1c66010);
      break;
  }

  return result;
}

inline static error_t Msp430SpiNoDmaP_0_Resource_request(uint8_t id) 
{
  return Msp430SpiNoDmaP_0_UsartResource_request(id);
}

static inline error_t CC2420SpiP_SpiResource_request( void ) 
{
  char unsigned result;
  result = Msp430SpiNoDmaP_0_Resource_request(Msp430Spi0C_0_CLIENT_ID);
  return result;
}

inline static void HplMsp430Usart0P_Usart_tx(uint8_t data) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _U0TXBUF = data;
  __nesc_atomic_end(__nesc_atomic);
}
}

static inline void Msp430SpiNoDmaP_0_Usart_tx(uint8_t data) 
{
  HplMsp430Usart0P_Usart_tx(data);
}

inline static bool HplMsp430Usart0P_Usart_isRxIntrPending( void ) 
{
  if(_IFG1 & (1 << 6))
  {
    return TRUE;
  }
  return FALSE;
}

static inline bool Msp430SpiNoDmaP_0_Usart_isRxIntrPending( void ) 
{
  char unsigned result;
  result = HplMsp430Usart0P_Usart_isRxIntrPending();
  return result;
}

inline static void HplMsp430Usart0P_Usart_clrRxIntr( void ) 
{
  _IFG1 &= ~1 << 6;
}

static inline void Msp430SpiNoDmaP_0_Usart_clrRxIntr( void ) 
{
  HplMsp430Usart0P_Usart_clrRxIntr();
}

static inline uint8_t Msp430SpiNoDmaP_0_Usart_rx( void ) 
{
  char unsigned result;
  result = HplMsp430Usart0P_Usart_rx();
  return result;
}

inline static void CC2420SpiP_ChipSpiResource_abortRelease( void ) 
{
  CC2420SpiP_release = FALSE;
}

static inline void CC2420TransmitP_ChipSpiResource_abortRelease( void ) 
{
  CC2420SpiP_ChipSpiResource_abortRelease();
}

inline static void CC2420TransmitP_ChipSpiResource_releasing( void ) 
{
  if(CC2420TransmitP_abortSpiRelease)
  {
    CC2420TransmitP_ChipSpiResource_abortRelease();
  }
}

static inline void CC2420SpiP_ChipSpiResource_releasing( void ) 
{
  CC2420TransmitP_ChipSpiResource_releasing();
}

inline static void ArbiterP_0_ResourceDefaultOwner_default_granted( void ) 
{
}

static inline void ArbiterP_0_ResourceDefaultOwner_granted( void ) 
{
  ArbiterP_0_ResourceDefaultOwner_default_granted();
}

static inline void Msp430SpiNoDmaP_0_Usart_resetUsart(bool reset) 
{
  HplMsp430Usart0P_Usart_resetUsart(reset);
}

static inline void Msp430SpiNoDmaP_0_Usart_disableSpi( void ) 
{
  HplMsp430Usart0P_Usart_disableSpi();
}

inline static void Msp430SpiNoDmaP_0_ResourceConfigure_unconfigure(uint8_t id) 
{
  Msp430SpiNoDmaP_0_Usart_resetUsart(TRUE);
  Msp430SpiNoDmaP_0_Usart_disableSpi();
  Msp430SpiNoDmaP_0_Usart_resetUsart(FALSE);
}

inline static void ArbiterP_0_ResourceConfigure_default_unconfigure(uint8_t id) 
{
}

static inline void ArbiterP_0_ResourceConfigure_unconfigure(uint8_t arg_0x1db4030) 
{
  switch (arg_0x1db4030) 
  {
    case Msp430Usart0C_0_CLIENT_ID:
      Msp430SpiNoDmaP_0_ResourceConfigure_unconfigure(Msp430Spi0C_0_CLIENT_ID);
      break;
    default :
      ArbiterP_0_ResourceConfigure_default_unconfigure(arg_0x1db4030);
      break;
  }

}

static inline error_t ArbiterP_0_grantedTask_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(ArbiterP_0_grantedTask);
  return result;
}

inline static resource_client_id_t FcfsResourceQueueC_0_FcfsQueue_dequeue( void ) 
{
{
  /* IRQ INSTR 25 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 26 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 27 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  if(FcfsResourceQueueC_0_qHead != FcfsResourceQueueC_0_NO_ENTRY)
  {
  /* IRQ INSTR 22 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 23 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 24 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
    uint8_t id = FcfsResourceQueueC_0_qHead;
  /* IRQ INSTR 16 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 17 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 18 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
    FcfsResourceQueueC_0_qHead = FcfsResourceQueueC_0_resQ[FcfsResourceQueueC_0_qHead];
  /* IRQ INSTR 19 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 20 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 21 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
    if(FcfsResourceQueueC_0_qHead == FcfsResourceQueueC_0_NO_ENTRY)
    {
      FcfsResourceQueueC_0_qTail = FcfsResourceQueueC_0_NO_ENTRY;
    }
    FcfsResourceQueueC_0_resQ[id] = FcfsResourceQueueC_0_NO_ENTRY;
  {
    char unsigned __nesc_temp = id;
    return __nesc_temp;
  }
  }
{
  char unsigned __nesc_temp = FcfsResourceQueueC_0_NO_ENTRY;
  return __nesc_temp;
}
}
}

static inline resource_client_id_t ArbiterP_0_Queue_dequeue( void ) 
{
  char unsigned result;
  result = FcfsResourceQueueC_0_FcfsQueue_dequeue();
  return result;
}

inline static bool FcfsResourceQueueC_0_FcfsQueue_isEmpty( void ) 
{
  /* IRQ INSTR 28 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 29 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 30 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  return FcfsResourceQueueC_0_qHead == FcfsResourceQueueC_0_NO_ENTRY;
}

static inline bool ArbiterP_0_Queue_isEmpty( void ) 
{
  char unsigned result;
  result = FcfsResourceQueueC_0_FcfsQueue_isEmpty();
  return result;
}

inline static error_t ArbiterP_0_Resource_release(uint8_t id) 
{
{
  if((ArbiterP_0_state == ArbiterP_0_RES_BUSY) && (ArbiterP_0_resId == id))
  {
    if(ArbiterP_0_Queue_isEmpty() == FALSE)
    {
      ArbiterP_0_reqResId = ArbiterP_0_Queue_dequeue();
      ArbiterP_0_resId = ArbiterP_0_NO_RES;
      ArbiterP_0_state = ArbiterP_0_RES_GRANTING;
      ArbiterP_0_grantedTask_postTask();
      ArbiterP_0_ResourceConfigure_unconfigure(id);
    }
    else 
    {
      ArbiterP_0_resId = ArbiterP_0_default_owner_id;
      ArbiterP_0_state = ArbiterP_0_RES_CONTROLLED;
      ArbiterP_0_ResourceConfigure_unconfigure(id);
      ArbiterP_0_ResourceDefaultOwner_granted();
    }
  {
    char unsigned __nesc_temp = SUCCESS;
    return __nesc_temp;
  }
  }
}
  return FAIL;
}

inline static error_t Msp430SpiNoDmaP_0_UsartResource_default_release(uint8_t id) 
{
  return FAIL;
}

static inline error_t Msp430SpiNoDmaP_0_UsartResource_release(uint8_t arg_0x1c66010) 
{
  char unsigned result;
  switch (arg_0x1c66010) 
  {
    case Msp430Spi0C_0_CLIENT_ID:
      result = ArbiterP_0_Resource_release(Msp430Usart0C_0_CLIENT_ID);
      break;
    default :
      result = Msp430SpiNoDmaP_0_UsartResource_default_release(arg_0x1c66010);
      break;
  }

  return result;
}

inline static error_t Msp430SpiNoDmaP_0_Resource_release(uint8_t id) 
{
  return Msp430SpiNoDmaP_0_UsartResource_release(id);
}

static inline error_t CC2420SpiP_SpiResource_release( void ) 
{
  char unsigned result;
  result = Msp430SpiNoDmaP_0_Resource_release(Msp430Spi0C_0_CLIENT_ID);
  return result;
}

inline static void HplMsp430GeneralIOP_17_IO_selectIOFunc( void ) 
{
  _P3SEL &= ~0x01 << 1;
}

static inline void HplMsp430Usart0P_SIMO_selectIOFunc( void ) 
{
  HplMsp430GeneralIOP_17_IO_selectIOFunc();
}

inline static void HplMsp430GeneralIOP_18_IO_selectIOFunc( void ) 
{
  _P3SEL &= ~0x01 << 2;
}

static inline void HplMsp430Usart0P_SOMI_selectIOFunc( void ) 
{
  HplMsp430GeneralIOP_18_IO_selectIOFunc();
}

inline static void HplMsp430GeneralIOP_19_IO_selectIOFunc( void ) 
{
  _P3SEL &= ~0x01 << 3;
}

static inline void HplMsp430Usart0P_UCLK_selectIOFunc( void ) 
{
  HplMsp430GeneralIOP_19_IO_selectIOFunc();
}

static inline cc2420_status_t CC2420TransmitP_STXONCCA_strobe( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Strobe_strobe(CC2420_STXONCCA);
  return result;
}

static inline cc2420_status_t CC2420TransmitP_STXON_strobe( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Strobe_strobe(CC2420_STXON);
  return result;
}

static inline cc2420_status_t CC2420TransmitP_SNOP_strobe( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Strobe_strobe(CC2420_SNOP);
  return result;
}

inline static error_t CC2420SpiP_ChipSpiResource_attemptRelease( void ) 
{
  return CC2420SpiP_attemptRelease();
}

static inline error_t CC2420TransmitP_ChipSpiResource_attemptRelease( void ) 
{
  char unsigned result;
  result = CC2420SpiP_ChipSpiResource_attemptRelease();
  return result;
}

inline static void HplMsp430GeneralIOP_25_IO_selectModuleFunc( void ) 
{
  _P4SEL |= 0x01 << 1;
}

static inline void GpioCaptureC_0_GeneralIO_selectModuleFunc( void ) 
{
  HplMsp430GeneralIOP_25_IO_selectModuleFunc();
}

inline static uint16_t Msp430TimerCapComP_4_CC2int(Msp430TimerCapComP_4_cc_t x) 
{
  union Msp430TimerCapComP_4___nesc_unnamed4352  {
  Msp430TimerCapComP_4_cc_t f;
  uint16_t t;
} c;
  c.f = x;
  return c.t;
}

inline static uint16_t Msp430TimerCapComP_4_captureControl(uint8_t l_cm) 
{
  Msp430TimerCapComP_4_cc_t x = {.cm = l_cm & 0x03, .ccis = 0, .clld = 0, .cap = 1, .scs = 1, .ccie = 0};
  return Msp430TimerCapComP_4_CC2int(x);
}

inline static void Msp430TimerCapComP_4_Control_setControlAsCapture(uint8_t cm) 
{
  _TBCCTL1 = Msp430TimerCapComP_4_captureControl(cm);
}

static inline void GpioCaptureC_0_Msp430TimerControl_setControlAsCapture(uint8_t cm) 
{
  Msp430TimerCapComP_4_Control_setControlAsCapture(cm);
}

inline static void Msp430TimerCapComP_4_Control_enableEvents( void ) 
{
  _TBCCTL1 |= 0x0010;
}

static inline void GpioCaptureC_0_Msp430TimerControl_enableEvents( void ) 
{
  Msp430TimerCapComP_4_Control_enableEvents();
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

__inline static uint16_t __nesc_ntoh_leuint16(void *source) 
{
  const uint8_t *base = source;
  return (( uint16_t )(base[1]) << 8) | base[0];
}

__inline static uint32_t __nesc_hton_uint32(void *target, uint32_t value) 
{
  uint8_t *base = target;
  base[3] = value;
  base[2] = value >> 8;
  base[1] = value >> 16;
  base[0] = value >> 24;
  return value;
}

__inline static uint32_t __nesc_ntoh_uint32(void *source) 
{
  const uint8_t *base = source;
  return (((( uint32_t )(base[0]) << 24) | (( uint32_t )(base[1]) << 16)) | (( uint32_t )(base[2]) << 8)) | base[3];
}

static inline void CC2420TransmitP_PacketTimeStamp_clear(message_t *msg) 
{
  CC2420PacketP_PacketTimeStamp32khz_clear(msg);
}

inline static void CC2420ReceiveP_CC2420Receive_sfd_dropped( void ) 
{
  if(CC2420ReceiveP_m_timestamp_size)
  {
    CC2420ReceiveP_m_timestamp_size--;
  }
}

static inline void CC2420TransmitP_CC2420Receive_sfd_dropped( void ) 
{
  CC2420ReceiveP_CC2420Receive_sfd_dropped();
}

inline static uint8_t HplMsp430GeneralIOP_25_IO_getRaw( void ) 
{
  return _P4IN & (0x01 << 1);
}

inline static bool HplMsp430GeneralIOP_25_IO_get( void ) 
{
  return HplMsp430GeneralIOP_25_IO_getRaw() != 0;
}

static inline bool Msp430GpioC_5_HplGeneralIO_get( void ) 
{
  char unsigned result;
  result = HplMsp430GeneralIOP_25_IO_get();
  return result;
}

inline static bool Msp430GpioC_5_GeneralIO_get( void ) 
{
  return Msp430GpioC_5_HplGeneralIO_get();
}

static inline bool CC2420TransmitP_SFD_get( void ) 
{
  char unsigned result;
  result = Msp430GpioC_5_GeneralIO_get();
  return result;
}

inline static void CC2420ReceiveP_CC2420Receive_sfd(uint32_t time) 
{
  if(CC2420ReceiveP_m_timestamp_size < CC2420ReceiveP_TIMESTAMP_QUEUE_SIZE)
  {
  /* IRQ INSTR 31 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
    uint8_t tail = (CC2420ReceiveP_m_timestamp_head + CC2420ReceiveP_m_timestamp_size) % CC2420ReceiveP_TIMESTAMP_QUEUE_SIZE;
    CC2420ReceiveP_m_timestamp_queue[tail] = time;
    CC2420ReceiveP_m_timestamp_size++;
  }
}

static inline void CC2420TransmitP_CC2420Receive_sfd(uint32_t time) 
{
  CC2420ReceiveP_CC2420Receive_sfd(time);
}

inline static error_t GpioCaptureC_0_Capture_captureFallingEdge( void ) 
{
  return GpioCaptureC_0_enableCapture(MSP430TIMER_CM_FALLING);
}

static inline error_t CC2420TransmitP_CaptureSFD_captureFallingEdge( void ) 
{
  char unsigned result;
  result = GpioCaptureC_0_Capture_captureFallingEdge();
  return result;
}

inline static cc2420_header_t *CC2420PacketP_CC2420PacketBody_getHeader(message_t *msg) 
{
  return (cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
}

static inline cc2420_header_t *CC2420TransmitP_CC2420PacketBody_getHeader(message_t *msg) 
{
  struct cc2420_header_t  *result;
  result = CC2420PacketP_CC2420PacketBody_getHeader(msg);
  return result;
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

static inline void CC2420TransmitP_BackoffTimer_stop( void ) 
{
  TransformAlarmC_0_Alarm_stop();
}

static inline void Msp430GpioC_1_HplGeneralIO_set( void ) 
{
  HplMsp430GeneralIOP_26_IO_set();
}

inline static void Msp430GpioC_1_GeneralIO_set( void ) 
{
  Msp430GpioC_1_HplGeneralIO_set();
}

static inline void CC2420TransmitP_CSN_set( void ) 
{
  Msp430GpioC_1_GeneralIO_set();
}

static inline cc2420_status_t CC2420TransmitP_TXFIFO_RAM_write(uint8_t offset, uint8_t *data, uint8_t length) 
{
  char unsigned result;
  result = CC2420SpiP_Ram_write(CC2420_RAM_TXFIFO, offset, data, length);
  return result;
}

static inline void Msp430GpioC_1_HplGeneralIO_clr( void ) 
{
  HplMsp430GeneralIOP_26_IO_clr();
}

inline static void Msp430GpioC_1_GeneralIO_clr( void ) 
{
  Msp430GpioC_1_HplGeneralIO_clr();
}

static inline void CC2420TransmitP_CSN_clr( void ) 
{
  Msp430GpioC_1_GeneralIO_clr();
}

inline static uint8_t CC2420PacketP_PacketTimeSyncOffset_get(message_t *msg) 
{
  return ((__nesc_ntoh_leuint8(CC2420PacketP_CC2420PacketBody_getHeader(msg)->length.data) + (sizeof( cc2420_header_t ) - MAC_HEADER_SIZE)) - MAC_FOOTER_SIZE) - sizeof( timesync_radio_t );
}

static inline uint8_t CC2420TransmitP_PacketTimeSyncOffset_get(message_t *msg) 
{
  char unsigned result;
  result = CC2420PacketP_PacketTimeSyncOffset_get(msg);
  return result;
}

__inline static uint8_t __nesc_ntoh_uint8(const void *source) 
{
  const uint8_t *base = source;
  return base[0];
}

__inline static int8_t __nesc_ntoh_int8(const void *source) 
{
  return __nesc_ntoh_uint8(source);
}

inline static cc2420_metadata_t *CC2420PacketP_CC2420PacketBody_getMetadata(message_t *msg) 
{
  return ( cc2420_metadata_t * )(msg->metadata);
}

inline static bool CC2420PacketP_PacketTimeSyncOffset_isSet(message_t *msg) 
{
  return __nesc_ntoh_int8(CC2420PacketP_CC2420PacketBody_getMetadata(msg)->timesync.data);
}

static inline bool CC2420TransmitP_PacketTimeSyncOffset_isSet(message_t *msg) 
{
  char unsigned result;
  result = CC2420PacketP_PacketTimeSyncOffset_isSet(msg);
  return result;
}

inline static void CC2420PacketP_PacketTimeStamp32khz_set(message_t *msg, uint32_t value) 
{
  __nesc_hton_uint32(CC2420PacketP_CC2420PacketBody_getMetadata(msg)->timestamp.data, value);
}

static inline void CC2420TransmitP_PacketTimeStamp_set(message_t *msg, CC2420TransmitP_PacketTimeStamp_size_type value) 
{
  CC2420PacketP_PacketTimeStamp32khz_set(msg, value);
}

static inline CC2420TransmitP_BackoffTimer_size_type CC2420TransmitP_BackoffTimer_getNow( void ) 
{
  long unsigned result;
  result = TransformAlarmC_0_Alarm_getNow();
  return result;
}

__inline static uint32_t CC2420TransmitP_getTime32(uint16_t time) 
{
  uint32_t recent_time = CC2420TransmitP_BackoffTimer_getNow();
  return recent_time + ( int16_t )(time - recent_time);
}

inline static void CC2420TransmitP_CaptureSFD_captured(uint16_t time) 
{
  char unsigned *__nesc_temp44;
  uint32_t time32;
  uint8_t sfd_state = 0;
{
  time32 = CC2420TransmitP_getTime32(time);
  /* IRQ INSTR 32 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 33 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  switch (CC2420TransmitP_m_state) 
  {
    case CC2420TransmitP_S_SFD:
      CC2420TransmitP_m_state = CC2420TransmitP_S_EFD;
    CC2420TransmitP_sfdHigh = TRUE;
    CC2420TransmitP_m_receiving = FALSE;
    CC2420TransmitP_CaptureSFD_captureFallingEdge();
    CC2420TransmitP_PacketTimeStamp_set(CC2420TransmitP_m_msg, time32);
    if(CC2420TransmitP_PacketTimeSyncOffset_isSet(CC2420TransmitP_m_msg))
    {
      uint8_t absOffset = (sizeof( message_header_t ) - sizeof( cc2420_header_t )) + CC2420TransmitP_PacketTimeSyncOffset_get(CC2420TransmitP_m_msg);
      timesync_radio_t *timesync = ( timesync_radio_t * )(( nx_uint8_t * )(CC2420TransmitP_m_msg) + absOffset);
      __nesc_temp44 = (*timesync).data, __nesc_hton_uint32(__nesc_temp44, __nesc_ntoh_uint32(__nesc_temp44) - time32);
      CC2420TransmitP_CSN_clr();
      CC2420TransmitP_TXFIFO_RAM_write(absOffset, ( uint8_t * )(timesync), sizeof( timesync_radio_t ));
      CC2420TransmitP_CSN_set();
    }
    if(__nesc_ntoh_leuint16(CC2420TransmitP_CC2420PacketBody_getHeader(CC2420TransmitP_m_msg)->fcf.data) & (1 << IEEE154_FCF_ACK_REQ))
    {
      CC2420TransmitP_abortSpiRelease = TRUE;
    }
    CC2420TransmitP_releaseSpiResource();
    CC2420TransmitP_BackoffTimer_stop();
    if(CC2420TransmitP_SFD_get())
    {
        break;
    }
    case CC2420TransmitP_S_EFD:
      CC2420TransmitP_sfdHigh = FALSE;
    CC2420TransmitP_CaptureSFD_captureRisingEdge();
    if(__nesc_ntoh_leuint16(CC2420TransmitP_CC2420PacketBody_getHeader(CC2420TransmitP_m_msg)->fcf.data) & (1 << IEEE154_FCF_ACK_REQ))
    {
      CC2420TransmitP_m_state = CC2420TransmitP_S_ACK_WAIT;
      CC2420TransmitP_BackoffTimer_start(CC2420_ACK_WAIT_DELAY);
    }
    else 
    {
      CC2420TransmitP_signalDone(SUCCESS);
    }
    if(!CC2420TransmitP_SFD_get())
    {
        break;
    }
    default :
      if(!CC2420TransmitP_m_receiving && (CC2420TransmitP_sfdHigh == FALSE))
      {
        CC2420TransmitP_sfdHigh = TRUE;
        CC2420TransmitP_CaptureSFD_captureFallingEdge();
        sfd_state = CC2420TransmitP_SFD_get();
        CC2420TransmitP_CC2420Receive_sfd(time32);
        CC2420TransmitP_m_receiving = TRUE;
        CC2420TransmitP_m_prev_time = time;
        if(CC2420TransmitP_SFD_get())
        {
          return;
        }
      }
    if(CC2420TransmitP_sfdHigh == TRUE)
    {
      CC2420TransmitP_sfdHigh = FALSE;
      CC2420TransmitP_CaptureSFD_captureRisingEdge();
      CC2420TransmitP_m_receiving = FALSE;
      if((sfd_state == 0) && ((time - CC2420TransmitP_m_prev_time) < 10))
      {
        CC2420TransmitP_CC2420Receive_sfd_dropped();
        if(CC2420TransmitP_m_msg)
        {
          CC2420TransmitP_PacketTimeStamp_clear(CC2420TransmitP_m_msg);
        }
      }
        break;
    }
  }

}
}

static inline void GpioCaptureC_0_Capture_captured(uint16_t time) 
{
  CC2420TransmitP_CaptureSFD_captured(time);
}

inline static void Msp430TimerCapComP_4_Capture_clearOverflow( void ) 
{
  _TBCCTL1 &= ~0x0002;
}

static inline void GpioCaptureC_0_Msp430Capture_clearOverflow( void ) 
{
  Msp430TimerCapComP_4_Capture_clearOverflow();
}

inline static void Msp430TimerCapComP_4_Control_clearPendingInterrupt( void ) 
{
  _TBCCTL1 &= ~0x0001;
}

static inline void GpioCaptureC_0_Msp430TimerControl_clearPendingInterrupt( void ) 
{
  Msp430TimerCapComP_4_Control_clearPendingInterrupt();
}

inline static void GpioCaptureC_0_Msp430Capture_captured(uint16_t time) 
{
  GpioCaptureC_0_Msp430TimerControl_clearPendingInterrupt();
  GpioCaptureC_0_Msp430Capture_clearOverflow();
  GpioCaptureC_0_Capture_captured(time);
}

static inline void Msp430TimerCapComP_4_Capture_captured(uint16_t time) 
{
  GpioCaptureC_0_Msp430Capture_captured(time);
}

inline static Msp430TimerCapComP_4_cc_t Msp430TimerCapComP_4_int2CC(uint16_t x) 
{
  union Msp430TimerCapComP_4___nesc_unnamed4353  {
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

static inline void TransformAlarmC_1_Alarm_fired( void ) 
{
  AlarmToTimerC_0_Alarm_fired();
}

inline static void TransformAlarmC_1_AlarmFrom_fired( void ) 
{
{
  /* IRQ INSTR 34 */
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
  union Msp430TimerCapComP_5___nesc_unnamed4354  {
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

static inline TransformCounterC_1_CounterFrom_size_type TransformCounterC_1_CounterFrom_get( void ) 
{
  int unsigned result;
  result = Msp430CounterC_0_Counter_get();
  return result;
}

static inline bool TransformCounterC_1_CounterFrom_isOverflowPending( void ) 
{
  char unsigned result;
  result = Msp430CounterC_0_Counter_isOverflowPending();
  return result;
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
  union Msp430TimerCapComP_6___nesc_unnamed4355  {
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
  union Msp430TimerCapComP_7___nesc_unnamed4356  {
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
  union Msp430TimerCapComP_8___nesc_unnamed4357  {
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
  union Msp430TimerCapComP_9___nesc_unnamed4358  {
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
  /* IRQ INSTR 35 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 36 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 37 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  memset(( void * )(SchedulerBasicP_m_next), SchedulerBasicP_NO_TASK, sizeof(SchedulerBasicP_m_next));
  SchedulerBasicP_m_head = SchedulerBasicP_NO_TASK;
  SchedulerBasicP_m_tail = SchedulerBasicP_NO_TASK;
}
}

static inline void RealMainP_Scheduler_init( void ) 
{
  SchedulerBasicP_Scheduler_init();
}

inline static void HplMsp430GeneralIOP_38_IO_set( void ) 
{
  _P5OUT |= 0x01 << 6;
}

static inline void Msp430GpioC_9_HplGeneralIO_set( void ) 
{
  HplMsp430GeneralIOP_38_IO_set();
}

inline static void Msp430GpioC_9_GeneralIO_set( void ) 
{
  Msp430GpioC_9_HplGeneralIO_set();
}

static inline void LedsP_Led2_set( void ) 
{
  Msp430GpioC_9_GeneralIO_set();
}

inline static void HplMsp430GeneralIOP_37_IO_set( void ) 
{
  _P5OUT |= 0x01 << 5;
}

static inline void Msp430GpioC_8_HplGeneralIO_set( void ) 
{
  HplMsp430GeneralIOP_37_IO_set();
}

inline static void Msp430GpioC_8_GeneralIO_set( void ) 
{
  Msp430GpioC_8_HplGeneralIO_set();
}

static inline void LedsP_Led1_set( void ) 
{
  Msp430GpioC_8_GeneralIO_set();
}

inline static void HplMsp430GeneralIOP_36_IO_set( void ) 
{
  _P5OUT |= 0x01 << 4;
}

static inline void Msp430GpioC_7_HplGeneralIO_set( void ) 
{
  HplMsp430GeneralIOP_36_IO_set();
}

inline static void Msp430GpioC_7_GeneralIO_set( void ) 
{
  Msp430GpioC_7_HplGeneralIO_set();
}

static inline void LedsP_Led0_set( void ) 
{
  Msp430GpioC_7_GeneralIO_set();
}

inline static void HplMsp430GeneralIOP_38_IO_makeOutput( void ) 
{
  _P5DIR |= 0x01 << 6;
}

static inline void Msp430GpioC_9_HplGeneralIO_makeOutput( void ) 
{
  HplMsp430GeneralIOP_38_IO_makeOutput();
}

inline static void Msp430GpioC_9_GeneralIO_makeOutput( void ) 
{
  Msp430GpioC_9_HplGeneralIO_makeOutput();
}

static inline void LedsP_Led2_makeOutput( void ) 
{
  Msp430GpioC_9_GeneralIO_makeOutput();
}

inline static void HplMsp430GeneralIOP_37_IO_makeOutput( void ) 
{
  _P5DIR |= 0x01 << 5;
}

static inline void Msp430GpioC_8_HplGeneralIO_makeOutput( void ) 
{
  HplMsp430GeneralIOP_37_IO_makeOutput();
}

inline static void Msp430GpioC_8_GeneralIO_makeOutput( void ) 
{
  Msp430GpioC_8_HplGeneralIO_makeOutput();
}

static inline void LedsP_Led1_makeOutput( void ) 
{
  Msp430GpioC_8_GeneralIO_makeOutput();
}

inline static void HplMsp430GeneralIOP_36_IO_makeOutput( void ) 
{
  _P5DIR |= 0x01 << 4;
}

static inline void Msp430GpioC_7_HplGeneralIO_makeOutput( void ) 
{
  HplMsp430GeneralIOP_36_IO_makeOutput();
}

inline static void Msp430GpioC_7_GeneralIO_makeOutput( void ) 
{
  Msp430GpioC_7_HplGeneralIO_makeOutput();
}

static inline void LedsP_Led0_makeOutput( void ) 
{
  Msp430GpioC_7_GeneralIO_makeOutput();
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

__inline static void MotePlatformC_uwait(uint16_t u) 
{
  uint16_t t0 = _TAR;
  //while((_TAR - t0) <= u)
  //;
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
  MotePlatformC_uwait(1024 * 10);
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

inline static uint16_t Msp430ClockP_test_calib_busywait_delta(int calib) 
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;
  Msp430ClockP_set_dco_calib(calib);
  while(aclk_count-- > 0)
  {
    _TBCCR0 = _TBR + Msp430ClockP_ACLK_CALIB_PERIOD;
    _TBCCTL0 &= ~0x0001;
//    while((_TBCCTL0 & 0x0001) == 0)
//    ;
    dco_prev = dco_curr;
    dco_curr = _TAR;
  }
  return dco_curr - dco_prev;
}

inline static void Msp430ClockP_busyCalibrateDco( void ) 
{
  int calib;
  int step;
  for(calib = 0, step = 0x800; step != 0; step >>= 1)
  {
    if(Msp430ClockP_test_calib_busywait_delta(calib | step) <= Msp430ClockP_TARGET_DCO_DELTA)
    {
      calib |= step;
    }
  }

  if((calib & 0x0e0) == 0x0e0)
  {
    calib &= ~0x01f;
  }
  Msp430ClockP_set_dco_calib(calib);
}

inline static void Msp430ClockP_Msp430ClockInit_defaultSetupDcoCalibrate( void ) 
{
  _TACTL = 0x0200 | 0x0020;
  _TBCTL = 0x0100 | 0x0020;
  _BCSCTL1 = 0x80 | 0x04;
  _BCSCTL2 = 0;
  _TBCCTL0 = 0x4000;
}

inline static void Msp430ClockP_Msp430ClockInit_default_setupDcoCalibrate( void ) 
{
  Msp430ClockP_Msp430ClockInit_defaultSetupDcoCalibrate();
}

static inline void Msp430ClockP_Msp430ClockInit_setupDcoCalibrate( void ) 
{
  Msp430ClockP_Msp430ClockInit_default_setupDcoCalibrate();
}

inline static error_t Msp430ClockP_Init_init( void ) 
{
  _TACTL = 0x0004;
  _TAIV = 0;
  _TBCTL = 0x0004;
  _TBIV = 0;
{
//  Msp430ClockP_Msp430ClockInit_setupDcoCalibrate();
//  Msp430ClockP_busyCalibrateDco();
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

static inline void DisseminatorP_1_DisseminationValue_changed( void ) 
{
  TestDisseminationC_Value16_changed();
}

inline static void DisseminatorP_1_changedTask_runTask( void ) 
{
  DisseminatorP_1_DisseminationValue_changed();
}

const inline static DisseminatorP_1_t *DisseminatorP_1_DisseminationValue_get( void ) 
{
  return &DisseminatorP_1_valueCache;
}

const static inline TestDisseminationC_Value16_t *TestDisseminationC_Value16_get( void ) 
{
  const int unsigned *result;
  result = DisseminatorP_1_DisseminationValue_get();
  return result;
}

static inline error_t TrickleTimerImplP_0_timerTask_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(TrickleTimerImplP_0_timerTask);
  return result;
}

inline static void DisseminationEngineImplP_TrickleTimer_fired(uint16_t key) 
{
  if(!DisseminationEngineImplP_m_running || DisseminationEngineImplP_m_bufBusy)
  {
    return;
  }
  DisseminationEngineImplP_sendObject(key);
}

inline static void TrickleTimerImplP_0_TrickleTimer_default_fired(uint8_t id) 
{
  return;
}

static inline void TrickleTimerImplP_0_TrickleTimer_fired(uint8_t arg_0x22e4c20) 
{
  switch (arg_0x22e4c20) 
  {
    case DisseminatorC_0_TIMER_ID:
      DisseminationEngineImplP_TrickleTimer_fired(4660U);
      break;
    case DisseminatorC_1_TIMER_ID:
      DisseminationEngineImplP_TrickleTimer_fired(9029U);
      break;
    default :
      TrickleTimerImplP_0_TrickleTimer_default_fired(arg_0x22e4c20);
      break;
  }

}

inline static uint16_t BitVectorC_0_getMask(uint16_t bitnum) 
{
  return 1 << (bitnum % BitVectorC_0_ELEMENT_SIZE);
}

inline static uint16_t BitVectorC_0_getIndex(uint16_t bitnum) 
{
  return bitnum / BitVectorC_0_ELEMENT_SIZE;
}

inline static void BitVectorC_0_BitVector_clear(uint16_t bitnum) 
{
  BitVectorC_0_m_bits[BitVectorC_0_getIndex(bitnum)] &= ~BitVectorC_0_getMask(bitnum);
}

static inline void TrickleTimerImplP_0_Pending_clear(uint16_t bitnum) 
{
  BitVectorC_0_BitVector_clear(bitnum);
}

inline static bool BitVectorC_0_BitVector_get(uint16_t bitnum) 
{
  return BitVectorC_0_m_bits[BitVectorC_0_getIndex(bitnum)] & BitVectorC_0_getMask(bitnum)? TRUE : FALSE;
}

static inline bool TrickleTimerImplP_0_Pending_get(uint16_t bitnum) 
{
  char unsigned result;
  result = BitVectorC_0_BitVector_get(bitnum);
  return result;
}

inline static void TrickleTimerImplP_0_timerTask_runTask( void ) 
{
  uint8_t i;
  for(i = 0; i < 2U; i++)
  {
    bool fire = FALSE;
  {
  /* IRQ INSTR 38 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 39 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 40 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 41 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 42 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
    __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  {
    if(TrickleTimerImplP_0_Pending_get(i))
    {
      TrickleTimerImplP_0_Pending_clear(i);
      fire = TRUE;
    }
  }
    __nesc_atomic_end(__nesc_atomic);
  }
    if(fire)
    {
      TrickleTimerImplP_0_TrickleTimer_fired(i);
      TrickleTimerImplP_0_timerTask_postTask();
      return;
    }
  }

}

static inline void *CC2420TinyosNetworkP_SubSend_getPayload(message_t *msg, uint8_t len) 
{
  void *result;
  result = CC2420CsmaP_Send_getPayload(msg, len);
  return result;
}

inline static void *CC2420TinyosNetworkP_Send_getPayload(message_t *msg, uint8_t len) 
{
  return CC2420TinyosNetworkP_SubSend_getPayload(msg, len);
}

static inline void *UniqueSendP_SubSend_getPayload(message_t *msg, uint8_t len) 
{
  void *result;
  result = CC2420TinyosNetworkP_Send_getPayload(msg, len);
  return result;
}

inline static void *UniqueSendP_Send_getPayload(message_t *msg, uint8_t len) 
{
  return UniqueSendP_SubSend_getPayload(msg, len);
}

static inline void *CC2420ActiveMessageP_SubSend_getPayload(message_t *msg, uint8_t len) 
{
  void *result;
  result = UniqueSendP_Send_getPayload(msg, len);
  return result;
}

inline static void *CC2420ActiveMessageP_Packet_getPayload(message_t *msg, uint8_t len) 
{
  return CC2420ActiveMessageP_SubSend_getPayload(msg, len);
}

inline static void *CC2420ActiveMessageP_AMSend_getPayload(am_id_t id, message_t *m, uint8_t len) 
{
  return CC2420ActiveMessageP_Packet_getPayload(m, len);
}

static inline void *AMQueueImplP_0_AMSend_getPayload(am_id_t arg_0x2246108, message_t *msg, uint8_t len) 
{
  void *result;
  result = CC2420ActiveMessageP_AMSend_getPayload(arg_0x2246108, msg, len);
  return result;
}

inline static void *AMQueueImplP_0_Send_getPayload(uint8_t id, message_t *m, uint8_t len) 
{
  return AMQueueImplP_0_AMSend_getPayload(0, m, len);
}

static inline void *AMQueueEntryP_0_Send_getPayload(message_t *msg, uint8_t len) 
{
  void *result;
  result = AMQueueImplP_0_Send_getPayload(0U, msg, len);
  return result;
}

inline static void *AMQueueEntryP_0_AMSend_getPayload(message_t *m, uint8_t len) 
{
  return AMQueueEntryP_0_Send_getPayload(m, len);
}

static inline void *DisseminationEngineImplP_AMSend_getPayload(message_t *msg, uint8_t len) 
{
  void *result;
  result = AMQueueEntryP_0_AMSend_getPayload(msg, len);
  return result;
}

inline static uint8_t CC2420CsmaP_Send_maxPayloadLength( void ) 
{
  return 28;
}

inline static void *DisseminatorP_0_DisseminationCache_requestData(uint8_t *size) 
{
  *size = sizeof( DisseminatorP_0_t );
  return &DisseminatorP_0_valueCache;
}

inline static void *DisseminatorP_1_DisseminationCache_requestData(uint8_t *size) 
{
  *size = sizeof( DisseminatorP_1_t );
  return &DisseminatorP_1_valueCache;
}

inline static void *DisseminationEngineImplP_DisseminationCache_default_requestData(uint16_t key, uint8_t *size) 
{
  return _IE1;
}

static inline void *DisseminationEngineImplP_DisseminationCache_requestData(uint16_t arg_0x21c4120, uint8_t *size) 
{
  void *result;
  switch (arg_0x21c4120) 
  {
    case 4660U:
      result = DisseminatorP_0_DisseminationCache_requestData(size);
      break;
    case 9029U:
      result = DisseminatorP_1_DisseminationCache_requestData(size);
      break;
    default :
      result = DisseminationEngineImplP_DisseminationCache_default_requestData(arg_0x21c4120, size);
      break;
  }

  return result;
}

inline static uint8_t CC2420ActiveMessageP_Packet_maxPayloadLength( void ) 
{
  return 28;
}

inline static uint8_t CC2420ActiveMessageP_AMSend_maxPayloadLength(am_id_t id) 
{
  return CC2420ActiveMessageP_Packet_maxPayloadLength();
}

static inline uint8_t AMQueueImplP_0_AMSend_maxPayloadLength(am_id_t arg_0x2246108) 
{
  char unsigned result;
  result = CC2420ActiveMessageP_AMSend_maxPayloadLength(arg_0x2246108);
  return result;
}

inline static uint8_t AMQueueImplP_0_Send_maxPayloadLength(uint8_t id) 
{
  return AMQueueImplP_0_AMSend_maxPayloadLength(0);
}

static inline uint8_t AMQueueEntryP_0_Send_maxPayloadLength( void ) 
{
  char unsigned result;
  result = AMQueueImplP_0_Send_maxPayloadLength(0U);
  return result;
}

inline static uint8_t AMQueueEntryP_0_AMSend_maxPayloadLength( void ) 
{
  return AMQueueEntryP_0_Send_maxPayloadLength();
}

static inline uint8_t DisseminationEngineImplP_AMSend_maxPayloadLength( void ) 
{
  char unsigned result;
  result = AMQueueEntryP_0_AMSend_maxPayloadLength();
  return result;
}

static inline error_t AMQueueImplP_0_AMSend_send(am_id_t arg_0x2246108, am_addr_t addr, message_t *msg, uint8_t len) 
{
  char unsigned result;
  result = CC2420ActiveMessageP_AMSend_send(arg_0x2246108, addr, msg, len);
  return result;
}

static inline am_addr_t AMQueueImplP_0_AMPacket_destination(message_t *amsg) 
{
  int unsigned result;
  result = CC2420ActiveMessageP_AMPacket_destination(amsg);
  return result;
}

static inline am_id_t AMQueueImplP_0_AMPacket_type(message_t *amsg) 
{
  char unsigned result;
  result = CC2420ActiveMessageP_AMPacket_type(amsg);
  return result;
}

__inline static uint8_t __nesc_hton_leuint8(void *target, uint8_t value) 
{
  uint8_t *base = target;
  base[0] = value;
  return value;
}

static inline cc2420_header_t *CC2420ActiveMessageP_CC2420PacketBody_getHeader(message_t *msg) 
{
  struct cc2420_header_t  *result;
  result = CC2420PacketP_CC2420PacketBody_getHeader(msg);
  return result;
}

inline static void CC2420ActiveMessageP_Packet_setPayloadLength(message_t *msg, uint8_t len) 
{
  __nesc_hton_leuint8(CC2420ActiveMessageP_CC2420PacketBody_getHeader(msg)->length.data, len + CC2420_SIZE);
}

static inline void AMQueueImplP_0_Packet_setPayloadLength(message_t *msg, uint8_t len) 
{
  CC2420ActiveMessageP_Packet_setPayloadLength(msg, len);
}

inline static error_t AMQueueImplP_0_Send_send(uint8_t clientId, message_t *msg, uint8_t len) 
{
  if(clientId >= 2)
  {
    return FAIL;
  }
  if(AMQueueImplP_0_queue[clientId].msg != _IE1)
  {
    return EBUSY;
  }
  AMQueueImplP_0_queue[clientId].msg = msg;
  AMQueueImplP_0_Packet_setPayloadLength(msg, len);
  if(AMQueueImplP_0_current >= 2)
  {
    error_t err;
    am_id_t amId = AMQueueImplP_0_AMPacket_type(msg);
    am_addr_t dest = AMQueueImplP_0_AMPacket_destination(msg);
    AMQueueImplP_0_current = clientId;
    err = AMQueueImplP_0_AMSend_send(amId, dest, msg, len);
    if(err != SUCCESS)
    {
      AMQueueImplP_0_current = 2;
      AMQueueImplP_0_queue[clientId].msg = _IE1;
    }
    return err;
  }
  else 
  {
  }
  return SUCCESS;
}

static inline error_t AMQueueEntryP_0_Send_send(message_t *msg, uint8_t len) 
{
  char unsigned result;
  result = AMQueueImplP_0_Send_send(0U, msg, len);
  return result;
}

inline static void CC2420ActiveMessageP_AMPacket_setType(message_t *amsg, am_id_t type) 
{
  cc2420_header_t *header = CC2420ActiveMessageP_CC2420PacketBody_getHeader(amsg);
  __nesc_hton_leuint8(header->type.data, type);
}

static inline void AMQueueEntryP_0_AMPacket_setType(message_t *amsg, am_id_t t) 
{
  CC2420ActiveMessageP_AMPacket_setType(amsg, t);
}

__inline static uint16_t __nesc_hton_leuint16(void *target, uint16_t value) 
{
  uint8_t *base = target;
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

inline static void CC2420ActiveMessageP_AMPacket_setDestination(message_t *amsg, am_addr_t addr) 
{
  cc2420_header_t *header = CC2420ActiveMessageP_CC2420PacketBody_getHeader(amsg);
  __nesc_hton_leuint16(header->dest.data, addr);
}

static inline void AMQueueEntryP_0_AMPacket_setDestination(message_t *amsg, am_addr_t addr) 
{
  CC2420ActiveMessageP_AMPacket_setDestination(amsg, addr);
}

inline static error_t AMQueueEntryP_0_AMSend_send(am_addr_t dest, message_t *msg, uint8_t len) 
{
  AMQueueEntryP_0_AMPacket_setDestination(msg, dest);
  AMQueueEntryP_0_AMPacket_setType(msg, 96);
  return AMQueueEntryP_0_Send_send(msg, len);
}

static inline error_t DisseminationEngineImplP_AMSend_send(am_addr_t addr, message_t *msg, uint8_t len) 
{
  char unsigned result;
  result = AMQueueEntryP_0_AMSend_send(addr, msg, len);
  return result;
}

inline static uint16_t CC2420ControlP_CC2420Config_getPanAddr( void ) 
{
{
  /* IRQ INSTR 43 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 44 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 45 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 46 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 47 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  int unsigned __nesc_temp = CC2420ControlP_m_pan;
{
  __nesc_atomic_end(__nesc_atomic);
  return __nesc_temp;
}
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static inline uint16_t CC2420ActiveMessageP_CC2420Config_getPanAddr( void ) 
{
  int unsigned result;
  result = CC2420ControlP_CC2420Config_getPanAddr();
  return result;
}

inline static void CC2420ActiveMessageP_SendNotifier_default_aboutToSend(am_id_t amId, am_addr_t addr, message_t *msg) 
{
}

static inline void CC2420ActiveMessageP_SendNotifier_aboutToSend(am_id_t arg_0x16fec58, am_addr_t dest, message_t *msg) 
{
  CC2420ActiveMessageP_SendNotifier_default_aboutToSend(arg_0x16fec58, dest, msg);
}

__inline static uint8_t __nesc_hton_uint8(void *target, uint8_t value) 
{
  uint8_t *base = target;
  base[0] = value;
  return value;
}

__inline static int8_t __nesc_hton_int8(void *target, int8_t value) 
{
  __nesc_hton_uint8(target, value);
  return value;
}

inline static error_t CC2420TransmitP_send(message_t *p_msg, bool cca) 
{
{
  /* IRQ INSTR 52 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 53 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 54 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 55 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 56 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  /* IRQ INSTR 48 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 49 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  if(CC2420TransmitP_m_state == CC2420TransmitP_S_CANCEL)
  {
  {
    char unsigned __nesc_temp = ECANCEL;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
  /* IRQ INSTR 50 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 51 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  if(CC2420TransmitP_m_state != CC2420TransmitP_S_STARTED)
  {
  {
    char unsigned __nesc_temp = FAIL;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
  CC2420TransmitP_m_state = CC2420TransmitP_S_LOAD;
  CC2420TransmitP_m_cca = cca;
  CC2420TransmitP_m_msg = p_msg;
  CC2420TransmitP_totalCcaChecks = 0;
}
  __nesc_atomic_end(__nesc_atomic);
}
  if(CC2420TransmitP_acquireSpiResource() == SUCCESS)
  {
    CC2420TransmitP_loadTXFIFO();
  }
  return SUCCESS;
}

inline static error_t CC2420TransmitP_Send_send(message_t *p_msg, bool useCca) 
{
  return CC2420TransmitP_send(p_msg, useCca);
}

static inline error_t CC2420CsmaP_CC2420Transmit_send(message_t *p_msg, bool useCca) 
{
  char unsigned result;
  result = CC2420TransmitP_Send_send(p_msg, useCca);
  return result;
}

inline static void CC2420ActiveMessageP_RadioBackoff_default_requestCca(am_id_t id, message_t *msg) 
{
}

static inline void CC2420ActiveMessageP_RadioBackoff_requestCca(am_id_t arg_0x16fd2e8, message_t *msg) 
{
  CC2420ActiveMessageP_RadioBackoff_default_requestCca(arg_0x16fd2e8, msg);
}

inline static void CC2420ActiveMessageP_SubBackoff_requestCca(message_t *msg) 
{
  CC2420ActiveMessageP_RadioBackoff_requestCca(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.data), msg);
}

static inline void CC2420CsmaP_RadioBackoff_requestCca(message_t *msg) 
{
  CC2420ActiveMessageP_SubBackoff_requestCca(msg);
}

inline static void StateImplP_State_forceState(uint8_t id, uint8_t reqState) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  StateImplP_state[id] = reqState;
  __nesc_atomic_end(__nesc_atomic);
}
}

static inline void CC2420CsmaP_SplitControlState_forceState(uint8_t reqState) 
{
  StateImplP_State_forceState(1U, reqState);
}

static inline bool CC2420CsmaP_SplitControlState_isState(uint8_t myState) 
{
  char unsigned result;
  result = StateImplP_State_isState(1U, myState);
  return result;
}

static inline cc2420_metadata_t *CC2420CsmaP_CC2420PacketBody_getMetadata(message_t *msg) 
{
  struct cc2420_metadata_t  *result;
  result = CC2420PacketP_CC2420PacketBody_getMetadata(msg);
  return result;
}

static inline cc2420_header_t *CC2420CsmaP_CC2420PacketBody_getHeader(message_t *msg) 
{
  struct cc2420_header_t  *result;
  result = CC2420PacketP_CC2420PacketBody_getHeader(msg);
  return result;
}

inline static error_t CC2420CsmaP_Send_send(message_t *p_msg, uint8_t len) 
{
  char unsigned *__nesc_temp43;
  char unsigned *__nesc_temp42;
  cc2420_header_t *header = CC2420CsmaP_CC2420PacketBody_getHeader(p_msg);
  cc2420_metadata_t *metadata = CC2420CsmaP_CC2420PacketBody_getMetadata(p_msg);
{
  /* IRQ INSTR 57 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 58 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 59 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 60 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 61 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(!CC2420CsmaP_SplitControlState_isState(CC2420CsmaP_S_STARTED))
  {
  {
    char unsigned __nesc_temp = FAIL;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
  CC2420CsmaP_SplitControlState_forceState(CC2420CsmaP_S_TRANSMITTING);
  CC2420CsmaP_m_msg = p_msg;
}
  __nesc_atomic_end(__nesc_atomic);
}
  __nesc_hton_leuint8(header->length.data, len + CC2420_SIZE);
  __nesc_temp42 = header->fcf.data, __nesc_hton_leuint16(__nesc_temp42, __nesc_ntoh_leuint16(__nesc_temp42) & (1 << IEEE154_FCF_ACK_REQ));
  __nesc_temp43 = header->fcf.data, __nesc_hton_leuint16(__nesc_temp43, __nesc_ntoh_leuint16(__nesc_temp43) | ((((IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE) | (1 << IEEE154_FCF_INTRAPAN)) | (IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE)) | (IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE)));
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

static inline error_t CC2420TinyosNetworkP_SubSend_send(message_t *msg, uint8_t len) 
{
  char unsigned result;
  result = CC2420CsmaP_Send_send(msg, len);
  return result;
}

static inline cc2420_header_t *CC2420TinyosNetworkP_CC2420PacketBody_getHeader(message_t *msg) 
{
  struct cc2420_header_t  *result;
  result = CC2420PacketP_CC2420PacketBody_getHeader(msg);
  return result;
}

inline static error_t CC2420TinyosNetworkP_Send_send(message_t *msg, uint8_t len) 
{
  __nesc_hton_leuint8(CC2420TinyosNetworkP_CC2420PacketBody_getHeader(msg)->network.data, 0x3f);
  return CC2420TinyosNetworkP_SubSend_send(msg, len);
}

static inline error_t UniqueSendP_SubSend_send(message_t *msg, uint8_t len) 
{
  char unsigned result;
  result = CC2420TinyosNetworkP_Send_send(msg, len);
  return result;
}

static inline cc2420_header_t *UniqueSendP_CC2420PacketBody_getHeader(message_t *msg) 
{
  struct cc2420_header_t  *result;
  result = CC2420PacketP_CC2420PacketBody_getHeader(msg);
  return result;
}

static inline error_t UniqueSendP_State_requestState(uint8_t reqState) 
{
  char unsigned result;
  result = StateImplP_State_requestState(2U, reqState);
  return result;
}

inline static error_t UniqueSendP_Send_send(message_t *msg, uint8_t len) 
{
  error_t error;
  if(UniqueSendP_State_requestState(UniqueSendP_S_SENDING) == SUCCESS)
  {
    __nesc_hton_leuint8(UniqueSendP_CC2420PacketBody_getHeader(msg)->dsn.data, UniqueSendP_localSendId++);
    if((error = UniqueSendP_SubSend_send(msg, len)) != (SUCCESS))
    {
      UniqueSendP_State_toIdle();
    }
    return error;
  }
  return EBUSY;
}

static inline error_t CC2420ActiveMessageP_SubSend_send(message_t *msg, uint8_t len) 
{
  char unsigned result;
  result = UniqueSendP_Send_send(msg, len);
  return result;
}

static inline cc2420_status_t CC2420TransmitP_TXCTRL_write(uint16_t data) 
{
  char unsigned result;
  result = CC2420SpiP_Reg_write(CC2420_TXCTRL, data);
  return result;
}

static inline error_t CC2420SpiP_SpiPacket_send(uint8_t *txBuf, uint8_t *rxBuf, uint16_t len) 
{
  char unsigned result;
  result = Msp430SpiNoDmaP_0_SpiPacket_send(Msp430Spi0C_0_CLIENT_ID, txBuf, rxBuf, len);
  return result;
}

static inline uint8_t CC2420SpiP_SpiByte_write(uint8_t tx) 
{
  char unsigned result;
  result = Msp430SpiNoDmaP_0_SpiByte_write(tx);
  return result;
}

inline static bool StateImplP_State_isIdle(uint8_t id) 
{
  return StateImplP_State_isState(id, StateImplP_S_IDLE);
}

static inline bool CC2420SpiP_WorkingState_isIdle( void ) 
{
  char unsigned result;
  result = StateImplP_State_isIdle(0U);
  return result;
}

inline static cc2420_status_t CC2420SpiP_Fifo_write(uint8_t addr, uint8_t *data, uint8_t len) 
{
  uint8_t status = 0;
{
  /* IRQ INSTR 62 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 63 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 64 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 65 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 66 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(CC2420SpiP_WorkingState_isIdle())
  {
  {
    char unsigned __nesc_temp = status;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
  CC2420SpiP_m_addr = addr;
  status = CC2420SpiP_SpiByte_write(CC2420SpiP_m_addr);
  CC2420SpiP_SpiPacket_send(data, _IE1, len);
  return status;
}

static inline cc2420_status_t CC2420TransmitP_TXFIFO_write(uint8_t *data, uint8_t length) 
{
  char unsigned result;
  result = CC2420SpiP_Fifo_write(CC2420_TXFIFO, data, length);
  return result;
}

inline static void HplMsp430Usart0P_Usart_enableRxIntr( void ) 
{
{
  /* IRQ INSTR 67 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 68 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 69 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 70 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 71 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  _IFG1 &= ~1 << 6;
  _IE1 |= 1 << 6;
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static inline void Msp430SpiNoDmaP_0_Usart_enableRxIntr( void ) 
{
  HplMsp430Usart0P_Usart_enableRxIntr();
}

static inline error_t Msp430SpiNoDmaP_0_signalDone_task_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(Msp430SpiNoDmaP_0_signalDone_task);
  return result;
}

__inline static uint16_t __nesc_hton_uint16(void *target, uint16_t value) 
{
  uint8_t *base = target;
  base[1] = value;
  base[0] = value >> 8;
  return value;
}

static inline void DisseminatorP_0_DisseminationValue_changed( void ) 
{
  TestDisseminationC_Value32_changed();
}

inline static void DisseminatorP_0_changedTask_runTask( void ) 
{
  DisseminatorP_0_DisseminationValue_changed();
}

inline static void DisseminationEngineImplP_AMSend_sendDone(message_t *msg, error_t error) 
{
  DisseminationEngineImplP_m_bufBusy = FALSE;
}

static inline void AMQueueEntryP_0_AMSend_sendDone(message_t *msg, error_t error) 
{
  DisseminationEngineImplP_AMSend_sendDone(msg, error);
}

inline static void AMQueueEntryP_0_Send_sendDone(message_t *m, error_t err) 
{
  AMQueueEntryP_0_AMSend_sendDone(m, err);
}

inline static void DisseminationEngineImplP_ProbeAMSend_sendDone(message_t *msg, error_t error) 
{
  DisseminationEngineImplP_m_bufBusy = FALSE;
}

static inline void AMQueueEntryP_1_AMSend_sendDone(message_t *msg, error_t error) 
{
  DisseminationEngineImplP_ProbeAMSend_sendDone(msg, error);
}

inline static void AMQueueEntryP_1_Send_sendDone(message_t *m, error_t err) 
{
  AMQueueEntryP_1_AMSend_sendDone(m, err);
}

inline static void AMQueueImplP_0_Send_default_sendDone(uint8_t id, message_t *msg, error_t err) 
{
}

static inline void AMQueueImplP_0_Send_sendDone(uint8_t arg_0x22476f8, message_t *msg, error_t error) 
{
  switch (arg_0x22476f8) 
  {
    case 0U:
      AMQueueEntryP_0_Send_sendDone(msg, error);
      break;
    case 1U:
      AMQueueEntryP_1_Send_sendDone(msg, error);
      break;
    default :
      AMQueueImplP_0_Send_default_sendDone(arg_0x22476f8, msg, error);
      break;
  }

}

inline static void AMQueueImplP_0_CancelTask_runTask( void ) 
{
  uint8_t i;
  uint8_t j;
  uint8_t mask;
  uint8_t last;
  message_t *msg;
  for(i = 0; i < ((2 / 8) + 1); i++)
  {
    if(AMQueueImplP_0_cancelMask[i])
    {
      for(mask = 1, j = 0; j < 8; j++)
      {
        if(AMQueueImplP_0_cancelMask[i] & mask)
        {
          last = (i * 8) + j;
          msg = AMQueueImplP_0_queue[last].msg;
          AMQueueImplP_0_queue[last].msg = _IE1;
          AMQueueImplP_0_cancelMask[i] &= ~mask;
          AMQueueImplP_0_Send_sendDone(last, msg, ECANCEL);
        }
        mask <<= 1;
      }

    }
  }

}

inline static void AMQueueImplP_0_errorTask_runTask( void ) 
{
  AMQueueImplP_0_sendDone(AMQueueImplP_0_current, AMQueueImplP_0_queue[AMQueueImplP_0_current].msg, FAIL);
}

static inline error_t AMQueueImplP_0_errorTask_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(AMQueueImplP_0_errorTask);
  return result;
}

inline static uint8_t CC2420ActiveMessageP_Packet_payloadLength(message_t *msg) 
{
  return __nesc_ntoh_leuint8(CC2420ActiveMessageP_CC2420PacketBody_getHeader(msg)->length.data) - CC2420_SIZE;
}

static inline uint8_t AMQueueImplP_0_Packet_payloadLength(message_t *msg) 
{
  char unsigned result;
  result = CC2420ActiveMessageP_Packet_payloadLength(msg);
  return result;
}

inline static void AMQueueImplP_0_nextPacket( void ) 
{
  uint8_t i;
  AMQueueImplP_0_current = (AMQueueImplP_0_current + 1) % 2;
  for(i = 0; i < 2; i++)
  {
    if((AMQueueImplP_0_queue[AMQueueImplP_0_current].msg == _IE1) || (AMQueueImplP_0_cancelMask[AMQueueImplP_0_current / 8] & (1 << (AMQueueImplP_0_current % 8))))
    {
      AMQueueImplP_0_current = (AMQueueImplP_0_current + 1) % 2;
    }
    else 
    {
        break;
    }
  }

  if(i >= 2)
  {
    AMQueueImplP_0_current = 2;
  }
}

inline static void AMQueueImplP_0_tryToSend( void ) 
{
  AMQueueImplP_0_nextPacket();
  if(AMQueueImplP_0_current < 2)
  {
    error_t nextErr;
    message_t *nextMsg = AMQueueImplP_0_queue[AMQueueImplP_0_current].msg;
    am_id_t nextId = AMQueueImplP_0_AMPacket_type(nextMsg);
    am_addr_t nextDest = AMQueueImplP_0_AMPacket_destination(nextMsg);
    uint8_t len = AMQueueImplP_0_Packet_payloadLength(nextMsg);
    nextErr = AMQueueImplP_0_AMSend_send(nextId, nextDest, nextMsg, len);
    if(nextErr != SUCCESS)
    {
      AMQueueImplP_0_errorTask_postTask();
    }
  }
}

static inline void AlarmToTimerC_0_Alarm_startAt(AlarmToTimerC_0_Alarm_size_type t0, AlarmToTimerC_0_Alarm_size_type dt) 
{
  TransformAlarmC_1_Alarm_startAt(t0, dt);
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

inline static void Msp430AlarmC_1_Alarm_stop( void ) 
{
  Msp430AlarmC_1_Msp430TimerControl_disableEvents();
}

static inline void TransformAlarmC_1_AlarmFrom_stop( void ) 
{
  Msp430AlarmC_1_Alarm_stop();
}

inline static void TransformAlarmC_1_Alarm_stop( void ) 
{
  TransformAlarmC_1_AlarmFrom_stop();
}

static inline void AlarmToTimerC_0_Alarm_stop( void ) 
{
  TransformAlarmC_1_Alarm_stop();
}

inline static void AlarmToTimerC_0_Timer_stop( void ) 
{
  AlarmToTimerC_0_Alarm_stop();
}

static inline void VirtualizeTimerC_0_TimerFrom_stop( void ) 
{
  AlarmToTimerC_0_Timer_stop();
}

static inline TransformAlarmC_1_Counter_size_type TransformAlarmC_1_Counter_get( void ) 
{
  long unsigned result;
  result = TransformCounterC_1_Counter_get();
  return result;
}

inline static TransformAlarmC_1_to_size_type TransformAlarmC_1_Alarm_getNow( void ) 
{
  return TransformAlarmC_1_Counter_get();
}

static inline AlarmToTimerC_0_Alarm_size_type AlarmToTimerC_0_Alarm_getNow( void ) 
{
  long unsigned result;
  result = TransformAlarmC_1_Alarm_getNow();
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

inline static void BitVectorC_0_BitVector_set(uint16_t bitnum) 
{
  BitVectorC_0_m_bits[BitVectorC_0_getIndex(bitnum)] |= BitVectorC_0_getMask(bitnum);
}

static inline void TrickleTimerImplP_0_Pending_set(uint16_t bitnum) 
{
  BitVectorC_0_BitVector_set(bitnum);
}

inline static uint32_t VirtualizeTimerC_0_Timer_getdt(uint8_t num) 
{
  return VirtualizeTimerC_0_m_timers[num].dt;
}

static inline uint32_t TrickleTimerImplP_0_Timer_getdt( void ) 
{
  long unsigned result;
  result = VirtualizeTimerC_0_Timer_getdt(1U);
  return result;
}

inline static void TrickleTimerImplP_0_Timer_fired( void ) 
{
  uint8_t i;
  uint32_t dt = TrickleTimerImplP_0_Timer_getdt();
  for(i = 0; i < 2U; i++)
  {
    uint32_t remaining = TrickleTimerImplP_0_trickles[i].time;
    if(remaining != 0)
    {
      remaining -= dt;
      if(remaining == 0)
      {
        if(TrickleTimerImplP_0_trickles[i].count < 1)
        {
        {
  /* IRQ INSTR 72 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 73 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 74 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 75 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 76 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
          __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          TrickleTimerImplP_0_Pending_set(i);
        }
          __nesc_atomic_end(__nesc_atomic);
        }
          TrickleTimerImplP_0_timerTask_postTask();
        }
        TrickleTimerImplP_0_generateTime(i);
        TrickleTimerImplP_0_trickles[i].count = 0;
      }
      else 
      {
        TrickleTimerImplP_0_trickles[i].time = remaining;
      }
    }
  }

  TrickleTimerImplP_0_adjustTimer();
}

inline static void HplMsp430GeneralIOP_38_IO_toggle( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P5OUT ^= 0x01 << 6;
  __nesc_atomic_end(__nesc_atomic);
}
}

static inline void Msp430GpioC_9_HplGeneralIO_toggle( void ) 
{
  HplMsp430GeneralIOP_38_IO_toggle();
}

inline static void Msp430GpioC_9_GeneralIO_toggle( void ) 
{
  Msp430GpioC_9_HplGeneralIO_toggle();
}

static inline void LedsP_Led2_toggle( void ) 
{
  Msp430GpioC_9_GeneralIO_toggle();
}

inline static void LedsP_Leds_led2Toggle( void ) 
{
  LedsP_Led2_toggle();
}

static inline void TestDisseminationC_Leds_led2Toggle( void ) 
{
  LedsP_Leds_led2Toggle();
}

const inline static DisseminatorP_0_t *DisseminatorP_0_DisseminationValue_get( void ) 
{
  return &DisseminatorP_0_valueCache;
}

const static inline TestDisseminationC_Value32_t *TestDisseminationC_Value32_get( void ) 
{
  const long unsigned *result;
  result = DisseminatorP_0_DisseminationValue_get();
  return result;
}

static inline error_t DisseminatorP_1_changedTask_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(DisseminatorP_1_changedTask);
  return result;
}

static inline void DisseminatorP_1_DisseminationCache_newData( void ) 
{
  DisseminationEngineImplP_DisseminationCache_newData(9029U);
}

inline static void DisseminatorP_1_DisseminationUpdate_change(DisseminatorP_1_t *newVal) 
{
  if(!DisseminatorP_1_m_running)
  {
    return;
  }
  memcpy(&DisseminatorP_1_valueCache, newVal, sizeof( DisseminatorP_1_t ));
  DisseminatorP_1_seqno = DisseminatorP_1_seqno >> 16;
  DisseminatorP_1_seqno++;
  if(DisseminatorP_1_seqno == DISSEMINATION_SEQNO_UNKNOWN)
  {
    DisseminatorP_1_seqno++;
  }
  DisseminatorP_1_seqno = DisseminatorP_1_seqno << 16;
  DisseminatorP_1_seqno += TOS_NODE_ID;
  DisseminatorP_1_DisseminationCache_newData();
  DisseminatorP_1_changedTask_postTask();
}

static inline void TestDisseminationC_Update16_change(TestDisseminationC_Update16_t *newVal) 
{
  DisseminatorP_1_DisseminationUpdate_change(newVal);
}

static inline error_t DisseminatorP_0_changedTask_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(DisseminatorP_0_changedTask);
  return result;
}

static inline void DisseminatorP_0_DisseminationCache_newData( void ) 
{
  DisseminationEngineImplP_DisseminationCache_newData(4660U);
}

inline static void DisseminatorP_0_DisseminationUpdate_change(DisseminatorP_0_t *newVal) 
{
  if(!DisseminatorP_0_m_running)
  {
    return;
  }
  memcpy(&DisseminatorP_0_valueCache, newVal, sizeof( DisseminatorP_0_t ));
  DisseminatorP_0_seqno = DisseminatorP_0_seqno >> 16;
  DisseminatorP_0_seqno++;
  if(DisseminatorP_0_seqno == DISSEMINATION_SEQNO_UNKNOWN)
  {
    DisseminatorP_0_seqno++;
  }
  DisseminatorP_0_seqno = DisseminatorP_0_seqno << 16;
  DisseminatorP_0_seqno += TOS_NODE_ID;
  DisseminatorP_0_DisseminationCache_newData();
  DisseminatorP_0_changedTask_postTask();
}

static inline void TestDisseminationC_Update32_change(TestDisseminationC_Update32_t *newVal) 
{
  DisseminatorP_0_DisseminationUpdate_change(newVal);
}

static inline void Msp430GpioC_8_HplGeneralIO_toggle( void ) 
{
  HplMsp430GeneralIOP_37_IO_toggle();
}

inline static void Msp430GpioC_8_GeneralIO_toggle( void ) 
{
  Msp430GpioC_8_HplGeneralIO_toggle();
}

static inline void LedsP_Led1_toggle( void ) 
{
  Msp430GpioC_8_GeneralIO_toggle();
}

inline static void LedsP_Leds_led1Toggle( void ) 
{
  LedsP_Led1_toggle();
}

static inline void TestDisseminationC_Leds_led1Toggle( void ) 
{
  LedsP_Leds_led1Toggle();
}

static inline void Msp430GpioC_7_HplGeneralIO_toggle( void ) 
{
  HplMsp430GeneralIOP_36_IO_toggle();
}

inline static void Msp430GpioC_7_GeneralIO_toggle( void ) 
{
  Msp430GpioC_7_HplGeneralIO_toggle();
}

static inline void LedsP_Led0_toggle( void ) 
{
  Msp430GpioC_7_GeneralIO_toggle();
}

inline static void LedsP_Leds_led0Toggle( void ) 
{
  LedsP_Led0_toggle();
}

static inline void TestDisseminationC_Leds_led0Toggle( void ) 
{
  LedsP_Leds_led0Toggle();
}

inline static void TestDisseminationC_Timer_fired( void ) 
{
  uint32_t newVal32 = 0xDEADBEEF;
  uint16_t newVal16 = 0xABCD;
  if((TOS_NODE_ID % 4) == 1)
  {
    TestDisseminationC_Leds_led0Toggle();
    TestDisseminationC_Leds_led1Toggle();
    TestDisseminationC_Update32_change(&newVal32);
    TestDisseminationC_Update16_change(&newVal16);
  }
  else 
  {
    const uint32_t *newVal = TestDisseminationC_Value32_get();
    if(*newVal == 123456)
    {
      TestDisseminationC_Leds_led2Toggle();
    }
  }
}

inline static void VirtualizeTimerC_0_Timer_default_fired(uint8_t num) 
{
}

static inline void VirtualizeTimerC_0_Timer_fired(uint8_t arg_0x20a3dd8) 
{
  switch (arg_0x20a3dd8) 
  {
    case 1U:
      TrickleTimerImplP_0_Timer_fired();
      break;
    case 2U:
      TestDisseminationC_Timer_fired();
      break;
    default :
      VirtualizeTimerC_0_Timer_default_fired(arg_0x20a3dd8);
      break;
  }

}

inline static uint16_t BitVectorC_1_getMask(uint16_t bitnum) 
{
  return 1 << (bitnum % BitVectorC_1_ELEMENT_SIZE);
}

inline static uint16_t BitVectorC_1_getIndex(uint16_t bitnum) 
{
  return bitnum / BitVectorC_1_ELEMENT_SIZE;
}

inline static void BitVectorC_1_BitVector_set(uint16_t bitnum) 
{
  BitVectorC_1_m_bits[BitVectorC_1_getIndex(bitnum)] |= BitVectorC_1_getMask(bitnum);
}

static inline void TrickleTimerImplP_0_Changed_set(uint16_t bitnum) 
{
  BitVectorC_1_BitVector_set(bitnum);
}

static inline uint16_t TrickleTimerImplP_0_Random_rand16( void ) 
{
  int unsigned result;
  result = RandomMlcgC_Random_rand16();
  return result;
}

inline static uint32_t VirtualizeTimerC_0_Timer_getNow(uint8_t num) 
{
  return VirtualizeTimerC_0_TimerFrom_getNow();
}

static inline uint32_t TrickleTimerImplP_0_Timer_getNow( void ) 
{
  long unsigned result;
  result = VirtualizeTimerC_0_Timer_getNow(1U);
  return result;
}

inline static uint32_t VirtualizeTimerC_0_Timer_gett0(uint8_t num) 
{
  return VirtualizeTimerC_0_m_timers[num].t0;
}

static inline uint32_t TrickleTimerImplP_0_Timer_gett0( void ) 
{
  long unsigned result;
  result = VirtualizeTimerC_0_Timer_gett0(1U);
  return result;
}

inline static bool BitVectorC_1_BitVector_get(uint16_t bitnum) 
{
  return BitVectorC_1_m_bits[BitVectorC_1_getIndex(bitnum)] & BitVectorC_1_getMask(bitnum)? TRUE : FALSE;
}

static inline bool TrickleTimerImplP_0_Changed_get(uint16_t bitnum) 
{
  char unsigned result;
  result = BitVectorC_1_BitVector_get(bitnum);
  return result;
}

inline static void BitVectorC_1_BitVector_clear(uint16_t bitnum) 
{
  BitVectorC_1_m_bits[BitVectorC_1_getIndex(bitnum)] &= ~BitVectorC_1_getMask(bitnum);
}

static inline void TrickleTimerImplP_0_Changed_clear(uint16_t bitnum) 
{
  BitVectorC_1_BitVector_clear(bitnum);
}

inline static void VirtualizeTimerC_0_Timer_startOneShot(uint8_t num, uint32_t dt) 
{
  VirtualizeTimerC_0_startTimer(num, VirtualizeTimerC_0_TimerFrom_getNow(), dt, TRUE);
}

static inline void TrickleTimerImplP_0_Timer_startOneShot(uint32_t dt) 
{
  VirtualizeTimerC_0_Timer_startOneShot(1U, dt);
}

static inline error_t VirtualizeTimerC_0_updateFromTimer_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(VirtualizeTimerC_0_updateFromTimer);
  return result;
}

inline static void VirtualizeTimerC_0_Timer_stop(uint8_t num) 
{
  VirtualizeTimerC_0_m_timers[num].isrunning = FALSE;
}

static inline void TrickleTimerImplP_0_Timer_stop( void ) 
{
  VirtualizeTimerC_0_Timer_stop(1U);
}

inline static void VirtualizeTimerC_0_TimerFrom_fired( void ) 
{
  VirtualizeTimerC_0_fireTimers(VirtualizeTimerC_0_TimerFrom_getNow());
}

static inline void AlarmToTimerC_0_Timer_fired( void ) 
{
  VirtualizeTimerC_0_TimerFrom_fired();
}

inline static TransformAlarmC_1_to_size_type TransformAlarmC_1_Alarm_getAlarm( void ) 
{
{
  /* IRQ INSTR 78 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 79 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 80 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 81 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 82 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  /* IRQ INSTR 77 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  TransformAlarmC_1_to_size_type __nesc_temp = TransformAlarmC_1_m_t0 + TransformAlarmC_1_m_dt;
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
  result = TransformAlarmC_1_Alarm_getAlarm();
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

inline static message_t *CC2420TinyosNetworkP_NonTinyosReceive_default_receive(uint8_t networkId, message_t *msg, void *payload, uint8_t len) 
{
  return msg;
}

static inline message_t *CC2420TinyosNetworkP_NonTinyosReceive_receive(uint8_t arg_0x218d9b8, message_t *msg, void *payload, uint8_t len) 
{
  struct message_t  *result;
  result = CC2420TinyosNetworkP_NonTinyosReceive_default_receive(arg_0x218d9b8, msg, payload, len);
  return result;
}

inline static message_t *CC2420ActiveMessageP_Snoop_default_receive(am_id_t id, message_t *msg, void *payload, uint8_t len) 
{
  return msg;
}

static inline message_t *CC2420ActiveMessageP_Snoop_receive(am_id_t arg_0x16df220, message_t *msg, void *payload, uint8_t len) 
{
  struct message_t  *result;
  result = CC2420ActiveMessageP_Snoop_default_receive(arg_0x16df220, msg, payload, len);
  return result;
}

__inline static uint16_t __nesc_ntoh_uint16(const void *source) 
{
  const uint8_t *base = source;
  return (( uint16_t )(base[0]) << 8) | base[1];
}

inline static void TrickleTimerImplP_0_TrickleTimer_incrementCounter(uint8_t id) 
{
  TrickleTimerImplP_0_trickles[id].count++;
}

inline static void DisseminationEngineImplP_TrickleTimer_default_incrementCounter(uint16_t key) 
{
}

static inline void DisseminationEngineImplP_TrickleTimer_incrementCounter(uint16_t arg_0x21c4c10) 
{
  switch (arg_0x21c4c10) 
  {
    case 4660U:
      TrickleTimerImplP_0_TrickleTimer_incrementCounter(DisseminatorC_0_TIMER_ID);
      break;
    case 9029U:
      TrickleTimerImplP_0_TrickleTimer_incrementCounter(DisseminatorC_1_TIMER_ID);
      break;
    default :
      DisseminationEngineImplP_TrickleTimer_default_incrementCounter(arg_0x21c4c10);
      break;
  }

}

inline static void DisseminationEngineImplP_TrickleTimer_default_reset(uint16_t key) 
{
}

static inline void DisseminationEngineImplP_TrickleTimer_reset(uint16_t arg_0x21c4c10) 
{
  switch (arg_0x21c4c10) 
  {
    case 4660U:
      TrickleTimerImplP_0_TrickleTimer_reset(DisseminatorC_0_TIMER_ID);
      break;
    case 9029U:
      TrickleTimerImplP_0_TrickleTimer_reset(DisseminatorC_1_TIMER_ID);
      break;
    default :
      DisseminationEngineImplP_TrickleTimer_default_reset(arg_0x21c4c10);
      break;
  }

}

inline static void DisseminationEngineImplP_DisseminationCache_default_storeData(uint16_t key, void *data, uint8_t size, uint32_t seqno) 
{
}

static inline void DisseminationEngineImplP_DisseminationCache_storeData(uint16_t arg_0x21c4120, void *data, uint8_t size, uint32_t seqno) 
{
  switch (arg_0x21c4120) 
  {
    case 4660U:
      DisseminatorP_0_DisseminationCache_storeData(data, size, seqno);
      break;
    case 9029U:
      DisseminatorP_1_DisseminationCache_storeData(data, size, seqno);
      break;
    default :
      DisseminationEngineImplP_DisseminationCache_default_storeData(arg_0x21c4120, data, size, seqno);
      break;
  }

}

inline static uint32_t DisseminatorP_0_DisseminationCache_requestSeqno( void ) 
{
  return DisseminatorP_0_seqno;
}

inline static uint32_t DisseminatorP_1_DisseminationCache_requestSeqno( void ) 
{
  return DisseminatorP_1_seqno;
}

inline static uint32_t DisseminationEngineImplP_DisseminationCache_default_requestSeqno(uint16_t key) 
{
  return DISSEMINATION_SEQNO_UNKNOWN;
}

static inline uint32_t DisseminationEngineImplP_DisseminationCache_requestSeqno(uint16_t arg_0x21c4120) 
{
  long unsigned result;
  switch (arg_0x21c4120) 
  {
    case 4660U:
      result = DisseminatorP_0_DisseminationCache_requestSeqno();
      break;
    case 9029U:
      result = DisseminatorP_1_DisseminationCache_requestSeqno();
      break;
    default :
      result = DisseminationEngineImplP_DisseminationCache_default_requestSeqno(arg_0x21c4120);
      break;
  }

  return result;
}

inline static message_t *DisseminationEngineImplP_Receive_receive(message_t *msg, void *payload, uint8_t len) 
{
  dissemination_message_t *dMsg = ( dissemination_message_t * )(payload);
  uint16_t key = __nesc_ntoh_uint16(dMsg->key.data);
  uint32_t incomingSeqno = __nesc_ntoh_uint32(dMsg->seqno.data);
  uint32_t currentSeqno = DisseminationEngineImplP_DisseminationCache_requestSeqno(key);
  if(!DisseminationEngineImplP_m_running)
  {
    return msg;
  }
  if((currentSeqno == DISSEMINATION_SEQNO_UNKNOWN) && (incomingSeqno != DISSEMINATION_SEQNO_UNKNOWN))
  {
    DisseminationEngineImplP_DisseminationCache_storeData(key, dMsg->data, len - sizeof( dissemination_message_t ), incomingSeqno);
    DisseminationEngineImplP_TrickleTimer_reset(key);
    return msg;
  }
  if((incomingSeqno == DISSEMINATION_SEQNO_UNKNOWN) && (currentSeqno != DISSEMINATION_SEQNO_UNKNOWN))
  {
    DisseminationEngineImplP_TrickleTimer_reset(key);
    return msg;
  }
  if(( int32_t )(incomingSeqno - currentSeqno) > 0)
  {
    DisseminationEngineImplP_DisseminationCache_storeData(key, dMsg->data, len - sizeof( dissemination_message_t ), incomingSeqno);
    DisseminationEngineImplP_TrickleTimer_reset(key);
  }
  else 
  {
    if(( int32_t )(incomingSeqno - currentSeqno) == 0)
    {
      DisseminationEngineImplP_TrickleTimer_incrementCounter(key);
    }
    else 
    {
      DisseminationEngineImplP_sendObject(key);
    }
  }
  return msg;
}

inline static message_t *DisseminationEngineImplP_ProbeReceive_receive(message_t *msg, void *payload, uint8_t len) 
{
  dissemination_probe_message_t *dpMsg = ( dissemination_probe_message_t * )(payload);
  if(!DisseminationEngineImplP_m_running)
  {
    return msg;
  }
  if(DisseminationEngineImplP_DisseminationCache_requestSeqno(__nesc_ntoh_uint16(dpMsg->key.data)) != DISSEMINATION_SEQNO_UNKNOWN)
  {
    DisseminationEngineImplP_sendObject(__nesc_ntoh_uint16(dpMsg->key.data));
  }
  return msg;
}

inline static message_t *CC2420ActiveMessageP_Receive_default_receive(am_id_t id, message_t *msg, void *payload, uint8_t len) 
{
  return msg;
}

static inline message_t *CC2420ActiveMessageP_Receive_receive(am_id_t arg_0x16e1b78, message_t *msg, void *payload, uint8_t len) 
{
  struct message_t  *result;
  switch (arg_0x16e1b78) 
  {
    case 96:
      result = DisseminationEngineImplP_Receive_receive(msg, payload, len);
      break;
    case 97:
      result = DisseminationEngineImplP_ProbeReceive_receive(msg, payload, len);
      break;
    default :
      result = CC2420ActiveMessageP_Receive_default_receive(arg_0x16e1b78, msg, payload, len);
      break;
  }

  return result;
}

inline static am_addr_t ActiveMessageAddressC_ActiveMessageAddress_amAddress( void ) 
{
  return ActiveMessageAddressC_amAddress();
}

static inline am_addr_t CC2420ActiveMessageP_ActiveMessageAddress_amAddress( void ) 
{
  int unsigned result;
  result = ActiveMessageAddressC_ActiveMessageAddress_amAddress();
  return result;
}

inline static am_addr_t CC2420ActiveMessageP_AMPacket_address( void ) 
{
  return CC2420ActiveMessageP_ActiveMessageAddress_amAddress();
}

inline static bool CC2420ActiveMessageP_AMPacket_isForMe(message_t *amsg) 
{
  return (CC2420ActiveMessageP_AMPacket_destination(amsg) == CC2420ActiveMessageP_AMPacket_address()) || (CC2420ActiveMessageP_AMPacket_destination(amsg) == AM_BROADCAST_ADDR);
}

static inline cc2420_metadata_t *CC2420ActiveMessageP_CC2420PacketBody_getMetadata(message_t *msg) 
{
  struct cc2420_metadata_t  *result;
  result = CC2420PacketP_CC2420PacketBody_getMetadata(msg);
  return result;
}

inline static message_t *CC2420ActiveMessageP_SubReceive_receive(message_t *msg, void *payload, uint8_t len) 
{
  if(!__nesc_ntoh_int8(CC2420ActiveMessageP_CC2420PacketBody_getMetadata(msg)->crc.data))
  {
    return msg;
  }
  if(CC2420ActiveMessageP_AMPacket_isForMe(msg))
  {
    return CC2420ActiveMessageP_Receive_receive(CC2420ActiveMessageP_AMPacket_type(msg), msg, payload, len);
  }
  else 
  {
    return CC2420ActiveMessageP_Snoop_receive(CC2420ActiveMessageP_AMPacket_type(msg), msg, payload, len);
  }
}

static inline message_t *UniqueReceiveP_Receive_receive(message_t *msg, void *payload, uint8_t len) 
{
  struct message_t  *result;
  result = CC2420ActiveMessageP_SubReceive_receive(msg, payload, len);
  return result;
}

inline static void UniqueReceiveP_insert(uint16_t msgSource, uint8_t msgDsn) 
{
  uint8_t element = UniqueReceiveP_recycleSourceElement;
  bool increment = FALSE;
{
  /* IRQ INSTR 83 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 84 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 85 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 86 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 87 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if((element == UniqueReceiveP_INVALID_ELEMENT) || (UniqueReceiveP_writeIndex == element))
  {
    element = UniqueReceiveP_writeIndex;
    increment = TRUE;
  }
  UniqueReceiveP_receivedMessages[element].source = msgSource;
  UniqueReceiveP_receivedMessages[element].dsn = msgDsn;
  if(increment)
  {
    UniqueReceiveP_writeIndex++;
    UniqueReceiveP_writeIndex %= 4;
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
}

inline static message_t *UniqueReceiveP_DuplicateReceive_default_receive(message_t *msg, void *payload, uint8_t len) 
{
  return msg;
}

static inline message_t *UniqueReceiveP_DuplicateReceive_receive(message_t *msg, void *payload, uint8_t len) 
{
  struct message_t  *result;
  result = UniqueReceiveP_DuplicateReceive_default_receive(msg, payload, len);
  return result;
}

inline static bool UniqueReceiveP_hasSeen(uint16_t msgSource, uint8_t msgDsn) 
{
  int i;
  UniqueReceiveP_recycleSourceElement = UniqueReceiveP_INVALID_ELEMENT;
{
  /* IRQ INSTR 88 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 89 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 90 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 91 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 92 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  for(i = 0; i < 4; i++)
  {
    if(UniqueReceiveP_receivedMessages[i].source == msgSource)
    {
      if(UniqueReceiveP_receivedMessages[i].dsn == msgDsn)
      {
      {
        char unsigned __nesc_temp = TRUE;
      {
        __nesc_atomic_end(__nesc_atomic);
        return __nesc_temp;
      }
      }
      }
      UniqueReceiveP_recycleSourceElement = i;
    }
  }

}
  __nesc_atomic_end(__nesc_atomic);
}
  return FALSE;
}

static inline cc2420_header_t *UniqueReceiveP_CC2420PacketBody_getHeader(message_t *msg) 
{
  struct cc2420_header_t  *result;
  result = CC2420PacketP_CC2420PacketBody_getHeader(msg);
  return result;
}

inline static message_t *UniqueReceiveP_SubReceive_receive(message_t *msg, void *payload, uint8_t len) 
{
  uint16_t msgSource = __nesc_ntoh_leuint16(UniqueReceiveP_CC2420PacketBody_getHeader(msg)->src.data);
  uint8_t msgDsn = __nesc_ntoh_leuint8(UniqueReceiveP_CC2420PacketBody_getHeader(msg)->dsn.data);
  if(UniqueReceiveP_hasSeen(msgSource, msgDsn))
  {
    return UniqueReceiveP_DuplicateReceive_receive(msg, payload, len);
  }
  else 
  {
    UniqueReceiveP_insert(msgSource, msgDsn);
    return UniqueReceiveP_Receive_receive(msg, payload, len);
  }
}

static inline message_t *CC2420TinyosNetworkP_Receive_receive(message_t *msg, void *payload, uint8_t len) 
{
  struct message_t  *result;
  result = UniqueReceiveP_SubReceive_receive(msg, payload, len);
  return result;
}

inline static message_t *CC2420TinyosNetworkP_SubReceive_receive(message_t *msg, void *payload, uint8_t len) 
{
  if(__nesc_ntoh_leuint8(CC2420TinyosNetworkP_CC2420PacketBody_getHeader(msg)->network.data) == 0x3f)
  {
    return CC2420TinyosNetworkP_Receive_receive(msg, payload, len);
  }
  else 
  {
    return CC2420TinyosNetworkP_NonTinyosReceive_receive(__nesc_ntoh_leuint8(CC2420TinyosNetworkP_CC2420PacketBody_getHeader(msg)->network.data), msg, payload, len);
  }
}

static inline message_t *CC2420ReceiveP_Receive_receive(message_t *msg, void *payload, uint8_t len) 
{
  struct message_t  *result;
  result = CC2420TinyosNetworkP_SubReceive_receive(msg, payload, len);
  return result;
}

static inline uint16_t CC2420ReceiveP_CC2420Config_getShortAddr( void ) 
{
  int unsigned result;
  result = CC2420ControlP_CC2420Config_getShortAddr();
  return result;
}

inline static bool CC2420ControlP_CC2420Config_isAddressRecognitionEnabled( void ) 
{
{
  /* IRQ INSTR 93 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 94 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 95 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 96 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 97 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  char unsigned __nesc_temp = CC2420ControlP_addressRecognition;
{
  __nesc_atomic_end(__nesc_atomic);
  return __nesc_temp;
}
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static inline bool CC2420ReceiveP_CC2420Config_isAddressRecognitionEnabled( void ) 
{
  char unsigned result;
  result = CC2420ControlP_CC2420Config_isAddressRecognitionEnabled();
  return result;
}

static inline cc2420_header_t *CC2420ReceiveP_CC2420PacketBody_getHeader(message_t *msg) 
{
  struct cc2420_header_t  *result;
  result = CC2420PacketP_CC2420PacketBody_getHeader(msg);
  return result;
}

inline static bool CC2420ReceiveP_passesAddressCheck(message_t *msg) 
{
  cc2420_header_t *header = CC2420ReceiveP_CC2420PacketBody_getHeader(msg);
  if(!CC2420ReceiveP_CC2420Config_isAddressRecognitionEnabled())
  {
    return TRUE;
  }
  return (__nesc_ntoh_leuint16(header->dest.data) == CC2420ReceiveP_CC2420Config_getShortAddr()) || (__nesc_ntoh_leuint16(header->dest.data) == AM_BROADCAST_ADDR);
}

static inline cc2420_metadata_t *CC2420ReceiveP_CC2420PacketBody_getMetadata(message_t *msg) 
{
  struct cc2420_metadata_t  *result;
  result = CC2420PacketP_CC2420PacketBody_getMetadata(msg);
  return result;
}

inline static void CC2420ReceiveP_receiveDone_task_runTask( void ) 
{
  cc2420_metadata_t *metadata = CC2420ReceiveP_CC2420PacketBody_getMetadata(CC2420ReceiveP_m_p_rx_buf);
  cc2420_header_t *header = CC2420ReceiveP_CC2420PacketBody_getHeader(CC2420ReceiveP_m_p_rx_buf);
  uint8_t length = __nesc_ntoh_leuint8(header->length.data);
  uint8_t tmpLen = sizeof(message_t ) - ((size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t *buf = ( uint8_t * )(header);
  __nesc_hton_int8(metadata->crc.data, buf[length] >> 7);
  __nesc_hton_uint8(metadata->lqi.data, buf[length] & 0x7f);
  __nesc_hton_uint8(metadata->rssi.data, buf[length - 1]);
  if(CC2420ReceiveP_passesAddressCheck(CC2420ReceiveP_m_p_rx_buf) && (length >= CC2420_SIZE))
  {
    CC2420ReceiveP_m_p_rx_buf = CC2420ReceiveP_Receive_receive(CC2420ReceiveP_m_p_rx_buf, CC2420ReceiveP_m_p_rx_buf->data, length - CC2420_SIZE);
  }
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  CC2420ReceiveP_receivingPacket = FALSE;
  __nesc_atomic_end(__nesc_atomic);
}
  CC2420ReceiveP_waitForNextPacket();
}

inline static uint8_t CC2420SpiP_Resource_isOwner(uint8_t id) 
{
{
  /* IRQ INSTR 98 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 99 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 100 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  char unsigned __nesc_temp = CC2420SpiP_m_holder == id;
  return __nesc_temp;
}
}

static inline bool CC2420ReceiveP_SpiResource_isOwner( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Resource_isOwner(CC2420SpiC_4_CLIENT_ID);
  return result;
}

static inline error_t CC2420ReceiveP_SpiResource_immediateRequest( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Resource_immediateRequest(CC2420SpiC_4_CLIENT_ID);
  return result;
}

static inline error_t CC2420ReceiveP_SpiResource_request( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Resource_request(CC2420SpiC_4_CLIENT_ID);
  return result;
}

static inline error_t CC2420SpiP_grant_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(CC2420SpiP_grant);
  return result;
}

inline static void CC2420SpiP_SpiResource_granted( void ) 
{
  CC2420SpiP_grant_postTask();
}

inline static void Msp430SpiNoDmaP_0_Resource_default_granted(uint8_t id) 
{
}

static inline void Msp430SpiNoDmaP_0_Resource_granted(uint8_t arg_0x1c691b8) 
{
  switch (arg_0x1c691b8) 
  {
    case Msp430Spi0C_0_CLIENT_ID:
      CC2420SpiP_SpiResource_granted();
      break;
    default :
      Msp430SpiNoDmaP_0_Resource_default_granted(arg_0x1c691b8);
      break;
  }

}

inline static void Msp430SpiNoDmaP_0_UsartResource_granted(uint8_t id) 
{
  Msp430SpiNoDmaP_0_Resource_granted(id);
}

inline static void ArbiterP_0_Resource_default_granted(uint8_t id) 
{
}

static inline void ArbiterP_0_Resource_granted(uint8_t arg_0x1db7230) 
{
  switch (arg_0x1db7230) 
  {
    case Msp430Usart0C_0_CLIENT_ID:
      Msp430SpiNoDmaP_0_UsartResource_granted(Msp430Spi0C_0_CLIENT_ID);
      break;
    default :
      ArbiterP_0_Resource_default_granted(arg_0x1db7230);
      break;
  }

}

inline static void ArbiterP_0_grantedTask_runTask( void ) 
{
{
  /* IRQ INSTR 104 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 105 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 106 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 107 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 108 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  /* IRQ INSTR 101 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 102 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 103 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  ArbiterP_0_resId = ArbiterP_0_reqResId;
  ArbiterP_0_state = ArbiterP_0_RES_BUSY;
}
  __nesc_atomic_end(__nesc_atomic);
}
  ArbiterP_0_ResourceConfigure_configure(ArbiterP_0_resId);
  ArbiterP_0_Resource_granted(ArbiterP_0_resId);
}

inline static void Msp430SpiNoDmaP_0_SpiPacket_default_sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error) 
{
}

static inline void Msp430SpiNoDmaP_0_SpiPacket_sendDone(uint8_t arg_0x1c67740, uint8_t *txBuf, uint8_t *rxBuf, uint16_t len, error_t error) 
{
  switch (arg_0x1c67740) 
  {
    case Msp430Spi0C_0_CLIENT_ID:
      CC2420SpiP_SpiPacket_sendDone(txBuf, rxBuf, len, error);
      break;
    default :
      Msp430SpiNoDmaP_0_SpiPacket_default_sendDone(arg_0x1c67740, txBuf, rxBuf, len, error);
      break;
  }

}

inline static void Msp430SpiNoDmaP_0_signalDone( void ) 
{
  /* IRQ INSTR 109 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 110 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  Msp430SpiNoDmaP_0_SpiPacket_sendDone(Msp430SpiNoDmaP_0_m_client, Msp430SpiNoDmaP_0_m_tx_buf, Msp430SpiNoDmaP_0_m_rx_buf, Msp430SpiNoDmaP_0_m_len, SUCCESS);
}

inline static void Msp430SpiNoDmaP_0_signalDone_task_runTask( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  Msp430SpiNoDmaP_0_signalDone();
  __nesc_atomic_end(__nesc_atomic);
}
}

inline static void CC2420TransmitP_TXFIFO_readDone(uint8_t *tx_buf, uint8_t tx_len, error_t error) 
{
}

static inline error_t CC2420ReceiveP_SpiResource_release( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Resource_release(CC2420SpiC_4_CLIENT_ID);
  return result;
}

static inline void CC2420ReceiveP_CSN_set( void ) 
{
  Msp430GpioC_1_GeneralIO_set();
}

static inline error_t CC2420ReceiveP_receiveDone_task_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(CC2420ReceiveP_receiveDone_task);
  return result;
}

static inline cc2420_metadata_t *CC2420TransmitP_CC2420PacketBody_getMetadata(message_t *msg) 
{
  struct cc2420_metadata_t  *result;
  result = CC2420PacketP_CC2420PacketBody_getMetadata(msg);
  return result;
}

inline static void CC2420TransmitP_CC2420Receive_receive(uint8_t type, message_t *ack_msg) 
{
  cc2420_header_t *ack_header;
  cc2420_header_t *msg_header;
  cc2420_metadata_t *msg_metadata;
  uint8_t *ack_buf;
  uint8_t length;
  if((type == IEEE154_TYPE_ACK) && CC2420TransmitP_m_msg)
  {
    ack_header = CC2420TransmitP_CC2420PacketBody_getHeader(ack_msg);
    msg_header = CC2420TransmitP_CC2420PacketBody_getHeader(CC2420TransmitP_m_msg);
  /* IRQ INSTR 111 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 112 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    if((CC2420TransmitP_m_state == CC2420TransmitP_S_ACK_WAIT) && (__nesc_ntoh_leuint8(msg_header->dsn.data) == __nesc_ntoh_leuint8(ack_header->dsn.data)))
    {
      CC2420TransmitP_BackoffTimer_stop();
      msg_metadata = CC2420TransmitP_CC2420PacketBody_getMetadata(CC2420TransmitP_m_msg);
      ack_buf = ( uint8_t * )(ack_header);
      length = __nesc_ntoh_leuint8(ack_header->length.data);
      __nesc_hton_int8(msg_metadata->ack.data, TRUE);
      __nesc_hton_uint8(msg_metadata->rssi.data, ack_buf[length - 1]);
      __nesc_hton_uint8(msg_metadata->lqi.data, ack_buf[length] & 0x7f);
      CC2420TransmitP_signalDone(SUCCESS);
    }
  }
}

static inline void CC2420ReceiveP_CC2420Receive_receive(uint8_t type, message_t *message) 
{
  CC2420TransmitP_CC2420Receive_receive(type, message);
}

static inline void CC2420ReceiveP_PacketTimeStamp_clear(message_t *msg) 
{
  CC2420PacketP_PacketTimeStamp32khz_clear(msg);
}

static inline void CC2420ReceiveP_PacketTimeStamp_set(message_t *msg, CC2420ReceiveP_PacketTimeStamp_size_type value) 
{
  CC2420PacketP_PacketTimeStamp32khz_set(msg, value);
}

inline static uint8_t HplMsp430GeneralIOP_0_IO_getRaw( void ) 
{
  return _P1IN & (0x01 << 0);
}

inline static bool HplMsp430GeneralIOP_0_IO_get( void ) 
{
  return HplMsp430GeneralIOP_0_IO_getRaw() != 0;
}

static inline bool Msp430GpioC_3_HplGeneralIO_get( void ) 
{
  char unsigned result;
  result = HplMsp430GeneralIOP_0_IO_get();
  return result;
}

inline static bool Msp430GpioC_3_GeneralIO_get( void ) 
{
  return Msp430GpioC_3_HplGeneralIO_get();
}

static inline bool CC2420ReceiveP_FIFOP_get( void ) 
{
  char unsigned result;
  result = Msp430GpioC_3_GeneralIO_get();
  return result;
}

inline static uint8_t HplMsp430GeneralIOP_3_IO_getRaw( void ) 
{
  return _P1IN & (0x01 << 3);
}

inline static bool HplMsp430GeneralIOP_3_IO_get( void ) 
{
  return HplMsp430GeneralIOP_3_IO_getRaw() != 0;
}

static inline bool Msp430GpioC_2_HplGeneralIO_get( void ) 
{
  char unsigned result;
  result = HplMsp430GeneralIOP_3_IO_get();
  return result;
}

inline static bool Msp430GpioC_2_GeneralIO_get( void ) 
{
  return Msp430GpioC_2_HplGeneralIO_get();
}

static inline bool CC2420ReceiveP_FIFO_get( void ) 
{
  char unsigned result;
  result = Msp430GpioC_2_GeneralIO_get();
  return result;
}

inline static error_t CC2420SpiP_Fifo_continueRead(uint8_t addr, uint8_t *data, uint8_t len) 
{
  return CC2420SpiP_SpiPacket_send(_IE1, data, len);
}

static inline error_t CC2420ReceiveP_RXFIFO_continueRead(uint8_t *data, uint8_t length) 
{
  char unsigned result;
  result = CC2420SpiP_Fifo_continueRead(CC2420_RXFIFO, data, length);
  return result;
}

static inline cc2420_status_t CC2420ReceiveP_RXFIFO_beginRead(uint8_t *data, uint8_t length) 
{
  char unsigned result;
  result = CC2420SpiP_Fifo_beginRead(CC2420_RXFIFO, data, length);
  return result;
}

static inline void CC2420ReceiveP_CSN_clr( void ) 
{
  Msp430GpioC_1_GeneralIO_clr();
}

static inline cc2420_status_t CC2420ReceiveP_SACK_strobe( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Strobe_strobe(CC2420_SACK);
  return result;
}

inline static bool CC2420ControlP_CC2420Config_isHwAutoAckDefault( void ) 
{
{
  char unsigned __nesc_temp = CC2420ControlP_hwAutoAckDefault;
  return __nesc_temp;
}
}

static inline bool CC2420ReceiveP_CC2420Config_isHwAutoAckDefault( void ) 
{
  char unsigned result;
  result = CC2420ControlP_CC2420Config_isHwAutoAckDefault();
  return result;
}

inline static bool CC2420ControlP_CC2420Config_isAutoAckEnabled( void ) 
{
{
  char unsigned __nesc_temp = CC2420ControlP_autoAckEnabled;
  return __nesc_temp;
}
}

static inline bool CC2420ReceiveP_CC2420Config_isAutoAckEnabled( void ) 
{
  char unsigned result;
  result = CC2420ControlP_CC2420Config_isAutoAckEnabled();
  return result;
}

inline static void CC2420ReceiveP_RXFIFO_readDone(uint8_t *rx_buf, uint8_t rx_len, error_t error) 
{
  cc2420_header_t *header = CC2420ReceiveP_CC2420PacketBody_getHeader(CC2420ReceiveP_m_p_rx_buf);
  uint8_t tmpLen = sizeof(message_t ) - ((size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t *buf = ( uint8_t * )(header);
  CC2420ReceiveP_rxFrameLength = buf[0];
  /* IRQ INSTR 125 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 126 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  switch (CC2420ReceiveP_m_state) 
  {
    case CC2420ReceiveP_S_RX_LENGTH:
      CC2420ReceiveP_m_state = CC2420ReceiveP_S_RX_FCF;
  /* IRQ INSTR 118 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
    if((CC2420ReceiveP_rxFrameLength + 1) > CC2420ReceiveP_m_bytes_left)
    {
      CC2420ReceiveP_flush();
    }
    else 
    {
      if(!CC2420ReceiveP_FIFO_get() && !CC2420ReceiveP_FIFOP_get())
      {
  /* IRQ INSTR 113 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
        CC2420ReceiveP_m_bytes_left -= CC2420ReceiveP_rxFrameLength + 1;
      }
  /* IRQ INSTR 117 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
      if(CC2420ReceiveP_rxFrameLength <= MAC_PACKET_SIZE)
      {
  /* IRQ INSTR 116 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
        if(CC2420ReceiveP_rxFrameLength > 0)
        {
  /* IRQ INSTR 115 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
          if(CC2420ReceiveP_rxFrameLength > CC2420ReceiveP_SACK_HEADER_LENGTH)
          {
            CC2420ReceiveP_RXFIFO_continueRead(buf + 1, CC2420ReceiveP_SACK_HEADER_LENGTH);
          }
          else 
          {
            CC2420ReceiveP_m_state = CC2420ReceiveP_S_RX_PAYLOAD;
  /* IRQ INSTR 114 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
            CC2420ReceiveP_RXFIFO_continueRead(buf + 1, CC2420ReceiveP_rxFrameLength);
          }
        }
        else 
        {
          CC2420ReceiveP_receivingPacket = FALSE;
          CC2420ReceiveP_CSN_set();
          CC2420ReceiveP_SpiResource_release();
          CC2420ReceiveP_waitForNextPacket();
        }
      }
      else 
      {
        CC2420ReceiveP_flush();
      }
    }
      break;
    case CC2420ReceiveP_S_RX_FCF:
      CC2420ReceiveP_m_state = CC2420ReceiveP_S_RX_PAYLOAD;
    if(CC2420ReceiveP_CC2420Config_isAutoAckEnabled() && !CC2420ReceiveP_CC2420Config_isHwAutoAckDefault())
    {
      if(((((__nesc_ntoh_leuint16(header->fcf.data) >> IEEE154_FCF_ACK_REQ) & 0x01) == 1) && ((__nesc_ntoh_leuint16(header->dest.data) == CC2420ReceiveP_CC2420Config_getShortAddr()) || (__nesc_ntoh_leuint16(header->dest.data) == AM_BROADCAST_ADDR))) && (((__nesc_ntoh_leuint16(header->fcf.data) >> IEEE154_FCF_FRAME_TYPE) & 7) == IEEE154_TYPE_DATA))
      {
        CC2420ReceiveP_CSN_set();
        CC2420ReceiveP_CSN_clr();
        CC2420ReceiveP_SACK_strobe();
        CC2420ReceiveP_CSN_set();
        CC2420ReceiveP_CSN_clr();
  /* IRQ INSTR 119 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
        CC2420ReceiveP_RXFIFO_beginRead((buf + 1) + CC2420ReceiveP_SACK_HEADER_LENGTH, CC2420ReceiveP_rxFrameLength - CC2420ReceiveP_SACK_HEADER_LENGTH);
        return;
      }
    }
  /* IRQ INSTR 120 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
    CC2420ReceiveP_RXFIFO_continueRead((buf + 1) + CC2420ReceiveP_SACK_HEADER_LENGTH, CC2420ReceiveP_rxFrameLength - CC2420ReceiveP_SACK_HEADER_LENGTH);
      break;
    case CC2420ReceiveP_S_RX_PAYLOAD:
      CC2420ReceiveP_CSN_set();
    if(!CC2420ReceiveP_m_missed_packets)
    {
      CC2420ReceiveP_SpiResource_release();
    }
  /* IRQ INSTR 123 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
    if((((CC2420ReceiveP_m_missed_packets && CC2420ReceiveP_FIFO_get()) || !CC2420ReceiveP_FIFOP_get()) || !CC2420ReceiveP_m_timestamp_size) || (CC2420ReceiveP_rxFrameLength <= 10))
    {
      CC2420ReceiveP_PacketTimeStamp_clear(CC2420ReceiveP_m_p_rx_buf);
    }
    else 
    {
      if(CC2420ReceiveP_m_timestamp_size == 1)
      {
  /* IRQ INSTR 121 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
        CC2420ReceiveP_PacketTimeStamp_set(CC2420ReceiveP_m_p_rx_buf, CC2420ReceiveP_m_timestamp_queue[CC2420ReceiveP_m_timestamp_head]);
      }
  /* IRQ INSTR 122 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
      CC2420ReceiveP_m_timestamp_head = (CC2420ReceiveP_m_timestamp_head + 1) % CC2420ReceiveP_TIMESTAMP_QUEUE_SIZE;
      CC2420ReceiveP_m_timestamp_size--;
      if(CC2420ReceiveP_m_timestamp_size > 0)
      {
        CC2420ReceiveP_PacketTimeStamp_clear(CC2420ReceiveP_m_p_rx_buf);
        CC2420ReceiveP_m_timestamp_head = 0;
        CC2420ReceiveP_m_timestamp_size = 0;
      }
    }
  /* IRQ INSTR 124 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
    if((buf[CC2420ReceiveP_rxFrameLength] >> 7) && rx_buf)
    {
      uint8_t type = (__nesc_ntoh_leuint16(header->fcf.data) >> IEEE154_FCF_FRAME_TYPE) & 7;
      CC2420ReceiveP_CC2420Receive_receive(type, CC2420ReceiveP_m_p_rx_buf);
      if(type == IEEE154_TYPE_DATA)
      {
        CC2420ReceiveP_receiveDone_task_postTask();
        return;
      }
    }
    CC2420ReceiveP_waitForNextPacket();
      break;
    default :
      CC2420ReceiveP_receivingPacket = FALSE;
    CC2420ReceiveP_CSN_set();
    CC2420ReceiveP_SpiResource_release();
      break;
  }

}

inline static void CC2420SpiP_Fifo_default_readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error) 
{
}

static inline void CC2420SpiP_Fifo_readDone(uint8_t arg_0x1b79698, uint8_t *data, uint8_t length, error_t error) 
{
  switch (arg_0x1b79698) 
  {
    case CC2420_TXFIFO:
      CC2420TransmitP_TXFIFO_readDone(data, length, error);
      break;
    case CC2420_RXFIFO:
      CC2420ReceiveP_RXFIFO_readDone(data, length, error);
      break;
    default :
      CC2420SpiP_Fifo_default_readDone(arg_0x1b79698, data, length, error);
      break;
  }

}

static inline cc2420_status_t CC2420ReceiveP_SFLUSHRX_strobe( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Strobe_strobe(CC2420_SFLUSHRX);
  return result;
}

inline static void CC2420ActiveMessageP_RadioBackoff_default_requestInitialBackoff(am_id_t id, message_t *msg) 
{
}

static inline void CC2420ActiveMessageP_RadioBackoff_requestInitialBackoff(am_id_t arg_0x16fd2e8, message_t *msg) 
{
  CC2420ActiveMessageP_RadioBackoff_default_requestInitialBackoff(arg_0x16fd2e8, msg);
}

inline static void CC2420ActiveMessageP_SubBackoff_requestInitialBackoff(message_t *msg) 
{
  CC2420ActiveMessageP_RadioBackoff_requestInitialBackoff(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.data), msg);
}

static inline void CC2420CsmaP_RadioBackoff_requestInitialBackoff(message_t *msg) 
{
  CC2420ActiveMessageP_SubBackoff_requestInitialBackoff(msg);
}

inline static void CC2420TransmitP_RadioBackoff_setInitialBackoff(uint16_t backoffTime) 
{
  CC2420TransmitP_myInitialBackoff = backoffTime + 1;
}

static inline void CC2420CsmaP_SubBackoff_setInitialBackoff(uint16_t backoffTime) 
{
  CC2420TransmitP_RadioBackoff_setInitialBackoff(backoffTime);
}

inline static void CC2420CsmaP_SubBackoff_requestInitialBackoff(message_t *msg) 
{
  CC2420CsmaP_SubBackoff_setInitialBackoff((CC2420CsmaP_Random_rand16() % (0x1F * CC2420_BACKOFF_PERIOD)) + CC2420_MIN_BACKOFF);
  CC2420CsmaP_RadioBackoff_requestInitialBackoff(msg);
}

static inline void CC2420TransmitP_RadioBackoff_requestInitialBackoff(message_t *msg) 
{
  CC2420CsmaP_SubBackoff_requestInitialBackoff(msg);
}

static inline error_t CC2420CsmaP_sendDone_task_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(CC2420CsmaP_sendDone_task);
  return result;
}

inline static void CC2420CsmaP_CC2420Transmit_sendDone(message_t *p_msg, error_t err) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  CC2420CsmaP_sendErr = err;
  __nesc_atomic_end(__nesc_atomic);
}
  CC2420CsmaP_sendDone_task_postTask();
}

static inline void CC2420TransmitP_Send_sendDone(message_t *p_msg, error_t error) 
{
  CC2420CsmaP_CC2420Transmit_sendDone(p_msg, error);
}

inline static void CC2420TransmitP_TXFIFO_writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error) 
{
  CC2420TransmitP_CSN_set();
  /* IRQ INSTR 127 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 128 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  if(CC2420TransmitP_m_state == CC2420TransmitP_S_CANCEL)
  {
  {
    CC2420TransmitP_CSN_clr();
    CC2420TransmitP_SFLUSHTX_strobe();
    CC2420TransmitP_CSN_set();
  }
    CC2420TransmitP_releaseSpiResource();
    CC2420TransmitP_m_state = CC2420TransmitP_S_STARTED;
    CC2420TransmitP_Send_sendDone(CC2420TransmitP_m_msg, ECANCEL);
  }
  else 
  {
    if(!CC2420TransmitP_m_cca)
    {
    {
      CC2420TransmitP_m_state = CC2420TransmitP_S_BEGIN_TRANSMIT;
    }
      CC2420TransmitP_attemptSend();
    }
    else 
    {
      CC2420TransmitP_releaseSpiResource();
    {
      CC2420TransmitP_m_state = CC2420TransmitP_S_SAMPLE_CCA;
    }
      CC2420TransmitP_RadioBackoff_requestInitialBackoff(CC2420TransmitP_m_msg);
      CC2420TransmitP_BackoffTimer_start(CC2420TransmitP_myInitialBackoff);
    }
  }
}

inline static void CC2420ReceiveP_RXFIFO_writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error) 
{
}

inline static void CC2420SpiP_Fifo_default_writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error) 
{
}

static inline void CC2420SpiP_Fifo_writeDone(uint8_t arg_0x1b79698, uint8_t *data, uint8_t length, error_t error) 
{
  switch (arg_0x1b79698) 
  {
    case CC2420_TXFIFO:
      CC2420TransmitP_TXFIFO_writeDone(data, length, error);
      break;
    case CC2420_RXFIFO:
      CC2420ReceiveP_RXFIFO_writeDone(data, length, error);
      break;
    default :
      CC2420SpiP_Fifo_default_writeDone(arg_0x1b79698, data, length, error);
      break;
  }

}

static inline cc2420_status_t CC2420ControlP_RXCTRL1_write(uint16_t data) 
{
  char unsigned result;
  result = CC2420SpiP_Reg_write(CC2420_RXCTRL1, data);
  return result;
}

static inline cc2420_status_t CC2420ControlP_IOCFG0_write(uint16_t data) 
{
  char unsigned result;
  result = CC2420SpiP_Reg_write(CC2420_IOCFG0, data);
  return result;
}

static inline cc2420_status_t CC2420ControlP_SXOSCON_strobe( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Strobe_strobe(CC2420_SXOSCON);
  return result;
}

inline static void HplMsp430InterruptP_Port14_enable( void ) 
{
  _P1IE |= 1 << 4;
}

static inline void Msp430InterruptC_0_HplInterrupt_enable( void ) 
{
  HplMsp430InterruptP_Port14_enable();
}

inline static void HplMsp430InterruptP_Port14_edge(bool l2h) 
{
{
  if(l2h)
  {
    _P1IES &= ~1 << 4;
  }
  else 
  {
    _P1IES |= 1 << 4;
  }
}
}

static inline void Msp430InterruptC_0_HplInterrupt_edge(bool low_to_high) 
{
  HplMsp430InterruptP_Port14_edge(low_to_high);
}

inline static void HplMsp430InterruptP_Port14_clear( void ) 
{
  _P1IFG &= ~1 << 4;
}

static inline void Msp430InterruptC_0_HplInterrupt_clear( void ) 
{
  HplMsp430InterruptP_Port14_clear();
}

inline static void HplMsp430InterruptP_Port14_disable( void ) 
{
  _P1IE &= ~1 << 4;
}

static inline void Msp430InterruptC_0_HplInterrupt_disable( void ) 
{
  HplMsp430InterruptP_Port14_disable();
}

inline static error_t Msp430InterruptC_0_Interrupt_disable( void ) 
{
{
  Msp430InterruptC_0_HplInterrupt_disable();
  Msp430InterruptC_0_HplInterrupt_clear();
}
  return SUCCESS;
}

inline static error_t Msp430InterruptC_0_enable(bool rising) 
{
{
  Msp430InterruptC_0_Interrupt_disable();
  Msp430InterruptC_0_HplInterrupt_edge(rising);
  Msp430InterruptC_0_HplInterrupt_enable();
}
  return SUCCESS;
}

inline static error_t Msp430InterruptC_0_Interrupt_enableRisingEdge( void ) 
{
  return Msp430InterruptC_0_enable(TRUE);
}

static inline error_t CC2420ControlP_InterruptCCA_enableRisingEdge( void ) 
{
  char unsigned result;
  result = Msp430InterruptC_0_Interrupt_enableRisingEdge();
  return result;
}

static inline cc2420_status_t CC2420ControlP_IOCFG1_write(uint16_t data) 
{
  char unsigned result;
  result = CC2420SpiP_Reg_write(CC2420_IOCFG1, data);
  return result;
}

inline static error_t CC2420ControlP_CC2420Power_startOscillator( void ) 
{
{
  /* IRQ INSTR 129 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 130 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 131 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 132 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 133 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(CC2420ControlP_m_state != CC2420ControlP_S_VREG_STARTED)
  {
  {
    char unsigned __nesc_temp = FAIL;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
  CC2420ControlP_m_state = CC2420ControlP_S_XOSC_STARTING;
  CC2420ControlP_IOCFG1_write(CC2420_SFDMUX_XOSC16M_STABLE << CC2420_IOCFG1_CCAMUX);
  CC2420ControlP_InterruptCCA_enableRisingEdge();
  CC2420ControlP_SXOSCON_strobe();
  CC2420ControlP_IOCFG0_write((1 << CC2420_IOCFG0_FIFOP_POLARITY) | (127 << CC2420_IOCFG0_FIFOP_THR));
  CC2420ControlP_writeFsctrl();
  CC2420ControlP_writeMdmctrl0();
  CC2420ControlP_RXCTRL1_write(((((((1 << CC2420_RXCTRL1_RXBPF_LOCUR) | (1 << CC2420_RXCTRL1_LOW_LOWGAIN)) | (1 << CC2420_RXCTRL1_HIGH_HGM)) | (1 << CC2420_RXCTRL1_LNA_CAP_ARRAY)) | (1 << CC2420_RXCTRL1_RXMIX_TAIL)) | (1 << CC2420_RXCTRL1_RXMIX_VCM)) | (2 << CC2420_RXCTRL1_RXMIX_CURRENT));
}
  __nesc_atomic_end(__nesc_atomic);
}
  return SUCCESS;
}

static inline error_t CC2420CsmaP_CC2420Power_startOscillator( void ) 
{
  char unsigned result;
  result = CC2420ControlP_CC2420Power_startOscillator();
  return result;
}

inline static void CC2420CsmaP_Resource_granted( void ) 
{
  CC2420CsmaP_CC2420Power_startOscillator();
}

static inline void CC2420ControlP_Resource_granted( void ) 
{
  CC2420CsmaP_Resource_granted();
}

static inline void CC2420ControlP_CSN_clr( void ) 
{
  Msp430GpioC_1_GeneralIO_clr();
}

inline static void CC2420ControlP_SpiResource_granted( void ) 
{
  CC2420ControlP_CSN_clr();
  CC2420ControlP_Resource_granted();
}

static inline error_t CC2420ControlP_syncDone_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(CC2420ControlP_syncDone);
  return result;
}

static inline error_t CC2420ControlP_SyncResource_release( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Resource_release(CC2420SpiC_1_CLIENT_ID);
  return result;
}

static inline void CC2420ControlP_CSN_set( void ) 
{
  Msp430GpioC_1_GeneralIO_set();
}

static inline cc2420_status_t CC2420ControlP_SRXON_strobe( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Strobe_strobe(CC2420_SRXON);
  return result;
}

static inline cc2420_status_t CC2420ControlP_SRFOFF_strobe( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Strobe_strobe(CC2420_SRFOFF);
  return result;
}

inline static void CC2420ControlP_SyncResource_granted( void ) 
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

inline static void CC2420ControlP_ReadRssi_default_readDone(error_t error, uint16_t data) 
{
}

static inline void CC2420ControlP_ReadRssi_readDone(error_t result, CC2420ControlP_ReadRssi_val_t val) 
{
  CC2420ControlP_ReadRssi_default_readDone(result, val);
}

static inline error_t CC2420ControlP_RssiResource_release( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Resource_release(CC2420SpiC_2_CLIENT_ID);
  return result;
}

inline static cc2420_status_t CC2420SpiP_Reg_read(uint8_t addr, uint16_t *data) 
{
  cc2420_status_t status = 0;
{
  /* IRQ INSTR 134 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 135 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 136 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 137 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 138 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(CC2420SpiP_WorkingState_isIdle())
  {
  {
    char unsigned __nesc_temp = status;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
  status = CC2420SpiP_SpiByte_write(addr | 0x40);
  *data = ( uint16_t )(CC2420SpiP_SpiByte_write(0)) << 8;
  *data |= CC2420SpiP_SpiByte_write(0);
  return status;
}

static inline cc2420_status_t CC2420ControlP_RSSI_read(uint16_t *data) 
{
  char unsigned result;
  result = CC2420SpiP_Reg_read(CC2420_RSSI, data);
  return result;
}

inline static void CC2420ControlP_RssiResource_granted( void ) 
{
  uint16_t data;
  CC2420ControlP_CSN_clr();
  CC2420ControlP_RSSI_read(&data);
  CC2420ControlP_CSN_set();
  CC2420ControlP_RssiResource_release();
  data += 0x7f;
  data &= 0x00ff;
  CC2420ControlP_ReadRssi_readDone(SUCCESS, data);
}

inline static void CC2420TransmitP_SpiResource_granted( void ) 
{
  uint8_t cur_state;
{
  /* IRQ INSTR 141 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 142 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 143 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 144 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 145 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  /* IRQ INSTR 139 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 140 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  cur_state = CC2420TransmitP_m_state;
}
  __nesc_atomic_end(__nesc_atomic);
}
  switch (cur_state) 
  {
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
  {
  /* IRQ INSTR 146 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 147 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 148 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 149 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 150 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
    __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  {
    CC2420TransmitP_m_state = CC2420TransmitP_S_STARTED;
  }
    __nesc_atomic_end(__nesc_atomic);
  }
    CC2420TransmitP_Send_sendDone(CC2420TransmitP_m_msg, ECANCEL);
      break;
    default :
      CC2420TransmitP_releaseSpiResource();
      break;
  }

}

inline static void CC2420ReceiveP_SpiResource_granted( void ) 
{
  CC2420ReceiveP_receive();
}

inline static void CC2420SpiP_Resource_default_granted(uint8_t id) 
{
}

static inline void CC2420SpiP_Resource_granted(uint8_t arg_0x1b7abd0) 
{
  switch (arg_0x1b7abd0) 
  {
    case CC2420SpiC_0_CLIENT_ID:
      CC2420ControlP_SpiResource_granted();
      break;
    case CC2420SpiC_1_CLIENT_ID:
      CC2420ControlP_SyncResource_granted();
      break;
    case CC2420SpiC_2_CLIENT_ID:
      CC2420ControlP_RssiResource_granted();
      break;
    case CC2420SpiC_3_CLIENT_ID:
      CC2420TransmitP_SpiResource_granted();
      break;
    case CC2420SpiC_4_CLIENT_ID:
      CC2420ReceiveP_SpiResource_granted();
      break;
    default :
      CC2420SpiP_Resource_default_granted(arg_0x1b7abd0);
      break;
  }

}

inline static void CC2420SpiP_grant_runTask( void ) 
{
  uint8_t holder;
{
  /* IRQ INSTR 154 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 155 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 156 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 157 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 158 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  /* IRQ INSTR 151 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 152 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 153 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  holder = CC2420SpiP_m_holder;
}
  __nesc_atomic_end(__nesc_atomic);
}
  CC2420SpiP_Resource_granted(holder);
}

static inline cc2420_status_t CC2420ControlP_FSCTRL_write(uint16_t data) 
{
  char unsigned result;
  result = CC2420SpiP_Reg_write(CC2420_FSCTRL, data);
  return result;
}

static inline cc2420_status_t CC2420ControlP_MDMCTRL0_write(uint16_t data) 
{
  char unsigned result;
  result = CC2420SpiP_Reg_write(CC2420_MDMCTRL0, data);
  return result;
}

static inline cc2420_status_t CC2420ControlP_PANID_write(uint8_t offset, uint8_t *data, uint8_t length) 
{
  char unsigned result;
  result = CC2420SpiP_Ram_write(CC2420_RAM_PANID, offset, data, length);
  return result;
}

inline static void CC2420ActiveMessageP_CC2420Config_syncDone(error_t error) 
{
}

inline static void CC2420ReceiveP_CC2420Config_syncDone(error_t error) 
{
}

static inline void CC2420ControlP_CC2420Config_syncDone(error_t error) 
{
  CC2420ReceiveP_CC2420Config_syncDone(error);
  CC2420ActiveMessageP_CC2420Config_syncDone(error);
}

inline static void CC2420ControlP_syncDone_runTask( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  CC2420ControlP_m_sync_busy = FALSE;
  __nesc_atomic_end(__nesc_atomic);
}
  CC2420ControlP_CC2420Config_syncDone(SUCCESS);
}

static inline error_t CC2420ControlP_SyncResource_request( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Resource_request(CC2420SpiC_1_CLIENT_ID);
  return result;
}

inline static error_t CC2420ControlP_CC2420Config_sync( void ) 
{
{
  /* IRQ INSTR 159 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 160 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 161 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 162 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 163 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(CC2420ControlP_m_sync_busy)
  {
  {
    char unsigned __nesc_temp = FAIL;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
  CC2420ControlP_m_sync_busy = TRUE;
  if(CC2420ControlP_m_state == CC2420ControlP_S_XOSC_STARTED)
  {
    CC2420ControlP_SyncResource_request();
  }
  else 
  {
    CC2420ControlP_syncDone_postTask();
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
  return SUCCESS;
}

inline static void CC2420ControlP_sync_runTask( void ) 
{
  CC2420ControlP_CC2420Config_sync();
}

inline static void AMQueueImplP_0_AMSend_sendDone(am_id_t id, message_t *msg, error_t err) 
{
  if(AMQueueImplP_0_current >= 2)
  {
    return;
  }
  if(AMQueueImplP_0_queue[AMQueueImplP_0_current].msg == msg)
  {
    AMQueueImplP_0_sendDone(AMQueueImplP_0_current, msg, err);
  }
  else 
  {
  }
}

static inline void CC2420ActiveMessageP_AMSend_sendDone(am_id_t arg_0x16e11b8, message_t *msg, error_t error) 
{
  AMQueueImplP_0_AMSend_sendDone(arg_0x16e11b8, msg, error);
}

inline static void CC2420ActiveMessageP_SubSend_sendDone(message_t *msg, error_t result) 
{
  CC2420ActiveMessageP_AMSend_sendDone(CC2420ActiveMessageP_AMPacket_type(msg), msg, result);
}

static inline void UniqueSendP_Send_sendDone(message_t *msg, error_t error) 
{
  CC2420ActiveMessageP_SubSend_sendDone(msg, error);
}

inline static void UniqueSendP_SubSend_sendDone(message_t *msg, error_t error) 
{
  UniqueSendP_State_toIdle();
  UniqueSendP_Send_sendDone(msg, error);
}

static inline void CC2420TinyosNetworkP_Send_sendDone(message_t *msg, error_t error) 
{
  UniqueSendP_SubSend_sendDone(msg, error);
}

inline static void CC2420TinyosNetworkP_SubSend_sendDone(message_t *msg, error_t error) 
{
  CC2420TinyosNetworkP_Send_sendDone(msg, error);
}

static inline void CC2420CsmaP_Send_sendDone(message_t *msg, error_t error) 
{
  CC2420TinyosNetworkP_SubSend_sendDone(msg, error);
}

static inline error_t CC2420CsmaP_stopDone_task_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(CC2420CsmaP_stopDone_task);
  return result;
}

inline static void HplMsp430GeneralIOP_29_IO_clr( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P4OUT &= ~0x01 << 5;
  __nesc_atomic_end(__nesc_atomic);
}
}

static inline void Msp430GpioC_6_HplGeneralIO_clr( void ) 
{
  HplMsp430GeneralIOP_29_IO_clr();
}

inline static void Msp430GpioC_6_GeneralIO_clr( void ) 
{
  Msp430GpioC_6_HplGeneralIO_clr();
}

static inline void CC2420ControlP_VREN_clr( void ) 
{
  Msp430GpioC_6_GeneralIO_clr();
}

inline static error_t CC2420ControlP_CC2420Power_stopVReg( void ) 
{
  CC2420ControlP_m_state = CC2420ControlP_S_VREG_STOPPED;
  CC2420ControlP_RSTN_clr();
  CC2420ControlP_VREN_clr();
  CC2420ControlP_RSTN_set();
  return SUCCESS;
}

static inline error_t CC2420CsmaP_CC2420Power_stopVReg( void ) 
{
  char unsigned result;
  result = CC2420ControlP_CC2420Power_stopVReg();
  return result;
}

inline static error_t ecombine(error_t r1, error_t r2) 
{
  return r1 == r2? r1 : FAIL;
}

inline static void HplMsp430InterruptP_Port10_clear( void ) 
{
  _P1IFG &= ~1 << 0;
}

static inline void Msp430InterruptC_1_HplInterrupt_clear( void ) 
{
  HplMsp430InterruptP_Port10_clear();
}

inline static void HplMsp430InterruptP_Port10_disable( void ) 
{
  _P1IE &= ~1 << 0;
}

static inline void Msp430InterruptC_1_HplInterrupt_disable( void ) 
{
  HplMsp430InterruptP_Port10_disable();
}

inline static error_t Msp430InterruptC_1_Interrupt_disable( void ) 
{
{
  Msp430InterruptC_1_HplInterrupt_disable();
  Msp430InterruptC_1_HplInterrupt_clear();
}
  return SUCCESS;
}

static inline error_t CC2420ReceiveP_InterruptFIFOP_disable( void ) 
{
  char unsigned result;
  result = Msp430InterruptC_1_Interrupt_disable();
  return result;
}

inline static error_t CC2420ReceiveP_StdControl_stop( void ) 
{
{
  /* IRQ INSTR 164 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 165 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 166 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 167 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 168 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  CC2420ReceiveP_m_state = CC2420ReceiveP_S_STOPPED;
  CC2420ReceiveP_reset_state();
  CC2420ReceiveP_CSN_set();
  CC2420ReceiveP_InterruptFIFOP_disable();
}
  __nesc_atomic_end(__nesc_atomic);
}
  return SUCCESS;
}

inline static void HplMsp430GeneralIOP_25_IO_selectIOFunc( void ) 
{
  _P4SEL &= ~0x01 << 1;
}

static inline void GpioCaptureC_0_GeneralIO_selectIOFunc( void ) 
{
  HplMsp430GeneralIOP_25_IO_selectIOFunc();
}

inline static void Msp430TimerCapComP_4_Control_disableEvents( void ) 
{
  _TBCCTL1 &= ~0x0010;
}

static inline void GpioCaptureC_0_Msp430TimerControl_disableEvents( void ) 
{
  Msp430TimerCapComP_4_Control_disableEvents();
}

inline static void GpioCaptureC_0_Capture_disable( void ) 
{
{
  GpioCaptureC_0_Msp430TimerControl_disableEvents();
  GpioCaptureC_0_GeneralIO_selectIOFunc();
}
}

static inline void CC2420TransmitP_CaptureSFD_disable( void ) 
{
  GpioCaptureC_0_Capture_disable();
}

inline static error_t CC2420TransmitP_StdControl_stop( void ) 
{
{
  /* IRQ INSTR 169 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 170 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 171 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 172 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 173 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  CC2420TransmitP_m_state = CC2420TransmitP_S_STOPPED;
  CC2420TransmitP_BackoffTimer_stop();
  CC2420TransmitP_CaptureSFD_disable();
  CC2420TransmitP_SpiResource_release();
  CC2420TransmitP_CSN_set();
}
  __nesc_atomic_end(__nesc_atomic);
}
  return SUCCESS;
}

static inline error_t CC2420CsmaP_SubControl_stop( void ) 
{
  char unsigned result;
  result = CC2420TransmitP_StdControl_stop();
  result = ecombine(result, CC2420ReceiveP_StdControl_stop());
  return result;
}

inline static void CC2420CsmaP_shutdown( void ) 
{
  CC2420CsmaP_SubControl_stop();
  CC2420CsmaP_CC2420Power_stopVReg();
  CC2420CsmaP_stopDone_task_postTask();
}

inline static void CC2420CsmaP_sendDone_task_runTask( void ) 
{
  error_t packetErr;
{
  /* IRQ INSTR 176 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 177 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 178 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 179 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 180 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  /* IRQ INSTR 174 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 175 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  packetErr = CC2420CsmaP_sendErr;
  __nesc_atomic_end(__nesc_atomic);
}
  if(CC2420CsmaP_SplitControlState_isState(CC2420CsmaP_S_STOPPING))
  {
    CC2420CsmaP_shutdown();
  }
  else 
  {
    CC2420CsmaP_SplitControlState_forceState(CC2420CsmaP_S_STARTED);
  }
  CC2420CsmaP_Send_sendDone(CC2420CsmaP_m_msg, packetErr);
}

inline static void TestDisseminationC_RadioControl_stopDone(error_t result) 
{
}

static inline void CC2420CsmaP_SplitControl_stopDone(error_t error) 
{
  TestDisseminationC_RadioControl_stopDone(error);
}

inline static void CC2420CsmaP_stopDone_task_runTask( void ) 
{
  CC2420CsmaP_SplitControlState_forceState(CC2420CsmaP_S_STOPPED);
  CC2420CsmaP_SplitControl_stopDone(SUCCESS);
}

static inline void TestDisseminationC_Timer_startPeriodic(uint32_t dt) 
{
  VirtualizeTimerC_0_Timer_startPeriodic(2U, dt);
}

static inline error_t DisseminatorP_0_DisseminationCache_start( void ) 
{
  char unsigned result;
  result = DisseminationEngineImplP_DisseminationCache_start(4660U);
  return result;
}

inline static error_t DisseminatorP_0_StdControl_start( void ) 
{
  error_t result = DisseminatorP_0_DisseminationCache_start();
  if(result == SUCCESS)
  {
    DisseminatorP_0_m_running = TRUE;
  }
  return result;
}

static inline error_t DisseminatorP_1_DisseminationCache_start( void ) 
{
  char unsigned result;
  result = DisseminationEngineImplP_DisseminationCache_start(9029U);
  return result;
}

inline static error_t DisseminatorP_1_StdControl_start( void ) 
{
  error_t result = DisseminatorP_1_DisseminationCache_start();
  if(result == SUCCESS)
  {
    DisseminatorP_1_m_running = TRUE;
  }
  return result;
}

inline static error_t DisseminationEngineImplP_DisseminatorControl_default_start(uint16_t id) 
{
  return FAIL;
}

static inline error_t DisseminationEngineImplP_DisseminatorControl_start(uint16_t arg_0x21c2738) 
{
  char unsigned result;
  switch (arg_0x21c2738) 
  {
    case DisseminatorC_0_TIMER_ID:
      result = DisseminatorP_0_StdControl_start();
      break;
    case DisseminatorC_1_TIMER_ID:
      result = DisseminatorP_1_StdControl_start();
      break;
    default :
      result = DisseminationEngineImplP_DisseminatorControl_default_start(arg_0x21c2738);
      break;
  }

  return result;
}

inline static error_t DisseminationEngineImplP_StdControl_start( void ) 
{
  uint8_t i;
  for(i = 0; i < DisseminationEngineImplP_NUM_DISSEMINATORS; i++)
  {
    DisseminationEngineImplP_DisseminatorControl_start(i);
  }

  DisseminationEngineImplP_m_running = TRUE;
  return SUCCESS;
}

static inline error_t TestDisseminationC_DisseminationControl_start( void ) 
{
  char unsigned result;
  result = DisseminationEngineImplP_StdControl_start();
  return result;
}

static inline error_t TestDisseminationC_RadioControl_start( void ) 
{
  char unsigned result;
  result = CC2420CsmaP_SplitControl_start();
  return result;
}

inline static void TestDisseminationC_RadioControl_startDone(error_t result) 
{
  if(result != SUCCESS)
  {
    TestDisseminationC_RadioControl_start();
  }
  else 
  {
    TestDisseminationC_DisseminationControl_start();
    if((TOS_NODE_ID % 4) == 1)
    {
      TestDisseminationC_Timer_startPeriodic(1024 * 20);
    }
    else 
    {
      TestDisseminationC_Timer_startPeriodic(1024);
    }
  }
}

static inline void CC2420CsmaP_SplitControl_startDone(error_t error) 
{
  TestDisseminationC_RadioControl_startDone(error);
}

static inline error_t CC2420ControlP_SpiResource_release( void ) 
{
  char unsigned result;
  result = CC2420SpiP_Resource_release(CC2420SpiC_0_CLIENT_ID);
  return result;
}

inline static error_t CC2420ControlP_Resource_release( void ) 
{
{
  /* IRQ INSTR 181 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 182 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 183 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 184 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 185 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  CC2420ControlP_CSN_set();
{
  char unsigned __nesc_temp = CC2420ControlP_SpiResource_release();
{
  __nesc_atomic_end(__nesc_atomic);
  return __nesc_temp;
}
}
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static inline error_t CC2420CsmaP_Resource_release( void ) 
{
  char unsigned result;
  result = CC2420ControlP_Resource_release();
  return result;
}

inline static error_t CC2420ControlP_CC2420Power_rxOn( void ) 
{
{
  /* IRQ INSTR 186 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 187 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 188 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 189 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 190 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(CC2420ControlP_m_state != CC2420ControlP_S_XOSC_STARTED)
  {
  {
    char unsigned __nesc_temp = FAIL;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
  CC2420ControlP_SRXON_strobe();
}
  __nesc_atomic_end(__nesc_atomic);
}
  return SUCCESS;
}

static inline error_t CC2420CsmaP_CC2420Power_rxOn( void ) 
{
  char unsigned result;
  result = CC2420ControlP_CC2420Power_rxOn();
  return result;
}

inline static void HplMsp430InterruptP_Port10_enable( void ) 
{
  _P1IE |= 1 << 0;
}

static inline void Msp430InterruptC_1_HplInterrupt_enable( void ) 
{
  HplMsp430InterruptP_Port10_enable();
}

inline static void HplMsp430InterruptP_Port10_edge(bool l2h) 
{
{
  if(l2h)
  {
    _P1IES &= ~1 << 0;
  }
  else 
  {
    _P1IES |= 1 << 0;
  }
}
}

static inline void Msp430InterruptC_1_HplInterrupt_edge(bool low_to_high) 
{
  HplMsp430InterruptP_Port10_edge(low_to_high);
}

inline static error_t Msp430InterruptC_1_enable(bool rising) 
{
{
  Msp430InterruptC_1_Interrupt_disable();
  Msp430InterruptC_1_HplInterrupt_edge(rising);
  Msp430InterruptC_1_HplInterrupt_enable();
}
  return SUCCESS;
}

inline static error_t Msp430InterruptC_1_Interrupt_enableFallingEdge( void ) 
{
  return Msp430InterruptC_1_enable(FALSE);
}

static inline error_t CC2420ReceiveP_InterruptFIFOP_enableFallingEdge( void ) 
{
  char unsigned result;
  result = Msp430InterruptC_1_Interrupt_enableFallingEdge();
  return result;
}

inline static error_t CC2420ReceiveP_StdControl_start( void ) 
{
{
  /* IRQ INSTR 191 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 192 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 193 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 194 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 195 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  CC2420ReceiveP_reset_state();
  CC2420ReceiveP_m_state = CC2420ReceiveP_S_STARTED;
  CC2420ReceiveP_receivingPacket = FALSE;
  CC2420ReceiveP_InterruptFIFOP_enableFallingEdge();
}
  __nesc_atomic_end(__nesc_atomic);
}
  return SUCCESS;
}

inline static error_t CC2420TransmitP_StdControl_start( void ) 
{
{
  /* IRQ INSTR 196 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 197 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 198 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 199 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 200 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  CC2420TransmitP_CaptureSFD_captureRisingEdge();
  CC2420TransmitP_m_state = CC2420TransmitP_S_STARTED;
  CC2420TransmitP_m_receiving = FALSE;
  CC2420TransmitP_abortSpiRelease = FALSE;
  CC2420TransmitP_m_tx_power = 0;
}
  __nesc_atomic_end(__nesc_atomic);
}
  return SUCCESS;
}

static inline error_t CC2420CsmaP_SubControl_start( void ) 
{
  char unsigned result;
  result = CC2420TransmitP_StdControl_start();
  result = ecombine(result, CC2420ReceiveP_StdControl_start());
  return result;
}

inline static void CC2420CsmaP_startDone_task_runTask( void ) 
{
  CC2420CsmaP_SubControl_start();
  CC2420CsmaP_CC2420Power_rxOn();
  CC2420CsmaP_Resource_release();
  CC2420CsmaP_SplitControlState_forceState(CC2420CsmaP_S_STARTED);
  CC2420CsmaP_SplitControl_startDone(SUCCESS);
}

static inline error_t CC2420CsmaP_SplitControlState_requestState(uint8_t reqState) 
{
  char unsigned result;
  result = StateImplP_State_requestState(1U, reqState);
  return result;
}

static inline void CC2420ControlP_StartupTimer_start(CC2420ControlP_StartupTimer_size_type dt) 
{
  TransformAlarmC_0_Alarm_start(dt);
}

inline static void HplMsp430GeneralIOP_29_IO_set( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P4OUT |= 0x01 << 5;
  __nesc_atomic_end(__nesc_atomic);
}
}

static inline void Msp430GpioC_6_HplGeneralIO_set( void ) 
{
  HplMsp430GeneralIOP_29_IO_set();
}

inline static void Msp430GpioC_6_GeneralIO_set( void ) 
{
  Msp430GpioC_6_HplGeneralIO_set();
}

static inline void CC2420ControlP_VREN_set( void ) 
{
  Msp430GpioC_6_GeneralIO_set();
}

inline static error_t CC2420ControlP_CC2420Power_startVReg( void ) 
{
{
  /* IRQ INSTR 201 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 202 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 203 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 204 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 205 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(CC2420ControlP_m_state != CC2420ControlP_S_VREG_STOPPED)
  {
  {
    char unsigned __nesc_temp = FAIL;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
  CC2420ControlP_m_state = CC2420ControlP_S_VREG_STARTING;
}
  __nesc_atomic_end(__nesc_atomic);
}
  CC2420ControlP_VREN_set();
  CC2420ControlP_StartupTimer_start(CC2420_TIME_VREN);
  return SUCCESS;
}

static inline error_t CC2420CsmaP_CC2420Power_startVReg( void ) 
{
  char unsigned result;
  result = CC2420ControlP_CC2420Power_startVReg();
  return result;
}

inline static error_t DisseminationEngineImplP_TrickleTimer_default_start(uint16_t key) 
{
  return FAIL;
}

static inline error_t DisseminationEngineImplP_TrickleTimer_start(uint16_t arg_0x21c4c10) 
{
  char unsigned result;
  switch (arg_0x21c4c10) 
  {
    case 4660U:
      result = TrickleTimerImplP_0_TrickleTimer_start(DisseminatorC_0_TIMER_ID);
      break;
    case 9029U:
      result = TrickleTimerImplP_0_TrickleTimer_start(DisseminatorC_1_TIMER_ID);
      break;
    default :
      result = DisseminationEngineImplP_TrickleTimer_default_start(arg_0x21c4c10);
      break;
  }

  return result;
}

inline static am_group_t ActiveMessageAddressC_ActiveMessageAddress_amGroup( void ) 
{
  am_group_t myGroup;
  myGroup = ActiveMessageAddressC_group;
  return myGroup;
}

static inline am_group_t CC2420ControlP_ActiveMessageAddress_amGroup( void ) 
{
  char unsigned result;
  result = ActiveMessageAddressC_ActiveMessageAddress_amGroup();
  return result;
}

static inline am_addr_t CC2420ControlP_ActiveMessageAddress_amAddress( void ) 
{
  int unsigned result;
  result = ActiveMessageAddressC_ActiveMessageAddress_amAddress();
  return result;
}

inline static void HplMsp430GeneralIOP_29_IO_makeOutput( void ) 
{
  _P4DIR |= 0x01 << 5;
}

static inline void Msp430GpioC_6_HplGeneralIO_makeOutput( void ) 
{
  HplMsp430GeneralIOP_29_IO_makeOutput();
}

inline static void Msp430GpioC_6_GeneralIO_makeOutput( void ) 
{
  Msp430GpioC_6_HplGeneralIO_makeOutput();
}

static inline void CC2420ControlP_VREN_makeOutput( void ) 
{
  Msp430GpioC_6_GeneralIO_makeOutput();
}

inline static void HplMsp430GeneralIOP_30_IO_makeOutput( void ) 
{
  _P4DIR |= 0x01 << 6;
}

static inline void Msp430GpioC_4_HplGeneralIO_makeOutput( void ) 
{
  HplMsp430GeneralIOP_30_IO_makeOutput();
}

inline static void Msp430GpioC_4_GeneralIO_makeOutput( void ) 
{
  Msp430GpioC_4_HplGeneralIO_makeOutput();
}

static inline void CC2420ControlP_RSTN_makeOutput( void ) 
{
  Msp430GpioC_4_GeneralIO_makeOutput();
}

inline static void HplMsp430GeneralIOP_26_IO_makeOutput( void ) 
{
  _P4DIR |= 0x01 << 2;
}

static inline void Msp430GpioC_1_HplGeneralIO_makeOutput( void ) 
{
  HplMsp430GeneralIOP_26_IO_makeOutput();
}

inline static void Msp430GpioC_1_GeneralIO_makeOutput( void ) 
{
  Msp430GpioC_1_HplGeneralIO_makeOutput();
}

static inline void CC2420ControlP_CSN_makeOutput( void ) 
{
  Msp430GpioC_1_GeneralIO_makeOutput();
}

inline static error_t CC2420ControlP_Init_init( void ) 
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

inline static error_t StateImplP_Init_init( void ) 
{
  int i;
  for(i = 0; i < 4U; i++)
  {
    StateImplP_state[i] = StateImplP_S_IDLE;
  }

  return SUCCESS;
}

inline static error_t FcfsResourceQueueC_0_Init_init( void ) 
{
  /* IRQ INSTR 206 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 207 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 208 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  memset(FcfsResourceQueueC_0_resQ, FcfsResourceQueueC_0_NO_ENTRY, sizeof(FcfsResourceQueueC_0_resQ));
  return SUCCESS;
}

inline static uint16_t Msp430TimerCapComP_3_CC2int(Msp430TimerCapComP_3_cc_t x) 
{
  union Msp430TimerCapComP_3___nesc_unnamed4359  {
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

inline static void HplMsp430GeneralIOP_25_IO_makeInput( void ) 
{
  _P4DIR &= ~0x01 << 1;
}

static inline void Msp430GpioC_5_HplGeneralIO_makeInput( void ) 
{
  HplMsp430GeneralIOP_25_IO_makeInput();
}

inline static void Msp430GpioC_5_GeneralIO_makeInput( void ) 
{
  Msp430GpioC_5_HplGeneralIO_makeInput();
}

static inline void CC2420TransmitP_SFD_makeInput( void ) 
{
  Msp430GpioC_5_GeneralIO_makeInput();
}

static inline void CC2420TransmitP_CSN_makeOutput( void ) 
{
  Msp430GpioC_1_GeneralIO_makeOutput();
}

inline static void HplMsp430GeneralIOP_4_IO_makeInput( void ) 
{
  _P1DIR &= ~0x01 << 4;
}

static inline void Msp430GpioC_0_HplGeneralIO_makeInput( void ) 
{
  HplMsp430GeneralIOP_4_IO_makeInput();
}

inline static void Msp430GpioC_0_GeneralIO_makeInput( void ) 
{
  Msp430GpioC_0_HplGeneralIO_makeInput();
}

static inline void CC2420TransmitP_CCA_makeInput( void ) 
{
  Msp430GpioC_0_GeneralIO_makeInput();
}

inline static error_t CC2420TransmitP_Init_init( void ) 
{
  CC2420TransmitP_CCA_makeInput();
  CC2420TransmitP_CSN_makeOutput();
  CC2420TransmitP_SFD_makeInput();
  return SUCCESS;
}

inline static error_t CC2420ReceiveP_Init_init( void ) 
{
  CC2420ReceiveP_m_p_rx_buf = &CC2420ReceiveP_m_rx_buf;
  return SUCCESS;
}

inline static uint16_t Msp430TimerCapComP_5_CC2int(Msp430TimerCapComP_5_cc_t x) 
{
  union Msp430TimerCapComP_5___nesc_unnamed4360  {
  Msp430TimerCapComP_5_cc_t f;
  uint16_t t;
} c;
  c.f = x;
  return c.t;
}

inline static uint16_t Msp430TimerCapComP_5_compareControl( void ) 
{
  Msp430TimerCapComP_5_cc_t x = {.cm = 1, .ccis = 0, .clld = 0, .cap = 0, .ccie = 0};
  return Msp430TimerCapComP_5_CC2int(x);
}

inline static void Msp430TimerCapComP_5_Control_setControlAsCompare( void ) 
{
  _TBCCTL2 = Msp430TimerCapComP_5_compareControl();
}

static inline void Msp430AlarmC_1_Msp430TimerControl_setControlAsCompare( void ) 
{
  Msp430TimerCapComP_5_Control_setControlAsCompare();
}

inline static error_t Msp430AlarmC_1_Init_init( void ) 
{
  Msp430AlarmC_1_Msp430TimerControl_disableEvents();
  Msp430AlarmC_1_Msp430TimerControl_setControlAsCompare();
  return SUCCESS;
}

inline static error_t RandomMlcgC_Init_init( void ) 
{
  RandomMlcgC_seed = ( uint32_t )(TOS_NODE_ID + 1);
  return SUCCESS;
}

static inline uint16_t UniqueSendP_Random_rand16( void ) 
{
  int unsigned result;
  result = RandomMlcgC_Random_rand16();
  return result;
}

inline static error_t UniqueSendP_Init_init( void ) 
{
  UniqueSendP_localSendId = UniqueSendP_Random_rand16();
  return SUCCESS;
}

inline static error_t UniqueReceiveP_Init_init( void ) 
{
  int i;
  for(i = 0; i < 4; i++)
  {
    UniqueReceiveP_receivedMessages[i].source = ( am_addr_t )(0xFFFF);
    UniqueReceiveP_receivedMessages[i].dsn = 0;
  }

  return SUCCESS;
}

inline static void BitVectorC_1_BitVector_clearAll( void ) 
{
  memset(BitVectorC_1_m_bits, 0, sizeof(BitVectorC_1_m_bits));
}

static inline void TrickleTimerImplP_0_Changed_clearAll( void ) 
{
  BitVectorC_1_BitVector_clearAll();
}

inline static void BitVectorC_0_BitVector_clearAll( void ) 
{
  memset(BitVectorC_0_m_bits, 0, sizeof(BitVectorC_0_m_bits));
}

static inline void TrickleTimerImplP_0_Pending_clearAll( void ) 
{
  BitVectorC_0_BitVector_clearAll();
}

inline static error_t TrickleTimerImplP_0_Init_init( void ) 
{
  int i;
  for(i = 0; i < 2U; i++)
  {
    TrickleTimerImplP_0_trickles[i].period = 1024;
    TrickleTimerImplP_0_trickles[i].count = 0;
    TrickleTimerImplP_0_trickles[i].time = 0;
    TrickleTimerImplP_0_trickles[i].remainder = 0;
  }

{
  TrickleTimerImplP_0_Pending_clearAll();
  TrickleTimerImplP_0_Changed_clearAll();
}
  return SUCCESS;
}

static inline error_t RealMainP_SoftwareInit_init( void ) 
{
  char unsigned result;
  result = TrickleTimerImplP_0_Init_init();
  result = ecombine(result, UniqueReceiveP_Init_init());
  result = ecombine(result, UniqueSendP_Init_init());
  result = ecombine(result, RandomMlcgC_Init_init());
  result = ecombine(result, Msp430AlarmC_1_Init_init());
  result = ecombine(result, CC2420ReceiveP_Init_init());
  result = ecombine(result, CC2420TransmitP_Init_init());
  result = ecombine(result, Msp430AlarmC_0_Init_init());
  result = ecombine(result, FcfsResourceQueueC_0_Init_init());
  result = ecombine(result, StateImplP_Init_init());
  result = ecombine(result, CC2420ControlP_Init_init());
  return result;
}

inline static void DisseminatorP_1_DisseminationValue_set(const DisseminatorP_1_t *val) 
{
  if(DisseminatorP_1_seqno == DISSEMINATION_SEQNO_UNKNOWN)
  {
    DisseminatorP_1_valueCache = *val;
  }
}

static inline void TestDisseminationC_Value16_set(const TestDisseminationC_Value16_t *arg_0x14ad7a8) 
{
  DisseminatorP_1_DisseminationValue_set(arg_0x14ad7a8);
}

inline static void DisseminatorP_0_DisseminationValue_set(const DisseminatorP_0_t *val) 
{
  if(DisseminatorP_0_seqno == DISSEMINATION_SEQNO_UNKNOWN)
  {
    DisseminatorP_0_valueCache = *val;
  }
}

static inline void TestDisseminationC_Value32_set(const TestDisseminationC_Value32_t *arg_0x14ad7a8) 
{
  DisseminatorP_0_DisseminationValue_set(arg_0x14ad7a8);
}

inline static void TestDisseminationC_Boot_booted( void ) 
{
  uint32_t initialVal32 = 123456;
  uint16_t initialVal16 = 1234;
  TestDisseminationC_Value32_set(&initialVal32);
  TestDisseminationC_Value16_set(&initialVal16);
  TestDisseminationC_RadioControl_start();
}

static inline void RealMainP_Boot_booted( void ) 
{
  TestDisseminationC_Boot_booted();
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
  /* IRQ INSTR 218 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 219 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 220 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  if(SchedulerBasicP_m_head != SchedulerBasicP_NO_TASK)
  {
  /* IRQ INSTR 215 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 216 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 217 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
    uint8_t id = SchedulerBasicP_m_head;
  /* IRQ INSTR 209 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 210 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 211 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
    SchedulerBasicP_m_head = SchedulerBasicP_m_next[SchedulerBasicP_m_head];
  /* IRQ INSTR 212 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 213 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 214 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
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
  /* IRQ INSTR 221 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 222 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 223 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 224 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 225 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
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

inline static void CC2420ReceiveP_InterruptFIFOP_fired( void ) 
{
  /* IRQ INSTR 226 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 227 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  if(CC2420ReceiveP_m_state == CC2420ReceiveP_S_STARTED)
  {
    CC2420ReceiveP_beginReceive();
  }
  else 
  {
    CC2420ReceiveP_m_missed_packets++;
  }
}

static inline void Msp430InterruptC_1_Interrupt_fired( void ) 
{
  CC2420ReceiveP_InterruptFIFOP_fired();
}

inline static void Msp430InterruptC_1_HplInterrupt_fired( void ) 
{
  Msp430InterruptC_1_HplInterrupt_clear();
  Msp430InterruptC_1_Interrupt_fired();
}

static inline void HplMsp430InterruptP_Port10_fired( void ) 
{
  Msp430InterruptC_1_HplInterrupt_fired();
}

inline static void HplMsp430InterruptP_Port11_clear( void ) 
{
  _P1IFG &= ~1 << 1;
}

inline static void HplMsp430InterruptP_Port11_default_fired( void ) 
{
  HplMsp430InterruptP_Port11_clear();
}

static inline void HplMsp430InterruptP_Port11_fired( void ) 
{
  HplMsp430InterruptP_Port11_default_fired();
}

inline static void HplMsp430InterruptP_Port12_clear( void ) 
{
  _P1IFG &= ~1 << 2;
}

inline static void HplMsp430InterruptP_Port12_default_fired( void ) 
{
  HplMsp430InterruptP_Port12_clear();
}

static inline void HplMsp430InterruptP_Port12_fired( void ) 
{
  HplMsp430InterruptP_Port12_default_fired();
}

inline static void HplMsp430InterruptP_Port13_clear( void ) 
{
  _P1IFG &= ~1 << 3;
}

inline static void HplMsp430InterruptP_Port13_default_fired( void ) 
{
  HplMsp430InterruptP_Port13_clear();
}

static inline void HplMsp430InterruptP_Port13_fired( void ) 
{
  HplMsp430InterruptP_Port13_default_fired();
}

static inline error_t CC2420CsmaP_startDone_task_postTask( void ) 
{
  char unsigned result;
  result = SchedulerBasicP_TaskBasic_postTask(CC2420CsmaP_startDone_task);
  return result;
}

inline static void CC2420CsmaP_CC2420Power_startOscillatorDone( void ) 
{
  CC2420CsmaP_startDone_task_postTask();
}

static inline void CC2420ControlP_CC2420Power_startOscillatorDone( void ) 
{
  CC2420CsmaP_CC2420Power_startOscillatorDone();
}

static inline error_t CC2420ControlP_InterruptCCA_disable( void ) 
{
  char unsigned result;
  result = Msp430InterruptC_0_Interrupt_disable();
  return result;
}

inline static void CC2420ControlP_InterruptCCA_fired( void ) 
{
  CC2420ControlP_m_state = CC2420ControlP_S_XOSC_STARTED;
  CC2420ControlP_InterruptCCA_disable();
  CC2420ControlP_IOCFG1_write(0);
  CC2420ControlP_writeId();
  CC2420ControlP_CSN_set();
  CC2420ControlP_CSN_clr();
  CC2420ControlP_CC2420Power_startOscillatorDone();
}

static inline void Msp430InterruptC_0_Interrupt_fired( void ) 
{
  CC2420ControlP_InterruptCCA_fired();
}

inline static void Msp430InterruptC_0_HplInterrupt_fired( void ) 
{
  Msp430InterruptC_0_HplInterrupt_clear();
  Msp430InterruptC_0_Interrupt_fired();
}

static inline void HplMsp430InterruptP_Port14_fired( void ) 
{
  Msp430InterruptC_0_HplInterrupt_fired();
}

inline static void HplMsp430InterruptP_Port15_clear( void ) 
{
  _P1IFG &= ~1 << 5;
}

inline static void HplMsp430InterruptP_Port15_default_fired( void ) 
{
  HplMsp430InterruptP_Port15_clear();
}

static inline void HplMsp430InterruptP_Port15_fired( void ) 
{
  HplMsp430InterruptP_Port15_default_fired();
}

inline static void HplMsp430InterruptP_Port16_clear( void ) 
{
  _P1IFG &= ~1 << 6;
}

inline static void HplMsp430InterruptP_Port16_default_fired( void ) 
{
  HplMsp430InterruptP_Port16_clear();
}

static inline void HplMsp430InterruptP_Port16_fired( void ) 
{
  HplMsp430InterruptP_Port16_default_fired();
}

inline static void HplMsp430InterruptP_Port17_clear( void ) 
{
  _P1IFG &= ~1 << 7;
}

inline static void HplMsp430InterruptP_Port17_default_fired( void ) 
{
  HplMsp430InterruptP_Port17_clear();
}

static inline void HplMsp430InterruptP_Port17_fired( void ) 
{
  HplMsp430InterruptP_Port17_default_fired();
}

inline static void HplMsp430InterruptP_Port20_clear( void ) 
{
  _P2IFG &= ~1 << 0;
}

inline static void HplMsp430InterruptP_Port20_default_fired( void ) 
{
  HplMsp430InterruptP_Port20_clear();
}

static inline void HplMsp430InterruptP_Port20_fired( void ) 
{
  HplMsp430InterruptP_Port20_default_fired();
}

inline static void HplMsp430InterruptP_Port21_clear( void ) 
{
  _P2IFG &= ~1 << 1;
}

inline static void HplMsp430InterruptP_Port21_default_fired( void ) 
{
  HplMsp430InterruptP_Port21_clear();
}

static inline void HplMsp430InterruptP_Port21_fired( void ) 
{
  HplMsp430InterruptP_Port21_default_fired();
}

inline static void HplMsp430InterruptP_Port22_clear( void ) 
{
  _P2IFG &= ~1 << 2;
}

inline static void HplMsp430InterruptP_Port22_default_fired( void ) 
{
  HplMsp430InterruptP_Port22_clear();
}

static inline void HplMsp430InterruptP_Port22_fired( void ) 
{
  HplMsp430InterruptP_Port22_default_fired();
}

inline static void HplMsp430InterruptP_Port23_clear( void ) 
{
  _P2IFG &= ~1 << 3;
}

inline static void HplMsp430InterruptP_Port23_default_fired( void ) 
{
  HplMsp430InterruptP_Port23_clear();
}

static inline void HplMsp430InterruptP_Port23_fired( void ) 
{
  HplMsp430InterruptP_Port23_default_fired();
}

inline static void HplMsp430InterruptP_Port24_clear( void ) 
{
  _P2IFG &= ~1 << 4;
}

inline static void HplMsp430InterruptP_Port24_default_fired( void ) 
{
  HplMsp430InterruptP_Port24_clear();
}

static inline void HplMsp430InterruptP_Port24_fired( void ) 
{
  HplMsp430InterruptP_Port24_default_fired();
}

inline static void HplMsp430InterruptP_Port25_clear( void ) 
{
  _P2IFG &= ~1 << 5;
}

inline static void HplMsp430InterruptP_Port25_default_fired( void ) 
{
  HplMsp430InterruptP_Port25_clear();
}

static inline void HplMsp430InterruptP_Port25_fired( void ) 
{
  HplMsp430InterruptP_Port25_default_fired();
}

inline static void HplMsp430InterruptP_Port26_clear( void ) 
{
  _P2IFG &= ~1 << 6;
}

inline static void HplMsp430InterruptP_Port26_default_fired( void ) 
{
  HplMsp430InterruptP_Port26_clear();
}

static inline void HplMsp430InterruptP_Port26_fired( void ) 
{
  HplMsp430InterruptP_Port26_default_fired();
}

inline static void HplMsp430InterruptP_Port27_clear( void ) 
{
  _P2IFG &= ~1 << 7;
}

inline static void HplMsp430InterruptP_Port27_default_fired( void ) 
{
  HplMsp430InterruptP_Port27_clear();
}

static inline void HplMsp430InterruptP_Port27_fired( void ) 
{
  HplMsp430InterruptP_Port27_default_fired();
}

static inline uint8_t Msp430UsartShareP_0_ArbiterInfo_userId( void ) 
{
  char unsigned result;
  result = ArbiterP_0_ArbiterInfo_userId();
  return result;
}

inline static void HplMsp430Usart0P_Usart_disableRxIntr( void ) 
{
  _IE1 &= ~1 << 6;
}

static inline void Msp430SpiNoDmaP_0_Usart_disableRxIntr( void ) 
{
  HplMsp430Usart0P_Usart_disableRxIntr();
}

inline static void Msp430SpiNoDmaP_0_UsartInterrupts_rxDone(uint8_t data) 
{
  if(Msp430SpiNoDmaP_0_m_rx_buf)
  {
    Msp430SpiNoDmaP_0_m_rx_buf[Msp430SpiNoDmaP_0_m_pos - 1] = data;
  }
  /* IRQ INSTR 228 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 229 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  if(Msp430SpiNoDmaP_0_m_pos < Msp430SpiNoDmaP_0_m_len)
  {
    Msp430SpiNoDmaP_0_continueOp();
  }
  else 
  {
    Msp430SpiNoDmaP_0_Usart_disableRxIntr();
    Msp430SpiNoDmaP_0_signalDone();
  }
}

inline static void Msp430UsartShareP_0_Interrupts_default_rxDone(uint8_t id, uint8_t data) 
{
}

static inline void Msp430UsartShareP_0_Interrupts_rxDone(uint8_t arg_0x1dac1d8, uint8_t data) 
{
  switch (arg_0x1dac1d8) 
  {
    case Msp430Usart0C_0_CLIENT_ID:
      Msp430SpiNoDmaP_0_UsartInterrupts_rxDone(data);
      break;
    default :
      Msp430UsartShareP_0_Interrupts_default_rxDone(arg_0x1dac1d8, data);
      break;
  }

}

static inline bool Msp430UsartShareP_0_ArbiterInfo_inUse( void ) 
{
  char unsigned result;
  result = ArbiterP_0_ArbiterInfo_inUse();
  return result;
}

inline static void Msp430UsartShareP_0_RawInterrupts_rxDone(uint8_t data) 
{
  if(Msp430UsartShareP_0_ArbiterInfo_inUse())
  {
    Msp430UsartShareP_0_Interrupts_rxDone(Msp430UsartShareP_0_ArbiterInfo_userId(), data);
  }
}

static inline void HplMsp430Usart0P_Interrupts_rxDone(uint8_t data) 
{
  Msp430UsartShareP_0_RawInterrupts_rxDone(data);
}

inline static bool HplMsp430I2C0P_HplI2C_isI2C( void ) 
{
{
  char unsigned __nesc_temp = ((_U0CTL & 0x20) && (_U0CTL & 0x04)) && (_U0CTL & 0x01);
  return __nesc_temp;
}
}

static inline bool HplMsp430Usart0P_HplI2C_isI2C( void ) 
{
  char unsigned result;
  result = HplMsp430I2C0P_HplI2C_isI2C();
  return result;
}

inline static void Msp430UsartShareP_0_I2CInterrupts_default_fired(uint8_t id) 
{
}

static inline void Msp430UsartShareP_0_I2CInterrupts_fired(uint8_t arg_0x1daca58) 
{
  Msp430UsartShareP_0_I2CInterrupts_default_fired(arg_0x1daca58);
}

inline static void Msp430UsartShareP_0_RawI2CInterrupts_fired( void ) 
{
  if(Msp430UsartShareP_0_ArbiterInfo_inUse())
  {
    Msp430UsartShareP_0_I2CInterrupts_fired(Msp430UsartShareP_0_ArbiterInfo_userId());
  }
}

static inline void HplMsp430Usart0P_I2CInterrupts_fired( void ) 
{
  Msp430UsartShareP_0_RawI2CInterrupts_fired();
}

inline static void Msp430SpiNoDmaP_0_UsartInterrupts_txDone( void ) 
{
}

inline static void Msp430UsartShareP_0_Interrupts_default_txDone(uint8_t id) 
{
}

static inline void Msp430UsartShareP_0_Interrupts_txDone(uint8_t arg_0x1dac1d8) 
{
  switch (arg_0x1dac1d8) 
  {
    case Msp430Usart0C_0_CLIENT_ID:
      Msp430SpiNoDmaP_0_UsartInterrupts_txDone();
      break;
    default :
      Msp430UsartShareP_0_Interrupts_default_txDone(arg_0x1dac1d8);
      break;
  }

}

inline static void Msp430UsartShareP_0_RawInterrupts_txDone( void ) 
{
  if(Msp430UsartShareP_0_ArbiterInfo_inUse())
  {
    Msp430UsartShareP_0_Interrupts_txDone(Msp430UsartShareP_0_ArbiterInfo_userId());
  }
}

static inline void HplMsp430Usart0P_Interrupts_txDone( void ) 
{
  Msp430UsartShareP_0_RawInterrupts_txDone();
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

static void Msp430TimerP_1_Event_fired(uint8_t arg_0x1593680) 
{
  switch (arg_0x1593680) 
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
      Msp430TimerP_1_Event_default_fired(arg_0x1593680);
      break;
  }

}

static void TransformAlarmC_0_Alarm_startAt(TransformAlarmC_0_to_size_type t0, TransformAlarmC_0_to_size_type dt) 
{
{
  /* IRQ INSTR 230 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 231 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 232 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 233 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 234 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  TransformAlarmC_0_m_t0 = t0;
  TransformAlarmC_0_m_dt = dt;
  TransformAlarmC_0_set_alarm();
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static void TransformAlarmC_0_set_alarm( void ) 
{
  TransformAlarmC_0_to_size_type now = TransformAlarmC_0_Counter_get();
  TransformAlarmC_0_to_size_type expires;
  TransformAlarmC_0_to_size_type remaining;
  /* IRQ INSTR 235 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 236 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  expires = TransformAlarmC_0_m_t0 + TransformAlarmC_0_m_dt;
  remaining = ( TransformAlarmC_0_to_size_type )(expires - now);
  /* IRQ INSTR 241 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 242 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  if(TransformAlarmC_0_m_t0 <= now)
  {
  /* IRQ INSTR 237 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 238 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    if((expires >= TransformAlarmC_0_m_t0) && (expires <= now))
    {
      remaining = 0;
    }
  }
  else 
  {
  /* IRQ INSTR 239 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 240 */
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
  /* IRQ INSTR 243 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 244 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    TransformAlarmC_0_m_t0 += TransformAlarmC_0_m_dt;
    TransformAlarmC_0_m_dt = 0;
  }
  TransformAlarmC_0_AlarmFrom_startAt(( TransformAlarmC_0_from_size_type )(now) << 0, ( TransformAlarmC_0_from_size_type )(remaining) << 0);
}

static TransformCounterC_0_to_size_type TransformCounterC_0_Counter_get( void ) 
{
  TransformCounterC_0_to_size_type rv = 0;
{
  /* IRQ INSTR 245 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 246 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 247 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 248 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 249 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
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
/*  if(1)
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

static void CC2420TransmitP_congestionBackoff( void ) 
{
{
  /* IRQ INSTR 250 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 251 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 252 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 253 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 254 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  CC2420TransmitP_RadioBackoff_requestCongestionBackoff(CC2420TransmitP_m_msg);
  CC2420TransmitP_BackoffTimer_start(CC2420TransmitP_myCongestionBackoff);
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static uint32_t RandomMlcgC_Random_rand32( void ) 
{
  uint32_t mlcg;
  uint32_t p;
  uint32_t q;
  uint64_t tmpseed;
{
  /* IRQ INSTR 255 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 256 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 257 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 258 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 259 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  tmpseed = ( uint64_t )(33614U) * ( uint64_t )(RandomMlcgC_seed);
  q = tmpseed;
  q = q >> 1;
  p = tmpseed >> 32;
  mlcg = p + q;
  if(mlcg & 0x80000000)
  {
    mlcg = mlcg & 0x7FFFFFFF;
    mlcg++;
  }
  RandomMlcgC_seed = mlcg;
}
  __nesc_atomic_end(__nesc_atomic);
}
  return mlcg;
}

static error_t CC2420TransmitP_acquireSpiResource( void ) 
{
  error_t error = CC2420TransmitP_SpiResource_immediateRequest();
  if(error != SUCCESS)
  {
    CC2420TransmitP_SpiResource_request();
  }
  return error;
}

static error_t CC2420SpiP_Resource_immediateRequest(uint8_t id) 
{
  error_t error;
{
  /* IRQ INSTR 260 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 261 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 262 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 263 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 264 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(CC2420SpiP_WorkingState_requestState(CC2420SpiP_S_BUSY) != SUCCESS)
  {
  {
    char unsigned __nesc_temp = EBUSY;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
  if(CC2420SpiP_SpiResource_isOwner())
  {
    CC2420SpiP_m_holder = id;
    error = SUCCESS;
  }
  else 
  {
    if((error = CC2420SpiP_SpiResource_immediateRequest()) == (SUCCESS))
    {
      CC2420SpiP_m_holder = id;
    }
    else 
    {
      CC2420SpiP_WorkingState_toIdle();
    }
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
  return error;
}

static error_t StateImplP_State_requestState(uint8_t id, uint8_t reqState) 
{
  error_t returnVal = FAIL;
{
  /* IRQ INSTR 268 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 269 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 270 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 271 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 272 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  /* IRQ INSTR 265 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 266 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 267 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  if((reqState == StateImplP_S_IDLE) || (StateImplP_state[id] == StateImplP_S_IDLE))
  {
    StateImplP_state[id] = reqState;
    returnVal = SUCCESS;
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
  return returnVal;
}

static uint8_t ArbiterP_0_Resource_isOwner(uint8_t id) 
{
{
  if((ArbiterP_0_resId == id) && (ArbiterP_0_state == ArbiterP_0_RES_BUSY))
  {
    char unsigned __nesc_temp = TRUE;
    return __nesc_temp;
  }
  else 
  {
    char unsigned __nesc_temp = FALSE;
    return __nesc_temp;
  }
}
}

static error_t ArbiterP_0_ResourceDefaultOwner_release( void ) 
{
{
  if(ArbiterP_0_resId == ArbiterP_0_default_owner_id)
  {
    if(ArbiterP_0_state == ArbiterP_0_RES_GRANTING)
    {
      ArbiterP_0_grantedTask_postTask();
    {
      char unsigned __nesc_temp = SUCCESS;
      return __nesc_temp;
    }
    }
    else 
    {
      if(ArbiterP_0_state == ArbiterP_0_RES_IMM_GRANTING)
      {
  /* IRQ INSTR 273 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 274 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 275 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
        ArbiterP_0_resId = ArbiterP_0_reqResId;
        ArbiterP_0_state = ArbiterP_0_RES_BUSY;
      {
        char unsigned __nesc_temp = SUCCESS;
        return __nesc_temp;
      }
      }
    }
  }
}
  return FAIL;
}

static error_t SchedulerBasicP_TaskBasic_postTask(uint8_t id) 
{
{
  /* IRQ INSTR 276 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 277 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 278 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 279 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 280 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
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

static void HplMsp430Usart0P_Usart_setModeSpi(msp430_spi_union_config_t *config) 
{
{
  /* IRQ INSTR 281 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 282 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 283 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 284 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 285 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
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
  __nesc_atomic_end(__nesc_atomic);
}
  return;
}

static error_t CC2420SpiP_Resource_request(uint8_t id) 
{
{
  /* IRQ INSTR 286 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 287 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 288 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 289 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 290 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(CC2420SpiP_WorkingState_requestState(CC2420SpiP_S_BUSY) == SUCCESS)
  {
    CC2420SpiP_m_holder = id;
    if(CC2420SpiP_SpiResource_isOwner())
    {
      CC2420SpiP_grant_postTask();
    }
    else 
    {
      CC2420SpiP_SpiResource_request();
    }
  }
  else 
  {
    CC2420SpiP_m_requests |= 1 << id;
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
  return SUCCESS;
}

static void CC2420TransmitP_attemptSend( void ) 
{
  uint8_t status;
  bool congestion = TRUE;
{
  /* IRQ INSTR 293 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 294 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 295 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 296 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 297 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  /* IRQ INSTR 291 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 292 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  if(CC2420TransmitP_m_state == CC2420TransmitP_S_CANCEL)
  {
    CC2420TransmitP_SFLUSHTX_strobe();
    CC2420TransmitP_releaseSpiResource();
    CC2420TransmitP_CSN_set();
    CC2420TransmitP_m_state = CC2420TransmitP_S_STARTED;
    CC2420TransmitP_Send_sendDone(CC2420TransmitP_m_msg, ECANCEL);
  {
    __nesc_atomic_end(__nesc_atomic);
    return;
  }
  }
  CC2420TransmitP_CSN_clr();
  status = CC2420TransmitP_m_cca? CC2420TransmitP_STXONCCA_strobe() : CC2420TransmitP_STXON_strobe();
  if(!status & CC2420_STATUS_TX_ACTIVE)
  {
    status = CC2420TransmitP_SNOP_strobe();
    if(status & CC2420_STATUS_TX_ACTIVE)
    {
      congestion = FALSE;
    }
  }
  CC2420TransmitP_m_state = congestion? CC2420TransmitP_S_SAMPLE_CCA : CC2420TransmitP_S_SFD;
  CC2420TransmitP_CSN_set();
}
  __nesc_atomic_end(__nesc_atomic);
}
  if(congestion)
  {
    CC2420TransmitP_totalCcaChecks = 0;
    CC2420TransmitP_releaseSpiResource();
    CC2420TransmitP_congestionBackoff();
  }
  else 
  {
    CC2420TransmitP_BackoffTimer_start(CC2420TransmitP_CC2420_ABORT_PERIOD);
  }
}

static cc2420_status_t CC2420SpiP_Strobe_strobe(uint8_t addr) 
{
{
  /* IRQ INSTR 298 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 299 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 300 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 301 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 302 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(CC2420SpiP_WorkingState_isIdle())
  {
  {
    char unsigned __nesc_temp = 0;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
  return CC2420SpiP_SpiByte_write(addr);
}

static bool StateImplP_State_isState(uint8_t id, uint8_t myState) 
{
  bool isState;
{
  /* IRQ INSTR 306 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 307 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 308 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 309 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 310 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  /* IRQ INSTR 303 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 304 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 305 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  isState = StateImplP_state[id] == myState;
  __nesc_atomic_end(__nesc_atomic);
}
  return isState;
}

static uint8_t Msp430SpiNoDmaP_0_SpiByte_write(uint8_t tx) 
{
  uint8_t byte;
  Msp430SpiNoDmaP_0_Usart_tx(tx);
  while(!Msp430SpiNoDmaP_0_Usart_isRxIntrPending())
  ;
  Msp430SpiNoDmaP_0_Usart_clrRxIntr();
  byte = Msp430SpiNoDmaP_0_Usart_rx();
  return byte;
}

static uint8_t HplMsp430Usart0P_Usart_rx( void ) 
{
  uint8_t value;
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  value = _U0RXBUF;
  __nesc_atomic_end(__nesc_atomic);
}
  return value;
}

static error_t CC2420SpiP_Resource_release(uint8_t id) 
{
  uint8_t i;
{
  /* IRQ INSTR 317 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 318 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 319 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 320 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 321 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  /* IRQ INSTR 311 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 312 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 313 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  if(CC2420SpiP_m_holder != id)
  {
  {
    char unsigned __nesc_temp = FAIL;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
  CC2420SpiP_m_holder = CC2420SpiP_NO_HOLDER;
  if(!CC2420SpiP_m_requests)
  {
    CC2420SpiP_WorkingState_toIdle();
    CC2420SpiP_attemptRelease();
  }
  else 
  {
  /* IRQ INSTR 314 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 315 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 316 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
    for(i = CC2420SpiP_m_holder + 1; ; i++)
    {
      i %= CC2420SpiP_RESOURCE_COUNT;
      if(CC2420SpiP_m_requests & (1 << i))
      {
        CC2420SpiP_m_holder = i;
        CC2420SpiP_m_requests &= ~1 << i;
        CC2420SpiP_grant_postTask();
      {
        char unsigned __nesc_temp = SUCCESS;
      {
        __nesc_atomic_end(__nesc_atomic);
        return __nesc_temp;
      }
      }
      }
    }

  }
}
  __nesc_atomic_end(__nesc_atomic);
}
  return SUCCESS;
}

static error_t CC2420SpiP_attemptRelease( void ) 
{
  /* IRQ INSTR 322 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 323 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 324 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  if(((CC2420SpiP_m_requests > 0) || (CC2420SpiP_m_holder != CC2420SpiP_NO_HOLDER)) || !CC2420SpiP_WorkingState_isIdle())
  {
    return FAIL;
  }
  CC2420SpiP_release = TRUE;
  CC2420SpiP_ChipSpiResource_releasing();
{
  if(CC2420SpiP_release)
  {
    CC2420SpiP_SpiResource_release();
  {
    char unsigned __nesc_temp = SUCCESS;
    return __nesc_temp;
  }
  }
}
  return EBUSY;
}

static void HplMsp430Usart0P_Usart_disableSpi( void ) 
{
{
  _ME1 &= ~1 << 6;
  HplMsp430Usart0P_SIMO_selectIOFunc();
  HplMsp430Usart0P_SOMI_selectIOFunc();
  HplMsp430Usart0P_UCLK_selectIOFunc();
}
}

static void HplMsp430GeneralIOP_26_IO_set( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P4OUT |= 0x01 << 2;
  __nesc_atomic_end(__nesc_atomic);
}
}

static void HplMsp430GeneralIOP_26_IO_clr( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P4OUT &= ~0x01 << 2;
  __nesc_atomic_end(__nesc_atomic);
}
}

static void CC2420TransmitP_signalDone(error_t err) 
{
  CC2420TransmitP_m_state = CC2420TransmitP_S_STARTED;
  CC2420TransmitP_abortSpiRelease = FALSE;
  CC2420TransmitP_ChipSpiResource_attemptRelease();
  CC2420TransmitP_Send_sendDone(CC2420TransmitP_m_msg, err);
}

static error_t GpioCaptureC_0_enableCapture(uint8_t mode) 
{
{
  GpioCaptureC_0_Msp430TimerControl_disableEvents();
  GpioCaptureC_0_GeneralIO_selectModuleFunc();
  GpioCaptureC_0_Msp430TimerControl_clearPendingInterrupt();
  GpioCaptureC_0_Msp430Capture_clearOverflow();
  GpioCaptureC_0_Msp430TimerControl_setControlAsCapture(mode);
  GpioCaptureC_0_Msp430TimerControl_enableEvents();
}
  return SUCCESS;
}

static void HplMsp430GeneralIOP_30_IO_clr( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P4OUT &= ~0x01 << 6;
  __nesc_atomic_end(__nesc_atomic);
}
}

static void HplMsp430GeneralIOP_30_IO_set( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P4OUT |= 0x01 << 6;
  __nesc_atomic_end(__nesc_atomic);
}
}

static cc2420_status_t CC2420SpiP_Ram_write(uint16_t addr, uint8_t offset, uint8_t *data, uint8_t len) 
{
  cc2420_status_t status = 0;
  uint8_t tmpLen = len;
  uint8_t *tmpData = ( uint8_t * )(data);
{
  /* IRQ INSTR 325 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 326 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 327 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 328 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 329 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(CC2420SpiP_WorkingState_isIdle())
  {
  {
    char unsigned __nesc_temp = status;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
  addr += offset;
  status = CC2420SpiP_SpiByte_write(addr | 0x80);
  CC2420SpiP_SpiByte_write((addr >> 1) & 0xc0);
  for(; len; len--)
  {
    CC2420SpiP_SpiByte_write(tmpData[tmpLen - len]);
  }

  return status;
}

static void CC2420PacketP_PacketTimeStamp32khz_clear(message_t *msg) 
{
  __nesc_hton_int8(CC2420PacketP_CC2420PacketBody_getMetadata(msg)->timesync.data, FALSE);
  __nesc_hton_uint32(CC2420PacketP_CC2420PacketBody_getMetadata(msg)->timestamp.data, CC2420_INVALID_TIMESTAMP);
}

static void TransformAlarmC_1_set_alarm( void ) 
{
  TransformAlarmC_1_to_size_type now = TransformAlarmC_1_Counter_get();
  TransformAlarmC_1_to_size_type expires;
  TransformAlarmC_1_to_size_type remaining;
  /* IRQ INSTR 330 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  expires = TransformAlarmC_1_m_t0 + TransformAlarmC_1_m_dt;
  remaining = ( TransformAlarmC_1_to_size_type )(expires - now);
  /* IRQ INSTR 333 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  if(TransformAlarmC_1_m_t0 <= now)
  {
  /* IRQ INSTR 331 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    if((expires >= TransformAlarmC_1_m_t0) && (expires <= now))
    {
      remaining = 0;
    }
  }
  else 
  {
  /* IRQ INSTR 332 */
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
  /* IRQ INSTR 334 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
    TransformAlarmC_1_m_t0 += TransformAlarmC_1_m_dt;
    TransformAlarmC_1_m_dt = 0;
  }
  TransformAlarmC_1_AlarmFrom_startAt(( TransformAlarmC_1_from_size_type )(now) << 5, ( TransformAlarmC_1_from_size_type )(remaining) << 5);
}

static TransformCounterC_1_to_size_type TransformCounterC_1_Counter_get( void ) 
{
  TransformCounterC_1_to_size_type rv = 0;
{
  /* IRQ INSTR 335 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 336 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 337 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 338 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 339 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  TransformCounterC_1_upper_count_type high = TransformCounterC_1_m_upper;
  TransformCounterC_1_from_size_type low = TransformCounterC_1_CounterFrom_get();
  if(TransformCounterC_1_CounterFrom_isOverflowPending())
  {
    high++;
    low = TransformCounterC_1_CounterFrom_get();
  }
{
  TransformCounterC_1_to_size_type high_to = high;
  TransformCounterC_1_to_size_type low_to = low >> TransformCounterC_1_LOW_SHIFT_RIGHT;
  rv = (high_to << TransformCounterC_1_HIGH_SHIFT_LEFT) | low_to;
}
}
  __nesc_atomic_end(__nesc_atomic);
}
  return rv;
}

int main( void ) 
{
{
  /* IRQ INSTR 340 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 341 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 342 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 343 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 344 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
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

static void Msp430ClockP_set_dco_calib(int calib) 
{
  _BCSCTL1 = (_BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  _DCOCTL = calib & 0xff;
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
    case CC2420CsmaP_startDone_task:
      CC2420CsmaP_startDone_task_runTask();
      break;
    case CC2420CsmaP_stopDone_task:
      CC2420CsmaP_stopDone_task_runTask();
      break;
    case CC2420CsmaP_sendDone_task:
      CC2420CsmaP_sendDone_task_runTask();
      break;
    case CC2420ControlP_sync:
      CC2420ControlP_sync_runTask();
      break;
    case CC2420ControlP_syncDone:
      CC2420ControlP_syncDone_runTask();
      break;
    case CC2420SpiP_grant:
      CC2420SpiP_grant_runTask();
      break;
    case Msp430SpiNoDmaP_0_signalDone_task:
      Msp430SpiNoDmaP_0_signalDone_task_runTask();
      break;
    case ArbiterP_0_grantedTask:
      ArbiterP_0_grantedTask_runTask();
      break;
    case CC2420ReceiveP_receiveDone_task:
      CC2420ReceiveP_receiveDone_task_runTask();
      break;
    case AlarmToTimerC_0_fired:
      AlarmToTimerC_0_fired_runTask();
      break;
    case VirtualizeTimerC_0_updateFromTimer:
      VirtualizeTimerC_0_updateFromTimer_runTask();
      break;
    case AMQueueImplP_0_CancelTask:
      AMQueueImplP_0_CancelTask_runTask();
      break;
    case AMQueueImplP_0_errorTask:
      AMQueueImplP_0_errorTask_runTask();
      break;
    case DisseminatorP_0_changedTask:
      DisseminatorP_0_changedTask_runTask();
      break;
    case TrickleTimerImplP_0_timerTask:
      TrickleTimerImplP_0_timerTask_runTask();
      break;
    case DisseminatorP_1_changedTask:
      DisseminatorP_1_changedTask_runTask();
      break;
    default :
      SchedulerBasicP_TaskBasic_default_runTask(arg_0x147bb38);
      break;
  }

}

static void TestDisseminationC_Value16_changed( void ) 
{
  const uint16_t *newVal = TestDisseminationC_Value16_get();
  if(*newVal == 0xABCD)
  {
    TestDisseminationC_Leds_led1Toggle();
  }
  else 
  {
  }
}

static void HplMsp430GeneralIOP_37_IO_toggle( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P5OUT ^= 0x01 << 5;
  __nesc_atomic_end(__nesc_atomic);
}
}

static void DisseminationEngineImplP_sendObject(uint16_t key) 
{
  void *object;
  uint8_t objectSize = 0;
  dissemination_message_t *dMsg = ( dissemination_message_t * )(DisseminationEngineImplP_AMSend_getPayload(&DisseminationEngineImplP_m_buf, sizeof( dissemination_message_t )));
  if(dMsg != _IE1)
  {
    DisseminationEngineImplP_m_bufBusy = TRUE;
    __nesc_hton_uint16(dMsg->key.data, key);
    __nesc_hton_uint32(dMsg->seqno.data, DisseminationEngineImplP_DisseminationCache_requestSeqno(key));
    if(__nesc_ntoh_uint32(dMsg->seqno.data) != DISSEMINATION_SEQNO_UNKNOWN)
    {
      object = DisseminationEngineImplP_DisseminationCache_requestData(key, &objectSize);
      if((objectSize + sizeof( dissemination_message_t )) > DisseminationEngineImplP_AMSend_maxPayloadLength())
      {
        objectSize = DisseminationEngineImplP_AMSend_maxPayloadLength() - sizeof( dissemination_message_t );
      }
      memcpy(dMsg->data, object, objectSize);
    }
    DisseminationEngineImplP_AMSend_send(AM_BROADCAST_ADDR, &DisseminationEngineImplP_m_buf, sizeof( dissemination_message_t ) + objectSize);
  }
}

static void *CC2420CsmaP_Send_getPayload(message_t *m, uint8_t len) 
{
  if(len <= CC2420CsmaP_Send_maxPayloadLength())
  {
    return ( void * )(m->data);
  }
  else 
  {
    return _IE1;
  }
}

static am_id_t CC2420ActiveMessageP_AMPacket_type(message_t *amsg) 
{
  cc2420_header_t *header = CC2420ActiveMessageP_CC2420PacketBody_getHeader(amsg);
  return __nesc_ntoh_leuint8(header->type.data);
}

static am_addr_t CC2420ActiveMessageP_AMPacket_destination(message_t *amsg) 
{
  cc2420_header_t *header = CC2420ActiveMessageP_CC2420PacketBody_getHeader(amsg);
  return __nesc_ntoh_leuint16(header->dest.data);
}

static error_t CC2420ActiveMessageP_AMSend_send(am_id_t id, am_addr_t addr, message_t *msg, uint8_t len) 
{
  cc2420_header_t *header = CC2420ActiveMessageP_CC2420PacketBody_getHeader(msg);
  if(len > CC2420ActiveMessageP_Packet_maxPayloadLength())
  {
    return ESIZE;
  }
  __nesc_hton_leuint8(header->type.data, id);
  __nesc_hton_leuint16(header->dest.data, addr);
  __nesc_hton_leuint16(header->destpan.data, CC2420ActiveMessageP_CC2420Config_getPanAddr());
  __nesc_hton_leuint16(header->src.data, CC2420ActiveMessageP_AMPacket_address());
  CC2420ActiveMessageP_SendNotifier_aboutToSend(id, addr, msg);
  return CC2420ActiveMessageP_SubSend_send(msg, len);
}

static am_addr_t ActiveMessageAddressC_amAddress( void ) 
{
  am_addr_t myAddr;
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  myAddr = ActiveMessageAddressC_addr;
  __nesc_atomic_end(__nesc_atomic);
}
  return myAddr;
}

static void CC2420TransmitP_loadTXFIFO( void ) 
{
  cc2420_header_t *header = CC2420TransmitP_CC2420PacketBody_getHeader(CC2420TransmitP_m_msg);
  uint8_t tx_power = __nesc_ntoh_uint8(CC2420TransmitP_CC2420PacketBody_getMetadata(CC2420TransmitP_m_msg)->tx_power.data);
  if(!tx_power)
  {
    tx_power = 31;
  }
  CC2420TransmitP_CSN_clr();
  if(CC2420TransmitP_m_tx_power != tx_power)
  {
    CC2420TransmitP_TXCTRL_write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (3 << CC2420_TXCTRL_PA_CURRENT)) | (1 << CC2420_TXCTRL_RESERVED)) | ((tx_power & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
  }
  CC2420TransmitP_m_tx_power = tx_power;
{
  uint8_t tmpLen = __nesc_ntoh_leuint8(header->length.data) - 1;
  CC2420TransmitP_TXFIFO_write(( uint8_t * )(header), __nesc_ntoh_leuint8(header->length.data) - 1);
}
}

static cc2420_status_t CC2420SpiP_Reg_write(uint8_t addr, uint16_t data) 
{
{
  /* IRQ INSTR 345 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 346 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 347 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 348 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 349 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(CC2420SpiP_WorkingState_isIdle())
  {
  {
    char unsigned __nesc_temp = 0;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
  CC2420SpiP_SpiByte_write(addr);
  CC2420SpiP_SpiByte_write(data >> 8);
  return CC2420SpiP_SpiByte_write(data & 0xff);
}

static error_t Msp430SpiNoDmaP_0_SpiPacket_send(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) 
{
  Msp430SpiNoDmaP_0_m_client = id;
  Msp430SpiNoDmaP_0_m_tx_buf = tx_buf;
  Msp430SpiNoDmaP_0_m_rx_buf = rx_buf;
  Msp430SpiNoDmaP_0_m_len = len;
  Msp430SpiNoDmaP_0_m_pos = 0;
  if(len)
  {
    Msp430SpiNoDmaP_0_Usart_enableRxIntr();
    Msp430SpiNoDmaP_0_continueOp();
  }
  else 
  {
    Msp430SpiNoDmaP_0_signalDone_task_postTask();
  }
  return SUCCESS;
}

static void Msp430SpiNoDmaP_0_continueOp( void ) 
{
  uint8_t end;
  uint8_t tmp;
{
  /* IRQ INSTR 360 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 361 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 362 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 363 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 364 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  /* IRQ INSTR 350 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 351 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  Msp430SpiNoDmaP_0_Usart_tx(Msp430SpiNoDmaP_0_m_tx_buf? Msp430SpiNoDmaP_0_m_tx_buf[Msp430SpiNoDmaP_0_m_pos] : 0);
  /* IRQ INSTR 352 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 353 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  end = Msp430SpiNoDmaP_0_m_pos + Msp430SpiNoDmaP_0_SPI_ATOMIC_SIZE;
  /* IRQ INSTR 356 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 357 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  if(end > Msp430SpiNoDmaP_0_m_len)
  {
  /* IRQ INSTR 354 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 355 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
    end = Msp430SpiNoDmaP_0_m_len;
  }
  while(++Msp430SpiNoDmaP_0_m_pos < end)
  {
    while(!Msp430SpiNoDmaP_0_Usart_isRxIntrPending())
    ;
    tmp = Msp430SpiNoDmaP_0_Usart_rx();
    if(Msp430SpiNoDmaP_0_m_rx_buf)
    {
      Msp430SpiNoDmaP_0_m_rx_buf[Msp430SpiNoDmaP_0_m_pos - 1] = tmp;
    }
  /* IRQ INSTR 358 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 359 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
    Msp430SpiNoDmaP_0_Usart_tx(Msp430SpiNoDmaP_0_m_tx_buf? Msp430SpiNoDmaP_0_m_tx_buf[Msp430SpiNoDmaP_0_m_pos] : 0);
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static void UniqueSendP_State_toIdle( void ) 
{
  StateImplP_State_toIdle(2U);
}

static void TestDisseminationC_Value32_changed( void ) 
{
  const uint32_t *newVal = TestDisseminationC_Value32_get();
  if(*newVal == 0xDEADBEEF)
  {
    TestDisseminationC_Leds_led0Toggle();
  }
  else 
  {
  }
}

static void HplMsp430GeneralIOP_36_IO_toggle( void ) 
{
{
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  _P5OUT ^= 0x01 << 4;
  __nesc_atomic_end(__nesc_atomic);
}
}

static void AMQueueImplP_0_sendDone(uint8_t last, message_t *msg, error_t err) 
{
  AMQueueImplP_0_queue[last].msg = _IE1;
  AMQueueImplP_0_tryToSend();
  AMQueueImplP_0_Send_sendDone(last, msg, err);
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

static void DisseminationEngineImplP_DisseminationCache_newData(uint16_t key) 
{
  if(!DisseminationEngineImplP_m_running || DisseminationEngineImplP_m_bufBusy)
  {
    return;
  }
  DisseminationEngineImplP_sendObject(key);
  DisseminationEngineImplP_TrickleTimer_reset(key);
}

static void TrickleTimerImplP_0_TrickleTimer_reset(uint8_t id) 
{
  TrickleTimerImplP_0_trickles[id].period = 1;
  TrickleTimerImplP_0_trickles[id].count = 0;
  if(TrickleTimerImplP_0_trickles[id].time != 0)
  {
  {
  /* IRQ INSTR 365 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 366 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 367 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 368 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 369 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
    __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
  {
    TrickleTimerImplP_0_Changed_set(id);
  }
    __nesc_atomic_end(__nesc_atomic);
  }
    TrickleTimerImplP_0_trickles[id].time = 0;
    TrickleTimerImplP_0_trickles[id].remainder = 0;
    TrickleTimerImplP_0_generateTime(id);
    TrickleTimerImplP_0_adjustTimer();
  }
  else 
  {
  }
}

static void TrickleTimerImplP_0_generateTime(uint8_t id) 
{
  uint32_t newTime;
  uint16_t rval;
  if(TrickleTimerImplP_0_trickles[id].time != 0)
  {
    TrickleTimerImplP_0_trickles[id].period *= 2;
    if(TrickleTimerImplP_0_trickles[id].period > 1024)
    {
      TrickleTimerImplP_0_trickles[id].period = 1024;
    }
  }
  TrickleTimerImplP_0_trickles[id].time = TrickleTimerImplP_0_trickles[id].remainder;
  newTime = TrickleTimerImplP_0_trickles[id].period;
  newTime = newTime << (10 - 1);
  rval = TrickleTimerImplP_0_Random_rand16() % (TrickleTimerImplP_0_trickles[id].period << (10 - 1));
  newTime += rval;
  TrickleTimerImplP_0_trickles[id].remainder = (TrickleTimerImplP_0_trickles[id].period << 10) - newTime;
  TrickleTimerImplP_0_trickles[id].time += newTime;
}

static void TrickleTimerImplP_0_adjustTimer( void ) 
{
  uint8_t i;
  uint32_t lowest = 0;
  bool set = FALSE;
  uint32_t elapsed = TrickleTimerImplP_0_Timer_getNow() - TrickleTimerImplP_0_Timer_gett0();
  for(i = 0; i < 2U; i++)
  {
    uint32_t timeRemaining = TrickleTimerImplP_0_trickles[i].time;
    if(timeRemaining != 0)
    {
    {
  /* IRQ INSTR 370 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 371 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 372 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 373 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 374 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
      __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if(!TrickleTimerImplP_0_Changed_get(i))
      {
        TrickleTimerImplP_0_Changed_clear(i);
        timeRemaining -= elapsed;
      }
    }
      __nesc_atomic_end(__nesc_atomic);
    }
      if(!set)
      {
        lowest = timeRemaining;
        set = TRUE;
      }
      else 
      {
        if(timeRemaining < lowest)
        {
          lowest = timeRemaining;
        }
      }
    }
  }

  if(set)
  {
    uint32_t timerVal = lowest;
    timerVal = timerVal;
    TrickleTimerImplP_0_Timer_startOneShot(timerVal);
  }
  else 
  {
    TrickleTimerImplP_0_Timer_stop();
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

static void TransformAlarmC_1_Alarm_startAt(TransformAlarmC_1_to_size_type t0, TransformAlarmC_1_to_size_type dt) 
{
{
  /* IRQ INSTR 375 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 376 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 377 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 378 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 379 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  TransformAlarmC_1_m_t0 = t0;
  TransformAlarmC_1_m_dt = dt;
  TransformAlarmC_1_set_alarm();
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static uint16_t CC2420ControlP_CC2420Config_getShortAddr( void ) 
{
{
  /* IRQ INSTR 380 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 381 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 382 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 383 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 384 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  int unsigned __nesc_temp = CC2420ControlP_m_short_addr;
{
  __nesc_atomic_end(__nesc_atomic);
  return __nesc_temp;
}
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static void DisseminatorP_1_DisseminationCache_storeData(void *data, uint8_t size, uint32_t newSeqno) 
{
  memcpy(&DisseminatorP_1_valueCache, data, size < sizeof( DisseminatorP_1_t )? size : sizeof( DisseminatorP_1_t ));
  DisseminatorP_1_seqno = newSeqno;
  DisseminatorP_1_DisseminationValue_changed();
}

static void DisseminatorP_0_DisseminationCache_storeData(void *data, uint8_t size, uint32_t newSeqno) 
{
  memcpy(&DisseminatorP_0_valueCache, data, size < sizeof( DisseminatorP_0_t )? size : sizeof( DisseminatorP_0_t ));
  DisseminatorP_0_seqno = newSeqno;
  DisseminatorP_0_DisseminationValue_changed();
}

static void CC2420ReceiveP_waitForNextPacket( void ) 
{
{
  /* IRQ INSTR 387 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 388 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 389 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 390 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 391 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  /* IRQ INSTR 385 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 386 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  if(CC2420ReceiveP_m_state == CC2420ReceiveP_S_STOPPED)
  {
    CC2420ReceiveP_SpiResource_release();
  {
    __nesc_atomic_end(__nesc_atomic);
    return;
  }
  }
  CC2420ReceiveP_receivingPacket = FALSE;
  if((CC2420ReceiveP_m_missed_packets && CC2420ReceiveP_FIFO_get()) || !CC2420ReceiveP_FIFOP_get())
  {
    if(CC2420ReceiveP_m_missed_packets)
    {
      CC2420ReceiveP_m_missed_packets--;
    }
    CC2420ReceiveP_beginReceive();
  }
  else 
  {
    CC2420ReceiveP_m_state = CC2420ReceiveP_S_STARTED;
    CC2420ReceiveP_m_missed_packets = 0;
    CC2420ReceiveP_SpiResource_release();
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static void CC2420ReceiveP_beginReceive( void ) 
{
  CC2420ReceiveP_m_state = CC2420ReceiveP_S_RX_LENGTH;
  CC2420ReceiveP_receivingPacket = TRUE;
  if(CC2420ReceiveP_SpiResource_isOwner())
  {
    CC2420ReceiveP_receive();
  }
  else 
  {
    if(CC2420ReceiveP_SpiResource_immediateRequest() == SUCCESS)
    {
      CC2420ReceiveP_receive();
    }
    else 
    {
      CC2420ReceiveP_SpiResource_request();
    }
  }
}

static void CC2420ReceiveP_receive( void ) 
{
  CC2420ReceiveP_CSN_clr();
  CC2420ReceiveP_RXFIFO_beginRead(( uint8_t * )(CC2420ReceiveP_CC2420PacketBody_getHeader(CC2420ReceiveP_m_p_rx_buf)), 1);
}

static cc2420_status_t CC2420SpiP_Fifo_beginRead(uint8_t addr, uint8_t *data, uint8_t len) 
{
  cc2420_status_t status = 0;
{
  /* IRQ INSTR 392 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 393 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 394 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 395 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 396 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  if(CC2420SpiP_WorkingState_isIdle())
  {
  {
    char unsigned __nesc_temp = status;
  {
    __nesc_atomic_end(__nesc_atomic);
    return __nesc_temp;
  }
  }
  }
}
  __nesc_atomic_end(__nesc_atomic);
}
  CC2420SpiP_m_addr = addr | 0x40;
  status = CC2420SpiP_SpiByte_write(CC2420SpiP_m_addr);
  CC2420SpiP_Fifo_continueRead(addr, data, len);
  return status;
}

static void CC2420SpiP_SpiPacket_sendDone(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error) 
{
  if(CC2420SpiP_m_addr & 0x40)
  {
    CC2420SpiP_Fifo_readDone(CC2420SpiP_m_addr & ~0x40, rx_buf, len, error);
  }
  else 
  {
    CC2420SpiP_Fifo_writeDone(CC2420SpiP_m_addr, tx_buf, len, error);
  }
}

static void CC2420ReceiveP_flush( void ) 
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

static void CC2420ReceiveP_reset_state( void ) 
{
  CC2420ReceiveP_m_bytes_left = CC2420ReceiveP_RXFIFO_SIZE;
  CC2420ReceiveP_receivingPacket = FALSE;
  CC2420ReceiveP_m_timestamp_head = 0;
  CC2420ReceiveP_m_timestamp_size = 0;
  CC2420ReceiveP_m_missed_packets = 0;
}

static void CC2420ControlP_writeFsctrl( void ) 
{
  uint8_t channel;
{
  /* IRQ INSTR 397 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 398 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 399 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 400 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 401 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  channel = CC2420ControlP_m_channel;
}
  __nesc_atomic_end(__nesc_atomic);
}
  CC2420ControlP_FSCTRL_write((1 << CC2420_FSCTRL_LOCK_THR) | ((((channel - 11) * 5) + 357) << CC2420_FSCTRL_FREQ));
}

static void CC2420ControlP_writeMdmctrl0( void ) 
{
{
  /* IRQ INSTR 402 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 403 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 404 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 405 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 406 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  CC2420ControlP_MDMCTRL0_write((((((((1 << CC2420_MDMCTRL0_RESERVED_FRAME_MODE) | ((CC2420ControlP_addressRecognition && CC2420ControlP_hwAddressRecognition) << CC2420_MDMCTRL0_ADR_DECODE)) | (2 << CC2420_MDMCTRL0_CCA_HYST)) | (3 << CC2420_MDMCTRL0_CCA_MOD)) | (1 << CC2420_MDMCTRL0_AUTOCRC)) | ((CC2420ControlP_autoAckEnabled && CC2420ControlP_hwAutoAckDefault) << CC2420_MDMCTRL0_AUTOACK)) | (0 << CC2420_MDMCTRL0_AUTOACK)) | (2 << CC2420_MDMCTRL0_PREAMBLE_LENGTH));
}
  __nesc_atomic_end(__nesc_atomic);
}
}

static void CC2420ControlP_writeId( void ) 
{
  nxle_uint16_t (id[2]);
{
  /* IRQ INSTR 407 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 408 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 409 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 410 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 411 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  __nesc_hton_leuint16(id[0].data, CC2420ControlP_m_pan);
  __nesc_hton_leuint16(id[1].data, CC2420ControlP_m_short_addr);
}
  __nesc_atomic_end(__nesc_atomic);
}
  CC2420ControlP_PANID_write(0, ( uint8_t * )(&id), sizeof(id));
}

static error_t CC2420CsmaP_SplitControl_start( void ) 
{
  if(CC2420CsmaP_SplitControlState_requestState(CC2420CsmaP_S_STARTING) == SUCCESS)
  {
    CC2420CsmaP_CC2420Power_startVReg();
    return SUCCESS;
  }
  else 
  {
    if(CC2420CsmaP_SplitControlState_isState(CC2420CsmaP_S_STARTED))
    {
      return EALREADY;
    }
    else 
    {
      if(CC2420CsmaP_SplitControlState_isState(CC2420CsmaP_S_STARTING))
      {
        return SUCCESS;
      }
    }
  }
  return EBUSY;
}

static error_t DisseminationEngineImplP_DisseminationCache_start(uint16_t key) 
{
  error_t result = DisseminationEngineImplP_TrickleTimer_start(key);
  DisseminationEngineImplP_TrickleTimer_reset(key);
  return result;
}

static error_t TrickleTimerImplP_0_TrickleTimer_start(uint8_t id) 
{
  if(TrickleTimerImplP_0_trickles[id].time != 0)
  {
    return EBUSY;
  }
  TrickleTimerImplP_0_trickles[id].time = 0;
  TrickleTimerImplP_0_trickles[id].remainder = 0;
  TrickleTimerImplP_0_trickles[id].count = 0;
  TrickleTimerImplP_0_generateTime(id);
{
  /* IRQ INSTR 412 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_TIMERB0_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 413 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT1_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 414 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0TX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 415 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_UART0RX_VECTOR(); _R2 |= 0x0008; }
  /* IRQ INSTR 416 */
//  if ((_R2 & 0x0008) != 0x0000) { _R2 &= 0xfff7; sig_PORT2_VECTOR(); _R2 |= 0x0008; }
  __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
{
  TrickleTimerImplP_0_Changed_set(id);
}
  __nesc_atomic_end(__nesc_atomic);
}
  TrickleTimerImplP_0_adjustTimer();
  return SUCCESS;
}

static void VirtualizeTimerC_0_Timer_startPeriodic(uint8_t num, uint32_t dt) 
{
  VirtualizeTimerC_0_startTimer(num, VirtualizeTimerC_0_TimerFrom_getNow(), dt, FALSE);
}

void sig_PORT1_VECTOR( void ) 
{
  volatile int n = _P1IFG & _P1IE;
  
  assert(0);
  
  if(n & (1 << 0))
  {
    HplMsp430InterruptP_Port10_fired();
    return;
  }
  if(n & (1 << 1))
  {
    HplMsp430InterruptP_Port11_fired();
    return;
  }
  if(n & (1 << 2))
  {
    HplMsp430InterruptP_Port12_fired();
    return;
  }
  if(n & (1 << 3))
  {
    HplMsp430InterruptP_Port13_fired();
    return;
  }
  if(n & (1 << 4))
  {
    HplMsp430InterruptP_Port14_fired();
    return;
  }
  if(n & (1 << 5))
  {
    HplMsp430InterruptP_Port15_fired();
    return;
  }
  if(n & (1 << 6))
  {
    HplMsp430InterruptP_Port16_fired();
    return;
  }
  if(n & (1 << 7))
  {
    HplMsp430InterruptP_Port17_fired();
    return;
  }
}

void sig_PORT2_VECTOR( void ) 
{
  volatile int n = _P2IFG & _P2IE;

  assert(0);

  if(n & (1 << 0))
  {
    HplMsp430InterruptP_Port20_fired();
    return;
  }
  if(n & (1 << 1))
  {
    HplMsp430InterruptP_Port21_fired();
    return;
  }
  if(n & (1 << 2))
  {
    HplMsp430InterruptP_Port22_fired();
    return;
  }
  if(n & (1 << 3))
  {
    HplMsp430InterruptP_Port23_fired();
    return;
  }
  if(n & (1 << 4))
  {
    HplMsp430InterruptP_Port24_fired();
    return;
  }
  if(n & (1 << 5))
  {
    HplMsp430InterruptP_Port25_fired();
    return;
  }
  if(n & (1 << 6))
  {
    HplMsp430InterruptP_Port26_fired();
    return;
  }
  if(n & (1 << 7))
  {
    HplMsp430InterruptP_Port27_fired();
    return;
  }
}

void sig_UART0RX_VECTOR( void ) 
{
  uint8_t temp = _U0RXBUF;

  assert(0);

  HplMsp430Usart0P_Interrupts_rxDone(temp);
}

static bool ArbiterP_0_ArbiterInfo_inUse( void ) 
{
{
  if(ArbiterP_0_state == ArbiterP_0_RES_CONTROLLED)
  {
    char unsigned __nesc_temp = FALSE;
    return __nesc_temp;
  }
}
  return TRUE;
}

static uint8_t ArbiterP_0_ArbiterInfo_userId( void ) 
{
{
  if(ArbiterP_0_state != ArbiterP_0_RES_BUSY)
  {
    char unsigned __nesc_temp = ArbiterP_0_NO_RES;
    return __nesc_temp;
  }
{
  char unsigned __nesc_temp = ArbiterP_0_resId;
  return __nesc_temp;
}
}
}

void sig_UART0TX_VECTOR( void ) 
{
  assert(0);

  if(HplMsp430Usart0P_HplI2C_isI2C())
  {
    HplMsp430Usart0P_I2CInterrupts_fired();
  }
  else 
  {
    HplMsp430Usart0P_Interrupts_txDone();
  }
}