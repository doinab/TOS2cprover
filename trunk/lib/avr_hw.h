/* AVR ATmega128 ans ATmega128L 
 */

typedef unsigned char 8b_reg_t;

/*
 * REGISTERS
 *
 *    AVR CPU General Purpose Working Registers
 *    32 8-bit registers; address range 0x00 - 0x1F
 */

8b_reg_t _R0;   /* 0x00 */
8b_reg_t _R1;   /* 0x01 */
8b_reg_t _R2;   /* 0x02 */
8b_reg_t _R3;   /* 0x03 */
8b_reg_t _R4;   /* 0x04 */
8b_reg_t _R5;   /* 0x05 */
8b_reg_t _R6;   /* 0x06 */
8b_reg_t _R7;   /* 0x07 */
8b_reg_t _R8;   /* 0x08 */
8b_reg_t _R9;   /* 0x09 */
8b_reg_t _R10;  /* 0x0A */
8b_reg_t _R11;  /* 0x0B */
8b_reg_t _R12;  /* 0x0C */
8b_reg_t _R13;  /* 0x0D */
8b_reg_t _R14;  /* 0x0E */
8b_reg_t _R15;  /* 0x0F */
8b_reg_t _R16;  /* 0x10 */
8b_reg_t _R17;  /* 0x11 */
8b_reg_t _R18;  /* 0x12 */
8b_reg_t _R19;  /* 0x13 */
8b_reg_t _R20;  /* 0x14 */
8b_reg_t _R21;  /* 0x15 */
8b_reg_t _R22;  /* 0x16 */
8b_reg_t _R23;  /* 0x17 */
8b_reg_t _R24;  /* 0x18 */
8b_reg_t _R25;  /* 0x19 */
8b_reg_t _R26;  /* 0x1A X-register low byte */
8b_reg_t _R27;  /* 0x1B X-register high byte */
8b_reg_t _R28;  /* 0x1C Y-register low byte */
8b_reg_t _R29;  /* 0x1D Y-register high byte */
8b_reg_t _R30;  /* 0x1E Z-register low byte */
8b_reg_t _R31;  /* 0x1F Z-register high byte */

/*
 * I/O REGISTERS
 *
 *     64 8-bit registers; address range 0x20 - 0x5F
 *     Timers, peripheral ports
 */

8b_reg_t _SREG;    /* 0x5F Status Register */
8b_reg_t _SPH;     /* 0x5E Stack Pointer High byte */
8b_reg_t _SPL;     /* 0x5D Stack Pointer Low byte */

8b_reg_t _XDIV;    /* 0x5C */
8b_reg_t _RAMPZ;   /* 0x5B */
8b_reg_t _EICRB;   /* 0x5A */
8b_reg_t _EIMSK;   /* 0x59 */
8b_reg_t _EIFR;    /* 0x58 */
8b_reg_t _TIMSK;   /* 0x57 */
8b_reg_t _TIFR;    /* 0x56 */
8b_reg_t _MCUCR;   /* 0x55 */
8b_reg_t _MCUCSR;  /* 0x54 */
8b_reg_t _TCCR0;   /* 0x53 */
8b_reg_t _TCNT0;   /* 0x52 Timer/Counter0 */
8b_reg_t _OCR0;    /* 0x51 Timer/Counter0 - Output Compare Register */
8b_reg_t _ASSR;    /* 0x50 */
8b_reg_t _TCCR1A;  /* 0x4F */
8b_reg_t _TCCR1B;  /* 0x4E */
8b_reg_t _TCNT1H;  /* 0x4D Timer/Counter1 - Counter Register High Byte */
8b_reg_t _TCNT1L;  /* 0x4C Timer/Counter1 - Counter Register Low Byte */
8b_reg_t _OCR1AH;  /* 0x4B Timer/Counter1 - Output Compare Register A High Byte */
8b_reg_t _OCR1AL;  /* 0x4A Timer/Counter1 - Output Compare Register A Low Byte */
8b_reg_t _OCR1BH;  /* 0x49 Timer/Counter1 - Output Compare Register B High Byte */
8b_reg_t _OCR1BL;  /* 0x48 Timer/Counter1 - Output Compare Register B Low Byte */
8b_reg_t _ICR1H;   /* 0x47 Timer/Counter1 - Input Capture Register High Byte */
8b_reg_t _ICR1L;   /* 0x46 Timer/Counter1 - Input Capture Register Low Byte */
8b_reg_t _TCCR2;   /* 0x45 */
8b_reg_t _TCNT2;   /* 0x44 Timer/Counter2 */
8b_reg_t _OCR2;    /* 0x43 Timer/Counter2 - Output Compare Register */
8b_reg_t _OCDR;    /* 0x42 */
8b_reg_t _WDTCR;   /* 0x41 */
8b_reg_t _SFIOR;   /* 0x40 */
8b_reg_t _EEARH;   /* 0x3F EEPROM address Register High Byte (bits 3-0) */
8b_reg_t _EEARL;   /* 0x3E EEPROM address Register Low Byte */
8b_reg_t _EEDR;    /* 0x3D EEPROM Data Register */
8b_reg_t _EECR;    /* 0x3C */

8b_reg_t _PORTA;   /* 0x3B I/O PORT A */
8b_reg_t _DDRA;    /* 0x3A */
8b_reg_t _PINA;    /* 0x39 */
8b_reg_t _PORTB;   /* 0x38 I/O PORT B */
8b_reg_t _DDRB;    /* 0x37 */
8b_reg_t _PINB;    /* 0x36 */
8b_reg_t _PORTC;   /* 0x35 I/O PORT C */
8b_reg_t _DDRC;    /* 0x34 */
8b_reg_t _PINC;    /* 0x33 */
8b_reg_t _PORTD;   /* 0x32 I/O PORT D */
8b_reg_t _DDRD;    /* 0x31 */
8b_reg_t _PIND;    /* 0x30 */
8b_reg_t _PORTE;   /* 0x23 I/O PORT E */
8b_reg_t _DDRE;    /* 0x22 */
8b_reg_t _PINE;    /* 0x21 */
8b_reg_t _PINF;    /* 0x20 I/O PORT F */

8b_reg_t _SPDR;    /* 0x2F SPI Data Register */
8b_reg_t _SPSR;    /* 0x2E */
8b_reg_t _SPCR;    /* 0x2D */
8b_reg_t _UDR0;    /* 0x2C USART0 I/O Data Register */
8b_reg_t _UCSR0A;  /* 0x2B */
8b_reg_t _UCSR0B;  /* 0x2A */
8b_reg_t _UBRR0L;  /* 0x29 USART0 Baud Rate Register Low */
8b_reg_t _ACSR;    /* 0x28 */
8b_reg_t _ADMUX;   /* 0x27 */
8b_reg_t _ACDSRA;  /* 0x26 */
8b_reg_t _ADCH;    /* 0x25 ADC Data Register High Byte */
8b_reg_t _ADCL;    /* 0x24 ADC Data Register Low Byte */

/*
 * EXTENDED I/O REGISTERS
 *
 *     160 8-bit registers; address range 0x60 - 0xFF
 */

8b_reg_t _DDRF;    /* 0x61 */