# addr has to be decimal
def get_memory_section(addr):
  if addr in range(0x0, 0xf):
    return 'Special Function Register'
  if addr in range(0x10, 0xff):
    return '8-bit Peripheral Module'
  if addr in range(0x100, 0x1ff, 2):
    return '16-bit Peripheral Module'

  return 'illegal address'

MSP430_map = {
  "0x0000":'_IE1',
  "0x0001":'_IE2',
  "0x0002":'_IFG1',
  "0x0003":'_IFG2',
  "0x0004":'_ME1',
  "0x0005":'_ME2',
  "0x0020":'_P1IN',
  "0x0021":'_P1OUT',
  "0x0022":'_P1DIR',
  "0x0023":'_P1IFG',
  "0x0024":'_P1IES',
  "0x0025":'_P1IE',
  "0x0026":'_P1SEL',
  "0x0028":'_P2IN',
  "0x0029":'_P2OUT',
  "0x002A":'_P2DIR',
  "0x002B":'_P2IFG',
  "0x002C":'_P2IES',
  "0x002D":'_P2IE',
  "0x002E":'_P2SEL',
  "0x0018":'_P3IN',
  "0x0019":'_P3OUT',
  "0x001A":'_P3DIR',
  "0x001B":'_P3SEL',
  "0x001C":'_P4IN',
  "0x001D":'_P4OUT',
  "0x001E":'_P4DIR',
  "0x001F":'_P4SEL',
  "0x0030":'_P5IN',
  "0x0031":'_P5OUT',
  "0x0032":'_P5DIR',
  "0x0033":'_P5SEL',
  "0x0034":'_P6IN',
  "0x0035":'_P6OUT',
  "0x0036":'_P6DIR',
  "0x0037":'_P6SEL',
  "0x0055":'_SVSCTL',
  "0x0058":'_BCSCTL2',
  "0x0057":'_BCSCTL1',
  "0x0056":'_DCOCTL',
  "0x005B":'_CAPD',
  "0x005A":'_CACTL2',
  "0x0077":'_U0TXBUF',
  "0x0076":'_U0RXBUF',
  "0x0075":'_U0BR1',
  "0x0074":'_U0BR0',
  "0x0073":'_U0MCTL',
  "0x0072":'_U0RCTL',
  "0x0071":'_U0TCTL',
  "0x0070":'_U0CTL',
  "0x007F":'_U1TXBUF',
  "0x007E":'_U1RXBUF',
  "0x007D":'_U1BR1',
  "0x007C":'_U1BR0',
  "0x007B":'_U1MCTL',
  "0x007A":'_U1RCTL',
  "0x0079":'_U1TCTL',
  "0x0078":'_U1CTL',
  "0x0120":'_WDTCTL',
  "0x012C":'_FCTL3',
  "0x012A":'_FCTL2',
  "0x0128":'_FCTL1',
  "0x013E":'_SUMEXT',
  "0x013C":'_RESHI',
  "0x013A":'_RESLO',
  "0x0138":'_OP2',
  "0x0136":'_MACS',
  "0x0134":'_MAC',
  "0x0132":'_MPYS',
  "0x0130":'_MPY',
  "0x0166":'_TACCTL2',
  "0x0164":'_TACCTL1',
  "0x0162":'_TACCTL0',
  "0x0160":'_TACTL',
  "0x012E":'_TAIV',
  "0x0176":'_TACCR2',
  "0x0174":'_TACCR1',
  "0x0172":'_TACCR0',
  "0x0170":'_TAR',
  "0x019C":'_TBCCR5',
  "0x019A":'_TBCCR4',
  "0x0198":'_TBCCR3',
  "0x0196":'_TBCCR2',
  "0x0194":'_TBCCR1',
  "0x0192":'_TBCCR0',
  "0x0190":'_TBR',
  "0x018E":'_TBCCTL6',
  "0x018C":'_TBCCTL5',
  "0x018A":'_TBCCTL4',
  "0x0188":'_TBCCTL3',
  "0x0186":'_TBCCTL2',
  "0x0184":'_TBCCTL1',
  "0x0182":'_TBCCTL0',
  "0x0180":'_TBCTL',
  "0x011E":'_TBIV',
  "0x01A8":'_ADC12IV',
  "0x01A6":'_ADC12IE',
  "0x01A4":'_ADC12IFG',
  "0x01A2":'_ADC12CTL1',
  "0x01A0":'_ADC12CTL0',
  "0x015E":'_ADC12MEM15',
  "0x015C":'_ADC12MEM14',
  "0x015A":'_ADC12MEM13',
  "0x0158":'_ADC12MEM12',
  "0x0156":'_ADC12MEM11',
  "0x0154":'_ADC12MEM10',
  "0x0152":'_ADC12MEM9',
  "0x0150":'_ADC12MEM8',
  "0x014E":'_ADC12MEM7',
  "0x014C":'_ADC12MEM6',
  "0x014A":'_ADC12MEM5',
  "0x0148":'_ADC12MEM4',
  "0x0146":'_ADC12MEM3',
  "0x0144":'_ADC12MEM2',
  "0x0142":'_ADC12MEM1',
  "0x0140":'_ADC12MEM0',
  "0x008F":'_ADC12MCTL15',
  "0x008E":'_ADC12MCTL14',
  "0x008D":'_ADC12MCTL13',
  "0x008C":'_ADC12MCTL12',
  "0x008B":'_ADC12MCTL11',
  "0x008A":'_ADC12MCTL10',
  "0x0089":'_ADC12MCTL9',
  "0x0088":'_ADC12MCTL8',
  "0x0087":'_ADC12MCTL7',
  "0x0086":'_ADC12MCTL6',
  "0x0085":'_ADC12MCTL5',
  "0x0084":'_ADC12MCTL4',
  "0x0083":'_ADC12MCTL3',
  "0x0082":'_ADC12MCTL2',
  "0x0081":'_ADC12MCTL1',
  "0x0080":'_ADC12MCTL0',
  "0x019E":'_TBCCR6',
  "0x01CA":'_DAC12_1DAT',
  "0x01C2":'_DAC12_1CTL',
  "0x01C8":'_DAC12_0DAT',
  "0x01C0":'_DAC12_0CTL',
  "0x01F6":'_DMA2SZ',
  "0x01F4":'_DMA2DA',
  "0x01F2":'_DMA2SA',
  "0x01F0":'_DMA2CTL',
  "0x01EE":'_DMA1SZ',
  "0x01EC":'_DMA1DA',
  "0x01EA":'_DMA1SA',
  "0x01E8":'_DMA1CTL',
  "0x01E6":'_DMA0SZ',
  "0x01E4":'_DMA0DA',
  "0x01E2":'_DMA0SA',
  "0x01E0":'_DMA0CTL',
  "0x0124":'_DMACTL1',
  "0x0122":'_DMACTL0'
}

""" A model of the MSP430 memory map and CPU functions 
 *
 * THE MSP430 MEMORY MAP
 *
 * PERIPHERALS
 *
-  8-bit SFR             0000h -> 000Fh      
-  8-bit peripherals     0010h -> 00FFh    
- 16-bit peripherals     0100h -> 01FFh  

The MSP430's memory mapped peripheral registers are termed Special Function
Registers (SFRs). Consider a peripheral a normal variable, which is simply
mapped to the specific memory location.
 *
 * 8-bit SFR:

unsigned char _IE1;    /* 0x0000 Interrupt Enable 1 */
unsigned char _IE2;    /* 0x0001 Interrupt Enable 2 */
unsigned char _IFG1;   /* 0x0002 Interrupt Flag 1   */
unsigned char _IFG2;   /* 0x0003 Interrupt Flag 2   */
unsigned char _ME1;    /* 0x0004 Module Enable 1    */
unsigned char _ME2;    /* 0x0005 Module Enable 2    */

/* 8-bit peripherals:
 *
 * Port P1 
 */
unsigned char _P1IN;     /* 0x0020 Input                 */
unsigned char _P1OUT;    /* 0x0021 Output                */
unsigned char _P1DIR;    /* 0x0022 Direction             */
unsigned char _P1IFG;    /* 0x0023 Interrupt Flag        */
unsigned char _P1IES;    /* 0x0024 Interrupt-Edge Select */
unsigned char _P1IE;     /* 0x0025 Interrupt Enable      */
unsigned char _P1SEL;    /* 0x0026 Selection             */

/*
 * Port P2
 */
unsigned char _P2IN;     /* 0x0028 Input                 */
unsigned char _P2OUT;    /* 0x0029 Output                */
unsigned char _P2DIR;    /* 0x002A Direction             */
unsigned char _P2IFG;    /* 0x002B Interrupt Flag        */
unsigned char _P2IES;    /* 0x002C Interrupt-Edge Select */
unsigned char _P2IE;     /* 0x002D Interrupt Enable      */
unsigned char _P2SEL;    /* 0x002E Selection             */

/*
 * Port P3
 */
unsigned char _P3IN;     /* 0x0018 Input                 */
unsigned char _P3OUT;    /* 0x0019 Output                */
unsigned char _P3DIR;    /* 0x001A Direction             */
unsigned char _P3SEL;    /* 0x001B Selection             */

/*
 * Port P4
 */
unsigned char _P4IN;     /* 0x001C Input                 */
unsigned char _P4OUT;    /* 0x001D Output                */
unsigned char _P4DIR;    /* 0x001E Direction             */
unsigned char _P4SEL;    /* 0x001F Selection             */

/*
 * Port P5
 */
unsigned char _P5IN;     /* 0x0030 Input                 */
unsigned char _P5OUT;    /* 0x0031 Output                */
unsigned char _P5DIR;    /* 0x0032 Direction             */
unsigned char _P5SEL;    /* 0x0033 Selection             */

/*
 * Port P6
 */
unsigned char _P6IN;     /* 0x0034 Input                 */
unsigned char _P6OUT;    /* 0x0035 Output                */
unsigned char _P6DIR;    /* 0x0036 Direction             */
unsigned char _P6SEL;    /* 0x0037 Selection             */

/*
 * BrownOUT, SVS
 */
unsigned char _SVSCTL;   /* 0x0055 SVS */ 

/*
 * Basic Clock
 */
unsigned char _BCSCTL2   /* 0x0058 Basic clock system control2 */
unsigned char _BCSCTL1   /* 0x0057 Basic clock system control1 */
unsigned char _DCOCTL    /* 0x0056 DCO clock frequency control */

/*
 * Comparator_A
 */
unsigned char _CAPD      /* 0x005B Comparator_A port disable   */
unsigned char _CACTL2    /* 0x005A Comparator_A control2       */
unsigned char _CACTL1    /* 0x0059 Comparator_A control1       */

/*
 * USART0 (UART or SPImode - the default)
 */
unsigned char _U0TXBUF;   /* 0x0077 Transmit buffer    */
unsigned char _U0RXBUF;   /* 0x0076 Receive buffer     */
unsigned char _U0BR1;     /* 0x0075 Baud rate          */
unsigned char _U0BR0;     /* 0x0074 Baud rate          */
unsigned char _U0MCTL;    /* 0x0073 Modulation control */
unsigned char _U0RCTL;    /* 0x0072 Receive control    */
unsigned char _U0TCTL;    /* 0x0071 Transmit control   */
unsigned char _U0CTL;     /* 0x0070 USART control      */

/*
 * USART1 (MSP430F16x and MSP430F161x only)
 */
unsigned short _U1TXBUF;  /* 0x007F Transmit buffer */
unsigned short _U1RXBUF;  /* 0x007E Receive buffer  */
unsigned short _U1BR1;    /* 0x007D Baud rate       */
unsigned short _U1BR0;    /* 0x007C Baud rate       */
unsigned short _U1MCTL;   /* 0x007B Modulation control      */
unsigned short _U1RCTL;   /* 0x007A Receive control */
unsigned short _U1TCTL;   /* 0x0079 Transmit control        */
unsigned short _U1CTL;    /* 0x0078 USART control   */

/*
 * Watchdog 
 */
unsigned short _WDTCTL;   /* 0x0120 Watchdog Timer control */

/*
 * Flash 
 */
unsigned short _FCTL3;  /* 0x012C Flash control 3        */
unsigned short _FCTL2;  /* 0x012A Flash control 2        */
unsigned short _FCTL1;  /* 0x0128 Flash control 1        */

/*
 * Hardware Multiplier (MSP430F16x and MSP430F161x only) 
 */

unsigned short _SUMEXT; /* 0x013E Sum extend                     */
unsigned short _RESHI;  /* 0x013C Result high word               */
unsigned short _RESLO;  /* 0x013A Result low word                */
unsigned short _OP2;    /* 0x0138 Second operand                 */
unsigned short _MACS;   /* 0x0136 Multiply signed +accumulate/operand1   */
unsigned short _MAC;    /* 0x0134 Multiply+accumulate/operand1   */
unsigned short _MPYS;   /* 0x0132 Multiply signed/operand1       */
unsigned short _MPY;    /* 0x0130 Multiply unsigned/operand1     */

/*
 * Timer_A3
 */
unsigned short _TACCTL2;  /* 0x0166 Capture/compare control 2      */
unsigned short _TACCTL1;  /* 0x0164 Capture/compare control 1      */
unsigned short _TACCTL0;  /* 0x0162 Capture/compare control 0      */
unsigned short _TACTL;    /* 0x0160 Timer_A control                */
unsigned short _TAIV;     /* 0x012E Timer_A interrupt vector       */
unsigned short _TACCR2;   /* 0x0176 Capture/compare register 2     */
unsigned short _TACCR1;   /* 0x0174 Capture/compare register 1     */
unsigned short _TACCR0;   /* 0x0172 Capture/compare register 0     */
unsigned short _TAR;      /* 0x0170 Timer_A register               */

/* 
 * Timer_B3 
 */
unsigned short _TBCCR5;   /* 0x019C Capture/compare register 5     */
unsigned short _TBCCR4;   /* 0x019A Capture/compare register 4     */
unsigned short _TBCCR3;   /* 0x0198 Capture/compare register 3     */
unsigned short _TBCCR2;   /* 0x0196 Capture/compare register 2     */
unsigned short _TBCCR1;   /* 0x0194 Capture/compare register 1     */
unsigned short _TBCCR0;   /* 0x0192 Capture/compare register 0     */
unsigned short _TBR;      /* 0x0190 Timer_B register               */
unsigned short _TBCCTL6;  /* 0x018E Capture/compare control 6      */
unsigned short _TBCCTL5;  /* 0x018C Capture/compare control 5      */
unsigned short _TBCCTL4;  /* 0x018A Capture/compare control 4      */
unsigned short _TBCCTL3;  /* 0x0188 Capture/compare control 3      */
unsigned short _TBCCTL2;  /* 0x0186 Capture/compare control 2      */
unsigned short _TBCCTL1;  /* 0x0184 Capture/compare control 1      */
unsigned short _TBCCTL0;  /* 0x0182 Capture/compare control 0      */
unsigned short _TBCTL;    /* 0x0180 Timer_B control                */
unsigned short _TBIV;     /* 0x011E Timer_B interrupt vector       */

/*
Reserved 016Eh 
Reserved 016Ch 
Reserved 016Ah 
Reserved 0168h 

Reserved 017Eh 
Reserved 017Ch 
Reserved 017Ah 
Reserved 0178h 
 */

/*
 * ADC12
 */
unsigned short _ADC12IV;        /* 0x01A8 Interrupt-vector-word register */
unsigned short _ADC12IE;        /* 0x01A6 Inerrupt-enable register       */
unsigned short _ADC12IFG;       /* 0x01A4 Inerrupt-flag register */
unsigned short _ADC12CTL1;      /* 0x01A2 Control register 1     */
unsigned short _ADC12CTL0;      /* 0x01A0 Control register 0     */
unsigned short _ADC12MEM15;     /* 0x015E Conversion memory 15   */
unsigned short _ADC12MEM14;     /* 0x015C Conversion memory 14   */
unsigned short _ADC12MEM13;     /* 0x015A Conversion memory 13   */
unsigned short _ADC12MEM12;     /* 0x0158 Conversion memory 12   */
unsigned short _ADC12MEM11;     /* 0x0156 Conversion memory 11   */
unsigned short _ADC12MEM10;     /* 0x0154 Conversion memory 10   */
unsigned short _ADC12MEM9;      /* 0x0152 Conversion memory 9    */
unsigned short _ADC12MEM8;      /* 0x0150 Conversion memory 8    */
unsigned short _ADC12MEM7;      /* 0x014E Conversion memory 7    */
unsigned short _ADC12MEM6;      /* 0x014C Conversion memory 6    */
unsigned short _ADC12MEM5;      /* 0x014A Conversion memory 5    */
unsigned short _ADC12MEM4;      /* 0x0148 Conversion memory 4    */
unsigned short _ADC12MEM3;      /* 0x0146 Conversion memory 3    */
unsigned short _ADC12MEM2;      /* 0x0144 Conversion memory 2    */
unsigned short _ADC12MEM1;      /* 0x0142 Conversion memory 1    */
unsigned short _ADC12MEM0;      /* 0x0140 Conversion memory 0    */
unsigned char  _ADC12MCTL15;    /* 0x008F ADC memory-control register15   */
unsigned char  _ADC12MCTL14;    /* 0x008E ADC memory-control register14   */
unsigned char  _ADC12MCTL13;    /* 0x008D ADC memory-control register13   */
unsigned char  _ADC12MCTL12;    /* 0x008C ADC memory-control register12   */
unsigned char  _ADC12MCTL11;    /* 0x008B ADC memory-control register11   */
unsigned char  _ADC12MCTL10;    /* 0x008A ADC memory-control register10   */
unsigned char  _ADC12MCTL9;     /* 0x0089 ADC memory-control register9    */
unsigned char  _ADC12MCTL8;     /* 0x0088 ADC memory-control register8    */
unsigned char  _ADC12MCTL7;     /* 0x0087 ADC memory-control register7    */
unsigned char  _ADC12MCTL6;     /* 0x0086 ADC memory-control register6    */
unsigned char  _ADC12MCTL5;     /* 0x0085 ADC memory-control register5    */
unsigned char  _ADC12MCTL4;     /* 0x0084 ADC memory-control register4    */
unsigned char  _ADC12MCTL3;     /* 0x0083 ADC memory-control register3    */
unsigned char  _ADC12MCTL2;     /* 0x0082 ADC memory-control register2    */
unsigned char  _ADC12MCTL1;     /* 0x0081 ADC memory-control register1    */
unsigned char  _ADC12MCTL0;     /* 0x0080 ADC memory-control register0    */
unsigned short _TBCCR6;         /* 0x019E Timer_B7/ Capture/compare register 6   */

/* 
 * DAC12
 */
unsigned short _DAC12_1DAT;   /* 0x01CA DAC12_1 data     */
unsigned short _DAC12_1CTL;   /* 0x01C2 DAC12_1 control  */
unsigned short _DAC12_0DAT;   /* 0x01C8 DAC12_0 data     */
unsigned short _DAC12_0CTL;   /* 0x01C0 DAC12_0 control  */

/* 
 * DMA 
 */
unsigned short _DMA2SZ;   /* 0x01F6 DMA channel 2 transfer size        */
unsigned short _DMA2DA;   /* 0x01F4 DMA channel 2 destination address  */
unsigned short _DMA2SA;   /* 0x01F2 DMA channel 2 source address       */
unsigned short _DMA2CTL;  /* 0x01F0 DMA channel 2 control              */
unsigned short _DMA1SZ;   /* 0x01EE DMA channel 1 transfer size        */
unsigned short _DMA1DA;   /* 0x01EC DMA channel 1 destination address  */
unsigned short _DMA1SA;   /* 0x01EA DMA channel 1 source address       */
unsigned short _DMA1CTL;  /* 0x01E8 DMA channel 1 control              */
unsigned short _DMA0SZ;   /* 0x01E6 DMA channel 0 transfer size        */
unsigned short _DMA0DA;   /* 0x01E4 DMA channel 0 destination address  */
unsigned short _DMA0SA;   /* 0x01E2 DMA channel 0 source address       */
unsigned short _DMA0CTL;  /* 0x01E0 DMA channel 0 control              */
unsigned short _DMACTL1;  /* 0x0124 DMA module control 1               */
unsigned short _DMACTL0;  /* 0x0122 DMA module control 0               */

/*
unsigned char int_reg;

// GIE: General Interrupt Enable
// 0: interrupts disabled; 1: interrupts enabled
// bit 0 in "int_reg"
#define   set_GIE int_reg |= 0x01;
#define unset_GIE int_reg &= 0xfe;
 */

"""
