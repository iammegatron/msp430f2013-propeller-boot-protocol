#define RXD       0x04                      // RXD on P1.2 (CCI1A)
#define TXD       0x02                      // TXD on P1.1 (general purpose IO)

#define RXBITS 8       // modifiy to change bit length
unsigned int RXData;  // received data
unsigned int TXData;  // send back data
unsigned char BitCnt;

int flag = 0; // state machine state


unsigned int T_slot_temp = 0;  // propeller protocol 
unsigned int T_slot = 0;       // propeller protocol: 1T


void RX_Ready (void);

#include  <msp430x20x3.h>

void main (void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer 
  IE1 |= WDTIE;                             // Enable WDT interrupt (for timeout)
  
  
  BCSCTL1 = CALBC1_16MHZ;                    // Set range
  DCOCTL = CALDCO_16MHZ;                     // Set DCO step + modulation */

  
  TACTL = TASSEL_2 + MC_2;                  // SMCLK, continuous UP mode
  
  P1SEL = RXD;                        // Capture function on RXD pin
  
  P1OUT |= TXD;          // pull up TXD
  // modify to add your input/output
  P1DIR = TXD;                        // TXD pin set as output
  

  
  // Mainloop
  for (;;)
  {
    RX_Ready();                               // UART ready to RX one Byte
    _BIS_SR(LPM0_bits + GIE);                 // Enter LPM0 w/ interrupt
  }
}


// Function Readies UART to Receive Character into RXTXData Buffer
void RX_Ready (void)
{
  BitCnt = RXBITS;                       // Load Bit counter
  CCTL1 = SCS +  CM1 + CAP + CCIE;       // Sync, falling Edge, Cap
  WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer 
}


// Timer A1 interrupt service routine
#pragma vector=TIMERA1_VECTOR
__interrupt void Timer_A (void)
{
  WDTCTL = WDT_MDLY_32;          // Set Watchdog Timer interval to ~2ms 
  switch (flag){ // flag0--flag3 calibration pulse: 0xF9
  case 0: // falling edge triggered
    T_slot_temp = CCR1;
    CCTL1 = SCS +  CM0 + CAP + CCIE;   // Sync, rising Edge, Cap     
    flag += 1;
    break;
  case 1: // rising edge triggered
    T_slot_temp = CCR1 - T_slot_temp; // 1T
    CCTL1 = SCS +  CM1 + CAP + CCIE;   // Sync, falling Edge, Cap
    flag += 1;
    break;
  case 2: // falling edge triggered
    T_slot = CCR1;
    CCTL1 = SCS +  CM0 + CAP + CCIE;   // Sync, rising Edge, Cap     
    flag += 1;
    break;
  case 3: // rising edge triggered
    WDTCTL = WDTPW + WDTHOLD;         // Stop watchdog timer; allow long break 
    T_slot = CCR1 - T_slot; // 2T
    CCTL1 = SCS +  CM1 + CAP + CCIE;   // Sync, falling Edge, Cap
    flag += 1;
    T_slot = (T_slot + T_slot_temp)>>1; // 1.5T    
    
    break;
  case 4:
    if(CCTL1 &  CM1) // falling edge triggered
    {
      T_slot_temp = CCR1;
      CCTL1 = SCS +  CM0 + CAP + CCIE;   // Sync, rising Edge, Cap     
    }
    else
    {
      T_slot_temp  = CCR1 - T_slot_temp;
      CCTL1 = SCS +  CM1 + CAP + CCIE;   // Sync, falling Edge, Cap
      RXData = RXData >> 1;          // LSB first
      if(T_slot_temp < T_slot)
        RXData |= 0x80;
      BitCnt --;                            
      if ( BitCnt == 0)         // All bits RXed?
      //>>>>>>>>>> Decode of Received Byte Here <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
      {
        WDTCTL = WDTPW + WDTHOLD;         // Stop watchdog timer; allow long break 
        if (RXData == 0xAA)
        {
          TXData = 1;   // send back 1
          flag = 5;
        }
        else if (RXData == 0xAB)
        {
          TXData = 0;  // send back 0
          flag = 5;          
        }
        else
        {
         CCTL1 &= ~ CCIE;                      // All bits RXed, disable interrupt
         _BIC_SR_IRQ(LPM0_bits);               // Clear LPM3 bits from 0(SR)
         flag = 4;           
        }
      }
    }   
    break;
  case 5: // falling edge trigger
    CCTL1 = SCS +  CM0 + CAP + CCIE;   // Sync, rising Edge, Cap
    flag += 1;
    P1OUT &= ~TXD; // pull TXD low   
    
    break;
  case 6:  // rising edge trigger
    CCTL1 = SCS +  CM1 + CAP + CCIE;   // Sync, falling Edge, Cap
    if(TXData & 0x01)
    {
      P1OUT |= TXD; // pull up TXD: 0xFF
    }    
    flag += 1;
    break;
  case 7: // falling edge trigger
    CCTL1 = SCS +  CM0 + CAP + CCIE;   // Sync, rising Edge, Cap
    P1OUT |= TXD; // pull up TXD
    flag += 1;
    break;
  case 8: // rising edge trigger
    CCTL1 = SCS +  CM1 + CAP + CCIE;   // Sync, falling Edge, Cap
    CCTL1 &= ~ CCIE;                      // All bits TXed, disable interrupt
    _BIC_SR_IRQ(LPM0_bits);               // Clear LPM3 bits from 0(SR)
    flag = 4;      
    WDTCTL = WDTPW + WDTHOLD;         // Stop watchdog timer; allow long break 
    break;
  default:
    flag = 0;
    break;
  }
}

// Watchdog Timer interrupt service routine
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
{                 // after timeout
  flag = 0;      
  RX_Ready();
}
