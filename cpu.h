#ifndef CPU_H_
#define CPU_H_
#ifdef __cplusplus
 extern "C" {
#endif

                                        // 8080
                                        // PIN                                     S-100 Pin     Arduino
                                        //  1  A10     Address Bus                 A10           
                                        //  2  GND     Power Supply                GND           
                                        //  3  D4      Bi-Directional Data Bus                   PF7
                                        //  4  D5      Bi-Directional Data Bus                   PF6
                                        //  5  D6      Bi-Directional Data Bus                   PF5
                                        //  6  D7      Bi-Directional Data Bus                   PF4
                                        //  7  D3      Bi-Directional Data Bus                   PF3
                                        //  8  D2      Bi-Directional Data Bus                   PF2
                                        //  9  D1      Bi-Directional Data Bus                   PF1
                                        // 10  D0      Bi-Directional Data Bus                   PF0
                                        // 11  -5V     Power Supply                              
const uint8_t resetPin = 53;            // 12  RESET   Timing and Control                        
const uint8_t holdPin = 0;            // 13  HOLD    Timing and Control          ~PHOLD        
const uint8_t intPin = 0;             // 14  INT     Timing and Control          ~PINT         
const uint8_t clock2Pin = 3;            // 15  O2      Timing and Control                        PE5
const uint8_t intePin = 0;            // 16  INTE    Timing and Control          PINTE         
const uint8_t dbinPin = 65;             // 17  DBIN    Timing and Control          PDBIN         
const uint8_t wrPin = 66;               // 18  ~WR     Timing and Control          ~PWR          
const uint8_t syncPin = 62;             // 19  SYNC    Timing and Control          ~PSYNC        
                                        // 20  +5V     Power Supply                              
const uint8_t hldaPin = 0;            // 21  HLDA    Timing and Control          PHLDA         
const uint8_t clock1Pin = 2;            // 22  O1      Timing and Control                        PE4
const uint8_t readyPin = 63;            // 23  READY   Timing and Control                        
const uint8_t waitPin = 64;             // 24  WAIT    Timing and Control          PWAIT         
                                        // 25  A0      Address Bus                 A0            
                                        // 26  A1      Address Bus                 A1            
                                        // 27  A2      Address Bus                 A2            
                                        // 28  +12V    Power Supply                              
                                        // 29  A3      Address Bus                 A3            
                                        // 30  A4      Address Bus                 A4            
                                        // 31  A5      Address Bus                 A5            
                                        // 32  A6      Address Bus                 A6            
                                        // 33  A7      Address Bus                 A7            
                                        // 34  A8      Address Bus                 A8            
                                        // 35  A9      Address Bus                 A9            
                                        // 36  A15     Address Bus                 A15           
                                        // 37  A12     Address Bus                 A12           
                                        // 38  A13     Address Bus                 A13           
                                        // 39  A14     Address Bus                 A14           
                                        // 40  A11     Address Bus                 A11           

typedef enum {
  INST_FETCH          = 0b10100010, // 0xA2  dbin
  MEMORY_READ         = 0b10000010, // 0x82  dbin
  MEMORY_WRITE        = 0b00000000, // 0x00
  STACK_READ          = 0b10000110, // 0x86  dbin
  STACK_WRITE         = 0b00000100, // 0x04
  INPUT_READ          = 0b01000010, // 0x42
  OUTPUT_WRITE        = 0b00010011, // 0x13
  INTE_AKCKNOWLEDGE   = 0b00100011, // 0x23
  HALT_ACKNOWLEDGE    = 0b10001010, // 0x8A  dbin
  INTE_ACK_WHILE_HALT = 0b00101011, // 0x2B  dbin
} StatusWord;

#ifdef __cplusplus
}
#endif
#endif // CPU_H_