#include "i8080.h"
#include "cpu.h"

// Temp program: JMP 0x0000; 3 bytes in length, 10 cycle duration
//static uint8_t memory[1000] = { 0xC3, 0x00, 0x00};

static volatile uint8_t clock1 = LOW;
//volatile byte clock2 = LOW;
static volatile uint8_t interrupt = LOW;

i8080 cpu;
static uint8_t t1;

void syncSignal(uint8_t status){
  CONTROL_PORT |= (1 << SYNCPINBIT);          // syncPin HIGH
  if(t1){
    status = INST_FETCH;
    t1 = false;
  } else if (interrupt){
    status = INT_AKCKNOWLEDGE;
    interrupt = false;
  }
  statusWord(status);
  CONTROL_PORT &= ~(1 << SYNCPINBIT);         // syncPin LOW
}

static void portReady(uint8_t port, uint8_t pin, uint8_t status){
  syncSignal(status);
  A8_A15_DIRECTION = 0xFF;                    // output, upper address byte
  A8_A15_PORT = port;
  CONTROL_PORT |= (1 << pin);
  while(!readyPin){
    CONTROL_PORT |= (1 << WAITPINBIT);        // waitPin HIGH
  }
  CONTROL_PORT &= ~(1 << WAITPINBIT);         // waitPin LOW
}

static void memoryReady(uint16_t addr, uint8_t pin, uint8_t status){
  syncSignal(status);
  A8_A15_DIRECTION = 0xFF;                    // output, upper address byte
  A0_A7_DIRECTION = 0xFF;                     // output, lower address byte
  A8_A15_PORT = addr >> 8;
  A0_A7_PORT = addr & 0xFF;
  CONTROL_PORT |= (1 << pin);
  while(!readyPin){
    CONTROL_PORT |= (1 << WAITPINBIT);        // waitPin HIGH
  }
  CONTROL_PORT &= ~(1 << WAITPINBIT);         // waitPin LOW
}

static uint8_t rb(void* userdata, uint16_t addr){
  uint8_t d0_d7; 
  memoryReady(addr, DBINPINBIT, MEMORY_READ);
  D0_D7_DIRECTION = 0x00;                     // input
  d0_d7 = D0_D7_PORT;
  //d0_d7 = memory[addr];                     // temp memory program
  CONTROL_PORT &= ~(1 << DBINPINBIT);         // dbinPin LOW
  return d0_d7;
}

static void wb(void* userdata, uint16_t addr, uint8_t val) {
  memoryReady(addr, WRPINBIT, MEMORY_WRITE);
  D0_D7_DIRECTION = 0xFF;                     // output 
  D0_D7_PORT = val;
  //memory[addr] = val;                       // temp memory program
  CONTROL_PORT |= (1 << WRPINBIT);            // wrPin HIGH
}

static uint8_t port_in(void* userdata, uint8_t port) {
  uint8_t d0_d7;
  portReady(port, DBINPINBIT, INPUT_READ);
  D0_D7_DIRECTION = 0x00;                     // input
  d0_d7 = D0_D7_PORT;
  CONTROL_PORT &= ~(1 << DBINPINBIT);         // dbinPin LOW
  return d0_d7;
}

static void port_out(void* userdata, uint8_t port, uint8_t value){
  portReady(port, WRPINBIT, OUTPUT_WRITE);
  D0_D7_DIRECTION = 0xFF;                     // output
  D0_D7_PORT = value;
  CONTROL_PORT |= (1 << WRPINBIT);            // wrPin HIGH
}

static uint8_t statusWord(byte sw){
  D0_D7_DIRECTION = 0xFF;                     // output
  D0_D7_PORT = sw;
}

static void checkHold(){
  if(holdPin == 1){
    // data and address bus high impedance state
    A8_A15_DIRECTION = 0x00;
    A0_A7_DIRECTION = 0x00;
    D0_D7_DIRECTION = 0x00;
    A8_A15_PORT = 0x00;
    A0_A7_PORT = 0x00;
    D0_D7_PORT = 0x00;
    while(holdPin == 1){
      CONTROL_PORT |= (1 << HLDAPINBIT);      // hldaPin HIGH
    }
    CONTROL_PORT &= ~(1 << HLDAPINBIT);       // hldaPin LOW
  }
}

static void intInt(){
  if(intePin == 1){
    interrupt = true;
  }
  if(cpu.halted == true){
    syncSignal(INTE_ACK_WHILE_HALT);
  }
}

static void clock1Int(){
  clock1 = HIGH;
}
/*
// unused at this time
static void clock2Int(){
  clock2 = HIGH;
}
*/
static void run(i8080* const c){
  // initialize the emulator
  i8080_init(c);
  c->userdata = c;
  c->read_byte = rb;
  c->write_byte = wb;
  c->port_in = port_in;
  c->port_out = port_out;
  c->pc = 0x0000;

  CONTROL_PORT &= ~(1 << WAITPINBIT);         // waitPin LOW
  CONTROL_PORT &= ~(1 << DBINPINBIT);         // dbinPin LOW
  CONTROL_PORT |= (1 << WRPINBIT);            // wrPin HIGH

  while(PORTB0 == 0){                         // while reset is not high execute instructions
    if(clock1 == HIGH){
      EXT_INT_MASK &= ~(1 << CLOCK1PINBIT);   // Disable clock1 intterupt
      checkHold();
      if(interrupt){
        i8080_interrupt(c, rb(c->userdata, c->pc));
      }
      t1 = true;
      i8080_step(c);
      clock1 = LOW;
      EXT_INT_MASK |= (1 << CLOCK1PINBIT);    // reenable clock1 interrupt
    }
  }
}

static void setup(){
  // Might change these to non-Arduino functions
  // Clock inputs
  attachInterrupt(digitalPinToInterrupt(clock1Pin), clock1Int, RISING);
  //attachInterrupt(digitalPinToInterrupt(clock2Pin), clock2Int, RISING);

  // Timing and control inputs
  attachInterrupt(digitalPinToInterrupt(intPin), intInt, RISING);
  pinMode(readyPin, INPUT);
  pinMode(holdPin, INPUT);
  pinMode(resetPin, INPUT);
  
  // Timing and control outputs
  pinMode(syncPin, OUTPUT);
  pinMode(dbinPin, OUTPUT);
  pinMode(waitPin, OUTPUT);
  pinMode(wrPin, OUTPUT);
  pinMode(hldaPin, OUTPUT);
  pinMode(intePin, OUTPUT);
}

static void loop() {
  run(&cpu);
}