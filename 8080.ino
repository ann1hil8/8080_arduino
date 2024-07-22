#include "i8080.h"
#include "cpu.h"

#define A8_A15_DIRECTION  DDRA
#define A0_A7_DIRECTION   DDRC
#define D0_D7_DIRECTION   DDRF
#define A8_A15_PORT       PORTA
#define A0_A7_PORT        PORTC
#define D0_D7_PORT        PORTF

// Temp program: JMP 0x0000; 3 bytes in length, 10 cycle duration
static uint8_t memory[1000] = { 0xC3, 0x00, 0x00};

volatile bool clock1 = LOW;
//volatile byte clock2 = LOW;
volatile bool interrupt = LOW;

static void syncSignal(){
  PORTK |= (1 << PORTK0); // syncPin HIGH
  statusWord(INST_FETCH); // TODO: Figure out the next status word docs sparse
  PORTK &= ~(1 << PORTK0); // syncPin LOW
}

static void portReady(uint8_t port, byte pin){
  syncSignal();
  A8_A15_DIRECTION = 0xFF; // Port A as output, upper address byte
  A8_A15_PORT = port;
  digitalWrite(pin, HIGH);
  while(!digitalRead(readyPin)){
    PORTK |= (1 << PORTK2); // waitPin HIGH
  }
  PORTK &= ~(1 << PORTK2); // waitPin LOW
}

static void memoryReady(uint16_t addr, byte pin){
  syncSignal();
  A8_A15_DIRECTION = 0xFF; // Port A as output, upper address byte
  A0_A7_DIRECTION = 0xFF; // Port C as output, lower address byte
  A8_A15_PORT = addr >> 8;
  A0_A7_PORT = addr & 0xFF;
  digitalWrite(pin, HIGH);
  while(!digitalRead(readyPin)){
    PORTK |= (1 << PORTK2); // waitPin HIGH
  }
  PORTK &= ~(1 << PORTK2); // waitPin LOW
}

static uint8_t rb(void* userdata, uint16_t addr){
  uint8_t d0_d7; 
  memoryReady(addr, dbinPin);
  D0_D7_DIRECTION = 0x00; // Port F as input
  d0_d7 = D0_D7_PORT;
  d0_d7 = memory[addr];
  PORTK &= ~(1 << PORTK3); // dbinPin LOW
  return d0_d7;
}

static void wb(void* userdata, uint16_t addr, uint8_t val) {
  memoryReady(addr, wrPin);
  D0_D7_DIRECTION = 0xFF;         // output 
  D0_D7_PORT = val;
  memory[addr] = val;
  PORTK |= (1 << PORTK4); // wrPin HIGH
}

static uint8_t port_in(void* userdata, uint8_t port) {
  uint8_t d0_d7;
  portReady(port, dbinPin);
  D0_D7_DIRECTION = 0x00;         // input
  d0_d7 = D0_D7_PORT;
  PORTK &= ~(1 << PORTK3); // dbinPin LOW
  return d0_d7;
}

static void port_out(void* userdata, uint8_t port, uint8_t value){
  portReady(port, wrPin);
  D0_D7_DIRECTION = 0xFF;         // output
  PORTF = value;
  PORTK |= (1 << PORTK4); // wrPin HIGH
}

uint8_t statusWord(byte sw){
  D0_D7_DIRECTION = 0xFF;         // output
  D0_D7_PORT = sw;
}

static void checkHold(){
  if(digitalRead(holdPin)){
    // data and address bus high impedance state
    A8_A15_DIRECTION = 0x00;
    A0_A7_DIRECTION = 0x00;
    D0_D7_DIRECTION = 0x00;
    A8_A15_PORT = 0x00;
    A0_A7_PORT = 0x00;
    D0_D7_PORT = 0x00;
    while(digitalRead(holdPin)){
      digitalWrite(hldaPin, HIGH);
    }
    digitalWrite(hldaPin, LOW);
  }
}

void intInt(){
  if(digitalRead(intePin)){
    interrupt = true;
  }
}

void clock1Int(){
  clock1 = HIGH;
}
/*
// unused at this time
void clock2Int(){
  clock2 = HIGH;
}

bool dbin(byte type){
  switch(type){
    case INST_FETCH:
    case MEMORY_READ:
    case STACK_READ:
    case INTE_AKCKNOWLEDGE:
    case INTE_ACK_WHILE_HALT:
    return true;
    default:
    return false;
  }
}

bool wr(byte type){
  switch(type){
    case MEMORY_WRITE:
    case STACK_WRITE:
    case OUTPUT_WRITE:
    return true;
    default:
    return false;
  }
}
*/
void run(i8080* const c){
  // initialize the emulator
  i8080_init(c);
  c->userdata = c;
  c->read_byte = rb;
  c->write_byte = wb;
  c->port_in = port_in;
  c->port_out = port_out;
  c->pc = 0x0000;

  PORTK &= ~(1 << PORTK2); // waitPin LOW
  PORTK &= ~(1 << PORTK3); // dbinPin LOW
  PORTK |= (1 << PORTK4); // wrPin HIGH

  while(PORTB0 == 0){ // while reset is not high execute instructions
    if(clock1 == HIGH){
      EIMSK &= ~(1 << INT4); // Disable O1 intterupt
      checkHold();
      if(interrupt){
        i8080_interrupt(c, rb(c->userdata, c->pc));
        interrupt = false;
      }
      i8080_step(c);
      clock1 = LOW;
      EIMSK |= (1 << INT4); // reenable clock1 interrupt
    }
  }
}

void setup(){
  attachInterrupt(digitalPinToInterrupt(clock1Pin), clock1Int, RISING);
  //attachInterrupt(digitalPinToInterrupt(clock2Pin), clock2Int, RISING);
  attachInterrupt(digitalPinToInterrupt(intPin), intInt, RISING);
  pinMode(resetPin, INPUT);
  pinMode(readyPin, INPUT);
  pinMode(holdPin, INPUT);
  
  pinMode(waitPin, OUTPUT);
  pinMode(dbinPin, OUTPUT);
  pinMode(wrPin, OUTPUT);
  pinMode(hldaPin, OUTPUT);
  pinMode(syncPin, OUTPUT);
  pinMode(intePin, OUTPUT);
}

void loop() {
  i8080 cpu;
  run(&cpu);
}