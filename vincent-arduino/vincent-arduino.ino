#include <serialize.h>

#include "packet.h"
#include "constants.h"
#include <math.h>

typedef enum {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;
volatile unsigned long leftTicks = 0, rightTicks = 0, requiredTicks = 0;
float speed = 0.0;

TResult readPacket(TPacket *packet) {
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);

}

void sendStatus() {
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftTicks;
  statusPacket.params[1] = rightTicks;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message) {
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void sendBadPacket() {
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}

void sendBadChecksum() {
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand() {
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse() {
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK() {
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet) {
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.
int readSerial(char *buffer) {
  int count = 0;

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code
void writeSerial(const char *buffer, int len) {
  Serial.write(buffer, len);
}

//KIV: can be placed in the RPi
int pwmVal(float speed) {
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

//interrupts for wheel encoder
ISR(PCINT0_vect) {
  //Serial.println(leftTicks); //to remove
  leftTicks++;
}
ISR(PCINT1_vect) {
  //Serial.println(rightTicks); //to remove
  rightTicks++;
}

ISR(TIMER0_COMPA_vect) { //left wheel, forward
  OCR0A = speed*0.8;//pwmVal(speed); //128
}
ISR(TIMER0_COMPB_vect) { //backward
  OCR0B = speed*0.8;//pwmVal(speed); //128
}
ISR(TIMER2_COMPA_vect) { //right wheel, forward
  OCR2A = speed;//pwmVal(speed); //128
}
ISR(TIMER2_COMPB_vect) { //backward
  OCR2B = speed;//pwmVal(speed); //128
}

void setup() {
  cli();
  Serial.begin(9600); //to remove
  
  DDRD |= 0b11111111;
  DDRB |= 0b11111111;

  //setup right wheel
  TCNT0 = 0;
  OCR0A = 128;
  OCR0B = 128; //set default of 50% duty cycle
  TIMSK0 |= 0b110;// Activates OCR0A and OCR0B compares match interrupt.

  //setup left wheel
  TCNT2 = 0;
  OCR2A = 128;
  OCR2B = 128; //set default of 50% duty cycle
  TIMSK2 |= 0b110;// Activates OCR2A and OCR2B compares match interrupt.

  //setup wheel encoders and set pull up resistors
  DDRC &= 0b11111110;// set PC0 as input
  DDRB &= 0b11011111;//set PB5 as input
  PORTC |= 0b00000001;//drive PC0 to high
  PORTB |= 0b00100000;//drive PB5 to high
  PCMSK0 = 0b00100000;//set PCINT5 to activate pin change interrupt 0
  PCMSK1 = 0b00000001;//set PCINT8 to activate pin change interrupt

  sei();

  //start PWM
  TCCR0B = 0b00000011;
  TCCR2B = 0b00000100;//setup prescalar64 to control period of PWM.

  //start wheel encoders
  PCICR = 0b00000011;//enable pin change interrupt 0 and 1

}


void rightForward() {
  TCCR0A = 0b10000001;//clear OC0A when upcounting and set when downcounting + set PWM
  //phase correct mode
  PORTD &= 0b11011111;// ensures that PD5 is off.
}

void rightBackward() {
  TCCR0A = 0b00100001;//clear OC0B when upcounting and set when downcounting + set PWM
  //phase correct mode
  PORTD &= 0b10111111;// ensures that PD6 is off.
}

void rightStop() {
  TCCR0A &= 0b00000000; //OC0A/B disconnected
  PORTD &= 100111111;// ensures that PD5 and 6 are off
}

void leftForward() {
  TCCR2A = 0b10000001;//clear OC2A when upcounting and set when downcounting + set PWM
  //phase correct mode
  PORTB &= 0b11110111;// ensures that PB3 is off.
}

void leftBackward() {
  TCCR2A = 0b00100001;//clear OC2B when upcounting and set when downcounting + set PWM
  //phase correct mode
  PORTD &= 0b11110111;// ensures that PD3 is off.
}

void leftStop() {
  TCCR2A &= 0b00000000; //OC2A/B disconnected
  PORTD &= 111110111;// ensures that PD3 is off.
  PORTB &= 111110111;// ensures PB3 is off
}

void moveStop() {
  rightStop();
  leftStop();
}

void moveForward() {
  leftTicks = 0;
  rightTicks = 0;
  leftForward();
  rightForward();
}

void moveBackward() { //I wrote this just in case we need it
  leftTicks = 0;
  rightTicks = 0;
  rightBackward();
  leftBackward();
}

void turnRight() {
  leftTicks = 0;
  rightTicks = 0;
  rightForward();
  leftBackward();
}

void turnLeft() {
  leftTicks = 0;
  rightTicks = 0;
  rightBackward();
  leftForward();
}

void handleCommand(TPacket *command) {
  switch (command->command) {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      requiredTicks = (float) command->params[0];
      speed = (float) command->params[1];
      moveForward();
      break;
    case COMMAND_TURN_LEFT:
      sendOK();
      requiredTicks = (float) command->params[0];
      speed = (float) command->params[1];
      turnLeft();
      break;
    case COMMAND_TURN_RIGHT:
      sendOK();
      requiredTicks = (float) command->params[0];
      speed = (float) command->params[1];
      turnRight();
      break;
    case COMMAND_REVERSE:
      sendOK();
      requiredTicks = (float) command->params[0];
      speed = (float) command->params[1];
      moveBackward();
      break;
    case COMMAND_STOP:
      sendOK();
      moveStop();
      break;
    case COMMAND_GET_STATS:
      sendStatus();
      break;
    default:
      sendBadCommand();
  }
}

void waitForHello() {
  int exit = 0;

  while (!exit) {
    TPacket hello;
    TResult result;

    do {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK) {
      if (hello.packetType == PACKET_TYPE_HELLO) {
        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD) {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}

void handlePacket(TPacket *packet) {
  switch (packet->packetType) {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD) {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD) {
    sendBadChecksum();
  }

  if ((leftTicks > requiredTicks)||(rightTicks > requiredTicks)) {
    moveStop();
  }
}
