#include <serialize.h>
#include "packet.h"
#include "constants.h"
#include <math.h>
#include <util/delay.h>

typedef enum {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4,
  LOCATION = 5
} TDirection;

volatile int irActivated = 0;
TDirection directionStack[50];
unsigned long tickStack[50];
volatile int size = 3;
volatile int size_clone = 3;


volatile TDirection dir = STOP;
volatile unsigned long leftTicks = 0, rightTicks = 0, requiredTicks = 0;

//Speed calculation
volatile unsigned long leftBeforeTicks = 0;
volatile unsigned long rightBeforeTicks = 0;
volatile unsigned int leftSpeed = 0, rightSpeed = 0;

//PID
int PWM_val_left = 150;
int PWM_val_right = 150;

float Kp = 26, Kd = 12; //PD tunning


int computePID(int cmd, int target, int current, char reset)   {
  float pidOutput = 0;
  int error = 0;
  static int last_error = 0;
  if (reset == 1) {
    last_error = 0;
    return PWM_val_left;
  }
  error = abs(target) - abs(current);
  pidOutput = (Kp * error) + (Kd * (error - last_error));
  last_error = error;
  return constrain(cmd + int(pidOutput), 0, 255);
}


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


//Send request to play sound on the PI
void sendPlaySound() {
  TPacket soundPacket;
  soundPacket.packetType = PACKET_TYPE_PLAY_SOUND;
  sendResponse(&soundPacket);
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

void handleCommand(TPacket *command) {
  switch (command->command) {
    // For movement commands, param[0] = distance
    case COMMAND_FORWARD:
      sendMessage("forward received");
      sendOK();
      requiredTicks = (float) command->params[0];
      moveForward();
      break;
    case COMMAND_TURN_LEFT:
      sendOK();
      requiredTicks = (float) command->params[0];
      turnLeft();
      break;
    case COMMAND_TURN_RIGHT:
      sendOK();
      requiredTicks = (float) command->params[0];
      turnRight();
      break;
    case COMMAND_REVERSE:
      sendOK();
      requiredTicks = (float) command->params[0];
      moveBackward();
      break;
    case COMMAND_STOP:
      sendOK();
      moveStop();
      break;
    case COMMAND_BACKTRACK:
      sendOK();
      sendMessage("Backtracking in progress");
      back_track();
      break;
    case COMMAND_MARK_LOCATION:
      sendOK();
      directionStack[size] = LOCATION;
      tickStack[size] = 0;
      size += 1;
      size_clone = size;
      sendMessage("Location marked");
      break;
    default:
      sendBadCommand();
  }
}

void back_track() {
  while (size--) {
    requiredTicks = tickStack[size];
    dir = directionStack[size];
    switch (dir) {
      case FORWARD:
        moveBackward();
        break;
      case BACKWARD:
        moveForward();
        break;
      case LEFT:
        turnRight();
        break;
      case RIGHT:
        turnLeft();
        break;
      case LOCATION:
        sendPlaySound();
        break;
    }
    while (dir != STOP) {
      if ((leftTicks > requiredTicks) || (rightTicks > requiredTicks)) {
        leftStop();
        rightStop();
        dir = STOP;
      }
    }
    _delay_ms(2000);
  }
  sendMessage("Backtracking done!");
  size = size_clone;
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
        sendMessage("HANDSHAKE DONE");
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


void leftForward(void)
{
  TCCR0A = 0b10000001;  //OC0A/PD6/Digital pin 6
  PORTD &= 0b11011111;
}
void leftReverse(void)
{
  TCCR0A = 0b00100001;  //OC0B/PD5/Digital pin 5
  PORTD &= 0b10111111;
}
void leftStop()
{
  TCCR0A = 0b00000001;
  PORTD &= 0b11011111;
  PORTD &= 0b10111111;
}
void rightForward(void)
{
  TCCR2A = 0b10000001;  //OC2A/PB3/Digital pin 11
  PORTD &= 0b11011111;
}

void rightReverse(void)
{
  TCCR2A = 0b00100001;  //OC2B/PD3/Digital pin 3
  PORTD &= 0b11011111;
}
void rightStop()
{
  TCCR2A = 0b00000001;
  PORTD &= 0b11011111;
  PORTD &= 0b11011111;
}


void moveStop() {
  rightStop();
  leftStop();

  //Store movement direction
  directionStack[size] = dir;
  tickStack[size] = (leftTicks > rightTicks) ? leftTicks : rightTicks;
  size += 1;
  size_clone = size;

  sendStatus();
  leftTicks = 0;
  rightTicks = 0;
  dir = STOP;
  PWM_val_left = computePID(PWM_val_left, rightSpeed, leftSpeed, 1);

  if (!irActivated) {
    cli();
    EICRA |= 0b00000011;
    EIMSK |= 0b00000001;
    irActivated = 1;
    sei();
  }
}

void moveForward() {
  leftTicks = 0;
  rightTicks = 0;

  dir = FORWARD;

  rightForward();
  leftForward();
}

void moveBackward() {
  leftTicks = 0;
  rightTicks = 0;

  dir = BACKWARD;

  leftReverse();
  rightReverse();
}

void turnRight() {
  leftTicks = 0;
  rightTicks = 0;

  dir = RIGHT;

  rightReverse();
  leftForward();
}

void turnLeft() {
  leftTicks = 0;
  rightTicks = 0;

  dir = LEFT;

  rightForward();
  leftReverse();
}


//Calculate speed and PID at the same time
ISR(TIMER1_COMPA_vect)
{
  leftSpeed =  leftTicks - leftBeforeTicks;
  rightSpeed = rightTicks - rightBeforeTicks;
  leftBeforeTicks = leftTicks;
  rightBeforeTicks = rightTicks;

  PWM_val_left = computePID(PWM_val_left, rightSpeed, leftSpeed, 0);

}
//interrupts for wheel encoder
ISR(PCINT0_vect) {
  leftTicks++;
}
ISR(PCINT1_vect) {
  rightTicks++;
}

ISR(TIMER0_COMPA_vect) {
  OCR0A = PWM_val_left; 
}
ISR(TIMER0_COMPB_vect) {
  OCR0B = PWM_val_left; 
}
ISR(TIMER2_COMPA_vect) {
  OCR2A = PWM_val_right; 
}
ISR(TIMER2_COMPB_vect) {
  OCR2B = PWM_val_right; 
}

//IR SENSOR
ISR(INT0_vect) {
  moveStop();
  EIMSK &=  0b11111110;
  irActivated = 0;
}


void setup() {
  cli();
  setupSerial();

  DDRD |= ((1 << DDD6) | (1 << DDD5) | (1 << DDD3));
  DDRB |= (1 << DDB3);


  //setup wheel encoders
  DDRC &= 0b11111110;// set PC0 as input
  DDRB &= 0b11111101;//set PB1 as input
  PORTC |= 0b00000001;//drive PC0 to high
  PORTB |= 0b00000010;//drive PB5 to high
  PCMSK1 = 0b00000001;//set PCINT8 to activate pin change interrup RIght encoder
  PCMSK0 = 0b00000010;//set PCINT5 to activate pin change interrupt 0 left encoder

  PCICR = 0b00000011;//enable pin change interrupt 0 and 1

  //Init PWM for wheels
  TCNT0 = 0;
  OCR0A = 200;
  OCR0B = 200;
  TIMSK0 |= 0b110;
  TCNT2 = 0;
  OCR2A = 200;
  OCR2B = 200;
  TIMSK2 |= 0b110;

  TCCR0B = 0b00000011;
  TCCR2B = 0b00000100;

  //Init timer 1 to CTC mode to sample speed of wheel at every 0.1s
  TCCR1A = 0b00000000;    //Set CTC mode (disconnect OC1A,OC1B)
  TCNT1 = 0;
  OCR1A = 3125;           //time interval = 16 microseconds * 3125 = 0.05s
  TCCR1B = 0b00001100;    //Set CTC mode and prescaler to 256
  TIMSK1 = 0b010;

  //Activate IR sensor isr
  EICRA |= 0b00000011;
  EIMSK =  0b00000001;
  irActivated = 1;
  sei();
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
  if (dir != STOP && ((leftTicks > requiredTicks) || (rightTicks > requiredTicks))) {
    moveStop();
  }
}
