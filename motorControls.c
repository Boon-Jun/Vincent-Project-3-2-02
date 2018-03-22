volatile unsigned long leftTicks = 0, rightTicks = 0, leftRequiredTicks = 0, rightRequiredTicks = 0;

void setup() {
  cli();
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
  OCR2B = 128;//set default of 50% duty cycle
  TIMSK2 |= 0b110;// Activates OCR2A and OCR2B compares match interrupt.

  //setup wheel encoders
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

ISR(PCINT0_vect) {
  leftTicks++;
}
ISR(PCINT1_vect) {
  rightTicks++;
}

ISR(TIMER0_COMPA_vect)
{
  OCR0A = 128;
}
ISR(TIMER0_COMPB_vect)
{
  OCR0B = 128;
}
ISR(TIMER2_COMPA_vect)
{
  OCR2A = 128;
}
ISR(TIMER2_COMPB_vect)
{
  OCR2B = 128;
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
  leftTicks = rightTicks = 0;
  rightForward();
  leftForward();
}

void moveBackward() {
  leftTicks = rightTicks = 0;
  rightBackward();
  leftBackward();
}

void turnRight() {
  leftTicks = rightTicks = 0;
  rightForward();
  leftBackward();
}

void turnLeft() {
  leftTicks = rightTicks = 0;
  rightBackward();
  leftForward();
}
