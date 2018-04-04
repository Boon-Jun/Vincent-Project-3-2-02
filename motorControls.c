volatile unsigned long leftTicks = 0, rightTicks = 0, leftRequiredTicks = 0, rightRequiredTicks = 0;

//Speed calculation
volatile unsigned long leftBeforeTicks = 0;
volatile unsigned long rightBeforeTicks = 0;
volatile unsigned int leftSpeed = 0, rightSpeed = 0;

int PWM_val_left = 200;
int PWM_val_right = 200;

//Calculate speed 
ISR(TIMER1_COMPA_vect)
{
  leftSpeed =  leftTicks - leftBeforeTicks;
  rightSpeed = rightTicks - rightBeforeTicks;
  leftBeforeTicks = leftTicks;
  rightBeforeTicks = rightTicks;
  }

ISR(PCINT0_vect) {
  leftTicks++;
}
ISR(PCINT1_vect) {
  rightTicks++;
}

ISR(TIMER0_COMPA_vect)
{
  OCR0A = PWM_val_left;
}
ISR(TIMER0_COMPB_vect)
{
  OCR0B = PWM_val_left;
}
ISR(TIMER2_COMPA_vect)
{
  OCR2A = PWM_val_right;
}
ISR(TIMER2_COMPB_vect)
{
  OCR2B = PWM_val_right;
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

void setup()
{
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
  OCR1A = 6250;          //time interval = 16 microseconds * 6250 = 0.1s
  TCCR1B = 0b00001100;    //Set CTC mode and prescaler to 256
  TIMSK1 = 0b010;
  
  sei();
}

void loop() {

}