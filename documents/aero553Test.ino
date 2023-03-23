int fTrig = 7;
float l = 65; // reciever spacing, mm
float y = 381; // distance from transmitter to receiver line, mm
float x0 = 0;
float x1 = 0;
float x = 0;
volatile byte overflows1 = 0;
float d0 = 0;
float d1 = 0;
float c = 145;
float m = .684;

void setup() {
  DDRB &= B00000011;
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(fTrig, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  
  overflows1 = 0;
  TCCR1A = 0;// set TIM1 registers to 0  
  TCCR1B = 0;  
  TCCR1C = 0;  
  TCNT1 = 0;
  TIMSK1 = _BV(TOIE1); //enable interrupt
    
  digitalWrite(fTrig, HIGH); // trigger transmitter
  delayMicroseconds(10);
  digitalWrite(fTrig, LOW);
  while(!( (PINB >> 1 & B00000010 >> 1) && (PINB & B00000001) )); // true if any echo pin is high
  TCCR1B = 1;// start timer 1
  while(( (PINB >> 1 & B00000010 >> 1) && (PINB & B00000001) ));// true if every echo pin is high
  TCCR1B = 0;    //stop timer 1
  unsigned int temp_timer1 = TCNT1;  //store passed ticks  
  unsigned long ticks1 = ((((unsigned long)overflows1) << 16) | (unsigned long)temp_timer1) - 4;  
  d0 = m*ticks1 * .343 /(F_CPU /1000000) + c;
  TCCR1B = 1;// start timer 1 
  
  while(( (PINB >> 1 & B00000010 >> 1) || (PINB & B00000001) )); // true if any echo pin is high
  TCCR1B = 0;    //stop timer 1
  temp_timer1 = TCNT1;  //store passed ticks  
  ticks1 = ((((unsigned long)overflows1) << 16) | (unsigned long)temp_timer1) - 4;  
  d1 = m*ticks1 * .343 /(F_CPU /1000000) + c;

  x0 = sqrt(abs(d0*d0 - y*y)) + l/2;
  x1 = sqrt(abs(d1*d1 - y*y)) - l/2;
  x = (x0 + x1)/2;

  Serial.println(x);

  d0 = 0.0;
  d1 = 0.0;
  x0 = 0.0;
  x1 = 0.0;
  x = 0.0;
}

ISR(TIMER1_OVF_vect)   
{  
  overflows1++;  
}//end ISR  
