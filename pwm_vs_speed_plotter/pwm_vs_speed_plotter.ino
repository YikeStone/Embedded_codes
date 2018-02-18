



int encoderPin1 = 2;
int encoderPin2 = 3;
 
volatile int A_lastEncoded = 0;
volatile long A_encoderValue = 0;
volatile int B_lastEncoded = 0;
volatile long B_encoderValue = 0;
  
long A_lastencoderValue = 0;
long B_lastencoderValue = 0;
  
int lastMSB = 0;
int lastLSB = 0;
ISR (PCINT1_vect){
    int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit
 
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (A_lastEncoded << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) A_encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) A_encoderValue --;
 
  A_lastEncoded = encoded;
  }
void setup() {
  Serial.begin (9600);
   *digitalPinToPCMSK(A5) |= bit (digitalPinToPCMSKbit(A5));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(A5)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(A5));
  *digitalPinToPCMSK(A4) |= bit (digitalPinToPCMSKbit(A4));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(A4)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(A4));
  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);
 
  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
  int i;
  for(i = 6; i<13; i++)
  pinMode(i,OUTPUT);
  digitalWrite(10,1);
  digitalWrite(11,0);

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);
 
}
 int j=100;
void loop(){ 
  //Do stuff here
  analogWrite(9,j);

  Serial.print(j);Serial.print("\t");Serial.println(B_encoderValue);
   j++;B_encoderValue=0;
  delay(500); //just here to slow down the output, and show it will work  even during a delay
if(j==256)return;
}
 
 
void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit
 
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (B_lastEncoded << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) B_encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) B_encoderValue --;
 
  B_lastEncoded = encoded;
  }
