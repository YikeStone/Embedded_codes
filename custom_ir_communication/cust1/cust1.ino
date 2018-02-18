uint32_t start=0;volatile int duration=0;
volatile bool chaR=false;volatile int i=0;
volatile int msg=0;
const int led=3;
const int n=70;
void setup(){
  Serial.begin(9600);
  pinMode(2,INPUT);
  pinMode(led,OUTPUT);
  attachInterrupt(0,startM,RISING);
  attachInterrupt(0,endM,FALLING);
  }

void startM()
{
  start=millis();
}

void endM(){
    duration = millis()-start;
    duration +=40;
    i++;
    msg=msg*10+duration/100;
  }
void loop(){
  if(i==4){
    Serial.print(decodeMsg(msg));i=0;msg=0;
    }
  if(Serial.available()>0)  //if the input field of the serial monitor has a value
  {
    char ch = Serial.read();
    sendCode(encodeMsg(ch));
  }
}

int encodeMsg(char ch){
  switch(ch){
    case '\n': return 1111;
    case 'a':  return 1112;
    case 'b':  return 1121;
    case 'c':  return 1332;
    case 'd':  return 1122;
    case 'e':  return 1211;
    case 'f':  return 1212;
    case 'g':  return 1221;
    case 'h':  return 1222;
    case 'i':  return 2111;
    case 'j':  return 2112;
    case 'k':  return 2121;
    case 'l':  return 2122;
    case 'm':  return 2211;
    case 'n':  return 2212;
    case 'o':  return 2221;
    case 'p':  return 2222;
    case 'q':  return 3111;
    case 'r':  return 3112;
    case 's':  return 3121;
    case 't':  return 3122;
    case 'u':  return 3211;
    case 'v':  return 3212;
    case 'w':  return 3221;
    case 'x':  return 3222;
    case 'y':  return 4111;
    case 'z':  return 1311;
    case '1':  return 4121;
    case '2':  return 4122;
    case '3':  return 4211;
    case '4':  return 4212;
    case '5':  return 4221;
    case '6':  return 4222;
    case '7':  return 5111;
    case '8':  return 5112;
    case '9':  return 5121;
    case '.':  return 5122;
    case '(':  return 5211;
    case ')':  return 5212;
    case '+':  return 5221;
    case '-':  return 5222;
    case '!':  return 4112;
    case '?':  return 1312;
    case ',':  return 1313;
    case '0':  return 1321;
    case 'A':  return 1322;
    case 'B':  return 1323;
    case 'C':  return 1331;
    case 'D':  return 1333;
    case 'E':  return 2311;
    case 'F':  return 2312;
    case 'G':  return 2313;
    case 'H':  return 2321;
    case 'I':  return 2322;
    case 'J':  return 2323;
    case 'K':  return 2331;
    case 'L':  return 2332;
    case 'M':  return 2333;
    case 'N':  return 3311;
    case 'O':  return 3312;
    case 'P':  return 3313;
    case 'Q':  return 3321;
    case 'R':  return 3322;
    case 'S':  return 3323;
    case 'T':  return 3331;
    case 'U':  return 3332;
    case 'V':  return 4311;
    case 'W':  return 4312;
    case 'X':  return 4313;
    case 'Y':  return 4321;
    case 'Z':  return 4322;



    // enter all the codes here should be exactly same as in letters array
    }
  }



char decodeMsg(int i){
   switch(i){
   //make all cases for all the values of i  
    case 1111:  return '\n';
    case 1112:  return 'a';
    case 1121:  return 'b';
    case 1332:  return 'c';
    case 1122:  return 'd';
    case 1211:  return 'e';
    case 1212:  return 'f';
    case 1221:  return 'g';
    case 1222:  return 'h';
    case 2111:  return 'i';
    case 2112:  return 'j';
    case 2121:  return 'k';
    case 2122:  return 'l';
    case 2211:  return 'm';
    case 2212:  return 'n';
    case 2221:  return 'o';
    case 2222:  return 'p';
    case 3111:  return 'q';
    case 3112:  return 'r';
    case 3121:  return 's';
    case 3122:  return 't';
    case 3211:  return 'u';
    case 3212:  return 'v';
    case 3221:  return 'w';
    case 3222:  return 'x';
    case 4111:  return 'y';
    case 1311:  return 'z';
    case 4121:  return '1';
    case 4122:  return '2';
    case 4211:  return '3';
    case 4212:  return '4';
    case 4221:  return '5';
    case 4222:  return '6';
    case 5111:  return '7';
    case 5112:  return '8';
    case 5121:  return '9';
    case 5122:  return '.';
    case 5211:  return '(';
    case 5212:  return ')';
    case 5221:  return '+';
    case 5222:  return '-';
    case 4112:  return '!';
    case 1312:  return '?';
    case 1313:  return ',';
    case 1321:  return '0';
    case 1322:  return 'A';
    case 1323:  return 'B';
    case 1331:  return 'C';
    case 1333:  return 'D';
    case 2311:  return 'E';
    case 2312:  return 'F';
    case 2313:  return 'G';
    case 2321:  return 'H';
    case 2322:  return 'I';
    case 2323:  return 'J';
    case 2331:  return 'K';
    case 2332:  return 'L';
    case 2333:  return 'M';
    case 3311:  return 'N';
    case 3312:  return 'O';
    case 3313:  return 'P';
    case 3321:  return 'Q';
    case 3322:  return 'R';
    case 3323:  return 'S';
    case 3331:  return 'T';
    case 3332:  return 'U';
    case 4311:  return 'V';
    case 4312:  return 'W';
    case 4313:  return 'X';
    case 4321:  return 'Y';
    case 4322:  return 'Z';

    }
  }


void sendCode(int code)    //convert the char to the corresponding element in the array
{
  int ch = 0;
  while((ch=(code%10))!=0)
  {
    digitalWrite(led, HIGH);  //turn on the LED
    dela(ch*100); 
    digitalWrite(led, LOW);  //turn off the LED
    dela(100);
    code = code/10;
  }
  dela(100);
}

void dela(int d)
{
     uint32_t t = millis();
     while((millis()-t)<d);
}
