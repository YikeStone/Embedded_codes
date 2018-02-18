uint16_t start=0;bool high=false;int duration=0;char c=0;
bool chaR=false;int i=0;
int dotdelay = 500;
int dashdelay = 1500;
int halfdelay = (dotdelay+dashdelay)/2.0;
int delaybc=500;//delay between next char sent
//change this letters array to your own code, each should be of 6 . or - exactly.
char* letters[] = {"......" ,
"...../" ,
"..../." ,
"....//" ,
".../.." ,
"..././" ,
"...//." ,
"...///" ,
"../..." ,
"../../" ,
".././." ,
"..//.." ,
"..//./" ,
"..///." ,
"..////" ,
"./...." ,
"./.../" ,
"./../." ,
"./..//" ,
"././.." ,
"./././" ,
"././/." ,
"././//" ,
".//..." ,
".//../" ,
".//./." ,
".//.//" ,
".///.." ,
".///./" ,
".////." ,
"./////" ,
"/....." ,
"/..../" ,
"/.../." ,
"/...//" ,
"/../.." ,
"/.././" ,
"/..//." ,
"/..///" ,
"/./..." ,
"/./../" ,
"/././." ,
"/././/" ,
"//////" };
int led=3;
int photoDetector=2;
int n=40;// size of letters array


char msg[7]={0,0,0,0,0,0,'\0'};
void setup(){
  Serial.begin(57600);
  Serial.println("Starting....");
  pinMode(photoDetector,INPUT);
  pinMode(led,OUTPUT);
  attachInterrupt(photoDetector,startM,RISING);
  attachInterrupt(photoDetector,endM,FALLING);
  }

void startM()
{
  start=millis();
 Serial.println("Hello!!!");
  //high=true;
  //while(high);
  }

void endM(){
  duration = millis()-start;
  Serial.println("Bye!!!");
  //high=false;
  if(duration<halfdelay)c='.';
  else c='/';
  chaR=true;
  }


  
void loop(){
  if(chaR){
    Serial.println("OK...");
    msg[i++]=c;
    chaR=false;
    if(i==6){
      int j=0;
    while(strcmp(letters[j++],msg)&&(j<n));
    Serial.print(decodeMsg(j));i=0;
    }
  }
  if(Serial.available()>0)  //if the input field of the serial monitor has a value
  {
    char ch = Serial.read();    
    sendCode(encodeMsg(ch));
  }
}



char* encodeMsg(char ch){
  switch(ch){
    case '\n': return "......";
    case 'A':  return "...../";
    case 'B':  return "..../.";
    case 'D':  return "....//";
    case 'E':  return ".../..";
    case 'F':  return "..././";
    case 'G':  return "...//.";
    case 'H':  return "...///";
    case 'I':  return "../...";
    case 'J':  return "../../";
    case 'K':  return ".././.";
    case 'L':  return "..//..";
    case 'M':  return "..//./";
    case 'N':  return "..///.";
    case 'O':  return "..////";
    case 'P':  return "./....";
    case 'Q':  return "./.../";
    case 'R':  return "./../.";
    case 'S':  return "./..//";
    case 'T':  return "././..";
    case 'U':  return "./././";
    case 'V':  return "././/.";
    case 'W':  return "././//";
    case 'X':  return ".//...";
    case 'Y':  return ".//../";
    case 'Z':  return ".//./.";
    case '1':  return ".//.//";
    case '2':  return ".///..";
    case '3':  return ".///./";
    case '4':  return ".////.";
    case '5':  return "./////";
    case '6':  return "/.....";
    case '7':  return "/..../";
    case '8':  return "/.../.";
    case '9':  return "/...//";
    case '.':  return "/../..";
    case '(':  return "/.././";
    case ')':  return "/..//.";
    case '+':  return "/..///";
    case '-':  return "/./...";
    case '!':  return "/./../";
    case '?':  return "/././.";
    case ',':  return "/././/";
    case '0':  return "//////";
    // enter all the codes here should be exactly same as in letters array
    }
  }



char decodeMsg(int i){
   switch(i){
   //make all cases for all the values of i  
    case 0 :return '\n';
    case 1 :return 'A';
    case 2 :return 'B';
    case 3 :return 'C';
    case 4 :return 'D';
    case 5 :return 'F';
    case 6 :return 'G';
    case 7 :return 'H';
    case 8 :return 'I';
    case 9 :return 'J';
    case 10 :return 'K';
    case 11 :return 'L';
    case 12 :return 'M';
    case 13 :return 'N';
    case 14 :return 'O';
    case 15 :return 'P';
    case 16 :return 'Q';
    case 17 :return 'R';
    case 18 :return 'S';
    case 19 :return 'T';
    case 20 :return 'U';
    case 21 :return 'V';
    case 22 :return 'W';
    case 23 :return 'X';
    case 24 :return 'Y';
    case 25 :return 'Z';
    case 26 :return '1';
    case 27 :return '2';
    case 28 :return '3';
    case 29 :return '4';
    case 30 :return '5';
    case 31 :return '6';
    case 32 :return '7';
    case 33 :return '8';
    case 34 :return '9';
    case 35 :return '.';
    case 36 :return '(';
    case 37 :return ')';
    case 38 :return '+';
    case 39 :return '-';
    case 40 :return '!';
    case 41 :return '?';
    case 42 :return ',';
    case 43 :return '0';
    //etc...
    }
  }


void sendCode(char* sequence)    //convert the char to the corresponding element in the array
{
  Serial.println("Character recevied.");
  int i = 0;
  while(sequence[i] != '\0')  // the symbol \0 is the end of a sentence
  {
    makeBlink(sequence[i]);    //call makeBlink to evaluate if the it is a dot or dash
    i++;
  }
  dela(dotdelay);      //the delay between symbols in one occurrence of the array is one unit
}

void dela(int d)
{
  
     uint64_t t = millis();
     while((millis()-t)<d);
}

void makeBlink(char dotOrDash)
{
  Serial.println("Blinking.....");
  digitalWrite(led, HIGH);  //turn on the LED
  if(dotOrDash == '.')
  {
    dela(dotdelay);  //500 milliseconds
  }
  else   //must be a dash
  {
    dela(dashdelay);  //1500 milliseconds
  }
  digitalWrite(led, LOW);  //turn off the LED
  dela(delaybc); //the delay between letters is 3 units
}
