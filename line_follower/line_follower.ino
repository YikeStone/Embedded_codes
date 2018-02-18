int in[7];
int ma[7]={0,0,0,0,0,0,0};
int mi[7]={500,500,500,500,500,500,500};
int md[]={10,11,9,6,8,7};
int cv;int k,pcv;
int perr;
long cerr;int pwma,pwmb,pwmm=255;
//float kp=0.28,ki=0.000008,kd=0.01;
float kp=0.19,ki=0.00008,kd=0.05;
void setup(){
  Serial.begin(9600);
  pinMode(13,INPUT);
  pinMode(12,INPUT);
  for(int i=0;i<5;i++)pinMode(md[i],OUTPUT);
  digitalWrite(md[0],HIGH);
  digitalWrite(md[1],LOW);
  digitalWrite(md[4],HIGH);
  digitalWrite(md[5],LOW);
  speedControl(255,255);
  for(int i=0;i<500;i++)
    takeInput();
  digitalWrite(md[0],LOW);
  digitalWrite(md[1],HIGH);
  digitalWrite(md[4],LOW);
  digitalWrite(md[5],HIGH);
  for(int i=0;i<1000;i++)
    takeInput();
  digitalWrite(md[0],HIGH);
  digitalWrite(md[1],LOW);
  digitalWrite(md[4],HIGH);
  digitalWrite(md[5],LOW);
  while(in[3]==0)
    takeInput();
  digitalWrite(md[0],HIGH);
  digitalWrite(md[1],LOW);
  digitalWrite(md[4],LOW);
  digitalWrite(md[5],HIGH);
}

void takeInput(){
  //in[0]=digitalRead(12);
  //in[7]=digitalRead(13);
  in[1]=analogRead(A1);
  in[2]=analogRead(A2);
  in[3]=analogRead(A3);
  in[4]=analogRead(A4);
  in[5]=analogRead(A5);
  for(int i=1;i<=5;i++){
    if(in[i]>ma[i])
      {ma[i]=in[i];}
    else
    if(in[i]<mi[i])
      mi[i]=in[i];
    
    in[i]=map(in[i],mi[i],ma[i],0,255);
  }
   in[3]=(in[3]>150)?1:0;
}
void setValues(){
  if(in[3]==1)k=(-in[1]*3-in[2]+in[4]+in[5]*3);
  if((in[1]>60||in[2]>60||in[4]>60||in[5]>60||in[3]==1))
  cv =k*in[3]
      +(1-in[3])*(((in[1]+in[2])>(in[4]+in[5]))?(k-(+(255-in[1])*3+(255-in[2]))):((255-in[4])+(255-in[5])*3+k));
   else cv=1800*(pcv/abs(pcv));
  pcv=cv;
  Serial.println(cv);
}
void loop (){
  /*for(int i=0;i<7;i++)
    {
      Serial.print(in[i]);Serial.print("  ");
    }*/
    
  takeInput();
  setValues();
int err = 0 - cv;
cerr+=err;
float pwm = err*kp + cerr*ki + (err-perr)*kd;
perr=err;
//Serial.println(pwm);
pwma=pwmm-pwm;pwmb=pwmm+pwm;
pwma=(pwma>pwmm)?pwmm:pwma;
pwmb=(pwmb>pwmm)?pwmm:pwmb;
pwma=(pwma<0)?0:pwma;
pwmb=(pwmb<0)?0:pwmb;
speedControl(pwma,pwmb);
}
void speedControl(int a, int b){

  analogWrite(md[2], map(a,0,255,0,255));//246
  analogWrite(md[3], map(b,0,255,0,250));}
