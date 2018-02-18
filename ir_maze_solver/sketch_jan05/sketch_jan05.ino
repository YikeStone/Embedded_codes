int A[11][11];
int b[6][2]={{7,3},{0,0}};
int ch[20][2];int nch=-1;
int nb=2,ib=0,M[18],im;
int ix,iy;
int X,Y,D;
int in[10];
int ma[7]={0,0,0,0,0,0,0};
int mi[7]={0,500,500,0,500,500,0};
int md[]={8,7,9,3,4,5};
int cv;int m,pcv;
float kp=0.19,ki=0.00008,kd=0.05;
int a;
int mov(){long tt=millis();
  int bbb=0;int bb1=0;int b=0;int aa=0;int perr;long cerr=0;int pwma=0,pwmb=0,pwmm=180;
  	while((b==0)||(in[7]==0)||(in[9]==0)){
  takeInput();long tt1=millis();int cc=0;
  if((in[0]==1)||(in[6]==1)&&((tt1-tt)>100)&&((tt1-tt)<1900)){bbb=1;aa=1;}
  if((bbb==1)&&in[7]==1||in[9]==1) bb1=1;
  if(bbb!=1&&in[7]==0&&in[9]==0)b=1;
  if(bb1==1&&in[7]==0&&in[9]==0)b=1;
  if((b==1)&&((in[7]==1)||(in[9]==1))){speedControl(0,0);return in[8];}
  if(in[0]==0&&in[6]==0) pwmm=180;
  setValues();
  int err = 0 - cv;
  cerr+=err;
  float pwm = err*kp + cerr*ki + (err-perr)*kd;
  perr=err;
  if(aa==1){pwm=0;aa=0;}
  pwma=pwmm-pwm;pwmb=pwmm+pwm;
  pwma=(pwma>pwmm)?pwmm:pwma;
  pwmb=(pwmb>pwmm)?pwmm:pwmb;
  pwma=(pwma<0)?0:pwma;
  pwmb=(pwmb<0)?0:pwmb;
  digitalWrite(md[0],HIGH);
  digitalWrite(md[1],LOW);
  digitalWrite(md[4],LOW);
  digitalWrite(md[5],HIGH);
  speedControl(pwma,pwmb);}
  speedControl(0,0);
  return in[8];
}
void setup(){
  D=1;X=1;Y=1;im=-1;
  Serial.begin(9600);
  pinMode(13,OUTPUT);pinMode(10,INPUT);
  pinMode(11,INPUT);pinMode(12,INPUT);
  pinMode(4,OUTPUT);pinMode(5,OUTPUT);//motor B
  pinMode(7,OUTPUT);pinMode(8,OUTPUT);//motor A
  pinMode(9,OUTPUT);//motor A pwm 
  pinMode(3,OUTPUT);//motor B pwm
  digitalWrite(13,HIGH);
 digitalWrite(13,HIGH);
  for(int i=0;i<500;i++)
  {
    takeInput();
    digitalWrite(md[0],HIGH);
    digitalWrite(md[1],LOW);
    digitalWrite(md[4],HIGH);
    digitalWrite(md[5],LOW);
    speedControl(255,255);
  }
  for(int i=0;i<1000;i++)
    {takeInput();
    digitalWrite(md[0],LOW);
    digitalWrite(md[1],HIGH);
    digitalWrite(md[4],LOW);
    digitalWrite(md[5],HIGH);
    speedControl(255,255);
  }
  while(in[3]==0){
    takeInput();
    digitalWrite(md[0],HIGH);
    digitalWrite(md[1],LOW);
    digitalWrite(md[4],HIGH);
    digitalWrite(md[5],LOW);
    speedControl(255,255);
  }
  speedControl(0,0);
  digitalWrite(md[0],HIGH);
  digitalWrite(md[1],LOW);
  digitalWrite(md[4],LOW);
  digitalWrite(md[5],HIGH);
  digitalWrite(13,LOW);
  ix=0;iy=0;
  for(int i=0;i<6;i++){printf("%d,%d\n",b[i][0]+1,b[i][1]+1);}

    for(int i=0;i<11;i++){
        A[i][0]=1000;A[0][i]=1000;A[10][i]=1000;A[i][10]=1000;
    }
    mov();
}
void calc(){
    for(int i=1;i<10;i++)

        for(int j=1;j<10;j++)

            if(A[i][j]!=500)

                A[i][j]=abs(b[ib][0]-i+1)+abs(b[ib][1]-j+1);
}

void takeInput(){
  in[0]=digitalRead(12);
  in[1]=analogRead(18);
  in[2]=analogRead(19);
  in[3]=digitalRead(11);
  in[4]=analogRead(20);
  in[5]=analogRead(21);
  in[6]=digitalRead(10);
  for(int i=1;i<6;i++){
    if(i==3)continue;
    if(in[i]>ma[i])
      {ma[i]=in[i];}
    else
    if(in[i]<mi[i])
      mi[i]=in[i];
    
    in[i]=map(in[i],mi[i],ma[i],0,255);
  }
   in[7]=analogRead(15);in[7]=((in[7]>500)?1:0);
   in[8]=analogRead(16);in[8]=((in[8]>500)?1:0);
   in[9]=analogRead(17);in[9]=((in[9]>150)?1:0);
}
void setValues(){
  if(in[3]==1)m=(-in[1]*3-in[2]+in[4]+in[5]*3);
  if((in[1]>60||in[2]>60||in[4]>60||in[5]>60||in[3]==1))
  cv =m*in[3]
      +(1-in[3])*(((in[1]+in[2])>(in[4]+in[5]))?(m-(+(255-in[1])*3+(255-in[2]))):((255-in[4])+(255-in[5])*3+m));
   else cv=1800*(pcv/abs(pcv));
  pcv=cv;
}
void turn(int c){
int e=0;
while(e==0||(in[3]==0))
  {
  takeInput();
  if(in[3]==0)e=1;
  if(c){
    digitalWrite(md[0],HIGH);
    digitalWrite(md[1],LOW);
    digitalWrite(md[4],HIGH);
    digitalWrite(md[5],LOW);
    
  }
  else{
    digitalWrite(md[0],LOW);
    digitalWrite(md[1],HIGH);
    digitalWrite(md[4],LOW);
    digitalWrite(md[5],HIGH);
    
  }
    speedControl(200,200);
  }
  if(c){D--;if(D==0)D=4;}
  else{D++;if(D==5)D=1;}
  speedControl(0,0);
  digitalWrite(md[0],HIGH);
  digitalWrite(md[1],LOW);
  digitalWrite(md[4],LOW);
  digitalWrite(md[5],HIGH);
}
void loop(){
  sp();
  if(!go())
   for(int i=0;i<50;i++){
    digitalWrite(6,HIGH);
    delay(100);
    digitalWrite(6,LOW);
    delay(100);
  ib++;}
  if(ib==nb)delay(100000);
}

int go(){
  for(int i=0;i<=im;i++){if(goto_(M[i]))return -1;}
  return 0;
}
void setp(int d){
switch(d){
    case 1:Y++;break;
    case 2:X++;break;
    case 3:Y--;break;
    case 4:X--;break;
}
}

int goto_(int d){
  
  if(d==D){if(!mov()){setp(D);turn(0);turn(0);mov();ch[++nch][0]=X;ch[nch][1]=Y;A[X][Y]=500;setp(D);return -1;}else {setp(D);return 0;}}
  if((abs(d-D)==2)){turn(1);turn(1);if(!mov()){setp(D);turn(0);turn(0);mov();ch[++nch][0]=X;ch[nch][1]=Y;A[X][Y]=500;setp(D);return -1;}else setp(D);return 0;}
  if((d==4&&D==1)||(d<D)){turn(1);if(!mov()){setp(D);turn(0);turn(0);mov();ch[++nch][0]=X;ch[nch][1]=Y;A[X][Y]=500;setp(D);return -1;}else setp(D);return 0;}
  if((d>D)||(D==4&&d==1)){turn(0);if(!mov()){setp(D);turn(0);turn(0);mov();ch[++nch][0]=X;ch[nch][1]=Y;A[X][Y]=500;setp(D);return -1;}else setp(D);return 0;}
}
void sp(){
    ix=X;iy=Y;im=-1;
    calc();
   int i=0,j=0,k=0;
    
                while(!((b[ib][0]==(ix-1))&&(b[ib][1]==(iy-1)))){
                    A[ix][iy]+=2;
                    nextd(min(A[ix][iy+1],min(A[ix-1][iy],min(A[ix+1][iy],A[ix][iy-1]))));
                    for(k=0;k<nch;k++){
                        if((ix==ch[k][0])&&(iy==ch[k][1])){
                            A[ix][iy]=500;
                            switch(M[im]){
                                case 1:A[ix][--iy]+=2;break;

                                case 2:A[--ix][iy]+=2;break;

                                case 3:A[ix][++iy]+=2;break;

                                case 4:A[++ix][iy]+=2;break;

                            }
                        im--;break;
                        }
                    }
                }
                for(i=1;i<im;i++){
                        if(abs(M[i]-M[i-1])==2){for(j=i-1;j<=im-2;j++)M[j]=M[j+2];im-=2;i=((i<2)?0:(i-2));}
                        else if(abs(M[i+1]-M[i-1])==2){M[i-1]=M[i];for(j=i;j<=im-2;j++)M[j]=M[j+2];im-=2;i=((i<2)?0:(i-2));}
                }
                
                
}
void nextd(int t1){
    if(im!=0){
        switch(M[im]){

            case 1:if(t1==A[ix][iy+1]){iy++;M[++im]=1;return;}break;

            case 2:if(t1==A[ix+1][iy]){ix++;M[++im]=2;return;}break;

            case 3:if(t1==A[ix][iy-1]){iy--;M[++im]=3;return;}break;

            case 4:if(t1==A[ix-1][iy]){ix--;M[++im]=4;return;}break;
        }
    }
     switch(D){
    
      case 1:
      if(t1==A[ix][iy+1]){
         iy++;M[++im]=1;return;
      }    
      if(t1==A[ix+1][iy]){
         ix++;M[++im]=2;return;
      }
      if(t1==A[ix-1][iy]){
          ix--;M[++im]=4;return;
      }
      if(t1==A[ix][iy-1]){
          iy--;M[++im]=3;return;
      }break;
      case 2:    
      if(t1==A[ix+1][iy]){
         ix++;M[++im]=2;return;
      }
      
      if(t1==A[ix][iy+1]){
         iy++;M[++im]=1;return;
      }
      if(t1==A[ix][iy-1]){
          iy--;M[++im]=3;return;
      }
      if(t1==A[ix-1][iy]){
          ix--;M[++im]=4;return;
      }
      break;
      
      case 3:
      
      if(t1==A[ix][iy-1]){
          iy--;M[++im]=3;return;
      }break;    
      if(t1==A[ix+1][iy]){
         ix++;M[++im]=2;return;
      }
      if(t1==A[ix-1][iy]){
          ix--;M[++im]=4;return;
      }
      if(t1==A[ix][iy+1]){
         iy++;M[++im]=1;return;
      }
      case 4:
      if(t1==A[ix-1][iy]){
          ix--;M[++im]=4;return;
      }
      if(t1==A[ix][iy-1]){
          iy--;M[++im]=3;return;
      }
      if(t1==A[ix][iy+1]){
         iy++;M[++im]=1;return;
      }
      if(t1==A[ix+1][iy]){
         ix++;M[++im]=2;return;
      }
      break;
    }

}  
void speedControl(int a, int b){
  analogWrite(9, map(a,0,255,0,145));
  analogWrite(3, map(b,0,255,0,150));
}
