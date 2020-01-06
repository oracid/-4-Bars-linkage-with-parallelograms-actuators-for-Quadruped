//FourBarParallelogram-1 is a 4 bars leg with parallelograms actuators - 04/01/2020
#include <Servo.h>
const int nbSrv=2;              // Number of servos
Servo Srv[nbSrv];               // Servos table
int SrvOn[nbSrv]={1,1};         // Servos table on/off
int Speed = 20;                 // speed
int errL=10, errR=4;            // servos point 0 correction

void  setup() {
  Serial.begin(9600);
  pinMode(0,INPUT_PULLUP);                    //start button attachment

  Srv[0].attach(4); //Srv[0].write(0+errL);
  Srv[1].attach(8); //Srv[1].write(0+errR);
  for (int i=85;i>=0;i--){Srv[0].write(i+errL);Srv[1].write(i+errR);delay(30);}
  Serial.print("\n\t To start, click on the Start button");
  while( digitalRead(0) );  delay(400);       // waiting for start button pressed 

  Serial.print("\n\t Started");
//  Square(100, 20);
}

void  loop(){
 Square(100, 20);
}

void Square(int lngth, int width){
  int minX=lngth/2*(-1), maxX=minX+lngth;
  int minY=10, maxY=minY+width;
  for (int i=minX; i<=maxX;i++) {InverseKinematics(i,minY,Srv[0],Srv[1],SrvOn[0],SrvOn[1]);}
  for (int i=minY;i<=maxY;i++)  {InverseKinematics(maxX,i,Srv[0],Srv[1],SrvOn[0],SrvOn[1]);}
  for (int i=maxX; i>=minX;i--) {InverseKinematics(i,maxY,Srv[0],Srv[1],SrvOn[0],SrvOn[1]);}
  for (int i=maxY;i>=minY;i--)  {InverseKinematics(minX,i,Srv[0],Srv[1],SrvOn[0],SrvOn[1]);}
}

void InverseKinematics(int Px, int Py, Servo srvL, Servo srvR, int srvLon, int srvRon){
  float A1x=0, A1y=144, A2x=0, A2y=144;                   // Values of servos positions
  float a1=112, c1=32, a2=112, c2=32;                     // Values of leg sides lengths

  int S1, S2;
  float d=A1y-Py, e=Px;
  float b=sqrt((d*d)+(e*e));
  float S=acos(d/b);
  float A1_2=acos(((b*b)+(c1*c1)-(a1*a1))/(2*b*c1));
  float A2_2=acos(((b*b)+(c2*c2)-(a2*a2))/(2*b*c2));
  float A1_1=(PI/2)-A1_2+S;
  float A2_1=(PI/2)-A2_2-S;
  if (e>=0){ 
    S1=round(A1_1*57.296);
    S2=round(A2_1*57.296);
  }else{
    S1=round(A2_1*57.296);
    S2=round(A1_1*57.296);
  }

//  Serial.print("\n\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);
//  Serial.print("\n\t d=");Serial.print(d);Serial.print("\t\t e=");Serial.print(e);Serial.print("\t\t b=");Serial.print(b);Serial.print("\t\t S=");Serial.print(S*57.296);
//  Serial.print("\n\t A1_1=");Serial.print(A1_1*57.296);Serial.print("\t\t A1_2=");Serial.print(A1_2*57.296);Serial.print("\t\t A2_2=");Serial.print(A2_2*57.296);Serial.print("\t\t A2_1=");Serial.print(A2_1*57.296);
//  Serial.print("\n\t Result of calculations, angles of the servos");
//  Serial.print("\n\t S1=");Serial.print(S1);Serial.print("°\t\t\t S2=");Serial.print(S2);Serial.print("°");


  if ( b>(a1+c1) ){
    Serial.print("\n\t Target point Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print("\t b=");Serial.print(b);Serial.print(" > ");
    Serial.print(a1+c1);Serial.print(" is too long. Target impossible to reach   !!!!!");
    return;
  }
  if (S1<0){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S1<0° is not reachable   !!!!!");
    return;
  }
  if (S2<0){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S2<0° is not reachable   !!!!!");
    return;
  }
  if (S1>140){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S1>140° is not reachable   !!!!!");
    return;
  }
  if (S2>140){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S2>140° is not reachable   !!!!!");
    return;
  }

//  Serial.print("\t executed command");
  if (srvLon) srvL.write(S1+errL);        // set target Left servo position if servo switch is On
  if (srvRon) srvR.write(S2+errR);        // set target Right servo position if servo switch is On
  delay(Speed);
}
