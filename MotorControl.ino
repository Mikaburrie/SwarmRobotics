#include <Wire.h> 
#include <LiquidCrystal_I2C.h> 
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); 
int leftF = 9; // in1
int leftB = 4; // in2
int rightF = 7; // in3
int rightB = 8; // in4
int enL = 5; // enA
int enR = 6; // enB
int buttons[] = {11,12}; // enA
int rightIn = 3;
int leftIn = 2;

const double robBased = .195;
const double wheelDiam = .065;
const double distOneTurn = wheelDiam * 3.14;
const int countOneTurn = 20;

double countR;
double countL;

volatile long cntrL, cntrR;
volatile long tempCntrL, tempCntrR;
int modeMax = 5; // different modes car can be in
int mode = 0; // current mode of car
int velC = 0;

boolean pressed[] = {false,false}; // debounce;

void leftWhlCnt() {

    cntrL++;
}

void rightWhlCnt() {

    cntrR++;

}
long lastPrint;
void setup() {
  for(int i = 4; i < 10; i++) { 
     
     pinMode(i, OUTPUT);
  }
  Serial.begin(9600);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  lcd.init();
  attachInterrupt(digitalPinToInterrupt(leftIn), leftWhlCnt, RISING);
  attachInterrupt(digitalPinToInterrupt(rightIn), rightWhlCnt, RISING);
  forward();
  setRatio(60*3.14/180, 1.2);
 
}

void stopW(){
  digitalWrite(leftB, LOW);
  digitalWrite(rightB, LOW);
  digitalWrite(rightF, LOW);
  digitalWrite(leftF, LOW);
  analogWrite(enR, 0);
  analogWrite(enL, 0);
}

void forward() {
  digitalWrite(leftB, LOW);
  digitalWrite(rightB, LOW);
  digitalWrite(rightF, HIGH);
  digitalWrite(leftF, HIGH);  
}

void backward() {
  digitalWrite(leftB, HIGH);
  digitalWrite(rightB, HIGH);
  digitalWrite(rightF, LOW);
  digitalWrite(leftF, LOW);
}

void rightOnly() {
  digitalWrite(leftB, LOW);
  digitalWrite(rightB, LOW);
  digitalWrite(rightF, HIGH);
  digitalWrite(leftF, LOW);
}

void leftOnly() {
  digitalWrite(leftB, LOW);
  digitalWrite(rightB, LOW);
  digitalWrite(rightF, LOW);
  digitalWrite(leftF, HIGH);
}

void setVel(int rSpeed, int lSpeed){
  
  analogWrite(enR, 155 + rSpeed);
  analogWrite(enL, 155 + lSpeed);
}

void setRatio(double theta, double radius) {
  
  countL = (int)abs((radius - robBased / 2) * theta / distOneTurn * countOneTurn);
  countR = (int)abs((radius + robBased / 2) * theta / distOneTurn * countOneTurn);
  Serial.println(countL);
  Serial.println(countR);
  if(countR > countL) {
    
    setVel((countL * 100) / countR, 100);
    
  } else if(countR < countL) {
    setVel(100, (countL * 100) / countR);
  } else {
    setVel(100,100);
  }
}

boolean set = false;

void loop() {
  lcd.clear(); 
  if(!set)  {
    tempCntrL = cntrL;
    tempCntrR = cntrR;
    set = true;
  }
  
  #if 1
  Serial.print("right: ");
  Serial.println((cntrR - tempCntrR) / countR);
  Serial.print("left: ");
  Serial.println((cntrL - tempCntrL) / countL);
  
    lcd.println((cntrR - tempCntrR) / countR);
    lcd.println((cntrL - tempCntrL) / countL);
    lastPrint = millis();
  
  if((cntrR - tempCntrR) / countR > (cntrL - tempCntrL) / countL) {
     leftOnly();
  } else if((cntrR - tempCntrR) / countR < (cntrL - tempCntrL) / countL) {
     rightOnly();
  } else {
     forward();
  }
  #endif
  
}
