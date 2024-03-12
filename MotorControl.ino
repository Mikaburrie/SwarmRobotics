int leftF = 7; // in1
int leftB = 4; // in2
int rightF = 9; // in3
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

void setup() {
  for(int i = 4; i < 10; i++) { 
     
     pinMode(i, OUTPUT);
  }
  Serial.begin(9600);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(leftIn), leftWhlCnt, RISING);
  attachInterrupt(digitalPinToInterrupt(rightIn), rightWhlCnt, RISING);
  
  spinTest(60*3.14/180,36 * 0.0254);
  spinTest(-45*3.14/180,25 * 0.0254);
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

void rightSpin() {
  digitalWrite(leftB, LOW);
  digitalWrite(rightB, HIGH);
  digitalWrite(rightF, LOW);
  digitalWrite(leftF, HIGH);
}

void leftSpin() {
  digitalWrite(leftB, HIGH);
  digitalWrite(rightB, LOW);
  digitalWrite(rightF, HIGH);
  digitalWrite(leftF, LOW);
}

void setVel(int rSpeed, int lSpeed){
  analogWrite(enR, rSpeed);
  analogWrite(enL, lSpeed);
}

void spinTest(double theta, double radius) {
  tempCntrR = cntrR;
  tempCntrL = cntrL;
  double countL = abs((radius - robBased / 2) * theta / distOneTurn * countOneTurn);
  double countR = abs((radius + robBased / 2) * theta / distOneTurn * countOneTurn);
  Serial.println(countR);
  Serial.println(countL);
  while(cntrR <= tempCntrR + countR && cntrL <= tempCntrL + countL) {
       setVel(255,255);
       forward();
       
  }
  stopW();
}
void loop() {
   
   
}
