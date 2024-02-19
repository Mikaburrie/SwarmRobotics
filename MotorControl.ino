int leftF = 4; // in1
int leftB = 7; // in2
int rightF = 8; // in3
int rightB = 9; // in4
int enL = 5;
int enR = 6; // enB
int buttons[] = {12,11}; // enA

int modeMax = 5; // different modes car can be in
int mode = 0; // current mode of car
int velC = 0;
boolean pressed[] = {false,false}; // debounce;

void setup() {
  for(int i = 2; i < 10; i++) { 
     if(i == 2 || i == 3 || i == 12) pinMode(i, INPUT);
     else pinMode(i, OUTPUT);
  }
  Serial.begin(9600);
}

void loop() {
   
   if(digitalRead(buttons[0]) == HIGH && !pressed[0]) {
       pressed[0] = true;
       mode++;
       mode%=modeMax;
       delay(100);
   } else if(digitalRead(buttons[0]) == LOW) {
        pressed[0] = false;
   }
   if(digitalRead(buttons[1]) == HIGH && !pressed[1]) {
       pressed[1] = true;
       velC++;
       velC%=modeMax;
       delay(100);
   } else if(digitalRead(buttons[1]) == LOW) {
        pressed[1] = false;
   }
  
   switch(mode) {
    case 0: // Stop
        digitalWrite(leftB, LOW);
        digitalWrite(rightB, LOW);
        digitalWrite(rightF, LOW);
        digitalWrite(leftF, LOW);
    break;
    case 1: // For
        digitalWrite(leftB, LOW);
        digitalWrite(rightB, LOW);
        digitalWrite(rightF, HIGH);
        digitalWrite(leftF, HIGH);
    break;
    case 2: // Back
        digitalWrite(leftB, HIGH);
        digitalWrite(rightB, HIGH);
        digitalWrite(rightF, LOW);
        digitalWrite(leftF, LOW);
    break;
    case 3: // Right
        digitalWrite(leftB, LOW);
        digitalWrite(rightB, HIGH);
        digitalWrite(rightF, LOW);
        digitalWrite(leftF, HIGH);
    break;
    case 4: // Lfft
        digitalWrite(leftB, HIGH);
        digitalWrite(rightB, LOW);
        digitalWrite(rightF, HIGH);
        digitalWrite(leftF, LOW);
    break;
   }
 
   int vel =  1023 - ((1023 * velC) / (5));
   Serial.print(vel);
   Serial.print(" ");
   Serial.println(mode);
   analogWrite(enR, vel);
   analogWrite(enL, vel);
 
 
   
}
