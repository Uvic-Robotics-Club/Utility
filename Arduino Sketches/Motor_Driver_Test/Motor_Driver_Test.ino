#include <Encoder.h>
#include <PID_v1.h>

double Input;
double Setpoint, Output;

int Output1, Output2;
String inputString = "";
auto mode = AUTOMATIC;

int MotorNumber = 0;

double Kp = 1, Ki = 1, Kd = 0;

#define Encoder_A 2
#define Encoder_B 3
#define PWM_A 5
#define PWM_B 6
#define Addr1 9
#define Addr2 10
#define Addr3 11

Encoder myEnc(Encoder_A, Encoder_B);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode (PWM_A, OUTPUT);
  pinMode (PWM_B, OUTPUT);
  // no need to setup encoder a and b, they will be setup in the Encoder class
  pinMode (Addr1, INPUT);
  pinMode (Addr2, INPUT);
  pinMode (Addr3, INPUT);
  if (!digitalRead(Addr1) & !digitalRead(Addr2) & !digitalRead(Addr3)) {
    MotorNumber = 1;
  }
  if (digitalRead(Addr1) & !digitalRead(Addr2) & !digitalRead(Addr3)) {
    MotorNumber = 2;
  }
  if (!digitalRead(Addr1) & digitalRead(Addr2) & !digitalRead(Addr3)) {
    MotorNumber = 3;
  }
  if (digitalRead(Addr1) & digitalRead(Addr2) & !digitalRead(Addr3)) {
    MotorNumber = 4;
  }
  if (!digitalRead(Addr1) & !digitalRead(Addr2) & digitalRead(Addr3)) {
    MotorNumber = 5;
  }
  if (digitalRead(Addr1) & !digitalRead(Addr2) & digitalRead(Addr3)) {
    MotorNumber = 6;
  }
  if (!digitalRead(Addr1) & digitalRead(Addr2) & digitalRead(Addr3)) {
    MotorNumber = 7;
  }
  if (digitalRead(Addr1) & digitalRead(Addr2) & digitalRead(Addr3)) {
    MotorNumber = 8;
  }


  myPID.SetOutputLimits(-255, 255);
  // turn on the pid
  myPID.SetMode(mode);

  Serial.begin(9600);
  Serial.println("Motor:" + String(MotorNumber));
}

long oldPosition  = 0;
long newPosition = 0;
unsigned long thisTime = 0;
unsigned long lastTime = 0;
unsigned long printTime = 0;
void loop() {
  thisTime = millis();
  newPosition = myEnc.read();
  Input = (oldPosition - newPosition); // This will generate the difference in steps over the delay, this will be the velocity
  myPID.Compute();

  if (Output > 0) {
    Output1 = 0;
    Output2 = (int) Output;
  }
  else {
    Output1 = (int) - Output;
    Output2 = 0;
  }
  if (abs(Output)<50){
    Output1 = 0;
    Output2 = 0;
  }


  analogWrite(PWM_A, Output1);
  analogWrite(PWM_B, Output2);
  if (thisTime - printTime > 500) {
    Serial.println("" + String(Input) + "," + String(Output) + "," + String(Setpoint));
    printTime = thisTime;
  }
  oldPosition = newPosition;
  lastTime = thisTime;
  delay(50);
}

void serialEvent() {
  char first = (char)Serial.read();
  wholeThing();
  if (first == 's') {
    Setpoint = inputString.toInt();
  }
  if (first == 'd') {
    Kd = inputString.toInt();
    myPID.SetTunings(Kp, Ki, Kd);
  }
  if (first == 'i') {
    Ki = inputString.toInt();
    myPID.SetTunings(Kp, Ki, Kd);
  }
  if (first == 'p') {
    Kp = inputString.toInt();
    myPID.SetTunings(Kp, Ki, Kd);
  }
  if (first == 'k') {
    mode = (mode == AUTOMATIC) ? MANUAL : AUTOMATIC;
    myPID.SetMode(mode);
    Output1 = 0;
    Output2 = 0;
  }


}

void wholeThing() {
  inputString = "";
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
  }
}

