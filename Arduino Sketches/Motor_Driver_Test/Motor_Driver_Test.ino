#include <Encoder.h>
#include <PID_v1.h>

double Input;
double Setpoint, Output;

int Output1, Output2;

double Kp = 0.1, Ki = 1, Kd = 0;

#define Encoder_A 2
#define Encoder_B 3
#define POT A0
#define PWM_A 5
#define PWM_B 6

Encoder myEnc(Encoder_A, Encoder_B);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode (POT, INPUT);
  pinMode (PWM_A, OUTPUT);
  pinMode (PWM_B, OUTPUT);
  // no need to setup encoder a and b, they will be setup in the Encoder class


  myPID.SetOutputLimits(-255, 255);
  // turn on the pid
  myPID.SetMode(AUTOMATIC);

  Serial.begin(9600);
}

long oldPosition  = 0;
long newPosition = 0;

void loop() {
  newPosition = myEnc.read();
  Input = oldPosition - newPosition; // This will generate the difference in steps over the delay, this will be the velocity
  Setpoint = map(analogRead(POT), 0, 1023, -3000, 3000);
  myPID.Compute();

  if (Output > 0) {
    Output1 = 0;
    Output2 = (int) Output;
  }
  else {
    Output1 = (int) - Output;
    Output2 = 0;
  }


  analogWrite(PWM_A, Output1);
  analogWrite(PWM_B, Output2);

  Serial.println("" + String(Input) + "," + String(Output) + "," + String(Setpoint));
  oldPosition = newPosition;
  delay(500);
}
