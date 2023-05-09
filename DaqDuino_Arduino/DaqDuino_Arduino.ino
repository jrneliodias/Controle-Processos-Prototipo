// DAQ-Duino, 2013-2016
// Author: Prof. Dr. Antonio Silveira (asilveira@ufpa.br)
// Laboratory of Control and Systems (LACOS), UFPA (www.ufpa.br)

float u[1] = {0.0};
float y[1] = {0.0};

#define DA6 (9) // PWM Pin 9

void setup() {
  pinMode(DA6, OUTPUT);
  digitalWrite(DA6, LOW);
  Serial.begin(115200); // Sets the data rate in bits per second (baud) for serial data transmission.
                       // For communicating with the computer, use one of these rates:
                       // 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200
  Serial.setTimeout(5); // Maximum milisecs to wait for
                        // Serial.parseFloat to timeout
}

void loop() {
  if (Serial.available() > 0) {
      u[0] = Serial.parseFloat();
             if (u[0] < 0) {u[0] = 0;}
             if (u[0] > 5) {u[0] = 5;}
      u[0] = u[0]*(255/5.0);
      analogWrite(DA6,u[0]);
    
      y[0] = analogRead(A0)*(5/1023.0); // Pin A0 as Input
             if (y[0] < 0) {y[0] = 0;}
             if (y[0] > 5) {y[0] = 5;}
    Serial.print(y[0]);
  }
}
