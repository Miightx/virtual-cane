// Define the vibrators
#define VIBRATOR_1 7
#define VIBRATOR_2 1

const int numVibrators = 2;


// Define the potentiometer
int pt = A1;

void setup() {
  // Start a serial communication between the Arduino board and the computer
  Serial.begin(115200);
  pinMode(VIBRATOR_1, OUTPUT);
  pinMode(VIBRATOR_2, OUTPUT);
}

void loop() {
  // Read the value of the potentiometer
  int potValue = analogRead(pt);

  // Send the value via the serial port
  Serial.println(potValue);

  if (Serial.available() > 0){
    int command = Serial.parseInt();
    if (command == 7){
      analogWrite(VIBRATOR_1, 120);  // Assuming you want to set it to maximum intensity
      analogWrite(VIBRATOR_2, 0);    // Turn off the other vibrator
    } 
    else if (command == 1){
      analogWrite(VIBRATOR_2, 50);  // Assuming you want to set it to maximum intensity
      analogWrite(VIBRATOR_1, 0);    // Turn off the other vibrator
    }
    else {
      analogWrite(VIBRATOR_1, 0);
      analogWrite(VIBRATOR_2, 0);
    }
  }
  // Wait for a short moment
  delay(200);
}
