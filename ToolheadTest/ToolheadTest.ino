#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <ezButton.h>

Servo axis;  // create servo object to control a servo

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
bool EndStopState = 0;
bool buttonState = 0; // Declare and initialize buttonState

// Data package for radio transmission
struct Data_Package {
  int t; // task type
  int d; // first value
  int e; // second value
  int f; // third value
};
Data_Package data; //Create a variable with the above structure

int type;

void setup() {
  axis.attach(4, 600, 2300);  // (pin, min, max)
  axis.write(0);  // tell servo to go to a particular angle
  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00001
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.setPALevel(RF24_PA_MIN);
}

void loop() {
  delay(5);
  radio.startListening();
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    // Action check endstop
    type = data.t;

    switch (type) {
      // Limitswitch
      case 11:
        // Add your code here for case 11
        break;
      // move 4th axis
      case 12:
        // Add your code here for case 12
        break;
    }

    delay(5);
    radio.stopListening();
    EndStopState = 0;
    radio.write(&EndStopState, sizeof(EndStopState));
  }

  axis.write(0);  // tell servo to go to a particular angle
  delay(2000);
  axis.write(45);  // tell servo to go to a particular angle
  delay(2000);
  axis.write(90);  // tell servo to go to a particular angle
  delay(2000);
}
