#include <SPI.h>
#include <RF24.h>
#include <ezButton.h>

ezButton limitSwitch(4); // Change this to the pin where your limit switch is connected

RF24 radio(7, 8); // CE, CSN
bool buttonPressed = false; //boolean to send the Information whether the button is pressed
const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  limitSwitch.setDebounceTime(50);
}

void loop() {

  limitSwitch.loop();

  int state = limitSwitch.getState();
  if(state == LOW){
    bool buttonPressed = true;
    radio.write(&buttonPressed, sizeof(buttonPressed)); 
  }  else {
    bool buttonPressed = false;
    radio.write(&buttonPressed, sizeof(buttonPressed));
  }

  /*if(limitSwitch.isPressed()){
    const char text[] = "Knopf gedrückt";
    radio.write(&text, sizeof(text)); 
    Serial.println("The limit switch: UNTOUCHED -> TOUCHED");
  }*/
  /*if(limitSwitch.isReleased()){
    const char text[] = "Knopf losgelassen";
    radio.write(&text, sizeof(text));
    Serial.println("The limit switch: TOUCHED -> UNTOUCHED");
    }*/

}