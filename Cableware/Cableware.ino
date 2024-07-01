#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>

#include <SPI.h>
#include <RF24.h>

#define NUM_STEPPERS 3; // Number of stepper motors

// Define Radio transmission
RF24 radio(7, 8); // CE, CSN pins
const byte addresses[][6] = {"0001", "0002"};
int startCommand = 0; // Endschalter initialisieren
int stopCommand = 1;
bool startCommandSent = false;

// Data package for radio transmission
struct Data_Package {
  int t; // task type
  int d; // first value
  int e; // second value
  int f; // third value
};
Data_Package data; //Create a variable with the above structure

// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, 2, 3); //  [A] (Typeof driver: with 2 pins, STEP, DIR)
AccelStepper stepper2(1, 4, 5); //  [B] (Typeof driver: with 2 pins, STEP, DIR)
AccelStepper stepper3(1, 6, 9); // [C] (Typeof driver: with 2 pins, STEP, DIR)

AccelStepper steppers[] = {stepper1, stepper2, stepper3}; // Array of stepper motors

MultiStepper steppersControl;  // Create instance of MultiStepper

long gotoposition[3]; // An array to store the target positions for each stepper motor
long values[3];
long currentposition[3]; // An array that stores the current position (Steps)
long stepsTaken[3]; // An array that stores the steps taken by each stepper motor
int backinfo;

int currentMotor = 0;

// Definition der Robot Parameter
float U = 53.88; // circumference of the winch
float R = 9.5; // Radius of the pulley
float UR2 = 29.845; // 
int S = 1600; // Steps per revolution
float SU = S/U; // Steps per mm
float a;
float b;
float c;

// Robot coordinates in mm
float A_winch_x = 0;
float A_winch_y = 461.88;
float B_winch_x = -400;
float B_winch_y = -230.94;
float C_winch_x = 400;
float C_winch_y = -230.94;
float Z_winch= 296.5;
// float L_0 = 502.918; calculated externaly

// tool head definition in mm
float A_toolhead_x = 0;
float A_toolhead_y = 16;
float B_toolhead_x = -13.856;
float B_toolhead_y = -8;
float C_toolhead_x = 13.856;
float C_toolhead_y = -8;
// Z Offset of toolhead, distance from lower cabale point to TCP. TCP must be in the tool center.
float Z_toolhead = 38; // Toolplate
float Z_Endstop = 48; // Touch-Sensor
float Z_Gripper = 70; // Servo Gripper


float A_d;
float B_d;
float C_d;
// calculating Z , valid for all winches
float Z;
// calculating the length between tool_[] and pulley_[]
float A_Hyp;
float B_Hyp;
float C_Hyp;
// calculating the lengt of the pulley cable from touch point to most upper point on the pulley
float A_Alpha;
float B_Alpha;
float C_Alpha;
// calculating the delta of cable lenngth to L_0
float A_L;
float B_L;
float C_L;

  // initial calculation of the "home length" of the cablebot
  float d_0 = sqrt(pow((A_winch_x - (A_toolhead_x + 0)),2)+pow((A_winch_y - (A_toolhead_y + 0)),2))-R;
  float Z_0 = Z_winch - Z_toolhead - 0;
  float Hyp_0 = sqrt(pow(d_0,2) + pow(Z_0,2));
  float Alpha_0 = (1-((180 - ((atan2(Z_0, Hyp_0) * 180.0 / PI) + (atan2(R, Hyp_0) * 180.0 / PI)))/180)) * UR2 ;
  float L_0 = (sqrt(pow(Hyp_0,2) - pow(R,2)) + Alpha_0);


void setup() {
   // define steppers
  Serial.begin(9600); // Initialisierung der seriellen Kommunikation mit einer Baudrate von 9600 bps
  stepper1.setMaxSpeed(1000); // Set maximum speed value for the stepper
  stepper1.setSpeed(500); // Set the speed
  stepper1.setAcceleration(10); // Set acceleration value for the stepper
  stepper1.setCurrentPosition(0); // Set the current position to 0 steps
  // [B]
  stepper2.setMaxSpeed(1000); // Set maximum speed value for the stepper
  stepper2.setSpeed(500); // Set the speed
  stepper2.setAcceleration(10); // Set acceleration value for the stepper
  stepper2.setCurrentPosition(0); // Set the current position to 0 steps
  // [C]
  stepper3.setMaxSpeed(1000); // Set maximum speed value for the stepper
  stepper3.setSpeed(500); // Set the speed
  stepper3.setAcceleration(10); // Set acceleration value for the stepper
  stepper3.setCurrentPosition(0); // Set the current position to 0 steps

  // Adding the 3 steppers to the steppersControl instance for multi stepper control
  steppersControl.addStepper(stepper1);
  steppersControl.addStepper(stepper2);
  steppersControl.addStepper(stepper3);

  // Setup Radio
  pinMode(12, OUTPUT);
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setPALevel(RF24_PA_MIN);

}

void loop() {
   if (Serial.available()) { // Überprüfung, ob Daten vom Seriellen Port verfügbar sind
    String receivedData = Serial.readStringUntil('\n'); // Lesen der Zeichenfolge bis zum Zeilenumbruch ('\n')
        
    // schreibe das empfangene array in die variabelen 
    int type, V1, V2, V3; // Deklaration von Variablen zur Aufnahme der empfangenen Werte
    sscanf(receivedData.c_str(), "%d,%d,%d,%d", &type, &V1, &V2, &V3); // Extrahieren der Werte aus der Zeichenfolge
    values[0] = V1 ; // in mm
    values[1] = V2 ; // in mm
    values[2] = V3 ; // in mm
    
    switch (type) {
    case 1:
        // set home position to 0
         // define steppers
        stepper1.setMaxSpeed(1000); // Set maximum speed value for the stepper
        stepper1.setAcceleration(10); // Set acceleration value for the stepper
        stepper1.setCurrentPosition(0); // Set the current position to 0 steps
        // [B]
        stepper2.setMaxSpeed(1000); // Set maximum speed value for the stepper
        stepper2.setAcceleration(10); // Set acceleration value for the stepper
        stepper2.setCurrentPosition(0); // Set the current position to 0 steps
        // [C]
        stepper3.setMaxSpeed(1000); // Set maximum speed value for the stepper
        stepper3.setAcceleration(10); // Set acceleration value for the stepper
        stepper3.setCurrentPosition(0); // Set the current position to 0 steps

        backinfo = 1;
        Serial.print(backinfo); // Senden des ersten Elements des Arrays
       
      break;
    case 2:
      // Set Position to values from gotoposition
        gotoposition[0] = values[0];
        gotoposition[1] = values[1];
        gotoposition[2] = values[2];
        stepper1.setCurrentPosition(gotoposition[0]); // Set the current position to V1
        stepper1.setCurrentPosition(gotoposition[1]); // Set the current position to V2
        stepper1.setCurrentPosition(gotoposition[2]); // Set the current position to V3
        backinfo = 2;
        Serial.print(backinfo); // Senden des ersten Elements des Arrays
      break;
    case 3:
      // move absolute by stepps
        gotoposition[0] = values[0];
        gotoposition[1] = values[1];
        gotoposition[2] = values[2];
        steppersControl.moveTo(gotoposition); // Calculates the required speed for all motors
        steppersControl.runSpeedToPosition(); // Blocks until all steppers are in position
        backinfo = 3;
        Serial.print(backinfo); // Senden des ersten Elements des Arrays
      break;
    case 4:
      // move absolute by mm
        gotoposition[0] = (int)(values[0] * SU);
        gotoposition[1] = (int)(values[1] * SU);
        gotoposition[2] = (int)(values[2] * SU);
        
        steppersControl.moveTo(gotoposition); // Calculates the required speed for all motors
        steppersControl.runSpeedToPosition(); // Blocks until all steppers are in position
        backinfo = 4;
        Serial.print(backinfo); // Senden des ersten Elements des Arrays
      break;
    case 5:
      // move relativ by value just tool head
        // calculating d
        A_d = sqrt(pow((A_winch_x - (A_toolhead_x + values[0])),2)+pow((A_winch_y - (A_toolhead_y + values[1])),2))-R;
        B_d = sqrt(pow((B_winch_x - (B_toolhead_x + values[0])),2)+pow((B_winch_y - (B_toolhead_y + values[1])),2))-R;
        C_d = sqrt(pow((C_winch_x - (C_toolhead_x + values[0])),2)+pow((C_winch_y - (C_toolhead_y + values[1])),2))-R;
        // calculating Z , valid for all winches
        Z = Z_winch - Z_toolhead - values[2];
        // calculating the length between tool_[] and pulley_[]
        A_Hyp = sqrt(pow(A_d,2) + pow(Z,2));
        B_Hyp = sqrt(pow(B_d,2) + pow(Z,2));
        C_Hyp = sqrt(pow(C_d,2) + pow(Z,2));
        // calculating the lengt of the pulley cable from touch point to most upper point on the pulley
        A_Alpha = (1-((180 - ((atan2(Z, A_Hyp) * 180.0 / PI) + (atan2(R, A_Hyp) * 180.0 / PI)))/180)) * UR2 ;
        B_Alpha = (1-((180 - ((atan2(Z, B_Hyp) * 180.0 / PI) + (atan2(R, B_Hyp) * 180.0 / PI)))/180)) * UR2 ;
        C_Alpha = (1-((180 - ((atan2(Z, C_Hyp) * 180.0 / PI) + (atan2(R, C_Hyp) * 180.0 / PI)))/180)) * UR2 ;
        // calculating the delta of cable lenngth to L_0
        A_L = (sqrt(pow(A_Hyp,2) - pow(R,2)) + A_Alpha) - L_0;
        B_L = (sqrt(pow(B_Hyp,2) - pow(R,2)) + B_Alpha) - L_0;
        C_L = (sqrt(pow(C_Hyp,2) - pow(R,2)) + C_Alpha) - L_0;
        // calculating 
        gotoposition[0] = (int)(A_L * SU*-1);
        gotoposition[1] = (int)(B_L * SU*-1);
        gotoposition[2] = (int)(C_L * SU*-1);


        steppersControl.moveTo(gotoposition); // Calculates the required speed for all motors
        steppersControl.runSpeedToPosition(); // Blocks until all steppers are in position

        backinfo = 5;
        Serial.print(backinfo); // Senden des ersten Elements des Arrays
      break;
    case 6:
      // Frei
      break;
    case 7:
      // Frei
      break;
    case 8:
      // Frei
      break;

    case 9:
        A_d = sqrt(pow((A_winch_x - (A_toolhead_x + values[0])),2)+pow((A_winch_y - (A_toolhead_y + values[1])),2))-R;
        B_d = sqrt(pow((B_winch_x - (B_toolhead_x + values[0])),2)+pow((B_winch_y - (B_toolhead_y + values[1])),2))-R;
        C_d = sqrt(pow((C_winch_x - (C_toolhead_x + values[0])),2)+pow((C_winch_y - (C_toolhead_y + values[1])),2))-R;
        // calculating Z , valid for all winches
        Z = Z_winch - Z_toolhead - values[2];
        // calculating the length between tool_[] and pulley_[]
        A_Hyp = sqrt(pow(A_d,2) + pow(Z,2));
        B_Hyp = sqrt(pow(B_d,2) + pow(Z,2));
        C_Hyp = sqrt(pow(C_d,2) + pow(Z,2));
        // calculating the lengt of the pulley cable from touch point to most upper point on the pulley
        A_Alpha = (1-((180 - ((atan2(Z, A_Hyp) * 180.0 / PI) + (atan2(R, A_Hyp) * 180.0 / PI)))/180)) * UR2 ;
        B_Alpha = (1-((180 - ((atan2(Z, B_Hyp) * 180.0 / PI) + (atan2(R, B_Hyp) * 180.0 / PI)))/180)) * UR2 ;
        C_Alpha = (1-((180 - ((atan2(Z, C_Hyp) * 180.0 / PI) + (atan2(R, C_Hyp) * 180.0 / PI)))/180)) * UR2 ;
        // calculating the delta of cable lenngth to L_0
        A_L = (sqrt(pow(A_Hyp,2) - pow(R,2)) + A_Alpha) - L_0;
        B_L = (sqrt(pow(B_Hyp,2) - pow(R,2)) + B_Alpha) - L_0;
        C_L = (sqrt(pow(C_Hyp,2) - pow(R,2)) + C_Alpha) - L_0;

        gotoposition[0] = (int)(A_L * SU*-1);
        gotoposition[1] = (int)(B_L * SU*-1);
        gotoposition[2] = (int)(C_L * SU*-1);

        Serial.print( gotoposition[0]); // Senden des ersten Elements des Arrays
        Serial.print(';');
        Serial.print( gotoposition[1]); // Senden des ersten Elements des Arrays
        Serial.print(';');
        Serial.print( gotoposition[2]); // Senden des ersten Elements des Arrays
        Serial.print(';');

      break;
// TOOL Cases
    case 11:
      // Homing Sequence
      // go through all steppers
      while (currentMotor < NUM_STEPPERS) {
        // if the start command is not sent yet, send it
        if (startCommandSent == false) {
          currentposition[currentMotor] = steppers[currentMotor].currentPosition();
          radio.stopListening();
          data.t = 11;
          data.d = startCommand;
          radio.write(&data, sizeof(Data_Package));
          radio.startListening();
          steppers[currentMotor].runSpeed();
          startCommandSent = true;
        } else {
          // if the start command is sent, check for the stop command
          if (radio.available()) {
            radio.read(&data, sizeof(Data_Package));
            // if the stop command is received, stop the stepper motor
            if (data.d == stopCommand) {
              // Stop the stepper motor
              steppers[currentMotor].stop();
              // Calculate steps taken for the current stepper motor
              stepsTaken[currentMotor] = steppers[currentMotor].currentPosition() - currentposition[currentMotor];
              // move back to the initial position
              steppers[currentMotor].moveTo(currentposition[currentMotor]);
              steppers[currentMotor].setSpeed(500);
              // Reset the startCommandSent flag to restart the loop for the next stepper motor
              startCommandSent = false;
              // Move to the next stepper motor
              currentMotor++;
            }
          } else {
            // if the stop command is not received, keep running the stepper motor
            steppers[currentMotor].runSpeed();
          }
        }
      } 
      // if all stepper motors are homed, print the steps taken by each stepper motor
      Serial.println("Homing Sequence completed");
      Serial.println("Steps taken by each stepper motor:");
      for (int i = 0; i < NUM_STEPPERS; i++) {
        Serial.print("Stepper ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(stepsTaken[i]);
      }
      break;
        // Rücksetzen auf 0 sollte wegfallen, da Homing nur einmalig ausgeführt wird
        // currentMotor = 0;
      }
    default:
      Serial.println("Ungültige Option ausgewählt");
      break;
  }
    
    
  } 
}
