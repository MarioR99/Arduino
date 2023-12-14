// Library for IR Remote
#include <IRremote.h>

// Define IR Remote Pin
int RECV_PIN = 3;
IRrecv irrecv(RECV_PIN);
decode_results results;

//Beacon Input Pins: 1 is Off(no beacon sensed) & 0 is Sensing
int inPinN = 7;
int inPinS = 6;iejwgbierw
int inPinE = 5;
int inPinW = 4;

// Define IR Remote Values
#define A 0x00FF30CF  // (Button "1" (Auto Control))
#define Forward 0x00FF629D
#define Backward 0x00FFA857
#define LEFT 0x00FF22DD
#define RIGHT 0x00FFC23D

// Define Motor Driver Pins
//Right Side
#define PWM1 8  //1 is right while facing direction of forward motion
#define AIN1 10 //1 is right while facing direction of forward motion
#define AIN2 11 //1 is right while facing direction of forward motion
// Left Side
#define PWM2 9  //2 is left while facing direction of forward motion
#define BIN1 13 //2 is left while facing direction of forward motion
#define BIN2 12 //2 is left while facing direction of forward motion

// Initiate Setup
 void setup(){

  // Start the receiver
  Serial.begin(9600);
  irrecv.enableIRIn(); 
   
  // Define Pin Modes
  pinMode(PWM1,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);

  pinMode(inPinN, INPUT);
  pinMode(inPinS, INPUT);
  pinMode(inPinE, INPUT);
  pinMode(inPinW, INPUT);

}

// Intitiate Loop for IR Control
 void loop()
 {
   if (irrecv.decode(&results))
   {
    irrecv.resume(); // get next value

     if (results.value == Forward){
       Serial.println("Forward");
        forward();
        delay(1000);
        stopMotors();
     }

     else if (results.value == Backward){
       Serial.println("Backward");
        backward();
        delay(1000);
        stopMotors();
     }

     else if (results.value == LEFT){
       Serial.println("LEFT");
        left();
        delay(500);
        stopMotors();
     }

     else if (results.value == RIGHT){
       Serial.println("RIGHT");
        right();
        delay(500);
        stopMotors();
     }

     else if (results.value == A){

      Serial.println("Autonomous Control");
      wallFollowing();

     }

    //  else {
    //    Serial.println("IR RECV Code Value Not Defined or Button was Held Down");
    //  }
    // Print Values to Screen
    // Serial.print("IR RECV Code = 0x ");
    // Serial.println(results.value, HEX);
    // delay(100);


  }
}


void forward() {
  analogWrite(PWM1,255);
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  analogWrite(PWM2,255);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
}

void backward() {
  analogWrite(PWM1,255);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  analogWrite(PWM2,255);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,HIGH);
}

void left() {
  analogWrite(PWM1,255);
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  analogWrite(PWM2,255);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,HIGH);
}

void right() {
  analogWrite(PWM1,255);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  analogWrite(PWM2,255);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
}

void stopMotors() {
  analogWrite(PWM1,0);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,LOW);
  analogWrite(PWM2,0);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,LOW);
}

//======================================================================================================
//  wallFollowing
//======================================================================================================
void wallFollowing(){

  // Obtain Distance From Wall Following Sensor
  float sensorValue1 = analogRead(A0); 
  float voltValue1 = sensorValue1 * (5.0 / 1023);
  float distance1 = 22.669 / (voltValue1-0.003);
  Serial.print(distance1);
  Serial.print("\t wallFollowing!!!!!!!!! \n");

  // Too Close! Left Correction
  if (distance1 < 20.0 || distance1 == 20.0){
  // Motor Control
  analogWrite(PWM1,200);
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  analogWrite(PWM2,100);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
  float sensorValue1 = analogRead(A0); 
  float voltValue1 = sensorValue1 * (5.0 / 1023);
  float distance1 = 22.669 / (voltValue1-0.003);
  delay(100); //500 original
  Serial.print(distance1);
  Serial.print("\t wallFollowingLEFT \n");
  wallFollowing();
  }

  // Too Far! Right Corrections
  else if (distance1 > 20.0 && distance1 < 30.0){
  // Motor Control
  analogWrite(PWM1,100);
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  analogWrite(PWM2,200);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
  float sensorValue1 = analogRead(A0); 
  float voltValue1 = sensorValue1 * (5.0 / 1023);
  float distance1 = 22.669 / (voltValue1-0.003);
  delay(100); //700 original
  Serial.print(distance1);
  Serial.print("\t wallFollowingRight \n");
  wallFollowing();
  }

  else{
    beaconFollowing();
  }

}

void beaconFollowing(){

Serial.print("beaconFollowing!!!!!!!!!!!! \n");
  //PhotoresistorCode!!!!!!!!!!!!!!
  // {
  //
  // }

      //=================================================
      //  DODGE OBSTACLES: Autonomous Control Code
      //=================================================

Serial.print("BoogyMan1 \n");

    // Read the input on analog pins:
    float sensorValue2 = analogRead(A1); // Closest to Wall Follower
    float sensorValue3 = analogRead(A2); // Middle
    float sensorValue4 = analogRead(A3); // Farthest from Wall Follower

    // Transform analogValue into voltage:
    float voltValue2 = sensorValue2 * (5.0 / 1023);
    float voltValue3 = sensorValue3 * (5.0 / 1023);
    float voltValue4 = sensorValue4 * (5.0 / 1023);

    // Convert Voltage to Distance
    float distance2 = 22.669 / (voltValue2-0.003);
    float distance3 = 22.669 / (voltValue3-0.003);
    float distance4 = 22.669 / (voltValue4-0.003);

     Serial.print("Sensor2: \n");
     Serial.println(distance2);
     Serial.print("\n");
     Serial.print("Sensor3: \n");
     Serial.println(distance3);
     Serial.print("\n");
     Serial.print("Sensor4: \n");
     Serial.println(distance4);
     Serial.print("\n");

Serial.print("BoogyMan2 \n");

        if (distance2 < 22.0 || distance3 < 22.0){
Serial.print("BoogyMan3 \n");
          //stop
          analogWrite(PWM1,0);
          digitalWrite(AIN1,LOW);
          digitalWrite(AIN2,LOW);
          analogWrite(PWM2,0);
          digitalWrite(BIN1,LOW);
          digitalWrite(BIN2,LOW);
          delay(1000);
          //turn left
          analogWrite(PWM1,255);
          digitalWrite(AIN1,HIGH);
          digitalWrite(AIN2,LOW);
          analogWrite(PWM2,255);
          digitalWrite(BIN1,LOW);
          digitalWrite(BIN2,HIGH);
          delay(1500);
          //go straight
          analogWrite(PWM1,255);
          digitalWrite(AIN1,HIGH);
          digitalWrite(AIN2,LOW);
          analogWrite(PWM2,255);
          digitalWrite(BIN1,HIGH);
          digitalWrite(BIN2,LOW);
          delay(1500);
          Serial.print("BoogyMan4 \n");
          beaconFollowing();
        }
        if (distance4 < 22.0){
          //stop
Serial.print("BoogyMan5 \n");
          analogWrite(PWM1,0);
          digitalWrite(AIN1,LOW);
          digitalWrite(AIN2,LOW);
          analogWrite(PWM2,0);
          digitalWrite(BIN1,LOW);
          digitalWrite(BIN2,LOW);
          delay(1000);
          //turn right
          analogWrite(PWM1,255);
          digitalWrite(AIN1,LOW);
          digitalWrite(AIN2,HIGH);
          analogWrite(PWM2,255);
          digitalWrite(BIN1,HIGH);
          digitalWrite(BIN2,LOW);
          delay(1500);
          //go straight
          analogWrite(PWM1,255);
          digitalWrite(AIN1,HIGH);
          digitalWrite(AIN2,LOW);
          analogWrite(PWM2,255);
          digitalWrite(BIN1,HIGH);
          digitalWrite(BIN2,LOW);
          delay(1500);
          Serial.print("BoogyMan6 \n");
          beaconFollowing();          
        }
        //=================================================
        //  BEACON MODE
        //=================================================

              //Beacon Input Pins: 1 is Off(no beacon sensed) & 0 is Sensing
              // int inPinN = 7;
              // int inPinS = 6;
              // int inPinE = 5;
              // int inPinW = 4;
              Serial.print("BoogyMan7 \n");
      while ( distance2 > 30.0 && distance3 > 30.0 && distance4 > 30.0 ){
Serial.print("BoogyMan8 \n");
        float sensorValue2 = analogRead(A1); // Closest to Wall Follower
        float sensorValue3 = analogRead(A2); // Middle
        float sensorValue4 = analogRead(A3); // Farthest from Wall Follower

        // Transform analogValue into voltage:
        float voltValue2 = sensorValue2 * (5.0 / 1023);
        float voltValue3 = sensorValue3 * (5.0 / 1023);
        float voltValue4 = sensorValue4 * (5.0 / 1023);

        // Convert Voltage to Distance
        float distance2 = 22.669 / (voltValue2-0.003);
        float distance3 = 22.669 / (voltValue3-0.003);
        float distance4 = 22.669 / (voltValue4-0.003);

          Serial.print("Sensor2: \n");
          Serial.println(distance2);
          Serial.print("\n");
          Serial.print("Sensor3: \n");
          Serial.println(distance3);
          Serial.print("\n");
          Serial.print("Sensor4: \n");
          Serial.println(distance4);
          Serial.print("\n");


          //BeaconSense
          boolean NVal= digitalRead(inPinN);
          boolean SVal= digitalRead(inPinS);
          boolean EVal= digitalRead(inPinW);
          boolean WVal= digitalRead(inPinE);

          Serial.println(NVal);
          Serial.print("\n");
          Serial.println(SVal);
          Serial.print("\n");
          Serial.println(EVal);
          Serial.print("\n");
          Serial.println(WVal);
          Serial.print("\n");

          Serial.print("BoogyMan9 \n");

          if (NVal == 0 && SVal == 1 && EVal == 1 && WVal == 1){
            Serial.print("BoogyMan10 \n");
            //Forward Slow
            analogWrite(PWM1,125);
            digitalWrite(AIN1,HIGH);
            digitalWrite(AIN2,LOW);
            analogWrite(PWM2,125);
            digitalWrite(BIN1,HIGH);
            digitalWrite(BIN2,LOW);
            delay(750);
            Serial.print("BoogyMan11 \n");
            beaconFollowing();
          }
          if (NVal == HIGH && SVal == LOW && EVal == HIGH && WVal == HIGH){
            Serial.print("BoogyMan12 \n");
            //Flip a 180 (turn right!)
            analogWrite(PWM1,255);
            digitalWrite(AIN1,LOW);
            digitalWrite(AIN2,HIGH);
            analogWrite(PWM2,255);
            digitalWrite(BIN1,HIGH);
            digitalWrite(BIN2,LOW);
            delay(750);
            Serial.print("BoogyMan13 \n");
            beaconFollowing();
          }
          if (NVal == 1 && SVal == 1 && EVal == 0 && WVal == 1){
            Serial.print("BoogyMan14 \n");
            //Right turn
            analogWrite(PWM1,255);
            digitalWrite(AIN1,LOW);
            digitalWrite(AIN2,HIGH);
            analogWrite(PWM2,255);
            digitalWrite(BIN1,HIGH);
            digitalWrite(BIN2,LOW);
            delay(750);
            Serial.print("BoogyMan15 \n");
            beaconFollowing();
          }
          if (NVal == 1 && SVal == 1 && EVal == 1 && WVal == 0){
            Serial.print("BoogyMan16 \n");
            //Left turn
            analogWrite(PWM1,255);
            digitalWrite(AIN1,HIGH);
            digitalWrite(AIN2,LOW);
            analogWrite(PWM2,255);
            digitalWrite(BIN1,LOW);
            digitalWrite(BIN2,HIGH);
            delay(750);
            Serial.print("BoogyMan17 \n");
            beaconFollowing();
          }
          if (NVal == 0 && SVal == 1 && EVal == 0 && WVal == 1){
            Serial.print("BoogyMan18 \n");
            //Slight Right
            analogWrite(PWM1,75);
            digitalWrite(AIN1,LOW);
            digitalWrite(AIN2,HIGH);
            analogWrite(PWM2,125);
            digitalWrite(BIN1,HIGH);
            digitalWrite(BIN2,LOW);
            delay(750);
            Serial.print("BoogyMan19 \n");
            beaconFollowing();
          }
          if (NVal == 0 && SVal == 1 && EVal == 1 && WVal == 0){
            Serial.print("BoogyMan20 \n");
            //Slight Left
            analogWrite(PWM1,125);
            digitalWrite(AIN1,LOW);
            digitalWrite(AIN2,HIGH);
            analogWrite(PWM2,75);
            digitalWrite(BIN1,HIGH);
            digitalWrite(BIN2,LOW);
            delay(750);
            Serial.print("BoogyMan21 \n");
            beaconFollowing();
          }
          if (NVal == 1 && SVal == 0 && EVal == 0 && WVal == 1){
            Serial.print("BoogyMan22 \n");
            //Right turn
            analogWrite(PWM1,255);
            digitalWrite(AIN1,LOW);
            digitalWrite(AIN2,HIGH);
            analogWrite(PWM2,255);
            digitalWrite(BIN1,HIGH);
            digitalWrite(BIN2,LOW);
            delay(750);
            Serial.print("BoogyMan23 \n");
            beaconFollowing();
          }
          if (NVal == 1 && SVal == 0 && EVal == 1 && WVal == 0){
            Serial.print("BoogyMan24 \n");
            //Left turn
            analogWrite(PWM1,255);
            digitalWrite(AIN1,HIGH);
            digitalWrite(AIN2,LOW);
            analogWrite(PWM2,255);
            digitalWrite(BIN1,LOW);
            digitalWrite(BIN2,HIGH);
            delay(750);
            Serial.print("BoogyMan25 \n");
            beaconFollowing();
          }
          else {
            Serial.print("NVal: \n");
            Serial.println(NVal);
            Serial.print("\n");

            Serial.print("SVal: \n");
            Serial.println(SVal);
            Serial.print("\n");

            Serial.print("EVal: \n");
            Serial.println(EVal);
            Serial.print("\n");

            Serial.print("WVal: \n");
            Serial.println(WVal);
            Serial.print("\n");


            Serial.print("BoogyMan26 \n");
            //Forward slow
            analogWrite(PWM1,125);
            digitalWrite(AIN1,HIGH);
            digitalWrite(AIN2,LOW);
            analogWrite(PWM2,125);
            digitalWrite(BIN1,HIGH);
            digitalWrite(BIN2,LOW);
            delay(750);

            Serial.print("NVal: \n");
            Serial.println(NVal);
            Serial.print("\n");

            Serial.print("SVal: \n");
            Serial.println(SVal);
            Serial.print("\n");

            Serial.print("EVal: \n");
            Serial.println(EVal);
            Serial.print("\n");

            Serial.print("WVal: \n");
            Serial.println(WVal);
            Serial.print("\n");




            Serial.print("BoogyMan27 \n");
            beaconFollowing();
          }
        }

  }

