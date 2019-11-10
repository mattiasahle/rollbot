//Author of normalization and calibration functions: Gudny Björk Odinsdottir
//Everything else: Mattias Ahle
//Kristianstad University 2019


//INDATA OPERATOR
//Number of laps
int numOfLaps = 2;


//Define the LED pins on the robot, used to control the calibration:
#define   LEFT_LED      3
#define   RIGHT_LED     2

//Global variables (arrays) used for the Normalization:
char sensors[5] = {A0, A1, A2, A3, A7}; /*Used to be able to loop through the 5 different sensors*/
int sensorvalues[5];    /*Used to store the raw sensor values*/
int normalizedsensorvalues[5];    /*Used to store the normalized sensor values*/
int maxsensorvalues[5] = {0, 0, 0, 0, 0};   /*Used to find the maximum value (over white surface) for each sensor*/
int minsensorvalues[5] = {1023, 1023, 1023, 1023, 1023};    /*Used to find the minimum value (over black surface) for each sensor*/
//int maxsensorvalues[5] = {600, 611, 659, 651, 569};   /*Used to find the maximum value (over white surface) for each sensor*/
//int minsensorvalues[5] = {68, 63, 91, 100, 68};    /*Used to find the minimum value (over black surface) for each sensor*/

//Pins used to control the motors:
int Left_Motor_Direction = 4;
int Right_Motor_Direction = 7;
int Left_Motor_Speed = 5;
int Right_Motor_Speed = 6;

boolean previousAllBlack = false; /* Used in lapCounter. Stores if the last sensor state was all black or not. */
int lapCount = 0;
float error = 0;
float P = 0;
float D = 0;
float PDValue = 0;
float previousError = 0;
int threshold = 512;

//Performance settings
int baseSpeed = 255;
float Kp = 0.3;
float Kd = 4;



void setup() {

  for (int i = 0; i < 5; i++) {
    pinMode(sensors[i], INPUT);
  }

  pinMode(LEFT_LED, OUTPUT);
  pinMode(RIGHT_LED, OUTPUT);

  pinMode(Left_Motor_Direction, OUTPUT);
  pinMode(Right_Motor_Direction, OUTPUT);
  pinMode(Left_Motor_Speed, OUTPUT);
  pinMode(Right_Motor_Speed, OUTPUT);

  digitalWrite(Left_Motor_Direction, LOW);
  digitalWrite(Right_Motor_Direction, HIGH);

  sensorCalibration();

  while (digitalRead(10) == HIGH) { /*Wait Until Button is Pressed*/
    digitalWrite(RIGHT_LED, HIGH);
  }
  delay(500);
  digitalWrite(RIGHT_LED, LOW);

  Serial.begin(9600);

}



void loop() {

  readSensors();

  calcPD();

  setMotorSpeeds();

  lapCounter();

}



void readSensors() {

  //Read raw sensor values
  for (int i = 0; i < 5; i++) {
    sensorvalues[i] = analogRead(sensors[i]);
  }

  normalizeSensorValues();
  //      for (int i = 0; i < 5; i++) {
  //        Serial.print(normalizedsensorvalues[i]);
  //        Serial.print("\t");
  //      }
  //      Serial.println();

}




void calcPD() {
  error = -normalizedsensorvalues[0] + -normalizedsensorvalues[1] + normalizedsensorvalues[3] + normalizedsensorvalues[4];
  //  Serial.print(-normalizedsensorvalues[0]);
  //  Serial.print("\t");
  //  Serial.print(-normalizedsensorvalues[1]);
  //  Serial.print("\t");
  //  Serial.print("|");
  //  Serial.print("\t");
  //  Serial.print(normalizedsensorvalues[3]);
  //  Serial.print("\t");
  //  Serial.print(normalizedsensorvalues[4]);
  //  Serial.print("\t=\t");
  //  Serial.println(error);

  P = error; //Proportional
  D = error - previousError; //Derivative

  PDValue = (Kp * P) + (Kd * D);

  previousError = error;
}



void setMotorSpeeds () {
  int leftMotorSpeed = baseSpeed - PDValue;
  int rightMotorSpeed = baseSpeed + PDValue;

  leftMotorSpeed = constrain(leftMotorSpeed, 0, baseSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, baseSpeed);

  //  Serial.print(leftMotorSpeed);
  //  Serial.print("\t");
  //  Serial.println(rightMotorSpeed);

  analogWrite(Left_Motor_Speed, leftMotorSpeed);
  analogWrite(Right_Motor_Speed, rightMotorSpeed);
}



void lapCounter() {
  if ((normalizedsensorvalues[0] > threshold ||
       normalizedsensorvalues[1] > threshold ||
       normalizedsensorvalues[2] > threshold ||
       normalizedsensorvalues[3] > threshold ||
       normalizedsensorvalues[4] > threshold) &&
      previousAllBlack == true) {
    lapCount++;
    //Serial.println(lapCount);
  }

  if (numOfLaps == lapCount) {
    analogWrite(Left_Motor_Speed, 0);
    analogWrite(Right_Motor_Speed, 0);
    digitalWrite(RIGHT_LED, HIGH);
    while (digitalRead(10) == HIGH) {
    }
    lapCount = 0;
    digitalWrite(RIGHT_LED, LOW);
    delay(500);
  }

  if (normalizedsensorvalues[0] < threshold &&
      normalizedsensorvalues[1] < threshold &&
      normalizedsensorvalues[2] < threshold &&
      normalizedsensorvalues[3] < threshold &&
      normalizedsensorvalues[4] < threshold) {
    previousAllBlack = true;
    digitalWrite(RIGHT_LED, HIGH);
  } else {
    previousAllBlack = false;
    digitalWrite(RIGHT_LED, LOW);
  }
}



//Calibration Function Used To Find Out Max- And Min Reading Value For Each Sensor:
void sensorCalibration() {
  Serial.begin(9600);
  Serial.println("Starting The Calibration!");
  Serial.println("Place the Robot On The Black Line, with sensor A2 on the Black Line (00X00)");
  Serial.println("Press Start Button To Start To Calibrate");
  digitalWrite(LEFT_LED, HIGH);    /*Turn on the left LED on the robot*/
  while (digitalRead(10) == HIGH) { /*Wait Until Button is Pressed*/
  }
  digitalWrite(RIGHT_LED, HIGH);   /*Turn on the right LED on the robot*/
  analogWrite(Left_Motor_Speed, 200);
  analogWrite(Right_Motor_Speed, 0);
  for (int i = 0; i < 200 ; i++) {
    for (int j = 0 ; j < 5; j++) {
      if (analogRead(sensors[j]) > maxsensorvalues[j]) {
        maxsensorvalues[j] = analogRead(sensors[j]);
        Serial.print("New max value: ");
        Serial.println(maxsensorvalues[j]);
      } if (analogRead(sensors[j]) < minsensorvalues[j]) {
        minsensorvalues[j] = analogRead(sensors[j]);
        Serial.print("New min value: ");
        Serial.println(minsensorvalues[j]);
      }
    }
  } analogWrite(Left_Motor_Speed, 0);
  analogWrite(Right_Motor_Speed, 0);
  digitalWrite(LEFT_LED, LOW);     /*Turn off the left LED on the robot*/
  Serial.println();
  Serial.println("The obtained maximum and minimum sensor values for each sensor:");
  for (int i = 0; i < 5; i++) {
    if (i == 4) {
      Serial.print("A7: ");
    } else {
      Serial.print("A");
      Serial.print(i);
      Serial.print(": ");
    }
    Serial.print(maxsensorvalues[i]);
    Serial.print("  ");
    Serial.println(minsensorvalues[i]);
  } Serial.println(); Serial.println("Calibration Is Over!");
  Serial.println("Press Start When You Have Placed The Robot On The Racing Track");
  while (digitalRead(10) == HIGH) {
    /*Wait Until Button is Pressed*/
  } digitalWrite(RIGHT_LED, LOW);    /*Turn off the right LED on the robot*/
  Serial.println("Start");
}



//Normalizes the Sensor Values:
void normalizeSensorValues() {

  //This functionality will take your sensorvalue and will map it to a value between 0 and 1023
  //depending on the minimum- respectively maximum value obtained from the Calibration Function.
  //In case the sensor value becomes less or greater than the obtained minimum- and maximum value
  //you will get a normalized value which is <0 or >1023. In this case you set your value to 0 respectively 1023
  for (int i = 0; i < 5; i++) {
    normalizedsensorvalues[i] = map(sensorvalues[i], minsensorvalues[i], maxsensorvalues[i], 0, 1023);
    if (normalizedsensorvalues[i] < 0) {
      normalizedsensorvalues[i] = 0;
    } else if (normalizedsensorvalues[i] > 1023) {
      normalizedsensorvalues[i] = 1023;
    }
  }
}
