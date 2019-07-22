#define ENA 11     // ENA speed pin for left motor
#define IN1 8       // IN1 on Motor controller board
#define IN2 9       // IN2

// Right motor pins
#define ENB 10     // ENB speed pin for right motor
#define IN3 12       // IN3 on Motor controller board
#define IN4 13       // IN4

int rightspeed = 110;
int leftspeed = 125;

#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(6, 7, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(2, 3, MAX_DISTANCE), 
  NewPing(4, 5, MAX_DISTANCE)
};

 
void setup() { 
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  
  pinMode(ENA, OUTPUT);  //set up left motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);//sets up right motor
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT);

  analogWrite(ENA, leftspeed); //set speed to mid
  analogWrite(ENB, rightspeed); //set speed to mid
}


void loop() {
    int distance[] = {0,0,0};
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    distance[i] = sonar[i].ping_cm();
    Serial.print(i);
    Serial.print("=");
    Serial.print(distance[i]);
    Serial.print("cm ");
    }
    Serial.println();

    //if ( (distance[0]==0 || distance[1]==0 || distance[2]==0) || (distance[0] > 15 && distance[1] > 15 && distance[2] > 15) ){
    if (distance[0] >= 1 && distance[0] <=15) {
      turnRight();
    //Serial.print("  Turning Right");
    }
    else if ( distance[2] > 0 && distance[2] <= 10 ) {
     turnLeft();
    //Serial.print("  Turning Left");
    }
    else if(distance[1] <= 10 && distance[1]>0){
     halt();
     delay(1000); 
     turnLeft();
     //with the controller --> robot will halt then the controller can control it again 
    }
    else {
      forward();
    }

    delay(500);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void forward() {
  digitalWrite(IN1, HIGH);//left motor forward
  digitalWrite(IN2, LOW);  //

  digitalWrite(IN3, LOW);//right motor forward
  digitalWrite(IN4, HIGH); 
}
void halt () {
  digitalWrite(IN1, LOW);//left motor stop
  digitalWrite(IN2, LOW);  //

  digitalWrite(IN3, LOW);//right motor stop
  digitalWrite(IN4, LOW); 
}
void turnRight(){ 
  digitalWrite(IN1, HIGH);//left motor forwards
  digitalWrite(IN2, LOW);  //

  digitalWrite(IN3, HIGH);//right motor backwards
  digitalWrite(IN4, LOW); 
}
void turnLeft(){
  digitalWrite(IN1, LOW);//left motor bakcwards
  digitalWrite(IN2, HIGH);  //

  digitalWrite(IN3, LOW);//right motor forwards
  digitalWrite(IN4, HIGH); 
}
