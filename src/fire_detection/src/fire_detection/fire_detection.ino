  /*-------defining Inputs------*/
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

bool angle; // Declare msg at a global or higher scope
void fireCallback(const std_msgs::Bool& msg);

ros::NodeHandle nh;

#define Left_S 8      // Left sensor
#define Right_S 10    // Right sensor
#define Forward_S 9   // Forward sensor
#define pump 7

std_msgs::Int8 fire_msg;                              // Variable fire_msg of the type std_msgs::Int32
ros::Publisher fire_pub("fire_detection", &fire_msg);  // Publisher is about the fire_msg variable on the topic 'fire_detection'. Publisher's name is 'fire_pub'.
ros::Subscriber<std_msgs::Bool> shoot_water("water_pump", &fireCallback);

/*-------Variables------*/
boolean fire = false;



void setup() {    
  Serial.begin(9600); // Start serial communication at 9600 baud
  pinMode(Left_S, INPUT);
  pinMode(Right_S, INPUT);
  pinMode(Forward_S, INPUT);
  pinMode(pump, OUTPUT);

  nh.initNode();
  nh.advertise(fire_pub);
  nh.subscribe(shoot_water);

}

void extinguishFire() {
  //delay(1000);
  digitalWrite(pump, HIGH);   // Start the water pump
  delay(10000);                 // Run pump for a duration (adjust as needed)
  digitalWrite(pump, LOW);
  delay(500);// Stop the water pump
}


void fireCallback(const std_msgs::Bool& msg) {
  // Callback function that will be called when a message is received
  // The received integer is in msg.data
  angle=msg.data;
  // Act upon the received integer (e.g., control the water pump)
  if (msg.data == 1) {
    extinguishFire();
  }
}


void loop() {
  int detectedSensor = 0;
  fire = digitalRead(Left_S) == 0 || digitalRead(Right_S) == 0 || digitalRead(Forward_S) == 0;
  if (fire) {
    if (digitalRead(Forward_S) == 0) {
      detectedSensor = 0; // Forward
    } else if (digitalRead(Left_S) == 0) {
      detectedSensor = -1; // Left
    } else if (digitalRead(Right_S) == 0) {
      detectedSensor = 1; // Right
    }
    fire_msg.data = detectedSensor;
    
    fire_pub.publish(&fire_msg);
    // Output the results
    Serial.print("Detected Fire! Sensor: ");
    Serial.print(detectedSensor);

    //extinguishFire();
  }
  nh.spinOnce();
  delay(300); // Slow down the loop
}
