#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <Encoder.h>
#include <math.h>

// Define motor pins
#define M1R_DIR 5    // RIGHT motor direction
#define M1R_PWM 4    // RIGHT motor speed
#define M2L_DIR 6    // LEFT motor direction
#define M2L_PWM 9    // LEFT motor speed

// Define range sensor pins (IR sensors)
#define IR_FRONT A2  // Front IR sensor
#define IR_RIGHT A3  // Right IR sensor
#define IR_LEFT  A4  // Left IR sensor

// Define encoder pins
#define ENC_M1R_A 18 // RIGHT encoder channel A
#define ENC_M1R_B 19 // RIGHT encoder channel B
#define ENC_M2L_A 3  // LEFT encoder channel A
#define ENC_M2L_B 2  // LEFT encoder channel B

// Define wheel and robot parameters
const float wheel_radius = 0.014;     // in meters
const float wheel_base = 0.095;       // distance between wheels in meters
const float ticks_per_revolution_right = 8590; // Right encoder ticks per revolution
const float ticks_per_revolution_left = 8000;  // Left encoder ticks per revolution
const float distance_per_tick_right = (2 * PI * wheel_radius) / ticks_per_revolution_right;
const float distance_per_tick_left = (2 * PI * wheel_radius) / ticks_per_revolution_left;

// Variables to store the robot's current position
float current_x = 0.0;
float current_y = 0.0;
float current_theta = 0.0;

// Variables to store the last encoder readings
long last_enc_m1r = 0;
long last_enc_m2l = 0;

long delta_enc_m1r = 0;
long delta_enc_m2l = 0;

Encoder encM1R(ENC_M1R_A, ENC_M1R_B); // Right encoder
Encoder encM2L(ENC_M2L_A, ENC_M2L_B); // Left encoder

// Create ROS node handle
ros::NodeHandle nh;

// Publisher for distance sensors
std_msgs::Float32 front_distance_msg;
ros::Publisher pub_front_distance("front_distance", &front_distance_msg);

std_msgs::Float32 right_distance_msg;
ros::Publisher pub_right_distance("right_distance", &right_distance_msg);

std_msgs::Float32 left_distance_msg;
ros::Publisher pub_left_distance("left_distance", &left_distance_msg);

// Publisher for robot pose
geometry_msgs::Pose pose_msg;
ros::Publisher pub_pose("pose", &pose_msg);

// Subscriber for velocity commands
geometry_msgs::Twist cmd_vel_msg;
void cmdVelCallback(const geometry_msgs::Twist& msg) {
    // Control robot's motors based on received message
    int linear_speed = msg.linear.x * 255;
    int angular_speed = msg.angular.z * 255;

    int right_speed = constrain(linear_speed - angular_speed, -255, 255);
    int left_speed = constrain(linear_speed + angular_speed, -255, 255);

    // Set motor directions and speeds
    digitalWrite(M1R_DIR, right_speed >= 0 ? LOW : HIGH);
    analogWrite(M1R_PWM, abs(right_speed));

    digitalWrite(M2L_DIR, left_speed >= 0 ? LOW : HIGH);
    analogWrite(M2L_PWM, abs(left_speed));
}

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &cmdVelCallback);

// Function to publish the robot's pose
void publishPose() {
    pose_msg.position.x = getCurrentX();
    pose_msg.position.y = getCurrentY();
    pose_msg.position.z = 0;  // Assuming robot operates in a 2D plane

    // Convert theta (yaw) to quaternion
    float qx, qy, qz, qw;
    thetaToQuaternion(current_theta, qx, qy, qz, qw);

    pose_msg.orientation.x = qx;
    pose_msg.orientation.y = qy;
    pose_msg.orientation.z = qz;
    pose_msg.orientation.w = qw;

    pub_pose.publish(&pose_msg);
}

unsigned long previousMillis = 0;
const long interval = 100; // Interval at which to run loop (100ms for 10Hz)

void setup() {
    nh.initNode();

    // Setup motor pins
    pinMode(M1R_DIR, OUTPUT);
    pinMode(M1R_PWM, OUTPUT);
    pinMode(M2L_DIR, OUTPUT);
    pinMode(M2L_PWM, OUTPUT);

    // Setup sensor pins
    pinMode(IR_FRONT, INPUT);
    pinMode(IR_RIGHT, INPUT);
    pinMode(IR_LEFT, INPUT);

    // Advertise publishers
    nh.advertise(pub_front_distance);
    nh.advertise(pub_right_distance);
    nh.advertise(pub_left_distance);
    nh.advertise(pub_pose);

    // Subscribe to topics
    nh.subscribe(sub_cmd_vel);
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Read sensor data and publish
        front_distance_msg.data = analogRead(IR_FRONT) * (5.0 / 1023.0); // Convert to voltage or distance
        pub_front_distance.publish(&front_distance_msg);

        right_distance_msg.data = analogRead(IR_RIGHT) * (5.0 / 1023.0);
        pub_right_distance.publish(&right_distance_msg);

        left_distance_msg.data = analogRead(IR_LEFT) * (5.0 / 1023.0);
        pub_left_distance.publish(&left_distance_msg);

        encupdate();
        publishPose();

        nh.spinOnce();
    }
}

float getCurrentX() {
    updatePose();
    return current_x;
}

float getCurrentY() {
    updatePose();
    return current_y;
}

float getCurrentTheta() {
    updatePose();
    return current_theta;
}

void encupdate() {
    long enc_m1r = encM1R.read();
    long enc_m2l = encM2L.read();

    delta_enc_m1r = enc_m1r - last_enc_m1r;
    delta_enc_m2l = enc_m2l - last_enc_m2l;

    last_enc_m1r = enc_m1r;
    last_enc_m2l = enc_m2l;
}

void updatePose() {
    float dt = 0.1; // Time interval between updates

    // Calculate linear velocity
    float v_right = delta_enc_m1r * distance_per_tick_right / dt;
    float v_left = delta_enc_m2l * distance_per_tick_left / dt;
    float v = (v_right + v_left) / 2;

    // Calculate angular velocity
    float w = (v_right - v_left) / wheel_base;

    // Update orientation
    current_theta = atan2(sin(current_theta + w * dt), cos(current_theta + w * dt));

    // Update position
    current_x += v * cos(current_theta) * dt;
    current_y += v * sin(current_theta) * dt;
}

void thetaToQuaternion(float theta, float &qx, float &qy, float &qz, float &qw) {
    qx = 0;
    qy = 0;
    qz = sin(theta / 2);
    qw = cos(theta / 2);
}
