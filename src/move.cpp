#include <ros/ros.h>
#include <wiringPi.h>
#include <unistd.h> // sleep()

// Mapping WiringPi pins ke GPIO
#define IN1 0 // GPIO17 - Motor kiri maju
#define IN2 1 // GPIO18 - Motor kiri mundur
#define IN3 2 // GPIO27 - Motor kanan maju
#define IN4 3 // GPIO22 - Motor kanan mundur

void setupMotorPins() {
    wiringPiSetup();
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void stopMotor() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_forward_gpio");
    ros::NodeHandle nh;

    setupMotorPins();
    ROS_INFO("Rover jalan maju 5 detik...");

    moveForward();
    sleep(5); // Tunggu 5 detik
    stopMotor();

    ROS_INFO("Rover berhenti.");
    return 0;
}
