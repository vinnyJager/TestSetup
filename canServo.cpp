#include "canServo.h"
#include <iostream>
#include <net/if.h>

#include <iostream>
#include <atomic>
#include <cstring>
#include <fstream>
#include <thread>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <ctime>
#include <cctype>  // for isdigit
#include <limits>
#include "canServo.h"
//
using namespace std;

ServoControl::ServoControl() {
    
}

void ServoControl::canSetup() {
    memset(&frame, 0, sizeof(struct can_frame));
    system("sudo ip link set can0 type can bitrate 500000");
    system("sudo ifconfig can0 up");
    cout << "This is a CAN send and receive demo" << endl;


    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    
    if (s < 0) {
    perror("socket PF_CAN failed");
    // Stop program?
    }

    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(s, SIOCGIFINDEX, &ifr);  // laatste toegevoegd
    if (ret < 0) {
        perror("ioctl failed");
    // Stop program?
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
    if (ret < 0) {
        perror("bind failed");
    // Stop program? was return 1;
    }
}

#ifndef CAN_SERVO_H
#define CAN_SERVO_H

#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

class ServoControl {
private:
public:

    int ret;
    int s, nbytes;
    struct ifreq ifr;
    struct sockaddr_can addr;
    struct can_frame frame;


    ServoControl(); // constructor
    void myPublicMethod();
    void setSpeed(int speed);
    void canSetup();
};

#endif // ServoControl
