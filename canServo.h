#ifndef CAN_SERVO_H
#define CAN_SERVO_H

#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <atomic>
#include <vector>

 #include <condition_variable>
 #include <mutex>

using namespace std;

class ServoControl {
public:

     std::condition_variable cv;
     std::mutex cv_m;
     std::atomic<bool> newMessageAdded{false};


    uint32_t mpc_total_rmc;
    uint8_t mpc_soc;
    int8_t mpc_cell_temp_max; 
    int8_t mpc_cell_temp_min; 
    int8_t mpc_bms_temp_max; 

    int16_t mpc_rec_charge;
    int16_t mpc_rec_discharge; 
    uint16_t mpc_cell_volt_max; 
    uint16_t mpc_cell_volt_min; 

    int32_t mpc_system_current;
    uint16_t mpc_system_voltage; 

    uint8_t mpc_pack_num;
    uint8_t mpc_active_pack_num;
    uint8_t mpc_error_pack_num;
    uint8_t mpc_status;
    uint32_t mpc_error_state;

    int brushValue = 0;
    int ret;
    int s, nbytes;
    struct ifreq ifr;
    struct sockaddr_can addr;
    struct can_frame frame, frame1;
    vector<can_frame> buffer;
    //struct can_frame frame1;
    struct Message{
        unsigned char data[8];
    };



    ServoControl(); 
    void myPublicMethod();
    void setSpeed(int speed);
    void canSetup();
    void setupBrush();
    void getBrushCommand();
    void sendCAN();
    void receiveCanMessage(std::atomic<bool>& exitFlag);
    void sortCanMessage(std::atomic<bool>& exitFlag);
};

#endif
