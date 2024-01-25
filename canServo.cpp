#include "canServo.h"
#include <iostream>
#include <net/if.h>
#include <vector>

//
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
void ServoControl::setupBrush() {
    frame.can_id = 0x123;
    frame.can_dlc = 8;
    vector<Message> messages;
    char values[] ={0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
    messages.push_back({0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00} );
    copy(begin(values), end(values), begin(frame.data));
    //servoController.frame.data = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
    //servoController.frame.data[0] = 1;
    //servoController.frame.data[1] = 2;
    //servoController.frame.data[2] = 3;
    //servoController.frame.data[3] = 4;
    //servoController.frame.data[4] = 5;
    //servoController.frame.data[5] = 6;
    //servoController.frame.data[6] = 7;
    //servoController.frame.data[7] = 9;

    int nbytes = write(s, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        cerr << "Send Error frame[0]!" << endl;
        // Handle error if needed
    }
}
void ServoControl::getBrushCommand(){
 while (true) {
        cout << "Current value: " << brushValue << endl;
        cout << "Enter a value (or press 'Esc' to exit): ";

        string userInput;

        // Check if there is input available
        if (cin >> userInput) {
            if (userInput == "\033") {  // Escape key represented by the string "\033"
                break;  // Exit the loop if the user presses Escape
            }
  
            // Convert the string to an integer
            try {
                int intValue = stoi(userInput);

                // Check if the input is within the desired range
                if (intValue >= 0 && intValue <= 100) {
                    brushValue = intValue;
                    //setVelocity(value);
                } else {
                    cout << "Invalid input. Please enter a value between 0 and 100." << endl;
                }
            } catch (const invalid_argument& e) {
                cout << "Invalid input. Please enter a numeric value." << endl;
            } catch (const out_of_range& e) {
                cout << "Invalid input. Value out of range." << endl;
            }
        }
    }
}
void ServoControl::sendCAN(){
    //int nbytes = write(s, &frame, sizeof(frame));
    nbytes = write(s, &frame1, sizeof(frame1));
    if (nbytes != sizeof(frame1)) {
        cerr << "Send Error frame[0]!" << endl;
        // Handle error if needed
    }   
}
void ServoControl::receiveCanMessage(atomic<bool>& exitFlag) {
    //this_thread::sleep_for(chrono::milliseconds(100));  // 100 necessary because cin might not work well otherwise.

    while (!exitFlag.load()) {
        //this_thread::sleep_for(chrono::milliseconds(100));  // 100 necessary because cin might not work well otherwise.
        
        //nbytes = read(s, &buffer[j], sizeof(buffer[j]));
        nbytes = read(s, &frame, sizeof(frame));
        if (nbytes > 0) {
        //buffer.push_back(frame);
        //cout << frame.can_id << " Is the ID " << endl;
        //cout << &buffer[j].can_id <<" Is the ID of buffer " << endl;
        //cout << hex << buffer[j].can_id;

        //std::unique_lock<std::mutex> lk(cv_m);
        buffer.push_back(frame);
        //newMessageAdded = true;
        //lk.unlock();
        // cv.notify_one(); // Notify the sorting thread

        // for (int j = 0; j < buffer.size()+1; j++) {
        // cout << "REC    ";
        // cout << hex << buffer[j].can_id << " ";
        // printf(" %01X ",buffer[j].can_dlc);
        // for (int i = 0; i < buffer[j].can_dlc; i++) {
        //    printf(" %02X",buffer[j].data[i]);
        // }
        // cout << endl;
        
        //cout << "buffersize: " << buffer.size() << endl;

        //}
    }
    }
}

void ServoControl::sortCanMessage(atomic<bool>& exitFlag) {
this_thread::sleep_for(chrono::milliseconds(10));  // 100 necessary because cin might not work well otherwise.

while (!exitFlag.load()) {
    //std::unique_lock<std::mutex> lk(cv_m);
    //cv.wait(lk, [this, &exitFlag]{ return newMessageAdded.load() || exitFlag.load(); });
    //if (exitFlag.load()) break; // Exit if flag is set

    // while (1) {
    // cout << "byt" << endl;
    // this_thread::sleep_for(chrono::milliseconds(5));  // Necessary because cin might not work well otherwise.
    // }
    canid_t can_id_181 = 385; 
    canid_t can_id_38B = 0x38B;
    canid_t can_id_28B = 0x28B;
    canid_t can_id_18B = 0x18B;
    canid_t can_id_48B = 0x48B;
    canid_t can_id5 = 0x38B;
    //if (newMessageAdded){
        //cout << "New data!" << endl;
        for (int i = 0; i < buffer.size();) {
            //this_thread::sleep_for(chrono::milliseconds(2));  // Necessary because cin might not work well otherwise.
            //cout << "Buffer size: " << buffer.size() << endl;
            // Check if the received CAN message has the desired ID (0x38B)
            if (buffer[i].can_id == can_id_38B) {
                cout << "earf" << endl;
                //cout << "can_id = 0x" << hex << frame.can_id
                     //<< "\tcan_dlc = " << dec << static_cast<int>(frame.can_dlc) << "\t";

                // Extracting data based on the provided table for TPDO3 (0x38B)
                mpc_total_rmc = *(reinterpret_cast<uint32_t*>(&buffer[i].data[0]));
                mpc_soc = buffer[i].data[4];
                mpc_cell_temp_max = *reinterpret_cast<int8_t*>(&buffer[i].data[5]);
                mpc_cell_temp_min = *reinterpret_cast<int8_t*>(&buffer[i].data[6]);
                mpc_bms_temp_max = *reinterpret_cast<int8_t*>(&buffer[i].data[7]);

                //cout << "MPC_TOTAL_RMC = " << dec << mpc_total_rmc << " [mAh]\t";
                //cout << "MPC_SOC = " << dec << static_cast<int>(mpc_soc) << " [%]\t\t";
                //cout << "MPC_CELL_TEMP_MAX = " << dec << static_cast<int>(mpc_cell_temp_max) << " [oC]\t";
                //cout << "MPC_CELL_TEMP_MIN = " << dec << static_cast<int>(mpc_cell_temp_min) << " [oC]\t";
                //cout << "MPC_BMS_TEMP_MAX = " << dec << static_cast<int>(mpc_bms_temp_max) << " [oC]\t";

                //cout << endl;
            } else if (frame.can_id == can_id_28B) {
                cout << "earf" << endl;
                // Extracting data based on the provided table for TPDO2 (0x28B)
                mpc_rec_charge = *(reinterpret_cast<int16_t*>(&buffer[i].data[0]));
                mpc_rec_discharge = *(reinterpret_cast<int16_t*>(&buffer[i].data[2]));
                mpc_cell_volt_max = *(reinterpret_cast<uint16_t*>(&buffer[i].data[4]));
                mpc_cell_volt_min = *(reinterpret_cast<uint16_t*>(&buffer[i].data[6]));

                //cout << "can_id = 0x" << hex << frame.can_id
                     //<< "\tcan_dlc = " << dec << static_cast<int>(frame.can_dlc) << "\t";
                //cout << "MPC_REC_CHARGE = " << dec << mpc_rec_charge << " [A]\t\t";
                //cout << "MPC_REC_DISCHARGE = " << dec << mpc_rec_discharge << " [A]\t";
                //cout << "MPC_CELL_VOLT_MAX = " << dec << mpc_cell_volt_max << " [mV]\t";
                //cout << "MPC_CELL_VOLT_MIN = " << dec << mpc_cell_volt_min << " [mV]\t";

                //cout << endl;
            } else if (frame.can_id == can_id_18B) {
                cout << "earf" << endl;
                // Extracting data based on the provided table for TPDO1 (0x18B)
                mpc_system_current = *(reinterpret_cast<int32_t*>(&buffer[i].data[0]));
                mpc_system_voltage = *(reinterpret_cast<uint16_t*>(&buffer[i].data[4]));
                //cout << *(reinterpret_cast<uint16_t*>(&frame.data[4]));

                //cout << "can_id = 0x" << hex << frame.can_id
                     //<< "\tcan_dlc = " << dec << static_cast<int>(frame.can_dlc) << "\t";
                //cout << "MPC_SYSTEM_CURRENT = " << dec << mpc_system_current << " [mA]\t";
                //cout << "MPC_SYSTEM_VOLTAGE = " << dec << mpc_system_voltage << " [mV]\t";

                //cout << endl;
            } else if(frame.can_id == can_id_48B) {
                cout << "earf" << endl;
                    // Extracting data based on the provided table for TPDO4 (0x48B)
                mpc_pack_num = *(reinterpret_cast<uint8_t*>(&buffer[i].data[0]));
                mpc_active_pack_num = *(reinterpret_cast<uint8_t*>(&buffer[i].data[1]));
                mpc_error_pack_num = *(reinterpret_cast<uint8_t*>(&buffer[i].data[2]));
                mpc_status = *(reinterpret_cast<uint8_t*>(&buffer[i].data[3]));
                mpc_error_state = *(reinterpret_cast<uint32_t*>(&buffer[i].data[4]));

                //cout << "can_id = 0x" << hex << frame.can_id
                    //<< "\tcan_dlc = " << dec << static_cast<int>(frame.can_dlc) << "\t";
                //cout << "MPC_PACK_NUM = " << dec << static_cast<int>(mpc_pack_num) << "\t\t";
                //cout << "MPC_ACTIVE_PACK_NUM = " << dec << static_cast<int>(mpc_active_pack_num) << "\t\t";
                //cout << "MPC_ERROR_PACK_NUM = " << dec << static_cast<int>(mpc_error_pack_num) << "\t\t";
                //cout << "MPC_STATUS = " << dec << static_cast<int>(mpc_status) << "\t\t\t";
                //cout << "MPC_ERROR_STATE = " << hex << mpc_error_state << "\t\t";
                //cout << "Error Bits: \n";
                for (int i = 0; i < 32; ++i) {
                    //cout << "earf" << endl;
                    bool error_bit = (mpc_error_state & (1 << i)) != 0;
                    if (error_bit) {
                        switch (i) {
                            case 0:  cout << "One or more packs temperature error "; break;
                            case 1:  cout << "One or more packs voltage error "; break;
                            case 2:  cout << "One or more packs current error "; break;
                            case 3:  cout << "One or more packs BMS error "; break;
                            case 4:  cout << "One or more packs MPC error "; break;
                            case 5:  cout << "One or more packs Under voltage error "; break;
                            case 6:  cout << "One or more packs rejecting a request "; break;
                            case 16: cout << "UNKNOWN ERROR "; break;
                            case 17: cout << "Initialization error "; break;
                            case 18: cout << "Communication error "; break;
                            case 19: cout << "State machine error "; break;
                            case 20: cout << "Node error "; break;
                            case 21: cout << "Configuration error "; break;
                            case 28: cout << "State machine error "; break;
                            case 29: cout << "State machine error "; break;
                            case 30: cout << "State machine error "; break;
                            case 31: cout << "State machine error "; break;
                            default: cout << "<reserved>"; break;
                        }
                    }
                }

            //cout << endl;
            } else if (buffer[i].can_id == can_id_181) {
                cout << "Sort:  ";
                    frame1.can_dlc = 8;
                    frame1.data[4] = buffer[i].data[0];
					frame1.data[5] = buffer[i].data[1];
                    frame1.can_id = 0x602;
                    frame1.data[0] = 0x2B;
                    frame1.data[1] = 0x71;
                    frame1.data[2] = 0x60;
                    frame1.data[3] = 0x00;
                    frame1.data[6] = 0x00;
                    frame1.data[7] = 0x00;   
					sendCAN();

                    printf("%03X ",frame1.can_id);
                for (int i = 0; i < frame1.can_dlc; i++) {
                    printf(" %02X",frame1.data[i]);
                }
                cout << "\n";
            }
            else{

                // printf("%02X ",buffer[i].data[0]);
                // printf("%02X ",buffer[i].data[1]);
                // printf("%02X ",buffer[i].data[2]);
                // printf("%02X ",buffer[i].data[3]);
                // printf("%02X ",buffer[i].data[4]);
                // printf("%02X ",buffer[i].data[5]);
                //printf("%01X ",buffer.size());
                cout << hex << buffer[i].can_id << "  ";
                for (int k = 0; k < buffer[i].can_dlc; k++) {
                    printf("%02X ",buffer[i].data[k]);
                }
                cout << endl;
                
                // Ignore CAN messages with IDs other than 0x38B, 0x28B, and 0x18B
                // cout << "Ignoring CAN message with ID 0x" << hex << frame.can_id << endl;

                // printf("%03X ",frame.can_id);
                // for (int i = 0; i < frame.can_dlc; i++) {
                //     printf(" %02X",frame.data[i]);
                // }

                    // cout << hex << frame.can_id << "\t";
                    // //cout << *reinterpret_cast<uint16_t*>(&frame.can_dlc) << " ";
                    // cout << *reinterpret_cast<uint16_t*>(&frame.data[0]) << " ";
                
        }
        //cout << "buffer size before: "<<buffer.size() << endl;
        buffer.erase(buffer.begin());
        cout <<  "buffer size : " << buffer.size() << endl;
        //this_thread::sleep_for(chrono::milliseconds(100));  // 100 necessary because cin might not work well otherwise.
        //newMessageAdded = false;
    //    }
    // for (auto it = buffer.begin(); it != buffer.end(); /* no increment */) {
    //     // Process *it
    //     it = buffer.erase(it); // This removes the processed element and advances the iterator
    // }
    
        // lk.unlock();

    }
    }
}



