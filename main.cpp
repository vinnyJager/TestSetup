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

using namespace std;

// Function to receive CAN message with timeout
void receiveCanMessage(int s);
void printWelcome();
void printManualMenu();
void printAutomaticMenu();
void printStartMenu();
void manualControl();
void automaticControl();
int getStartMenuAnswer();
int getManualMenuAnswer();
int getAutomaticMenuAnswer();
string getFileName();
string getCPUTemperature();
atomic<bool> exitFlag(false);
bool exitFlagTime = false;   
struct LogEntry {
    double motorCurrent;
    double voltage;
    };
void logData(ofstream& logFile, double motorCurrent, double voltage) {

    LogEntry entry;

    entry.motorCurrent = motorCurrent;
    entry.voltage = voltage;

    auto now = chrono::high_resolution_clock::now();
    static auto start_time = now;
    auto elapsed_time = chrono::duration_cast<chrono::seconds>(now - start_time);

    int hours = chrono::duration_cast<chrono::hours>(elapsed_time).count();
    int minutes = chrono::duration_cast<chrono::minutes>(elapsed_time % chrono::hours(1)).count();
    int seconds = chrono::duration_cast<chrono::seconds>(elapsed_time % chrono::minutes(1)).count();
    string cpuTemperature = getCPUTemperature();
    logFile << hours << "h" << minutes << "m" << seconds << "s" << "," << cpuTemperature << "," << entry.motorCurrent << "," << entry.voltage << endl;
}
void dataLogger() {
    while (!exitFlag.load()) {
        string fileName = getFileName(); 
        ofstream logFile(fileName);

        if (!logFile.is_open()) {
                cerr << "Failed to open log file.\n";
                exitFlag.store(true);
                return;
        }
        auto lastLogFileCreationTime = chrono::system_clock::now();

        while (!exitFlagTime) {
            if (chrono::duration_cast<chrono::hours>(chrono::system_clock::now() - lastLogFileCreationTime).count() >= 1){
                exitFlagTime = true;
            }
            else {
                double motorCurrent = 1;
                double voltage = 2;

                try {
                logData(logFile, motorCurrent, voltage);
                } 
                catch (const exception& e) {
                    cerr << "Exception during data logging: " << e.what() << endl;
                    exitFlag.store(true);  
                }   
                this_thread::sleep_for(chrono::milliseconds(500));
            } 
        }
        logFile.close();
        exitFlagTime = false;
    }
    exitFlag.store(false);
}


int main() {
    bool mainExit = false;
    thread loggerThread(dataLogger);
    ServoControl servoController;
    
    memset(&servoController.frame, 0, sizeof(struct can_frame));

    system("sudo ip link set can0 type can bitrate 500000");
    system("sudo ifconfig can0 up");
    cout << "This is a CAN send and receive demo" << endl;
    servoController.canSetup();

    // 1. Create socket
    servoController.s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (servoController.s < 0) {
    perror("socket PF_CAN failed");
    return 1;
    }

    // 2. Specify can0 devices
    strcpy(servoController.ifr.ifr_name, "can0");
    strcpy(ifr.ifr_name, "can0");
    servoController.ret = ioctl(servoController.s, SIOCGIFINDEX, &servoController.ifr);
    if (servoController.ret < 0) {
        perror("ioctl failed");
        return 1;
    }

    // 3. Bind the socket to can0
    servoController.addr.can_family = AF_CAN;
    servoController.addr.can_ifindex = servoController.ifr.ifr_ifindex;
    servoController.ret = bind(servoController.s, reinterpret_cast<struct sockaddr *>(&servoController.addr), sizeof(servoController.addr));
    if (servoController.ret < 0) {
        //perror("bind failed");
        //return 1;
    }

    // 4. Disable filtering rules for now (you can adjust this based on your requirements)
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    while(!mainExit) {
        
        printWelcome();
        printStartMenu();
        
        switch(getStartMenuAnswer()) {
            case 1:
            manualControl();
            break;

            case 2:
            automaticControl();
            break;

            case 3: //Exit
            exitFlagTime = true;
            exitFlag.store(true);

            if (loggerThread.joinable()) {
                loggerThread.join();
            }
            mainExit = true;
            break;
        }
    }
        mainExit = false; 

    // 5. Set send data

    frame.can_id = 0x123;
    frame.can_dlc = 8;
    frame.data[0] = 1;
    frame.data[1] = 2;
    frame.data[2] = 3;
    frame.data[3] = 4;
    frame.data[4] = 5;
    frame.data[5] = 6;
    frame.data[6] = 7;
    frame.data[7] = 9;

    int nbytes = write(s, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        cerr << "Send Error frame[0]!" << endl;
        // Handle error if needed
    }


    // Create thread for receiving
    thread receiveThread(receiveCanMessage, servoController.s); 

    // Wait for threads to finish
    receiveThread.join();

    // Close the socket and can0
    close(servoController.s);
    system("sudo ifconfig can0 down");

    return 0;
}


void printWelcome(){
    cout<< "Welcome to the test setup.\n";
}
void printStartMenu(){
    cout << "What do you want to do?\n";
    cout << "1. Manual controll\n";
    cout << "2. Automatic tests\n";
    cout << "3. Exit\n";
}
int getStartMenuAnswer(){
    int answer;

    while(!(cin >> answer) || answer < 1 || answer > 5) {
        cin.clear(); 
        cin.ignore(numeric_limits<streamsize>::max(), '\n'); 
        cout << "That is not an option. Please check your input and try again:\n";
    }
    return answer;  
}
void printManualMenu(){
    cout << "What do you want to control?\n";
    cout << "1. Brush motor drive.\n";
    cout << "2. Traction motor drive.\n";
    cout << "3. Sill motor drive.\n";
    cout << "4. Water pump relay.\n";
    cout << "5. Return.\n";
}
void printAutomaticMenu(){
    cout << "What test would you like to perform?\n";
    cout << "1. Capacity test.\n";
    cout << "2. Internal resistance test.\n";
    cout << "3. Functionality test.\n";
    cout << "4. Return \n";
}
void manualControl() {
    bool manualExit = false;

    while (!manualExit) {
        printManualMenu();
        switch (getManualMenuAnswer()) {
            case 1: {
                int value = 0;

                while (true) {
                    cout << "Current value: " << value << endl;
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
                                value = intValue;
                                // SEND COMMAND!
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
                break;
            }

            case 2:
                // control traction motor
                break;

            case 3:
                // control sill motor
                break;

            case 4:
                // control water pump
                break;

            case 5:
                manualExit = true;
                break;
        }
    }
}
void automaticControl(){
    bool automaticExit = false;
    while(!automaticExit) {
         printAutomaticMenu();

        switch(getAutomaticMenuAnswer()) {
            case 1:
            // Capacity test
            break;

            case 2:
            // Internal resistance test
            break;

            case 3:
            // Functionality test
            break;

            case 4:     
            automaticExit = true;
            break;
        }
    }

    automaticExit = false;

}
int getManualMenuAnswer() {
    int answer;
    
    while(!(cin >> answer) || answer < 1 || answer > 5) {
        cin.clear(); 
        cin.ignore(numeric_limits<streamsize>::max(), '\n'); 
        cout << "That is not an option. Please check your input and try again:\n";
    }
    return answer;  
}
int getAutomaticMenuAnswer() {
    int answer;
    
    while(!(cin >> answer) || answer < 1 || answer > 4) {
        cin.clear(); 
        cin.ignore(numeric_limits<streamsize>::max(), '\n'); 
        cout << "That is not an option. Please check your input and try again:\n";
    }
    return answer;  
}
string getFileName() {
    auto now = chrono::system_clock::now();

    time_t now_time_t = chrono::system_clock::to_time_t(now);
    tm* now_tm = localtime(&now_time_t);

    int year = now_tm->tm_year + 1900;  
    int month = now_tm->tm_mon + 1;      
    int day = now_tm->tm_mday;           
    int hour = now_tm->tm_hour;          
    int minute = now_tm->tm_min;        

    string fileName = "logFiles/" + to_string((year-2000)) + "_" + to_string(month) + "_" + to_string(day) + "_" + to_string(hour) + ":" + to_string(minute) + ".log";

    return fileName;
}
string getCPUTemperature() {
    ifstream tempFile("/sys/class/thermal/thermal_zone0/temp");
    if (!tempFile) {
        cerr << "Error opening temperature file." << endl;
        return "Error";
    }
    string temperature;
    tempFile >> temperature;
    tempFile.close();

    float tempCelsius = stof(temperature) / 1000.0;

    return to_string(tempCelsius);
}
void receiveCanMessage(int s) {
    struct can_frame frame;
    ssize_t nbytes;

    cout << "starting" << endl;
    while (true) {
        nbytes = read(s, &frame, sizeof(frame));
        if (nbytes > 0) {
            cout << "Received a CAN frame:" << endl;
            cout << "can_id  = 0x" << hex << frame.can_id << endl;
            cout << "can_dlc = " << dec << frame.can_dlc << endl;
            for (int i = 0; i < 8; i++)
                cout << "data[" << i << "] = " << dec << static_cast<int>(frame.data[i]) << endl;
        } else if (nbytes < 0) {
            perror("Read Error");
            // Handle error if needed
        } else {
            cout << "No data received." << endl;
        }
    }
}
