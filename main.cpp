#include <iostream>
#include <atomic>
#include <cstring>
#include <fstream>
#include <thread>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <ctime>
#include <cctype>  // for isdigit
#include <limits>
#include "canServo.h"
#include <chrono>
#include <algorithm>
#include <vector>
#include <pigpio.h>
#include <mutex>
#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <iomanip>  // for setprecision and fixed

#define LED_PIN 18
#define FLOW_PIN 17
#define FADE_STEPS 100
#define MAX_DURATION_MICROSECONDS 150000 // Maximum allowed duration (0.2 seconds)
#define FREQ_TO_FLOW_SLOPE 10.0 / 235.0  // Slope of the linear relationship

using namespace std;

struct Message {
    unsigned char data[8];
};

vector<Message> initMessages() {
    vector<Message> messages;
	messages.push_back({0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00}); //0 ready to switch on
	messages.push_back({0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00}); //1 switched on
	messages.push_back({0x2B, 0x40, 0x60, 0x00, 0x0F, 0x01, 0x00, 0x00}); //2 operation enabled
	messages.push_back({0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00}); //3 set velocity mode
	messages.push_back({0x2F, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00}); //4 set torque mode
	messages.push_back({0x23, 0xFF, 0x60, 0x00, 0x60, 0xCC, 0x05, 0x00}); //5 set velocity = 380 000 counts/s
	messages.push_back({0x23, 0x83, 0x60, 0x00, 0x20, 0xA1, 0x07, 0x00}); //6 set accel = 500 000counts/s/s
	messages.push_back({0x23, 0x84, 0x60, 0x00, 0x20, 0xA1, 0x07, 0x00}); //7 set decel = 500 000counts/s/s
	messages.push_back({0x2B, 0x16, 0x22, 0x00, 0x5E, 0x00, 0x00, 0x00}); //8 set torque constant = 94mNm/A => berekent
	messages.push_back({0x2B, 0x71, 0x60, 0x00, 0xC8, 0x00, 0x00, 0x00}); //9 set torque target = 200mNm CW
	messages.push_back({0x23, 0x87, 0x60, 0x00, 0x58, 0x02, 0x00, 0x00}); //10 set torque slope = 600mNm/sec => berekent op basis van 200mNm en 1m accel/decel
	messages.push_back({0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00}); //11 start motion
	messages.push_back({0x2B, 0x40, 0x60, 0x00, 0x0F, 0x01, 0x00, 0x00}); //12 stop motion
	messages.push_back({0x23, 0xFF, 0x60, 0x00, 0xA0, 0x33, 0xFA, 0xFF}); //13 set velocity = -380 000 counts/s
	messages.push_back({0x2B, 0x71, 0x60, 0x00, 0x38, 0xFF, 0x00, 0x00}); //14 set torque target = 200mNm CCW
	messages.push_back({0x2B, 0x71, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}); //15 set torque target = 0mNm

	return messages;
}

Message getMessage(const vector<Message>& messages, int input) {
    return messages[input];
}

std::atomic<int> pulseCount(0);
std::atomic<std::chrono::steady_clock::time_point> lastPulseTime;
std::atomic<double> flowFrequency(0.0);
std::atomic<double> flowRate(0.0);
bool firstLog = true;

int waterPWM;
bool exitFlagTime = false;  

std::mutex flowMutex;
std::mutex cinMutex;

void setupPWM();
void printWelcome();
void printManualMenu();
void printAutomaticMenu();
void printStartMenu();
void manualControl();
void automaticControl();

void INIT_Axis1();
void INIT_Axis2();
void Start_Motion();
void Stop_Motion();

int getStartMenuAnswer();
int getManualMenuAnswer();
int getAutomaticMenuAnswer();

string getFileName();
string getCPUTemperature();

atomic<bool> exitFlag(false);
 
struct LogEntry {
    double motorCurrent;
    double voltage;
};


void getPumpCommand();
void getBrushCommand();
void logData(ofstream& logFile, ServoControl& servoController);
void dataLogger();
void setupBrush();
void exitProgramme();
void capacityTest();
void readFlow();
void flowCallback(int gpio, int level, uint32_t tick);
void calculateFlowRate(double frequency);
ServoControl servoController;

int main() {

    bool mainExit = false;

    thread loggerThread(dataLogger);
    thread receiveThread(&ServoControl::receiveCanMessage, &servoController, std::ref(exitFlag));
    thread sortThread(&ServoControl::sortCanMessage, &servoController, std::ref(exitFlag));
    thread readerThread(readFlow);
    servoController.canSetup();
    setupPWM();

    
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

            close(servoController.s);
            system("sudo ifconfig can0 down");

            if (loggerThread.joinable()) {
                loggerThread.join();
            }
            if (receiveThread.joinable()) {
                receiveThread.join();
            }
            // Ensure the PWM is off before program termination
            gpioHardwarePWM(LED_PIN, 0, 0);

            // Cleanup and close pigpio library
            gpioTerminate();
            mainExit = true;

            break;
        }
    } 

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
int getStartMenuAnswer() {

    int answer;
    cin >> answer;
    cout << "You entered: " << answer << endl;
    while ( answer < 1 || answer > 3) {
        cout << "That is not an option. Please check your input and try again:\n";
        cin >> answer;
    }
    return answer;  
}
void printManualMenu(){
    cout << "What do you want to control?\n";
    cout << "1. Brush motor.\n";
    cout << "2. Traction motor.\n";
    cout << "3. Sill motor.\n";
    cout << "4. Water pump.\n";
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
            case 1: 
                servoController.setupBrush();
                servoController.getBrushCommand();
                break;
            
            case 2:
                // control traction motor

                //operation mode motor 1
                servoController.frame.can_id = 0x000;
                servoController.frame.can_dlc = 2;
                servoController.frame.data[0] = 1;
                servoController.frame.data[1] = 1;
                servoController.sendCAN();
                servoController.frame.can_dlc = 8;
                ///////

                //operation mode motor 2
                servoController.frame.can_id = 0x000;
                servoController.frame.can_dlc = 2;
                servoController.frame.data[0] = 1;
                servoController.frame.data[1] = 2;
                servoController.sendCAN();
                servoController.frame.can_dlc = 8;

                //init motor 1
                INIT_Axis1();


                //init motor 2
                INIT_Axis2();




                //start motion
                //Start_Motion();


                //stop motion
                //Stop_Motion();

                break;

            case 3:
                // control sill motor
                break;

            case 4:
                getPumpCommand();
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
            capacityTest();
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
        //cin.clear(); 
        //cin.ignore(numeric_limits<streamsize>::max(), '\n'); 
        cout << "That is not an option. Please check your input and try again:\n";
    }
    return answer;  
}
int getAutomaticMenuAnswer() {
    int answer;
    
    while(!(cin >> answer) || answer < 1 || answer > 4) {
        //cin.clear(); 
        //cin.ignore(numeric_limits<streamsize>::max(), '\n'); 
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

    // Format the temperature string to have only one decimal place
    stringstream ss;
    ss << fixed << setprecision(1) << tempCelsius;

    return ss.str();
}
void setupPWM(){
    if (gpioInitialise() < 0) {
        cerr << "Failed to initialize pigpio" << endl;
    }

    // Set the GPIO pin as an ALT0 function for PWM
    gpioSetMode(LED_PIN, PI_ALT0);
    gpioSetMode(FLOW_PIN, PI_INPUT);

    // Setup hardware PWM on GPIO 18 with a frequency of 488 Hz
    gpioHardwarePWM(LED_PIN, 488, 0);
    gpioSetAlertFunc(FLOW_PIN, flowCallback);
}
void getPumpCommand(){

    while (true) {
        cout << "Current value: " << waterPWM << "%"<<endl;
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
                    waterPWM = intValue;
                    gpioHardwarePWM(LED_PIN, 488, 10000*waterPWM);
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
void logData(ofstream& logFile, ServoControl& servoController) {

    auto now = chrono::high_resolution_clock::now();
    static auto start_time = now;
    auto elapsed_time = chrono::duration_cast<chrono::seconds>(now - start_time);

    int hours = chrono::duration_cast<chrono::hours>(elapsed_time).count();
    int minutes = chrono::duration_cast<chrono::minutes>(elapsed_time % chrono::hours(1)).count();
    int seconds = chrono::duration_cast<chrono::seconds>(elapsed_time % chrono::minutes(1)).count();
    string cpuTemperature = getCPUTemperature();
    // logFile << hours << ",hours," << minutes << ",minutes," << seconds << ",seconds," << cpuTemperature << ",celsius," 
    // << servoController.mpc_total_rmc << ",[mAh]," << servoController.mpc_soc << ",[%],"
    // << servoController.mpc_system_voltage << ",[mV]," << servoController.mpc_system_current <<  ",[mA],"
    // << servoController.mpc_cell_temp_max << ",[celsius]," << servoController.mpc_cell_temp_min <<  ",[celsius],"
    // << servoController.mpc_bms_temp_max << ",[celsius]," << servoController.mpc_rec_charge <<  ",[A]"
    // << servoController.mpc_rec_discharge << ",[A]," << servoController.mpc_cell_volt_max <<  ",[mV],"
    // << servoController.mpc_cell_volt_min << ",[mV]," << servoController.mpc_pack_num <<  ",[-]"
    // << servoController.mpc_active_pack_num << ",[-]" << servoController.mpc_error_pack_num <<  ",[-],"
    // << servoController.mpc_status << ",[-]" << servoController.mpc_error_state << ",[-],"
    // << endl;
    if(firstLog) {
    logFile << "Hours," << "Minutes," << "Seconds," << "CPU temp," << "Remaining capacity," << "State of charge," << "Pack voltage," << "Pack current,"
    << "max cell temp," << "min cell temp," << "bms temp max," << "recommended charge current,"<< "recommended discharge current," << "max cell voltage," << "min cell voltage,"
    << "pack number," << "active number packs," << "pack with errors," << "mpc_status," << "MPC error staus," << "flowrate," << "flow frequentie," << endl;
    logFile << "[h]," << "[m]," << "[s]," << "[deg C]," << "[mAh]," << "[%]," << "[V]," << "[mA],"
    << "[deg C]," << "[deg C]," << "[deg C]," << "[A],"<< "[A]," << "[mV]," << "[mV],"
    << "[-]," << "[-]," << "[-]," << "[-]," << "[-]," << "[Lpm]," << "[Hz]," << endl;
    firstLog = false;
    }
    logFile << hours << ","
    << minutes << ","
    << seconds << ","
    << cpuTemperature << ","
    << static_cast<int>(servoController.mpc_total_rmc) << ","
    << static_cast<int>(servoController.mpc_soc) << ","
    << static_cast<uint16_t>(servoController.mpc_system_voltage) << ","
    << static_cast<int32_t>(servoController.mpc_system_current) << ","
    << static_cast<int>(servoController.mpc_cell_temp_max) << ","
    << static_cast<int>(servoController.mpc_cell_temp_min) << ","
    << static_cast<int>(servoController.mpc_bms_temp_max) << ","
    << static_cast<int>(servoController.mpc_rec_charge) << ","
    << static_cast<int>(servoController.mpc_rec_discharge) << ","
    << static_cast<int>(servoController.mpc_cell_volt_max) << ","
    << static_cast<int>(servoController.mpc_cell_volt_min) << ","
    << static_cast<int>(servoController.mpc_pack_num) << ","
    << static_cast<int>(servoController.mpc_active_pack_num) << ","
    << static_cast<int>(servoController.mpc_error_pack_num) << ","
    << static_cast<int>(servoController.mpc_status) << ","
    << static_cast<int>(servoController.mpc_error_state) << ","
    << fixed << setprecision(1) << flowRate << ","  // One decimal place for flow rate
    << fixed << setprecision(1) << flowFrequency << ","  // One decimal place for flow frequency
    << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
    << static_cast<unsigned>(servoController.frame.can_id) << "," 
    << static_cast<unsigned>(servoController.frame.can_dlc) << "," 
    << static_cast<unsigned>(servoController.frame.can_id) << ","
    << static_cast<unsigned>(servoController.frame.data[0]) << ","
    << static_cast<unsigned>(servoController.frame.data[1]) << ","
    << static_cast<unsigned>(servoController.frame.data[2]) << ","
    << static_cast<unsigned>(servoController.frame.data[3]) << ","
    << static_cast<unsigned>(servoController.frame.data[4]) << ","
    << static_cast<unsigned>(servoController.frame.data[5]) << ","
    << static_cast<unsigned>(servoController.frame.data[6]) << ","
    << static_cast<unsigned>(servoController.frame.data[7]) << ","
    << std::dec << endl;


}
void dataLogger() {
    while (!exitFlag.load()) {
        string fileName = getFileName(); 
        ofstream logFile(fileName,ios::out | ios::binary);
        logFile << u8"\ufeff";

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
                //double motorCurrent = 1;
                //double voltage = 2;

                try {
                logData(logFile, servoController);
                } 
                catch (const exception& e) {
                    cerr << "Exception during data logging: " << e.what() << endl;
                    exitFlag.store(true);  
                }   
                //this_thread::sleep_for(chrono::milliseconds(500)); //500 for 2 hertz //100 for 10 hertz // 10 for 100 hertz //
                this_thread::sleep_for(chrono::microseconds(1000)); //500 for 2 hertz //100 for 10 hertz // 10 for 100 hertz //
            } 
        }
        logFile.close();
        exitFlagTime = false;
    }
    exitFlag.store(false);
}
void capacityTest(){
    char answer;
    cout << " Is the water pump connected properly? y\n" << endl;
    cin >> answer;
    cout << " Is the flow sensor connected properly? y\n" << endl;
    cin >> answer;
    cout << "The test will start now" << endl;
    while(servoController.mpc_soc > 1) {
        gpioHardwarePWM(LED_PIN, 50, 100000);  //1000000
        
        //  for (int i = 0; i <= FADE_STEPS; ++i) {
        //      gpioHardwarePWM(LED_PIN, 50, i * (1000000 / FADE_STEPS));
        //     usleep(10000);  // 10ms delay between steps
        // }
        // for (int i = FADE_STEPS; i >= 0; --i) {
        //     gpioHardwarePWM(LED_PIN, 50, i * (1000000 / FADE_STEPS));
        //     usleep(10000);  // 10ms delay between steps
        // }

    }
    cout << "The test has ended." << endl;
}
void readFlow() {
    // Infinite loop for continuous frequency calculation
    while (!exitFlag.load()) {
        auto currentTime = std::chrono::steady_clock::now();
        auto lastPulse = lastPulseTime.load();

        // Calculate time duration since the last pulse
        auto durationSinceLastPulse = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - lastPulse).count();

        // Check if the duration is greater than the maximum allowed duration
        if (durationSinceLastPulse > MAX_DURATION_MICROSECONDS) {
            // Protect shared variables with a lock
            std::lock_guard<std::mutex> lock(flowMutex);
            flowFrequency = 0.0;
        }

        // Get the frequency and calculate the flow rate
        double frequency;
        {
            // Protect shared variables with a lock
            std::lock_guard<std::mutex> lock(flowMutex);
            frequency = flowFrequency.load();
        }

        calculateFlowRate(frequency);
        //std::cout << "Current Frequency: " << static_cast<int>(frequency) << " Hz, Flow Rate: " << flowRate << " lpm" << std::endl;

        // Sleep for a short duration before the next iteration
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
void flowCallback(int gpio, int level, uint32_t tick) {
    if (level == 1) {
        auto currentTime = std::chrono::steady_clock::now();
        auto previousTime = lastPulseTime.exchange(currentTime);

        // Calculate time duration between two pulses
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - previousTime).count();
        
        // Calculate frequency based on the time duration
        flowFrequency = 1.0e6 / duration;
    }
}
void calculateFlowRate(double frequency) {
    // Linear relationship between frequency and flow rate
     flowRate = FREQ_TO_FLOW_SLOPE * frequency;
}

void INIT_Axis1() {
    //servoController.frame.can_dlc = 8;
	vector<Message> messages = initMessages();
	int bericht[] = {0, 1, 2, 3, 5, 6, 7};
	for (int i = 0; i < 7; i++) {
		this_thread::sleep_for(chrono::milliseconds(100));
        servoController.frame1.can_id = 0x601;
		Message result = getMessage(messages, bericht[i]);
		for (int i = 0; i < 8; i++) {
		servoController.frame1.data[i] = result.data[i];
		}
		servoController.sendCAN();

        printf("%03X ",servoController.frame1.can_id);
                for (int i = 0; i < servoController.frame1.can_dlc; i++) {
                    printf(" %02X",servoController.frame1.data[i]);
                    
                }
                cout << "\n";
	}
}

void INIT_Axis2() {
    //servoController.frame1.can_dlc = 8;
	vector<Message> messages = initMessages();
	int bericht[] = {0, 1, 2, 4, 8, 10};
	for (int i = 0; i < 6; i++) {
		this_thread::sleep_for(chrono::milliseconds(100));
		servoController.frame1.can_id = 0x602;
		Message result = getMessage(messages, bericht[i]);
		for (int i = 0; i < 8; i++) {
		servoController.frame1.data[i] = result.data[i];
		}
		servoController.sendCAN();

        printf("%03X ",servoController.frame1.can_id);
                for (int i = 0; i < servoController.frame1.can_dlc; i++) {
                    printf(" %02X",servoController.frame1.data[i]);
                    
                }
                cout << "\n";
	}
}

void Start_Motion() {
	//motion_start = true;
	vector<Message> messages = initMessages();
	int ID[] = {0x601, 0x602};
	for (int i = 0; i < 2; i++) {
		servoController.frame1.can_id = ID[i];
		Message result = getMessage(messages, 11);
		for (int i = 0; i < 8; i++) {
			servoController.frame1.data[i] = result.data[i];
			}
		servoController.sendCAN();
	}
}

void Stop_Motion() {
	//motion_start = false;
	vector<Message> messages = initMessages();
	int ID[] = {0x601, 0x602};
	for (int i = 0; i < 2; i++) {
		servoController.frame1.can_id = ID[i];
		Message result = getMessage(messages, 12);
		for (int i = 0; i < 8; i++) {
			servoController.frame1.data[i] = result.data[i];
			}
		servoController.sendCAN();
	}
}


