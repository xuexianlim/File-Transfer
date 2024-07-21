#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <cstring>
#include <atomic>
#include <sstream>
#include "winch.h"
#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"
#include "plog/Appenders/ColorConsoleAppender.h"
#include "DynamixelServo.h"

std::atomic<bool> quit(false);    // signal flag

void got_signal(int)
{
    // Signal handler function.
    // Set the flag and return.
    // Never do real work inside this function.
    // See also: man 7 signal-safety
    quit.store(true);
}

std::string datetime()
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%Y_%m_%d_%H-%M-%S",timeinfo);
    return std::string(buffer);
}

std::vector<std::string>
splitString(const std::string& target, char c) {
    std::string temp;
    std::stringstream stringstream { target };
    std::vector<std::string> result;

    while (std::getline(stringstream, temp, c)) {
        result.push_back(temp);
    }

    return result;
}

void handle_input() {
    std::string passed_string;
    double m;
    PLOG_INFO << "Your input:";
    std::getline(std::cin, passed_string);
    if (passed_string.empty()) {
        return;
    }
    std::vector<std::string> parsed_command = splitString(passed_string, ' ');
    // stop
    if (parsed_command[0] == "s") {
        PLOG.printf("Hello")
    }

}

// General Settings
struct conf {
    // Main Dynamixel Settings
    int DYNAMIXEL_DIRECTION = 1;           // 1: old WB, -1: new wb
    std::string DYNAMIXEL_PORT = "";       // "" for reg-ex check in /dev/serial/by-id. Otherwise port of the U2D2 board, /dev/ttyUSBX or /dev/serial/by-id/...
    int DYNAMIXEL_ID = 1;                  // ID of the Dynamixel motor
    int DYNAMIXEL_BAUDRATE = 1000000;      // Baudrate of the Dynamixel motor
    int DYNAMIXEL_OPERATING_MODE = 4;      // Operating mode the Dynamixel motor
    int DYNAMIXEL_VELOCITY_LIMIT = 1620;   // XL330-M077T: 1620, XL330-M288T: 445, XL430-W240T: 306

    // Additional Dynamixel Settings
    int MAX_CURRENT = 1100;                // Max current after which to stop the motor to avoid overload

    // Additional Dynamixel Settings for (extended) Position Control Mode
    int POSITION_THRESHOLD = 100;          // Position threshold
    int PROFILE_ACCELERATION = 150;        // Default acceleration profile, 0 = maximum
    int PROFILE_VELOCITY = 32767;          // Default velocity for raising / lowering, 0 = maximum
    int POSITION_GAIN_P = 400;             // Default: XL330-M077T - 400
    int POSITION_GAIN_I = 50;              // Default: XL330-M077T - 0
    int POSITION_GAIN_D = 400;             // Default: XL330-M077T - 400
};
conf config;

// Dynamixel
dynamixel_servo::DynamixelServo dyn;    // Dynamixel servo object
dynamixel::PortHandler * portHandler = nullptr;   // portHandler object

int main(int argc, char* argv[]) {
    // Signal handler
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = got_signal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL);

    // Initialize General logger
    static plog::RollingFileAppender<plog::WinchFormatter> fileAppender("log_info.csv", 0, 0);
    static plog::ColorConsoleAppender<plog::WinchFormatter> ColorConsoleAppender; // log to console with color code
    // static plog::ConsoleAppender<plog::TxtFormatter> ConsoleAppender; // log to Console without color code
    plog::init(plog::debug, &fileAppender).addAppender(&ColorConsoleAppender);

//    // Initialize data logger
//    std::string logname = datetime() + ".csv";
//    PLOG_INFO << "Logname:" << logname;
//    static plog::RollingFileAppender<plog::WinchFormatter> fileAppenderData(logname.c_str(), 0, 0);
//    plog::init<1>(plog::info, &fileAppenderData);

    // Welcome
    PLOG_INFO << "Welcome to the dynamixel sdk example!";

    PLOG_INFO.printf("--Initializing--");
    // Define Dynamixel Answer (for error handling)
    int dynAnswer;

    if (config.DYNAMIXEL_PORT.empty()) {
        // Regex check of U2D2 board
        PLOG_INFO.printf("No Dynamixel port supplied. Checking port.");
        std::string path = "/dev/serial/by-id";
        std::regex regex_u2d2("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_.*");
        std::regex regex_uart("/dev/serial/by-id/usb-Silicon_Labs_CP2102_.*");
        for (const auto & entry : std::filesystem::directory_iterator(path)) {
            if ( std::regex_match( entry.path().c_str(), regex_u2d2) ) {
                PLOG_INFO.printf("Found Dynamixel (U2D2) at port: %s.", entry.path().c_str() );
                config.DYNAMIXEL_PORT = entry.path().c_str();
                break;
            }
            else if ( std::regex_match( entry.path().c_str(), regex_uart) ) {
                PLOG_INFO.printf("Found USB-to-UART bridge at port: %s.", entry.path().c_str() );
                config.DYNAMIXEL_PORT = entry.path().c_str();
                break;
            }
        }
        if (config.DYNAMIXEL_PORT.empty()) {
            PLOG_INFO.printf("No U2D2 nor UART-to-USB bridge found.");
            return EXIT_FAILURE;
        }
    }

    // Check if the portHandler already exists
    if (portHandler != nullptr) {
        dyn = dynamixel_servo::DynamixelServo(portHandler, state.dynamixelBaudRate, config.DYNAMIXEL_ID, 2.0, config.DYNAMIXEL_VELOCITY_LIMIT);
    }
    else{
        dyn = dynamixel_servo::DynamixelServo(config.DYNAMIXEL_PORT, state.dynamixelBaudRate, config.DYNAMIXEL_ID, 2.0, config
                .DYNAMIXEL_VELOCITY_LIMIT);
    }

    // Connect to the Dynamixel, also checks the hardware status internally
    dynAnswer = dyn.connect();

//
    int i = 0;
    while ( !quit.load() ) {
        handle_input();
    }
    return 0;
}