//
// Created by florian on 31.05.22.
// Adapted by Steffen - April 2023
//

#include "stdio.h"
#include "math.h"
#include "DynamixelServo.h"
#include <iostream>
#include <string.h>
#include <bitset>

// Control table address
#define ADDR_MODEL_NUMBER       0   // 2 Byte, 1070: XC430-W150, 1190: XL330-M077T
#define ADDR_BAUD_RATE          1   // 1 Byte, 0-6, 0: 9600, 1: 57600, 2: 112000, 3: 1M, 4: 2M, 5: 3M,6: 4M, 7: 4.5M
#define ADDR_OPERATING_MODE     11  // 1 byte, uint8; 0: Current control, 1: Velocity control, 3: Position Control,
// 4: Extended Position Control, 5: Current-based Position, 16: PWM control
#define ADDR_HOMING_OFFSET      20  // 4 bytes, int32_t; -1,044,479 ~ 1,044,479
#define ADDR_MOVING_THRESHOLD    24  // 4 bytes, uint32; 0 ~ 1,023, 0.229 [rev/min], initial value: 10
#define ADDR_SPEED_LIMIT        44  // 4 bytes, uint16; Maximum allowed speed
#define ADDR_TORQUE_ENABLE      64  // 1 byte, uint8; 0: Disable, 1: Enable
#define ADDR_LED_ENABLE         65  // 1 byte, uint8; 0: Disable, 1: Enable
#define ADDR_HARDWARE_STATUS    70  // 1 byte, uint8; Bit 0: Input voltage error, Bit 2: Overheating Error, Bit 4: Electrical Shock Error, Bit 5: Overload error
#define ADDR_POSITION_GAIN_D    80  // 2 byte, uint16, 0~16383
#define ADDR_POSITION_GAIN_I    82  // 2 byte, uint16, 0~16383
#define ADDR_POSITION_GAIN_P    84  // 2 byte, uint16, 0~16383
#define ADDR_BUS_WATCHDOG       90  // 1 byte, int8; 0: No watchdog, 1-127 Watchdog activated (unit 20ms), -1 watchdog error status
#define ADDR_GOAL_VELOCITY      104 // 4 bytes, int32; Unit: 0.229 rev/min
#define ADDR_PROFILE_ACCELERATION   108 // 4 bytes, int32, Unit: 214.577 rev/min^2 (0 ~ 32767)
#define ADDR_PROFILE_VELOCITY   112 // 4 bytes, int32, Unit: 0.229 rev/min (0 ~ 32767)
#define ADDR_GOAL_POSITION      116 // 4 bytes, int32;
#define ADDR_MOVING             122 // 1 byte, uint8; 0: Stopped, 1: Moving
#define ADDR_MOVING_STATUS      123 // 1 byte, uint8;
#define ADDR_PRESENT_PWM        124 // 2 bytes, int16; Percentage, Unit: about 0.113% per LSB
#define ADDR_PRESENT_CURRENT    126 // 2 bytes, int16; Unit: 1mA per LSB
#define ADDR_PRESENT_VELOCITY   128 // 4 bytes, int32; Unit 0.229 rev/min per LSB
#define ADDR_PRESENT_POSITION   132 // 4 bytes, int32; Pulse, ~0.088 deg/Pulse, 1 rev 0 - 4095
#define ADDR_PRESENT_VOLTAGE    144 // 2 bytes, uint16; Unit: 0.1V per LSB
#define ADDR_PRESENT_TEMP       146 // 2 bytes, uint16; Unit: 1°C per LSB

#define MIN_POSITION_VALUE      0     // Min position value of Dynamixel Motors
#define MAX_POSITION_VALUE      4095  // Max position value of Dynamixel Motors

#define MIN_EXTENDED_POSITION_VALUE   -1048575     // 4 bytes, int32 ?, 0.088 DEG per UNIT
#define MAX_EXTENDED_POSITION_VALUE    1048575     // 4 bytes, int32 ?, 0.088 DEG per UNIT

#define PWM_UNIT        ((float) 0.113)     // %
#define CURRENT_UNIT    ((float) 0.001)     // A (1 mA)
#define VELOCITY_UNIT   ((float) 0.02398)   // rad/s (0.229 rev/min)
#define POSITION_UNIT   ((float) 0.001536)  // rad (0.088 deg)
#define VOLTAGE_UNIT    ((float) 0.1)       // V
#define TEMP_UNIT       ((float) 1)         // °C

#define CHECK_BIT(var,pos) (((var)>>(pos)) & 1)
// Define ERRORS
#define ERRBIT_ALERT 128 // Used to check for (other) error with hardware error https://github
// .com/ROBOTIS-GIT/DynamixelSDK/blob/6ae113ab5a2b1133ee081c8d110be62a098c051c/c%2B%2B/src/dynamixel_sdk/protocol2_packet_handler.cpp#L110

namespace dynamixel_servo {
    DynamixelServo::DynamixelServo(std::string connection_device, int baudrate, int ID, float protocol, uint32_t max_speedLimit) :
    connection_device(connection_device.c_str()),
    baudrate(baudrate),
    ID(ID),
    protocol(protocol),
    max_speedLimit(max_speedLimit),
    portInUse(false)
    {
        // Define connection depending on the inputs
        portHandler = dynamixel::PortHandler::getPortHandler(this->connection_device);
        packetHandler = dynamixel::PacketHandler::getPacketHandler(this->protocol);
    }

    DynamixelServo::DynamixelServo(dynamixel::PortHandler * portHandler, int baudrate, int ID, float protocol, uint32_t max_speedLimit) :
        portHandler(portHandler),
        baudrate(baudrate),
        ID(ID),
        protocol(protocol),
        max_speedLimit(max_speedLimit),
        portInUse(false)
    {
        // Define connection depending on the inputs
        packetHandler = dynamixel::PacketHandler::getPacketHandler(this->protocol);
    }

    DynamixelServo::~DynamixelServo() {
        // Close the connection
        disconnect();
    }

    int DynamixelServo::connect() {
        // Initialize errorcode
        int dynAnswer;
        // Open the connection
        if (!portHandler->openPort()) {
            PLOG_ERROR.printf( "[DYN ID:%03d] Dynamixel Servo failed to open the specified Port: %s!", ID,
                    connection_device);
            return ERROR_DYN_PORT;
        }
        //TODO: Handle what is happening if port is already opened -- CHECK IF DYNAMIXEL IS STILL REACHABLE?

        if (!portHandler->setBaudRate(this->baudrate)) {
            PLOG_ERROR.printf( "[DYN ID:%03d] Dynamixel Servo failed to set the baudrate: %d!", ID, baudrate);
            return ERROR_DYN;
        }

        // Check Hardware Status to check if all is fine
        dynAnswer = getHardwareStatus();
        if (dynAnswer == ERROR_DYN_COM){
            PLOG_ERROR.printf("Cannot find Dynamixel with baudrate specified in config: %d", baudrate);
            // Check for other BAUDRATES
            int baudrates[4] = {9600, 57600, 115200, 1000000};
            for (int rate : baudrates) {
                PLOG_INFO.printf("Trying other baudrate: %d", rate);
                portHandler->setBaudRate(rate);
                dynAnswer = getHardwareStatus();
                // Break if other baudrate works
                if (dynAnswer == EXIT_SUCCESS) {
                    PLOG_INFO.printf("Found Dynamixel with baudrate: %d", rate);
                    break;
                }
            }
            return dynAnswer;
//            // Reboot if hardware error
//            reboot();
//            // Check Hardware Status again
//            dynAnswer = getHardwareStatus();
//            if (dynAnswer != EXIT_SUCCESS){
//                // Exit if it still persists
//                return dynAnswer;
//            }
        }

        dynAnswer = getHardwareStatus();
        if (dynAnswer != EXIT_SUCCESS) {
            return dynAnswer;
        }

//        // Disable torque if it had been enabled
//        // TODO Check if this is necessary - Usually we want torque to be enabled at startup
//        dynAnswer = enableTorque(false);
//        if (dynAnswer != EXIT_SUCCESS) {
//            return dynAnswer;
//        }

//        // Get the velocity limit
//        speedLimit = getVelocityLimit();
//        if (speedLimit == 0) { // TODO:: Use exception rather than 0 here.
//            return dynAnswer;
//        }
        return EXIT_SUCCESS;
    }


    int DynamixelServo::disconnect() {
        portHandler->closePort();
        return EXIT_SUCCESS;
    }

    int16_t DynamixelServo::getModelNumber() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Current Value of X series is 2 byte data.
        int16_t model_number = 0;

        // Set portInUse flag
        portInUse = true;
        // Read Present Current (length : 2 bytes)
        dxl_comm_result = packetHandler->read2ByteTxRx(
                portHandler, ID, ADDR_MODEL_NUMBER, (uint16_t *) &model_number, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                               packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel model number! Result: %s", ID,
                               packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        // Unset portInUse flag
        portInUse = false;

        return model_number;
    }

    int DynamixelServo::setBaudRate(uint8_t baud_rate) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Set portInUse flag
        portInUse = true;
        // Write Mode (1 Byte)
        dxl_comm_result = packetHandler->write1ByteTxRx(
                portHandler, ID, ADDR_BAUD_RATE, baud_rate, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                               packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to change dynamixel baud rate to %i! Result: %s", ID, baud_rate,
                               packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        // Set portInUse flag
        portInUse = false;
        return EXIT_SUCCESS;
    }


    int DynamixelServo::setOperatingMode(uint8_t mode) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Set portInUse flag
        portInUse = true;
        // Write Mode (1 Byte)
        dxl_comm_result = packetHandler->write1ByteTxRx(
                portHandler, ID, ADDR_OPERATING_MODE, mode, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to set dynamixel servo mode! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        // Set portInUse flag
        portInUse = false;

        return EXIT_SUCCESS;
    }


    uint8_t DynamixelServo::getOperatingMode() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        uint8_t mode = 10; // 10 means error

        // Set portInUse flag
        portInUse = true;
        // Write Mode (1 Byte)
        dxl_comm_result = packetHandler->read1ByteTxRx(
                portHandler, ID, ADDR_OPERATING_MODE, &mode, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel operating mode! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            //TODO: Throw proper exception
            return 9;
        }
        // Set portInUse flag
        portInUse = false;
        PLOG_INFO.printf( "[DYN ID:%03d] Current dynamixel operating mode is: %d", ID, mode);
        return mode;
    }

    uint8_t DynamixelServo::getMoving() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        uint8_t moving = 0; // 10 means error

        // Set portInUse flag
        portInUse = true;
        // Write Mode (1 Byte)
        dxl_comm_result = packetHandler->read1ByteTxRx(
                portHandler, ID, ADDR_MOVING, &moving, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                               packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel moving value! Result: %s", ID,
                               packetHandler->getRxPacketError(dxl_error));
            //TODO: Throw proper exception
            return 9;
        }
        // Set portInUse flag
        portInUse = false;
        return moving;
    }

    uint8_t DynamixelServo::getMovingStatus() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        uint8_t movingStatus = 0; // 10 means error

        // Set portInUse flag
        portInUse = true;
        // Write Mode (1 Byte)
        dxl_comm_result = packetHandler->read1ByteTxRx(
                portHandler, ID, ADDR_MOVING, &movingStatus, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                               packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel moving status! Result: %s", ID,
                               packetHandler->getRxPacketError(dxl_error));
            //TODO: Throw proper exception
            return 9;
        }
        // Set portInUse flag
        portInUse = false;
        return movingStatus;
    }

    int DynamixelServo::setHomingOffset(int32_t offset) {
        // Saturate limit to the maximum value
        if (abs(offset) > 1044479) {
            return ERROR_DYN_BOUNDS;
        }
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Set portInUse flag
        portInUse = true;
        // Write Speed limit (4 Byte)
        dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler, ID, ADDR_HOMING_OFFSET, offset, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                               packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to set dynamixel servo homing offset! Result: %s", ID,
                               packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        // Set portInUse flag
        portInUse = false;

        homingOffset = offset;
        return EXIT_SUCCESS;
    }

    int DynamixelServo::setMovingThreshold(int32_t threshold) {
        // Saturate limit to the maximum value
        if (abs(threshold) > 1023) {
            return ERROR_DYN_BOUNDS;
        }
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Set portInUse flag
        portInUse = true;
        // Write Speed limit (4 Byte)
        dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler, ID, ADDR_MOVING_THRESHOLD, threshold, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                               packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to set dynamixel servo motion threshold! Result: %s", ID,
                               packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        // Set portInUse flag
        portInUse = false;

        movingThreshold = threshold;
        return EXIT_SUCCESS;
    }

    uint32_t DynamixelServo::getMovingThreshold() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        uint32_t movingThreshold_tmp = 0;

        // Set portInUse flag
        portInUse = true;
        // Read moving threshold (4 Byte)
        dxl_comm_result = packetHandler->read4ByteTxRx(
                portHandler, ID, ADDR_MOVING_THRESHOLD, &movingThreshold_tmp, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                               packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel moving threshold! Result: %s", ID,
                               packetHandler->getRxPacketError(dxl_error));
            //TODO: Throw proper exception
            return 0;
        }
        // Set portInUse flag
        portInUse = false;

        movingThreshold = movingThreshold_tmp;
        return movingThreshold_tmp;
    }

    int DynamixelServo::setVelocityLimit(uint32_t limit) {
        // Saturate limit to the maximum value
        if (limit > max_speedLimit) {
            limit = max_speedLimit;
        }
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Set portInUse flag
        portInUse = true;
        // Write Speed limit (4 Byte)
        dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler, ID, ADDR_SPEED_LIMIT, limit, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to set dynamixel servo speed limit! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }

        speedLimit = getVelocityLimit();
        // Set portInUse flag
        portInUse = false;
        return EXIT_SUCCESS;
    }

    int DynamixelServo::setProfileVelocity(uint32_t limit) {
        // Saturate limit to the maximum value
        if (limit > 32767) {
            PLOG_ERROR.printf("[DYN ID:%03d] Profile velocity out of bounds: %d", ID, limit);
            return ERROR_DYN_BOUNDS;
        }

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Write Speed limit (4 Byte)
        dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler, ID, ADDR_PROFILE_VELOCITY, limit, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to set dynamixel servo profile velocity! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        return EXIT_SUCCESS;
    }

    uint32_t DynamixelServo::getProfileVelocity() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        uint32_t profileVelocity_tmp = 0;

        // Read profile velocity (4 Byte)
        dxl_comm_result = packetHandler->read4ByteTxRx(
                portHandler, ID, ADDR_PROFILE_VELOCITY, &profileVelocity_tmp, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                               packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel profile velocity! Result: %s", ID,
                               packetHandler->getRxPacketError(dxl_error));
            //TODO: Throw proper exception
            return 0;
        }
        return profileVelocity_tmp;
    }

    int DynamixelServo::setProfileAcceleration(uint32_t limit) {
        // Saturate limit to the maximum value
        if (limit > 32767) {
            limit = 32767;
        }

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Write Speed limit (4 Byte)
        dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler, ID, ADDR_PROFILE_ACCELERATION, limit, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to set dynamixel servo profile acceleration! Result: %s",
                    ID,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        return EXIT_SUCCESS;
    }

    uint32_t DynamixelServo::getProfileAcceleration() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        uint32_t profileAcceleration_tmp = 0;

        // Read profile velocity (4 Byte)
        dxl_comm_result = packetHandler->read4ByteTxRx(
                portHandler, ID, ADDR_PROFILE_ACCELERATION, &profileAcceleration_tmp, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                               packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel profile velocity! Result: %s", ID,
                               packetHandler->getRxPacketError(dxl_error));
            //TODO: Throw proper exception
            return 0;
        }
        return profileAcceleration_tmp;
    }

    int DynamixelServo::setGain(int addr, uint16_t gain) {
        // Saturate limit to the maximum value
        if (gain > 16383) {
            gain = 16383;
        }

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Write Speed limit (4 Byte)
        dxl_comm_result = packetHandler->write2ByteTxRx(
                portHandler, ID, addr, gain, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to set dynamixel gain in address %d! Result: %s", ID, addr,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        return EXIT_SUCCESS;
    }

    int DynamixelServo::setPositionPGain(uint16_t gain) {
        return setGain(ADDR_POSITION_GAIN_P, gain);
    }

    int DynamixelServo::setPositionIGain(uint16_t gain) {
        return setGain(ADDR_POSITION_GAIN_I, gain);
    }

    int DynamixelServo::setPositionDGain(uint16_t gain) {
        return setGain(ADDR_POSITION_GAIN_D, gain);
    }



    uint32_t DynamixelServo::getVelocityLimit() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        uint32_t speedLimit_tmp = 0;

        // Set portInUse flag
        portInUse = true;
        // Read Speed limit (4 Byte)
        dxl_comm_result = packetHandler->read4ByteTxRx(
                portHandler, ID, ADDR_SPEED_LIMIT, &speedLimit_tmp, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel servo speed limit! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            //TODO: Throw proper exception
            return 0;
        }
        // Set portInUse flag
        portInUse = false;
        return speedLimit_tmp;
    }

    int32_t DynamixelServo::getVelocity() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        uint32_t speed_tmp = 0;

        // Set portInUse flag
        portInUse = true;
        // Read Speed limit (4 Byte)
        dxl_comm_result = packetHandler->read4ByteTxRx(
                portHandler, ID, ADDR_PRESENT_VELOCITY, &speed_tmp, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel current velocity! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            //TODO: Throw proper exception
            return 0;
        }
        // Set portInUse flag
        portInUse = false;
        return speed_tmp;
    }

    int DynamixelServo::enableTorque(bool enable) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Set portInUse flag
        portInUse = true;
        // Write Enable Torque (1 Byte)
        dxl_comm_result = packetHandler->write1ByteTxRx(
                portHandler, ID, ADDR_TORQUE_ENABLE, (uint8_t) enable, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to set dynamixel servo torque enable state! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        // Set portInUse flag
        portInUse = false;

        return EXIT_SUCCESS;
    }

    int DynamixelServo::enableLED(bool enable) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Set portInUse flag
        portInUse = true;
        // Write Enable LED (1 Byte)
        dxl_comm_result = packetHandler->write1ByteTxRx(
                portHandler, ID, ADDR_LED_ENABLE, (uint8_t) enable, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to set dynamixel servo LED enable state! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        // Set portInUse flag
        portInUse = false;

        return EXIT_SUCCESS;
    }

// SHOULD NOT BE USED AT THE MOMENT
//    int DynamixelServo::setSpeed(float speed_in_rad_per_s) {
//        int32_t velocity = (int32_t) speed_in_rad_per_s/VELOCITY_UNIT;
//        return setVelocity(velocity);
//    }

    int DynamixelServo::setVelocity(int32_t velocity) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        if (abs(velocity) > speedLimit){
            velocity = speedLimit * (1 - 2*(velocity < 0));
            PLOG_ERROR.printf( "[DYN ID:%03d] Velocity beyond min/max allowable velocity limit.", ID);
            return ERROR_DYN_BOUNDS;
        }

        // Set portInUse flag
        portInUse = true;
        // Write Velocity (4 Bytes)
        dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler, ID, ADDR_GOAL_VELOCITY, velocity, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to set dynamixel servo velocity! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        // Set portInUse flag
        portInUse = false;

        return EXIT_SUCCESS;
    }

    int DynamixelServo::setPosition(int32_t positionInUnits) {
        /*
         * Set absolute goal position of Dynamixel when in Position Mode
         *
         * @params positionInRadians Absolute Goal Position in Radians
         * @return bool of success / fail
         *
         */
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        int32_t goal_position = (int32_t) positionInUnits; // /POSITION_UNIT;

        // Catch position value out of bounds
        if ((goal_position < MIN_POSITION_VALUE) || (goal_position > MAX_POSITION_VALUE)) {
            PLOG_ERROR.printf( "[DYN ID:%03d] Goal Position beyond min/max allowable position value.", ID);
            return ERROR_DYN_BOUNDS;
        }

        // Set portInUse flag
        portInUse = true;
        // Write goal position
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, ID, ADDR_GOAL_POSITION, goal_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to set dynamixel goal position! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        // Set portInUse flag
        portInUse = false;

        return EXIT_SUCCESS;
    }

    int DynamixelServo::setExtendedPosition(int32_t positionInUnits) {
        /*
         * Set extended goal position of Dynamixel when in Extended Position Mode
         *
         * @params positionInRadians Extended Goal Position in Radians
         * @return bool of success / fail
         *
         */
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        int32_t goal_position = (int32_t) positionInUnits; // InRadians/POSITION_UNIT;

        // Catch position value out of bounds
        if ((goal_position < MIN_EXTENDED_POSITION_VALUE) || (goal_position > MAX_EXTENDED_POSITION_VALUE)) {
            PLOG_ERROR.printf( "[DYN ID:%03d] Goal Position beyond min/max allowable position value.", ID);
            return ERROR_DYN_BOUNDS;
        }

        // Set portInUse flag
        portInUse = true;
        // Write goal position
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, ID, ADDR_GOAL_POSITION, goal_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to set dynamixel goal position! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        // Set portInUse flag
        portInUse = false;

        return EXIT_SUCCESS;
    }

    uint8_t DynamixelServo::getHardwareStatus() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Hardware status Value of X series is 1 byte data.
        uint8_t status = 0;

        // Set portInUse flag
        portInUse = true;
        // Read hardware status (length : 1 byte)
        dxl_comm_result = packetHandler->read1ByteTxRx(
                portHandler, ID, ADDR_HARDWARE_STATUS, &status, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        // Set portInUse flag
        portInUse = false;

// dxl_error is set when a HW error occurs, so we ignore it here and get the actual hardware error.
//        else if (dxl_error != 0)
//        {
//            // Internal error in Dynamixel
//            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel servo hardware status! Result: %s", ID,
//                    packetHandler->getRxPacketError(dxl_error));
//            return ERROR_DYN;
//        }

        const char *error = nullptr;
        if (CHECK_BIT(status, 0)) {
            error="Input Voltage Error";
            return ERROR_DYN_VOLTAGE;
        }
        if (CHECK_BIT(status, 2)) {
            error="Overheating Error";
            return ERROR_DYN_OVERHEATING;
        }
        if (CHECK_BIT(status, 4)) {
            error="Electric Shock Error";
            return ERROR_DYN_ESHOCK;
        }
        if (CHECK_BIT(status, 5)) {
            error="Overload Error";
            return ERROR_DYN_OVERLOAD;
        }
        if (error) {
            PLOG_ERROR.printf( "[DYN ID:%03d] Dynamixel Hardware Error: %s", ID, error);
        }
        return status;

    }

    int DynamixelServo::setWatchdog(int8_t timeout) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Set portInUse flag
        portInUse = true;
        // Write Watchdog (1 Byte)
        dxl_comm_result = packetHandler->write1ByteTxRx(
                portHandler, ID, ADDR_BUS_WATCHDOG, timeout, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to set dynamixel servo watchdog state! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        // Set portInUse flag
        portInUse = false;

        return EXIT_SUCCESS;
    }

    int DynamixelServo::getWatchdogError() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Hardware status Value of X series is 1 byte data.
        uint8_t watchdogError = 0;

        // Set portInUse flag
        portInUse = true;
        // Read hardware status (length : 1 byte)
        dxl_comm_result = packetHandler->read1ByteTxRx(
                portHandler, ID, ADDR_BUS_WATCHDOG, &watchdogError, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel servo watchdog status! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        // Set portInUse flag
        portInUse = false;

        return watchdogError == -1;
    }

    float DynamixelServo::getPWM() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Current Value of X series is 2 byte data.
        int16_t pwm = 0;

        // Set portInUse flag
        portInUse = true;
        // Read Present PWM (length : 2 bytes)
        dxl_comm_result = packetHandler->read2ByteTxRx(
                portHandler, ID, ADDR_PRESENT_PWM, (uint16_t *) &pwm, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return NAN;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel servo PWM! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return NAN;
        }
        // Set portInUse flag
        portInUse = false;

        return pwm*PWM_UNIT;
    }

    float DynamixelServo::getCurrent() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Current Value of X series is 2 byte data.
        int16_t current = 0;

        // Set portInUse flag
        portInUse = true;
        // Read Present Current (length : 2 bytes)
        dxl_comm_result = packetHandler->read2ByteTxRx(
                portHandler, ID, ADDR_PRESENT_CURRENT, (uint16_t *) &current, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return NAN;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel servo current! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return NAN;
        }
        // Set portInUse flag
        portInUse = false;

        return current*CURRENT_UNIT;
    }

    float DynamixelServo::getSpeed() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Velocity Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
        int32_t velocity = 0;

        // Set portInUse flag
        portInUse = true;
        // Read Present Velocity (length : 4 bytes) and Convert uint32 -> int32
        dxl_comm_result = packetHandler->read4ByteTxRx(
                portHandler, ID, ADDR_PRESENT_VELOCITY, (uint32_t *) &velocity, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return NAN;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel servo velocity! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return NAN;
        }
        // Set portInUse flag
        portInUse = false;

        return velocity*VELOCITY_UNIT;
    }

    float DynamixelServo::getPosition() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
        int32_t position = 0;

        // Set portInUse flag
        portInUse = true;
        // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
        dxl_comm_result = packetHandler->read4ByteTxRx(
                portHandler, ID, ADDR_PRESENT_POSITION, (uint32_t *) &position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return NAN;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel servo position! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return NAN;
        }
        // Set portInUse flag
        portInUse = false;

        return position; //*POSITION_UNIT;
    }

    float DynamixelServo::getVoltage() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Current Value of X series is 2 byte data.
        uint16_t voltage = 0;

        // Set portInUse flag
        portInUse = true;
        // Read Present Voltage (length : 2 bytes)
        dxl_comm_result = packetHandler->read2ByteTxRx(
                portHandler, ID, ADDR_PRESENT_VOLTAGE, &voltage, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return NAN;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel servo voltage! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return NAN;
        }
        // Set portInUse flag
        portInUse = false;

        return voltage*VOLTAGE_UNIT;
    }

    float DynamixelServo::getTemperature() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Current Value of X series is 2 byte data.
        uint8_t temp = 0;

        // Set portInUse flag
        portInUse = true;
        // Read Present Temperature (length : 1 byte)
        dxl_comm_result = packetHandler->read1ByteTxRx(
                portHandler, ID, ADDR_PRESENT_TEMP, &temp, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return NAN;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to get dynamixel servo temperature! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return NAN;
        }
        // Set portInUse flag
        portInUse = false;

        return temp*TEMP_UNIT;
    }

    int DynamixelServo::clearMultiTurn() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Set portInUse flag
        portInUse = true;
        // Clear Multi-Turn Information
        dxl_comm_result = packetHandler->clearMultiTurn(portHandler, ID, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // Communication error
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID,
                    packetHandler->getTxRxResult(dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if (dxl_error != 0)
        {
            // Internal error in Dynamixel
            PLOG_ERROR.printf( "[DYN ID:%03d] Failed to clear multi turn information! Result: %s", ID,
                    packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN;
        }
        // Set portInUse flag
        portInUse = false;

        return EXIT_SUCCESS;
    }

    int DynamixelServo::reboot() {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Set portInUse flag
        portInUse = true;
        // Try reboot
        // Dynamixel LED will flicker while it reboots
        dxl_comm_result = packetHandler->reboot(portHandler, ID, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            PLOG_ERROR.printf( "[DYN ID:%03d] Communication error: %s", ID, packetHandler->getTxRxResult
            (dxl_comm_result));
            return ERROR_DYN_COM;
        }
        else if ((dxl_error & ~ERRBIT_ALERT) != 0) // https://github
            // .com/ROBOTIS-GIT/DynamixelSDK/blob/6ae113ab5a2b1133ee081c8d110be62a098c051c/c%2B%2B/src/dynamixel_sdk/protocol2_packet_handler.cpp#L110
        {
            PLOG_ERROR.printf("[DYN ID:%03d] Reboot error: %s", ID, packetHandler->getRxPacketError(dxl_error));
            return ERROR_DYN_REBOOT;
        }
        // Set portInUse flag
        portInUse = false;

        PLOG_INFO.printf("[DYN ID:%03d] Reboot Succeeded", ID);
        return EXIT_SUCCESS;
    }

    dynamixel::PortHandler *DynamixelServo::getPortHandler() {
        return portHandler;
    }
} // dynamixel_servo