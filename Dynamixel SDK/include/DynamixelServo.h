//
// Created by florian on 31.05.22.
//

#ifndef SRC_DYNAMIXEL_SERVO_H
#define SRC_DYNAMIXEL_SERVO_H

#include <cstdint>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "errorCodes.h"
#include <plog/Log.h>

namespace dynamixel_servo {

    class DynamixelServo {
    public:
        DynamixelServo(std::string connection_device = "/dev/ttyUSB0",
                       int baudrate = 57600,
                       int ID = 1,
                       float protocol = 2.0,
                       uint32_t max_speedLimit = 2047);

        DynamixelServo(dynamixel::PortHandler * portHandler,
                       int baudrate = 57600,
                       int ID = 1,
                       float protocol = 2.0,
                       uint32_t max_speedLimit = 2047);

        ~DynamixelServo();

        int connect();
        int disconnect();
        int16_t getModelNumber();
        int setBaudRate(uint8_t baud_rate);
        int setOperatingMode(uint8_t mode);
        int setVelocityLimit(uint32_t limit);
        uint32_t getVelocityLimit();
        int enableTorque(bool enable);
        int enableLED(bool enable);
        int reboot();

        // int setSpeed(float speed); // Should not be used how it is at the moment
        int setVelocity(int32_t velocity);
        int setPosition(int32_t positionInUnits);
        int setExtendedPosition(int32_t positionInUnits);
        int setProfileVelocity(uint32_t limit);
        int setProfileAcceleration(uint32_t limit);
        int setGain(int addr, uint16_t gain);
        int setPositionPGain(uint16_t gain);
        int setPositionIGain(uint16_t gain);
        int setPositionDGain(uint16_t gain);
        int clearMultiTurn();

        uint8_t getHardwareStatus();
        int setWatchdog(int8_t timeout);
        int getWatchdogError();

        float getPWM();
        float getCurrent();
        float getSpeed();
        float getPosition();
        float getVoltage();
        float getTemperature();
        uint8_t getOperatingMode();
        uint8_t getMoving();
        uint8_t getMovingStatus();
        int32_t getVelocity();
        uint32_t getProfileVelocity();
        uint32_t getProfileAcceleration();
        int setHomingOffset(int32_t offset);
        int setMovingThreshold(int32_t threshold);
        uint32_t getMovingThreshold();

        dynamixel::PortHandler * getPortHandler();
        bool portInUse = false;


    private:
        const char * connection_device;
        int baudrate;
        uint8_t ID;
        float protocol;
        uint32_t max_speedLimit;
        uint32_t speedLimit;
        int32_t homingOffset;
        int32_t movingThreshold;

        dynamixel::PortHandler * portHandler;
        dynamixel::PacketHandler * packetHandler;
    };

} // dynamixel_servo

#endif //SRC_DYNAMIXEL_SERVO_H
