#ifndef __IDEVICE_HPP__
#define __IDEVICE_HPP__

#include <vector>
#include <cstdint>
#include <map>

namespace idevice
{
    class Servo
    {
    public:
        virtual ~Servo() = default;
        virtual bool connect() = 0;
        virtual void disconnect() = 0;
        virtual bool isConnected() const = 0;
        virtual bool sendCmd(uint8_t cmd, const std::vector<uint8_t> &params) = 0;
        virtual bool recv(std::vector<uint8_t> &response) = 0;
        virtual bool recvTimeout(std::vector<uint8_t> &response, int timeout) = 0;
        virtual bool moveServo(const std::vector<std::tuple<uint8_t, double>> &servos, uint16_t time) = 0;
        virtual bool getBatteryVoltage(uint16_t &voltage) = 0;
        virtual bool powerOffServos(const std::vector<uint8_t> &servo_ids) = 0;
        virtual std::map<uint8_t, double> readServoPositions(const std::vector<uint8_t> &servo_ids) = 0;
        virtual bool runActionGroup(uint8_t group_id, uint16_t repetitions) = 0;
        virtual bool stopActionGroup() = 0;
        virtual bool setActionGroupSpeed(uint8_t group_id, uint16_t speed) = 0;
        virtual bool isActionGroupRunning(uint8_t &group_id, uint16_t &repetitions) = 0;
        virtual bool isActionGroupStopped() = 0;
        virtual bool isActionGroupComplete(uint8_t &group_id, uint16_t &repetitions) = 0;
    };

} // namespace idevice

#endif // __IDEVICE_HPP__