#ifndef __LSC_SERVO_HPP__
#define __LSC_SERVO_HPP__

#include "lsc_servo/visibility_control.h"
#include "idevice/idevice.hpp"
#include "hidraw/hidraw.hpp"

namespace lsc_servo
{
  enum CMD : uint8_t
  {
    CMD_SERVO_MOVE = 0x03,
    CMD_GET_BATTERY_VOLTAGE = 0x0F,
    CMD_MULT_SERVO_UNLOAD = 0x14,
    CMD_MULT_SERVO_POS_READ = 0x15,
    CMD_ACTION_GROUP_RUN = 0x06,
    CMD_ACTION_STOP = 0x07,
    CMD_ACTION_SPEED = 0x0B,
    CMD_ACTION_GROUP_STOP = 0x07,
    CMD_ACTION_GROUP_COMPLETE = 0x08
  };

  class Lsc_servo : public idevice::Servo
  {
  public:
    explicit Lsc_servo();
    virtual ~Lsc_servo();

    bool connect() override;
    void disconnect() override;
    bool isConnected() const override;

    bool sendCmd(uint8_t cmd, const std::vector<uint8_t> &params) override;
    bool recv(std::vector<uint8_t> &response) override;
    bool recvTimeout(std::vector<uint8_t> &response, int timeout = 500) override;

    bool moveServo(const std::vector<std::tuple<uint8_t, double>> &servos, uint16_t time) override;
    bool getBatteryVoltage(uint16_t &voltage) override;
    bool powerOffServos(const std::vector<uint8_t> &servo_ids) override;
    std::map<uint8_t, double> readServoPositions(const std::vector<uint8_t> &servo_ids) override;

    bool runActionGroup(uint8_t group_id, uint16_t repetitions) override;
    bool stopActionGroup() override;
    bool setActionGroupSpeed(uint8_t group_id, uint16_t speed) override;

    bool isActionGroupRunning(uint8_t &group_id, uint16_t &repetitions) override;
    bool isActionGroupStopped() override;
    bool isActionGroupComplete(uint8_t &group_id, uint16_t &repetitions) override;

  private:
    hidraw::Hidraw *phidraw;
    static constexpr uint16_t VENDOR_ID = 0x0483;
    static constexpr uint16_t PRODUCT_ID = 0x5750;
    static constexpr uint8_t HEADER[] = {0x55, 0x55};
    std::vector<uint8_t> buildCommandPacket(uint8_t cmd, const std::vector<uint8_t> &params);
  };

} // namespace lsc_servo

#endif // __LSC_SERVO_HPP__
