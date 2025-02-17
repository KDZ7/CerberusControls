#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include "pluginlib/class_loader.hpp"
#include "idevice/idevice.hpp"

int main()
{
    // Initialize the plugin loader for the servo interface "iservo"
    pluginlib::ClassLoader<idevice::Servo> loader("idevice", "idevice::Servo");
    try
    {
        std::shared_ptr<idevice::Servo> servo = loader.createSharedInstance("lsc_servo::Lsc_servo");
        // Connect to the servo controller
        if (!servo->connect())
        {
            std::cout << "Failed to connect to servo" << std::endl;
            return 1;
        }
        std::cout << "Connected to servo" << std::endl;

        // Test battery voltage reading
        // uint16_t voltage = 0;
        // if (!servo->getBatteryVoltage(voltage))
        // {
        //     std::cout << "Failed to get voltage" << std::endl;
        //     return 1;
        // }
        // std::cout << "Voltage: " << voltage << " mV" << std::endl;

        // Define positions for all servos (initial position at 0 radians)
        std::vector<std::tuple<uint8_t, double>> positions = {
            {11, 0.0},
            {12, 0.0},
            {13, 0.0},
            {21, 0.0},
            {22, 0.0},
            {23, 0.0},
            {31, 0.0},
            {32, 0.0},
            {33, 0.0},
            {41, 0.0},
            {42, 0.0},
            {43, 0.0},
        };

        // Move all servos to their initial positions over 1000ms
        if (!servo->moveServo(positions, 1000))
        {
            std::cout << "Failed to move servo" << std::endl;
            return 1;
        }
        std::cout << "Servo moved" << std::endl;

        //     // Wait for movement to complete
        //     std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        //     // Read current positions of all servos
        //     std::vector<uint8_t> servo_ids = {11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43};
        //     std::cout << "Reading servo positions..." << std::endl;
        //     std::map<uint8_t, double> current_positions = servo->readServoPositions(servo_ids);

        //     // Display the read positions
        //     if (current_positions.empty())
        //         std::cout << "Failed to read positions or no servos responded" << std::endl;
        //     else
        //         for (const auto &position : current_positions)
        //             std::cout << "Servo " << (int)position.first << ": " << position.second << " rad" << std::endl;

        //     // Test raw data reception (useful for debugging)
        //     std::vector<uint8_t> response;
        //     std::cout << "Testing raw receive with timeout..." << std::endl;
        //     if (servo->recvTimeout(response, 500))
        //     {
        //         std::cout << "Received " << response.size() << " bytes: ";
        //         for (const auto &byte : response)
        //             std::cout << std::hex << (int)byte << " ";
        //         std::cout << std::dec << std::endl;
        //     }
        //     else
        //         std::cout << "No data received within timeout" << std::endl;

        //     // Test sending a command and receiving a response
        //     std::cout << "Testing send command and receive response..." << std::endl;
        //     uint8_t cmd = 0x0F; // Command code for reading battery voltage
        //     std::vector<uint8_t> params;
        //     if (servo->sendCmd(cmd, params))
        //         if (servo->recvTimeout(response, 500))
        //         {
        //             std::cout << "Command response received: ";
        //             for (const auto &byte : response)
        //                 std::cout << std::hex << (int)byte << " ";
        //             std::cout << std::dec << std::endl;
        //         }
        //         else
        //             std::cout << "No response to command" << std::endl;
        //     else
        //         std::cout << "Failed to send command" << std::endl;

        //     // Test powering off selected servos
        //     std::vector<uint8_t> servos_to_power_off = {11, 21, 31, 41};
        //     std::cout << "Powering off selected servos..." << std::endl;
        //     if (servo->powerOffServos(servos_to_power_off))
        //         std::cout << "Servos powered off successfully" << std::endl;
        //     else
        //         std::cout << "Failed to power off servos" << std::endl;

        //     // Disconnect from the servo controller
        //     servo->disconnect();
        //     std::cout << "Disconnected from servo" << std::endl;
        // }
        // catch (pluginlib::PluginlibException &ex)
        // {
        //     std::cerr << "Plugin exception: " << ex.what() << std::endl;
        //     return 1;
    }
    catch (std::exception &ex)
    {
        std::cerr << "Standard exception: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}