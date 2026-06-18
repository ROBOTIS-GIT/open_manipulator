#include <cstdlib>
#include <iostream>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"

int main(int argc, char ** argv)
{
  const std::string port_name = argc > 1 ? argv[1] : "/dev/ttyACM0";
  const int baudrate = argc > 2 ? std::atoi(argv[2]) : 1000000;
  const int first_id = argc > 3 ? std::atoi(argv[3]) : 1;
  const int last_id = argc > 4 ? std::atoi(argv[4]) : 30;

  dynamixel::PortHandler * port = dynamixel::PortHandler::getPortHandler(port_name.c_str());
  dynamixel::PacketHandler * packet = dynamixel::PacketHandler::getPacketHandler(2.0);

  if (!port->openPort()) {
    std::cerr << "Failed to open " << port_name << '\n';
    return 1;
  }
  if (!port->setBaudRate(baudrate)) {
    std::cerr << "Failed to set baudrate " << baudrate << '\n';
    port->closePort();
    return 1;
  }

  std::cout << "Scanning " << port_name << " at " << baudrate
            << " bps, IDs " << first_id << ".." << last_id << '\n';

  bool any = false;
  for (int id = first_id; id <= last_id; ++id) {
    uint16_t model_number = 0;
    uint8_t dxl_error = 0;
    const int result = packet->ping(port, id, &model_number, &dxl_error);
    if (result == COMM_SUCCESS) {
      any = true;
      std::cout << "ID " << id << " OK model " << model_number;
      if (dxl_error != 0) {
        std::cout << " dxl_error " << static_cast<int>(dxl_error);
      }
      std::cout << '\n';
    }
  }

  if (!any) {
    std::cout << "No Dynamixel IDs responded.\n";
  }
  port->closePort();
  return any ? 0 : 2;
}
