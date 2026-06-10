#include <pluginlib/class_loader.hpp>
#include <hardware_interface/system_interface.hpp>
#include <iostream>

int main(int argc, char **argv)
{
  try
  {
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "corebot_hardware",
      "hardware_interface::SystemInterface"
    );

    auto classes = loader.getDeclaredClasses();

    std::cout << "Found hardware plugins:\n";
    for (const auto & cls : classes)
    {
      std::cout << "  - " << cls << std::endl;
    }
  }
  catch (const std::exception & e)
  {
    std::cerr << "Pluginlib error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}