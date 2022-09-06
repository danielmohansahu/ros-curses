/* Save the current ROS state to a file.
 */

// STL
#include <iostream>
#include <unistd.h>
#include <string>

// ros_curses
#include "parsers/ros1.h"

using namespace ros_curses;

int main(int argc, char** argv)
{
  // first argument should be the filename
  if (argc != 2)
  {
    std::cerr << "Usage: \n\t save-state FILENAME.yaml" << std::endl;
    return 1;
  }
  const auto filename = std::string(argv[1]);

  // construct ROS parser
  ros1::ROS1Parser parser {};

  // exit if there's no state
  if (const auto cg = parser.poll(); !cg)
  {
    std::cerr << "Failed to load computational graph; is a ROS instance running?" << std::endl;
    return 2;
  }
  else if (!cg->save(filename))
  {
    std::cerr << "Failed to save state to " << filename << std::endl;
    return 3;
  }

  // success!
}