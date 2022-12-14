/* ros-curses Command Line debugging of the ROS Computational Graph
 *
 *
 *
 */

// STL
#include <unistd.h>

// ros_curses
#include "display.h"
#include "parsers/ros1.h"

using namespace ros_curses;

int main(int argc, char** argv)
{
  // construct ROS parser
  ros1::ROS1Parser parser {};

  // construct basic display class
  auto display = Display();

  // process until shutdown
  while (display.process(parser.poll()) != Action::EXIT)
    usleep(10000);
}