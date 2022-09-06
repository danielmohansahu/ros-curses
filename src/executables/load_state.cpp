/* ros-curses Command Line debugging of the ROS Computational Graph
 *
 *
 *
 */

// STL
#include <iostream>
#include <unistd.h>

// ros_curses
#include "display.h"
#include "parsers/ros1.h"

using namespace ros_curses;

int main(int argc, char** argv)
{
  // first argument should be the filename
  if (argc != 2)
  {
    std::cerr << "Usage: \n\t load-state FILENAME.yaml" << std::endl;
    return 1;
  }
  const auto filename = std::string(argv[1]);

  // construct computational graph and spin
  if (auto cg = ComputationalGraph::load(filename); !cg)
  {
    std::cerr << "Failed to load state from " << filename << std::endl;
    return 2;
  }
  else
  {
    // construct basic display class and spin until shutdown
    auto display = Display {};
    while (display.process(cg) != Action::EXIT)
      usleep(10000);
  }
}