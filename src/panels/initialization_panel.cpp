/*
 *
 *
 * 
 */

#include <cstdlib>

// access to environment variables
extern char **environ;

// ros_curses
#include "panels/initialization_panel.h"

namespace ros_curses::panels
{

ActionPacket InitializationPanel::render(const std::optional<ComputationalGraph>&)
{
  // let the user know we haven't received any information
  print_line(2, " No ROS connection found; waiting...");

  // try to help the user with potentially useful debugging info
  print_line(4, " (maybe) useful information:");
  size_t idx = 5;

  // look for variables that might be helpful in debugging:
  char **evs = environ;
  for (; *evs; evs++)
    if (const auto& str = std::string(*evs); str.find("ROS_") != std::string::npos)
      print_line(idx++, "\t" + str);

  // redraw border (it might've gotten messed up)
  draw_border();

  // no user action required
  return NULL_ACTION;
}


} // namespace ros_curses::panels