/*
 *
 *
 * 
 */

// ros_curses
#include "panels/header_panel.h"

namespace ros_curses::panels
{

ActionPacket HeaderPanel::render(const std::optional<ComputationalGraph>&)
{
  // add status information
  print_line(1, " Status: " + _status);
  print_line(2, " Active: " + std::to_string(_active));

  // redraw border
  draw_border();

  // add this after the border, to override it. I think it looks nice...
  mvwaddnstr(_window, 0, 1, "ros-curses: Command line introspection of the ROS computational graph.", _cols - 2 * BORDER);
  return NULL_ACTION;
}

} // namespace ros_curses::panels