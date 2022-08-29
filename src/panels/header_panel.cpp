/*
 *
 *
 * 
 */

// ros_curses
#include "panels/header_panel.h"

namespace ros_curses::panels
{

HeaderPanel::HeaderPanel()
{
}

void HeaderPanel::render(const std::optional<ComputationalGraph>&)
{
  mvwaddstr(_window, 0, 0, "ros-curses: Command line introspection of the ROS computational graph.");
  mvwprintw(_window, 1, 0, "  Status: %s", _status.c_str());
  mvwprintw(_window, 2, 0, "  Active: %u", _active);
  touchline(_window, 0, 3);
}

} // namespace ros_curses::panels