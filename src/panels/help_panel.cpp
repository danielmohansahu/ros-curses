/*
 *
 *
 * 
 */

// STL
#include <algorithm>

// ros_curses
#include "panels/help_panel.h"

namespace ros_curses::panels
{

HelpPanel::HelpPanel()
 : _items({
  ""
  " Navigation Commands",
  " -------------------",
  "          UP : move cursor up",
  "        DOWN : move cursor down",
  "        LEFT : switch active display",
  "       RIGHT : switch active display",
  "         TAB : cycle panels",
  "        h, ? : toggle help menu",
  "   CTRL-C, q : quit / exit",
  "",
  " Node Info Commands",
  " ------------------",
  "       ENTER : switch to selected item's panel",
  "",
  " Topic Info Commands",
  " -------------------",
  "           H : run 'rostopic hz'",
  "           E : run 'rostopic echo --noarr -n1'",
  "           B : run 'rostopic bw'",
  "",
  " About",
  " -----",
  " This project targets users of ROS who have",
  " spent too many hours manually running",
  " 'rostopic info', 'rosnode info -q',",
  " 'rostopic hz', etc. while debugging a live ",
  " Computational Graph.",
  "",
  " There are good tools for ROS introspection,",
  " like 'rqt_graph' - why create another one?",
  " Let alone one with an antiquated technology",
  " like curses? My reasons are the following:",
  " 1. Many robotic platforms don't support GUIs,",
  "    or need to be accessed remotely via SSH in",
  "    environments with spotty comms.",
  " 2. There's an argument to be made that GUIs",
  "    slow down debugging. I believe the success",
  "    of tools like 'htop', 'bmon', and 'ncdu'",
  "    (to name a few) supports this argument.",
  "",
  " Anyway, I hope it's useful. Please submit",
  " technical feedback on Github:",
  "  https://github.com/danielmohansahu/ros-curses"
  })
{
}

void HelpPanel::render(const std::optional<ComputationalGraph>&)
{
  // completely redraw, to start with a blank slate
  redraw();

  // visible region parameters
  const size_t visible_start_idx = 1;
  const size_t visible_end_buffer = 1;
  const size_t visible_region_size = _rows - visible_start_idx - visible_end_buffer;

  // scroll parameters
  const bool scroll_required = visible_region_size < _items.size();
  _scroll_offset = (scroll_required) ? std::clamp(_scroll_offset, 0UL, _items.size() - visible_region_size) : 0;
  
  // loop parameters
  const size_t start_idx = _scroll_offset;
  const size_t end_idx = start_idx + std::min(visible_region_size - 1, _items.size());

  // iterate through visible section of items
  for (size_t i = start_idx; i != end_idx; ++i)
    print_line(visible_start_idx + i - start_idx, _items[i]);

  // add a little blurb if we're scrolling
  if (scroll_required)
  {
    if (_scroll_offset != _items.size() - visible_region_size)
      print_line_center(visible_start_idx + end_idx - start_idx, "-- more --");
    else
      print_line_center(visible_start_idx + end_idx - start_idx, "-- end --");
  }

  // redraw border, in case our lines overrode them
  draw_border();

  // help panel header - not scrollable. this intentionally overrides part of the border
  mvwaddstr(_window, 0, 1, "ros-curses help");
}

void HelpPanel::handle_key_up()
{
  if (_scroll_offset > 0)
    --_scroll_offset;
}

void HelpPanel::handle_key_down()
{
  ++_scroll_offset;
}

void HelpPanel::set_visible(const bool visible)
{
  // reset our selected index and call parent method
  _scroll_offset = 0;
  PanelBase::set_visible(visible);
}


} // namespace ros_curses::panels