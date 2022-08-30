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

  // the start index of our scrolling region (global coordinate frame) is just after the border
  const size_t scroll_start_idx = BORDER;

  // get visible range (local coordinate frame)
  const auto [begin, end] = _scroll.range_from_start(_items.size(), _scroll_offset);
  _scroll_offset = begin;

  // iterate through visible section of items
  for (size_t i = begin; i != end; ++i)
    print_line(scroll_start_idx + i - begin, _items[i]);

  // add a little blurb at the end if we're scrolling
  if (_scroll.scroll_required(_items.size()))
  {
    if (end < _items.size() - 1)
      print_line_center(scroll_start_idx + end - begin, "-- more --");
    else
      print_line_center(scroll_start_idx + end - begin, "-- end --");
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

void HelpPanel::move_and_resize(const size_t rows, const size_t cols, const size_t y, const size_t x)
{
  // call parent method and update scroll size
  PanelBase::move_and_resize(rows, cols, y, x);
  _scroll.update_size(rows - BORDER * 2);
}


} // namespace ros_curses::panels