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
 : PanelBase(false), _items({
  ""
  " Navigation Commands",
  " -------------------",
  "          UP : move active display up",
  "        DOWN : move active display down",
  "        PGUP : page active display up",
  "      PGDOWN : page active display down",  
  "        LEFT : switch active display",
  "       RIGHT : switch active display",
  "         TAB : cycle panels",
  "        h, ? : toggle help menu",
  "   CTRL-C, q : quit / exit",
  "",
  " (Topic|Node|Service|Param)Info Panel Commands",
  " ---------------------------------------------",
  "       ENTER : switch to selected item's panel",
  "           / : enter 'filter' mode",
  "         ESC : exit 'filter' mode",
  "",
  " Legend",
  " ------",
  " HIGHLIGHTED : currently active selection",
  "         DIM : element not found on last poll,",
  "               probably deleted or killed",
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

ActionPacket HelpPanel::render(const std::optional<ComputationalGraph>&)
{
  // completely redraw, to start with a blank slate
  redraw();

  // get visible indices
  const auto [begin, end] = _scroll.update(_items.size());

  for (size_t i = begin; i != end; ++i)
    print_line(BORDER + i - begin, _items[i]);

  // add a little blurb at the end if we're scrolling
  if (_scroll.scroll_required(_items.size()))
    print_line_center(BORDER + end - begin, (end < _items.size()) ? "-- more --" : "-- end --");

  // redraw border, in case our lines overrode them
  draw_border();

  // help panel header - not scrollable. this intentionally overrides part of the border
  mvwaddstr(_window, 0, 1, "ros-curses help");

  // no user action required
  return NULL_ACTION;
}

} // namespace ros_curses::panels