/*
 *
 *
 * 
 */

// ros_curses
#include "panels/param_info_panel.h"

namespace ros_curses::panels
{

ActionPacket ParamInfoPanel::render(const std::optional<ComputationalGraph>& graph)
{
  // start from a blank slate
  redraw();
  mvwaddnstr(_window, 0, 1, "ROS Param Summary", _cols - 2 * BORDER);

  // if we don't have a graph or topic selected we can't do anything
  if (!graph || !_external_selection || graph->params().find(*_external_selection) == graph->params().end())
    return NULL_ACTION;
  
  // otherwise print some info about this topic
  const auto& param = graph->params().at(*_external_selection);

  // header information
  size_t current_idx = 1;
  print_line(current_idx++, *_external_selection + ((param->active) ? " (active)" : " (inactive)"));
  print_line(current_idx++, "  Value: " + param->value);
  print_line(current_idx++, "");

  // redraw border and header, in case it got borked
  draw_border();
  mvwaddnstr(_window, 0, 1, "ROS Param Summary", _cols - 2 * BORDER);

  // indicate we want to display information about this topic
  return NULL_ACTION;
}

} // namespace ros_curses::panels