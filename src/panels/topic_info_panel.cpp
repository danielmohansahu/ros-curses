/*
 *
 *
 * 
 */

// ros_curses
#include "panels/topic_info_panel.h"

namespace ros_curses::panels
{

ActionPacket TopicInfoPanel::render(const std::optional<ComputationalGraph>& graph)
{
  // start from a blank slate
  redraw();
  mvwaddnstr(_window, 0, 1, "ROS Topic Summary", _cols - 2 * BORDER);

  // if we don't have a graph or topic selected we can't do anything
  if (!graph || !_selection || graph->topics().find(*_selection) == graph->topics().end())
    return NULL_ACTION;
  
  // otherwise print some info about this topic
  const auto& topic = graph->topics().at(*_selection);

  // header information
  print_line(1, *_selection + ((topic->active) ? " (active)" : " (inactive)"));
  print_line(2, "  Message Type: " + topic->type);
  print_line(3, "  Publishers:   " + std::to_string(topic->publishers.size()));
  print_line(4, "  Subscribers:  " + std::to_string(topic->subscribers.size()));

  // redraw border and header, in case it got borked
  draw_border();
  mvwaddnstr(_window, 0, 1, "ROS Topic Summary", _cols - 2 * BORDER);

  // indicate we want to display information about this topic
  return {Action::SELECT_TOPIC, _selection};}

} // namespace ros_curses::panels