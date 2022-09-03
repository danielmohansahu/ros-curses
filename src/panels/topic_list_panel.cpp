/*
 *
 *
 * 
 */

// STL
#include <algorithm>
#include <assert.h>

// ros_curses
#include "panels/topic_list_panel.h"

namespace ros_curses::panels
{

ActionPacket TopicListPanel::render(const std::optional<ComputationalGraph>& graph)
{
  // start from a blank slate
  redraw();
  mvwaddnstr(_window, 0, 1, "ROS Topics", _cols - 2 * BORDER);

  // if we don't have a graph we can't do anything
  if (!graph)
    return NULL_ACTION;
  
  // if we don't have topics we also can't do anything
  std::vector<std::string> topics;
  if (topics = graph->topic_list(); topics.size() == 0)
    return NULL_ACTION;

  // construct a list of indices for the visible region
  std::vector<size_t> indices(topics.size());
  std::iota(indices.begin(), indices.end(), 0);

  // get the region bounds
  const auto [begin, end, selection_idx] = _scroll.update(topics, indices, _step, _page);
  _step = 0; _page = 0;

  // iterate through visible section of items and print
  for (size_t i = begin; i != end; ++i)
    print_line(BORDER + i - begin, " - " + topics[i], format(i == selection_idx, !graph->topics().at(topics[i])->active));

  // add a little blurb if we're scrolling
  if (_scroll.scroll_required(topics.size()))
    print_line_center(BORDER + end - begin, (end < topics.size()) ? "-- more --" : "-- end --");

  // redraw border, in case it got borked
  draw_border();

  // write panel name over border
  mvwaddnstr(_window, 0, 1, "ROS Topics", _cols - 2 * BORDER);

  // indicate we want to display information about this topic
  return {Action::SELECT_TOPIC, topics[selection_idx]};
}

} // namespace ros_curses::panels