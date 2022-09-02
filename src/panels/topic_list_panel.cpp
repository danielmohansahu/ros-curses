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

  // check if we can shift to a valid topic
  size_t topic_idx;
  if (const auto idx = _scroll.shift(_selection, topics, _shift); !idx)
    return NULL_ACTION;
  else
  {
    // update selections
    topic_idx = *idx;
    _selection = topics[*idx];
    _shift = 0;
  }

  // start of the scrollable region (global coordinates)
  const size_t scroll_start_idx = BORDER;

  // get the region bounds
  const auto [begin, end] = _scroll.range_from_selection(topics.size(), _last_start_idx, topic_idx);
  _last_start_idx = begin;

  // iterate through visible section of items and print
  for (size_t i = begin; i != end; ++i)
  {
    // format display
    auto format = A_NORMAL;
    if (i == topic_idx)
      format = A_STANDOUT;
    else if (!graph->topics().at(topics[i])->active)
      format |= A_DIM;
    print_line(scroll_start_idx + i - begin, " - " + topics[i], format);
  }

  // add a little blurb if we're scrolling
  if (_scroll.scroll_required(topics.size()))
    print_line_center(scroll_start_idx + end - begin, (end < topics.size()) ? "-- more --" : "-- end --");

  // redraw border, in case it got borked
  draw_border();

  // write panel name over border
  mvwaddnstr(_window, 0, 1, "ROS Topics", _cols - 2 * BORDER);

  // indicate we want to display information about this topic
  return {Action::SELECT_TOPIC, _selection};
}

void TopicListPanel::set_visible(const bool visible)
{
  // reset our selected index
  _shift = 0;
  _selection = std::nullopt;

  // call parent method
  PanelBase::set_visible(visible);
}

} // namespace ros_curses::panels