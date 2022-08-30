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

void TopicListPanel::render(const std::optional<ComputationalGraph>& graph)
{
  // start from a blank slate
  redraw();
  mvwaddnstr(_window, 0, 1, "ROS Topics", _cols - 2 * BORDER);

  // if we don't have a graph we can't do anything
  if (!graph)
    return;
  
  // if we don't have topics we also can't do anything
  std::vector<std::string> topics;
  if (topics = graph->topic_list(); topics.size() == 0)
    return;

  // check if we can shift to a valid topic
  size_t topic_idx;
  if (const auto idx = _scroll.shift(_selection, topics, _shift); !idx)
    return;
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
    print_line(scroll_start_idx + i - begin, " - " + topics[i], (i == topic_idx) ? A_STANDOUT : A_NORMAL);

  // add a little blurb if we're scrolling
  if (_scroll.scroll_required(topics.size()))
    print_line_center(scroll_start_idx + end - begin, (end < topics.size() - 1) ? "-- more --" : "-- end --");

  // redraw border, in case it got borked
  draw_border();

  // write panel name over border
  mvwaddnstr(_window, 0, 1, "ROS Topics", _cols - 2 * BORDER);
}

void TopicListPanel::set_visible(const bool visible)
{
  // reset our selected index
  _shift = 0;
  _selection = std::nullopt;

  // call parent method
  PanelBase::set_visible(visible);
}

void TopicListPanel::move_and_resize(const size_t rows, const size_t cols, const size_t y, const size_t x)
{
  // call parent method and update scroll size
  PanelBase::move_and_resize(rows, cols, y, x);
  _scroll.update_size(rows - BORDER * 2);
}

} // namespace ros_curses::panels