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

size_t TopicListPanel::update_selection(const std::vector<std::string>& names)
{
  // update the current selection, ensuring it is valid, and return new index

  // set selection to a valid state, e.g. if just starting
  if (!_selection || std::find(names.begin(), names.end(), *_selection) == names.end())
  {
    _selection = names[0];
    return 0;
  }

  // find current selection in the list
  const auto& it = std::find(names.begin(), names.end(), *_selection);
  size_t idx = std::distance(names.begin(), it);

  // check if user is trying to navigate to a new selection
  if (_shift == 0)
  {
    // we're not shifting; return current index
    return idx;
  }
  else
  {
    // get element _shift elements away from current selection    
    const size_t new_idx = (idx + _shift) % (names.size() - 1);
    _selection = names[new_idx];

    // reset shift amount
    _shift = 0;

    return new_idx;
  }
}

void TopicListPanel::render(const std::optional<ComputationalGraph>& graph)
{
  // start from a blank slate
  redraw();

  // handle case where there's no information
  if (!graph || graph->topics().size() == 0)
  {
    mvwaddnstr(_window, 0, 1, "ROS Topics", _cols - 2 * BORDER);
    print_line(1, "no topics found");
    return;
  }

  // otherwise we have valid data; render it

  // get ordered set of topics
  std::vector<std::string> topics;
  topics.reserve(graph->topics().size());
  for (const auto& kv : graph->topics())
    topics.push_back(kv.first);
  std::sort(topics.begin(), topics.end());

  // make sure our selection is valid
  const size_t topic_idx = update_selection(topics);

  // handle scrolling

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