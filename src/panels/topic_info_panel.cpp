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

  // get a list of all this topic's information as tuples
  std::vector<std::pair<std::string,std::string>> fields;
  for (const auto& pub : topic->publishers)
    fields.emplace_back("publisher", pub->name);
  for (const auto& sub : topic->subscribers)
    fields.emplace_back("subscriber", sub->name);

  // header information
  size_t current_idx = 1;
  print_line(current_idx++, *_selection + ((topic->active) ? " (active)" : " (inactive)"));
  print_line(current_idx++, "  Message Type: " + topic->type);
  print_line(current_idx++, "  Publishers:   " + std::to_string(topic->publishers.size()));
  print_line(current_idx++, "  Subscribers:  " + std::to_string(topic->subscribers.size()));
  print_line(current_idx++, "");

  // check if we can shift to a valid sub-selection
  size_t highlighted_idx;
  if (const auto idx = _scroll.shift(_sub_selection, fields, _shift); !idx)
    return NULL_ACTION;
  else
  {
    // update selections
    highlighted_idx = *idx;
    _sub_selection = fields[*idx];
    _shift = 0;
  }

  // start of the scrollable region (global coordinates)
  const size_t scroll_start_idx = current_idx;

  // get the region bounds
  const auto [begin, end] = _scroll.range_from_selection(fields.size(), _last_start_idx, highlighted_idx);
  _last_start_idx = begin;

  // iterate through visible section of items and print
  for (size_t i = begin; i != end; ++i)
    print_line(scroll_start_idx + i - begin,
               fields[i].first + ": " + fields[i].second,
               (i == highlighted_idx) ? A_STANDOUT : A_NORMAL);

  // add a little blurb if we're scrolling
  if (_scroll.scroll_required(fields.size()))
    print_line_center(scroll_start_idx + end - begin, (end < fields.size() - 1) ? "-- more --" : "-- end --");

  // redraw border and header, in case it got borked
  draw_border();
  mvwaddnstr(_window, 0, 1, "ROS Topic Summary", _cols - 2 * BORDER);

  // indicate we want to display information about this topic
  return NULL_ACTION;
}

ActionPacket TopicInfoPanel::handle_enter()
{
  // do nothing if nothing is highlighted
  if (!_sub_selection)
    return NULL_ACTION;

  // switch to selected node
  return {DISPLAY_NODE, _sub_selection->second};
}

void TopicInfoPanel::set_visible(const bool visible)
{
  // reset our selected index
  _shift = 0;
  _selection = std::nullopt;
  _sub_selection = std::nullopt;

  // call parent method
  PanelBase::set_visible(visible);
}

void TopicInfoPanel::move_and_resize(const size_t rows, const size_t cols, const size_t y, const size_t x)
{
  // call parent method and update scroll size
  PanelBase::move_and_resize(rows, cols, y, x);
  _scroll.update_size(rows - BORDER * 2);
}

} // namespace ros_curses::panels