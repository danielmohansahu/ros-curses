/*
 *
 *
 * 
 */

// ros_curses
#include "panels/service_info_panel.h"

namespace ros_curses::panels
{

ActionPacket ServiceInfoPanel::render(const std::optional<ComputationalGraph>& graph)
{
  // start from a blank slate
  redraw();
  mvwaddnstr(_window, 0, 1, "ROS Service Summary", _cols - 2 * BORDER);

  // if we don't have a graph or topic selected we can't do anything
  if (!graph || !_selection || graph->services().find(*_selection) == graph->services().end())
    return NULL_ACTION;
  
  // otherwise print some info about this topic
  const auto& service = graph->services().at(*_selection);

  // get a list of all this topic's information as tuples
  std::vector<std::pair<std::string,std::string>> fields;
  for (const auto& node : service->advertisers)
    fields.emplace_back("advertiser", node->name);

  // header information
  size_t current_idx = 1;
  print_line(current_idx++, *_selection + ((service->active) ? " (active)" : " (inactive)"));
  print_line(current_idx++, "  Message Type: " + service->type);
  print_line(current_idx++, "  Publishers:   " + std::to_string(service->advertisers.size()));
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
  mvwaddnstr(_window, 0, 1, "ROS Service Summary", _cols - 2 * BORDER);

  // indicate we want to display information about this topic
  return NULL_ACTION;
}

ActionPacket ServiceInfoPanel::handle_enter()
{
  // do nothing if nothing is highlighted
  if (!_sub_selection)
    return NULL_ACTION;

  // switch to selected node
  return {DISPLAY_NODE, _sub_selection->second};
}

void ServiceInfoPanel::set_visible(const bool visible)
{
  // reset our selected index
  _shift = 0;
  _selection = std::nullopt;
  _sub_selection = std::nullopt;

  // call parent method
  PanelBase::set_visible(visible);
}

} // namespace ros_curses::panels