/*
 *
 *
 * 
 */

// STL
#include <assert.h>

// ros_curses
#include "panels/node_info_panel.h"

namespace ros_curses::panels
{

ActionPacket NodeInfoPanel::render(const std::optional<ComputationalGraph>& graph)
{
  // start from a blank slate
  redraw();
  mvwaddnstr(_window, 0, 1, "ROS Node Summary", _cols - 2 * BORDER);

  // if we don't have a graph or topic selected we can't do anything
  if (!graph || !_selection || graph->nodes().find(*_selection) == graph->nodes().end())
    return NULL_ACTION;
  
  // otherwise print some info about this topic
  const auto& node = graph->nodes().at(*_selection);

  // get a list of all this topic's information as tuples
  std::vector<std::pair<std::string,std::string>> fields;
  for (const auto& pub : node->publications)
    fields.emplace_back("publication", pub);
  for (const auto& sub : node->subscriptions)
    fields.emplace_back("subscription", sub);
  for (const auto& srv : node->services)
    fields.emplace_back("service", srv);
  for (const auto& param : node->parameters)
    fields.emplace_back("param", param);

  // header information
  size_t current_idx = 1;
  print_line(current_idx++, *_selection + ((node->active) ? " (active)" : " (inactive)"));
  print_line(current_idx++, "  Publications:  " + std::to_string(node->publications.size()));
  print_line(current_idx++, "  Subscriptions: " + std::to_string(node->subscriptions.size()));
  print_line(current_idx++, "  Services:      " + std::to_string(node->services.size()));
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
    print_line(scroll_start_idx + i - begin, fields[i].first + ": " + fields[i].second, format(i == highlighted_idx));

  // add a little blurb if we're scrolling
  if (_scroll.scroll_required(fields.size()))
    print_line_center(scroll_start_idx + end - begin, (end < fields.size()) ? "-- more --" : "-- end --");

  // redraw border and header, in case it got borked
  draw_border();
  mvwaddnstr(_window, 0, 1, "ROS Node Summary", _cols - 2 * BORDER);

  // indicate we want to display information about this topic
  return NULL_ACTION;
}

ActionPacket NodeInfoPanel::handle_enter()
{
  // do nothing if nothing is highlighted
  if (!_sub_selection)
    return NULL_ACTION;

  // switch to the current selection
  if (_sub_selection->first == "publication" || _sub_selection->first == "subscription")
    return {DISPLAY_TOPIC, _sub_selection->second};
  else if (_sub_selection->first == "service")
    return {DISPLAY_SERVICE, _sub_selection->second};
  else if (_sub_selection->first == "param")
    return {DISPLAY_PARAM, _sub_selection->second};
  else
    assert(false);
}

void NodeInfoPanel::set_visible(const bool visible)
{
  // reset our selected index
  _shift = 0;
  _selection = std::nullopt;
  _sub_selection = std::nullopt;

  // call parent method
  PanelBase::set_visible(visible);
}

} // namespace ros_curses::panels