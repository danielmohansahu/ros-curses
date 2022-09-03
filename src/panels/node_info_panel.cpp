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
  if (!graph || !_external_selection || graph->nodes().find(*_external_selection) == graph->nodes().end())
    return NULL_ACTION;
  
  // otherwise print some info about this topic
  const auto& node = graph->nodes().at(*_external_selection);

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
  print_line(current_idx++, *_external_selection + ((node->active) ? " (active)" : " (inactive)"));
  print_line(current_idx++, "  Publications:  " + std::to_string(node->publications.size()));
  print_line(current_idx++, "  Subscriptions: " + std::to_string(node->subscriptions.size()));
  print_line(current_idx++, "  Services:      " + std::to_string(node->services.size()));
  print_line(current_idx++, "");

  // construct indices corresponding to all selectable items
  std::vector<size_t> indices(fields.size());
  std::iota(indices.begin(), indices.end(), 0);

  // get the visible region bounds
  const auto [begin, end, selection_idx] = _scroll.update(fields, indices, _step, _page);
  _step = 0; _page = 0;
  _sub_selection = fields[indices[selection_idx]];

  // iterate through visible section of items and print
  for (size_t i = begin; i != end; ++i)
    print_line(current_idx + i - begin, fields[i].first + ": " + fields[i].second, format(i == selection_idx));

  // add a little blurb if we're scrolling
  if (_scroll.scroll_required(fields.size()))
    print_line_center(current_idx + end - begin, (end < fields.size()) ? "-- more --" : "-- end --");

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

} // namespace ros_curses::panels