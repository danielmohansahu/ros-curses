/*
 *
 *
 * 
 */

// STL
#include <algorithm>
#include <assert.h>

// ros_curses
#include "panels/service_list_panel.h"

namespace ros_curses::panels
{

ActionPacket ServiceListPanel::render(const std::optional<ComputationalGraph>& graph)
{
  // start from a blank slate
  redraw();
  mvwaddnstr(_window, 0, 1, "ROS Services", _cols - 2 * BORDER);

  // if we don't have a graph we can't do anything
  if (!graph)
    return NULL_ACTION;
  
  // if we don't have topics we also can't do anything
  std::vector<std::string> all_services;
  if (all_services = graph->service_list(); all_services.size() == 0)
    return NULL_ACTION;

  // optionally filter nodes
  std::vector<std::string> services;
  services.reserve(all_services.size());
  if (_filter && _filter->size() != 0)
  {
    for (const auto& service : all_services)
      if (service.find(*_filter) != std::string::npos)
        services.emplace_back(service);
  }
  else
    services = all_services;

  // add a little blurb if we're filtering
  if (_filter)
    print_line(_rows - BORDER * 2, "Showing matches for: '" + *_filter + "'");

  // if we filtered everything there's not much to do
  if (services.size() == 0)
    return NULL_ACTION;

  // construct a list of indices for the visible region
  std::vector<size_t> indices(services.size());
  std::iota(indices.begin(), indices.end(), 0);

  // get the region bounds
  const auto [begin, end, selection_idx, highlight_idx] = _scroll.update(services, indices, services.size());

  // iterate through visible section of items and print
  for (size_t i = begin; i != end; ++i)
    print_line(BORDER + i - begin, " - " + services[i], format(i == highlight_idx, !graph->services().at(services[i])->active));

  // add a little blurb if we're scrolling
  if (_scroll.scroll_required(services.size()))
    print_line_center(BORDER + end - begin, (end < services.size()) ? "-- more --" : "-- end --");

  // redraw border, in case it got borked
  draw_border();

  // write panel name over border
  mvwaddnstr(_window, 0, 1, "ROS Services", _cols - 2 * BORDER);

  // indicate we want to display information about this topic
  return {Action::SELECT_SERVICE, services[selection_idx]};
}

} // namespace ros_curses::panels