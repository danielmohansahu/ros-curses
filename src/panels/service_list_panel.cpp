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
  std::vector<std::string> services;
  if (services = graph->service_list(); services.size() == 0)
    return NULL_ACTION;

  // check if we can shift to a valid topic
  size_t service_idx;
  if (const auto idx = _scroll.shift(_selection, services, _shift); !idx)
    return NULL_ACTION;
  else
  {
    // update selections
    service_idx = *idx;
    _selection = services[*idx];
    _shift = 0;
  }

  // start of the scrollable region (global coordinates)
  const size_t scroll_start_idx = BORDER;

  // get the region bounds
  const auto [begin, end] = _scroll.range_from_selection(services.size(), _last_start_idx, service_idx);
  _last_start_idx = begin;

  // iterate through visible section of items and print
  for (size_t i = begin; i != end; ++i)
  {
    // format display
    auto format = A_NORMAL;
    if (i == service_idx)
      format = A_STANDOUT;
    else if (!graph->services().at(services[i])->active)
      format |= A_DIM;
    print_line(scroll_start_idx + i - begin, " - " + services[i], format);
  }

  // add a little blurb if we're scrolling
  if (_scroll.scroll_required(services.size()))
    print_line_center(scroll_start_idx + end - begin, (end < services.size() - 1) ? "-- more --" : "-- end --");

  // redraw border, in case it got borked
  draw_border();

  // write panel name over border
  mvwaddnstr(_window, 0, 1, "ROS Services", _cols - 2 * BORDER);

  // indicate we want to display information about this topic
  return {Action::SELECT_SERVICE, _selection};
}

void ServiceListPanel::set_visible(const bool visible)
{
  // reset our selected index
  _shift = 0;
  _selection = std::nullopt;

  // call parent method
  PanelBase::set_visible(visible);
}

void ServiceListPanel::move_and_resize(const size_t rows, const size_t cols, const size_t y, const size_t x)
{
  // call parent method and update scroll size
  PanelBase::move_and_resize(rows, cols, y, x);
  _scroll.update_size(rows - BORDER * 2);
}

} // namespace ros_curses::panels