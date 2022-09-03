/*
 *
 *
 * 
 */

// STL
#include <algorithm>
#include <assert.h>

// ros_curses
#include "panels/param_list_panel.h"

namespace ros_curses::panels
{

ActionPacket ParamListPanel::render(const std::optional<ComputationalGraph>& graph)
{
  // start from a blank slate
  redraw();
  mvwaddnstr(_window, 0, 1, "ROS Params", _cols - 2 * BORDER);

  // if we don't have a graph we can't do anything
  if (!graph)
    return NULL_ACTION;
  
  // if we don't have topics we also can't do anything
  std::vector<std::string> all_params;
  if (all_params = graph->param_list(); all_params.size() == 0)
    return NULL_ACTION;

  // optionally filter nodes
  std::vector<std::string> params;
  params.reserve(all_params.size());
  if (_filter && _filter->size() != 0)
  {
    for (const auto& param : all_params)
      if (param.find(*_filter) != std::string::npos)
        params.emplace_back(param);
  }
  else
    params = all_params;

  // add a little blurb if we're filtering
  if (_filter)
    print_line(_rows - BORDER * 2, "Showing matches for: '" + *_filter + "'");

  // if we filtered everything there's not much to do
  if (params.size() == 0)
    return NULL_ACTION;

  // construct a list of indices for the visible region
  std::vector<size_t> indices(params.size());
  std::iota(indices.begin(), indices.end(), 0);

  // get the region bounds
  const auto [begin, end, selection_idx, highlight_idx] = _scroll.update(params, indices, params.size());

  // iterate through visible section of items and print
  for (size_t i = begin; i != end; ++i)
    print_line(BORDER + i - begin, " - " + params[i], format(i == highlight_idx, !graph->params().at(params[i])->active));

  // add a little blurb if we're scrolling
  if (_scroll.scroll_required(params.size()))
    print_line_center(BORDER + end - begin, (end < params.size()) ? "-- more --" : "-- end --");

  // redraw border, in case it got borked
  draw_border();

  // write panel name over border
  mvwaddnstr(_window, 0, 1, "ROS Params", _cols - 2 * BORDER);

  // indicate we want to display information about this topic
  return {Action::SELECT_PARAM, params[selection_idx]};
}

} // namespace ros_curses::panels