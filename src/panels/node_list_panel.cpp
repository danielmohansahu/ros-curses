/*
 *
 *
 * 
 */

// STL
#include <algorithm>
#include <numeric>
#include <assert.h>

// ros_curses
#include "panels/node_list_panel.h"

namespace ros_curses::panels
{

ActionPacket NodeListPanel::render(const std::optional<ComputationalGraph>& graph)
{
  // start from a blank slate
  redraw();
  mvwaddnstr(_window, 0, 1, "ROS Nodes", _cols - 2 * BORDER);

  // if we don't have a graph we can't do anything
  if (!graph)
    return NULL_ACTION;
  
  // if we don't have topics we also can't do anything
  std::vector<std::string> nodes;
  if (nodes = graph->node_list(); nodes.size() == 0)
    return NULL_ACTION;

  // construct a list of indices for the visible region
  std::vector<size_t> indices(nodes.size());
  std::iota(indices.begin(), indices.end(), 0);

  // get the start -> end of the scrollable region, as well as current selection index
  const auto [begin, end, selection_idx, highlight_idx] = _scroll.update(nodes, indices, nodes.size());

  // iterate through visible section of items and print
  for (size_t i = begin; i != end; ++i)
    print_line(BORDER + i - begin, " - " + nodes[i], format(i == highlight_idx, !graph->nodes().at(nodes[i])->active));

  // add a little blurb if we're scrolling
  if (_scroll.scroll_required(nodes.size()))
    print_line_center(BORDER + end - begin, (end < nodes.size()) ? "-- more --" : "-- end --");

  // redraw border, in case it got borked
  draw_border();

  // write panel name over border
  mvwaddnstr(_window, 0, 1, "ROS Nodes", _cols - 2 * BORDER);

  // indicate we want to display information about this topic
  return {Action::SELECT_NODE, nodes[selection_idx]};
}

} // namespace ros_curses::panels