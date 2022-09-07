/*
 *
 *
 *
 */

// STL
#include <algorithm>
#include <assert.h>

// ros_curses
#include "panels/node_info_panel.h"

namespace ros_curses::panels
{

void NodeInfoPanel::construct(const auto& node,
                              std::vector<std::string>& full_text,
                              std::vector<std::pair<ROSType, std::string>>& selectables,
                              std::vector<size_t>& selectable_indices)
{
  // construct full representation of our desired text

  // initialize vectors
  const size_t selectable_length = node->publications.size() + node->subscriptions.size()+ node->services.size()+ node->parameters.size();
  const size_t length = /*header=*/4 + /*section_headers=*/4 * 2 + selectable_length;
  selectables.reserve(selectable_length);
  selectable_indices.reserve(selectable_length);
  full_text.reserve(length);

  // running tally variables
  size_t selectable_idx = 0;

  // add fixed header information
  full_text.emplace_back(*_active_node + ((node->active) ? " (active)" : " (inactive)"));
  full_text.emplace_back("  Total Publications:  " + std::to_string(node->publications.size()));
  full_text.emplace_back("  Total Subscriptions: " + std::to_string(node->subscriptions.size()));
  full_text.emplace_back("  Total Services:      " + std::to_string(node->services.size()));
  full_text.emplace_back("");
  full_text.emplace_back("Publications:");

  // add selectable items (publications)
  selectable_idx = full_text.size();
  for (const auto& item : node->publications)
  {
    selectables.emplace_back(ROSType::PUBLICATION, item);
    full_text.emplace_back(" " + item);
    selectable_indices.emplace_back(selectable_idx++);
  }

  // add sub-header
  full_text.emplace_back("");
  full_text.emplace_back("Subscriptions:");
  selectable_idx += 2;

  // add selectable items (subscriptions)
  for (const auto& item : node->subscriptions)
  {
    selectables.emplace_back(ROSType::SUBSCRIPTION, item);
    full_text.emplace_back(" " + item);
    selectable_indices.emplace_back(selectable_idx++);
  }

  // add sub-header
  full_text.emplace_back("");
  full_text.emplace_back("Services:");
  selectable_idx += 2;

  // add selectable items (services)
  for (const auto& item : node->services)
  {
    selectables.emplace_back(ROSType::SERVICE, item);
    full_text.emplace_back(" " + item);
    selectable_indices.emplace_back(selectable_idx++);
  }

  // add sub-header
  full_text.emplace_back("");
  full_text.emplace_back("Parameters (in shared namespace):");
  selectable_idx += 2;

  // add selectable items (parameters)
  for (const auto& item : node->parameters)
  {
    selectables.emplace_back(ROSType::PARAMETER, item);
    full_text.emplace_back(" " + item);
    selectable_indices.emplace_back(selectable_idx++);
  }
}

ActionPacket NodeInfoPanel::render(const std::optional<ComputationalGraph>& graph)
{
  // start from a blank slate
  redraw();
  mvwaddnstr(_window, 0, 1, "ROS Node Summary", _cols - 2 * BORDER);

  // if we don't have a graph or topic selected we can't do anything
  if (!graph || !_active_node || graph->nodes().find(*_active_node) == graph->nodes().end())
    return NULL_ACTION;

  // otherwise print some info about this topic
  const auto& node = graph->nodes().at(*_active_node);

  // construct desired textual representation and metadata
  std::vector<std::string> full_text;
  std::vector<std::pair<ROSType, std::string>> selectables;
  std::vector<size_t> selectable_indices;
  construct(node, full_text, selectables, selectable_indices);

  // get the visible region bounds
  const auto [begin, end, selection_idx, highlight_idx] = _scroll.update(selectables, selectable_indices, full_text.size());

  // update selectable region, if we got something valid
  if (selection_idx < selectables.size())
    _sub_selection = selectables[selection_idx];
  else
    _sub_selection = std::nullopt;

  // iterate through and display the visible section of text
  for (size_t i = begin; i != end; ++i)
    print_line(BORDER + i - begin, full_text[i], format(i == highlight_idx));

  // add a little blurb if we're scrolling
  if (_scroll.scroll_required(full_text.size()))
    print_line_center(BORDER + end - begin, (end < full_text.size()) ? "-- more --" : "-- end --");

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
  switch (_sub_selection->first)
  {
    case ROSType::PUBLICATION:
    case ROSType::SUBSCRIPTION:
      return {DISPLAY_TOPIC, _sub_selection->second};
    case ROSType::SERVICE:
      return {DISPLAY_SERVICE, _sub_selection->second};
    case ROSType::PARAMETER:
      return {DISPLAY_PARAM, _sub_selection->second};
    default:
      assert(false);
      return NULL_ACTION;
  }
}

} // namespace ros_curses::panels