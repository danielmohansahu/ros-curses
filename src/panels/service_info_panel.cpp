/*
 *
 *
 *
 */

// ros_curses
#include "panels/service_info_panel.h"

namespace ros_curses::panels
{

void ServiceInfoPanel::construct(const auto& service,
                                 std::vector<std::string>& full_text,
                                 std::vector<std::pair<ROSType, std::string>>& selectables,
                                 std::vector<size_t>& selectable_indices)
{
  // construct full representation of our desired text

  // initialize vectors
  const size_t selectable_length = service->advertisers.size();
  const size_t length = /*header=*/3 + /*section_headers=*/1 * 2 + selectable_length;
  selectables.reserve(selectable_length);
  selectable_indices.reserve(selectable_length);
  full_text.reserve(length);

  // running tally variables
  size_t selectable_idx = 0;

  // add fixed header information
  full_text.emplace_back(*_active_service + ((service->active) ? " (active)" : " (inactive)"));
  full_text.emplace_back("  Service Type: " + service->type);
  full_text.emplace_back("  Advertisers:  " + std::to_string(service->advertisers.size()));
  full_text.emplace_back("");
  full_text.emplace_back("Advertisers:");

  // add selectable items (advertisers)
  selectable_idx = full_text.size();
  for (const auto& item : service->advertisers)
  {
    selectables.emplace_back(ROSType::ADVERTISER, item);
    full_text.emplace_back(" " + item);
    selectable_indices.emplace_back(selectable_idx++);
  }
}

ActionPacket ServiceInfoPanel::render(const std::optional<ComputationalGraph>& graph)
{
  // start from a blank slate
  redraw();
  mvwaddnstr(_window, 0, 1, "ROS Service Summary", _cols - 2 * BORDER);

  // if we don't have a graph or topic selected we can't do anything
  if (!graph || !_active_service || graph->services().find(*_active_service) == graph->services().end())
    return NULL_ACTION;

  // otherwise print some info about this topic
  const auto& service = graph->services().at(*_active_service);

  // construct desired textual representation and metadata
  std::vector<std::string> full_text;
  std::vector<std::pair<ROSType, std::string>> selectables;
  std::vector<size_t> selectable_indices;
  construct(service, full_text, selectables, selectable_indices);

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

} // namespace ros_curses::panels