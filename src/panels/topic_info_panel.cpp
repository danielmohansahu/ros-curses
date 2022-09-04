/*
 *
 *
 * 
 */

// ros_curses
#include "panels/topic_info_panel.h"

namespace ros_curses::panels
{

void TopicInfoPanel::construct(const auto& topic,
                               std::vector<std::string>& full_text,
                               std::vector<std::pair<ROSType, std::string>>& selectables,
                               std::vector<size_t>& selectable_indices)
{
  // construct full representation of our desired text

  // initialize vectors
  const size_t selectable_length = topic->publishers.size() + topic->subscribers.size();
  const size_t length = /*header=*/4 + /*section_headers=*/2 * 2 + /*enable_polling=*/ 3 + selectable_length;
  selectables.reserve(selectable_length);
  selectable_indices.reserve(selectable_length);
  full_text.reserve(length);

  // running tally variables
  size_t selectable_idx = 0;

  // add fixed header information
  full_text.emplace_back(*_active_topic + ((topic->active) ? " (active)" : " (inactive)"));
  full_text.emplace_back("  Message Type: " + topic->type);
  full_text.emplace_back("  Publishers:   " + std::to_string(topic->publishers.size()));
  full_text.emplace_back("  Subscribers:  " + std::to_string(topic->subscribers.size()));
  full_text.emplace_back("");
  full_text.emplace_back("Publishers:");

  // add selectable items (publihsers)
  selectable_idx = full_text.size();
  for (const auto& item : topic->publishers)
  {
    selectables.emplace_back(ROSType::PUBLISHER, item);
    full_text.emplace_back(" " + item);
    selectable_indices.emplace_back(selectable_idx++);
  }

  // add sub-header
  full_text.emplace_back("");
  full_text.emplace_back("Subscribers:");
  selectable_idx += 2;

  // add selectable items (subscriptions)
  for (const auto& item : topic->subscribers)
  {
    selectables.emplace_back(ROSType::SUBSCRIBER, item);
    full_text.emplace_back(" " + item);
    selectable_indices.emplace_back(selectable_idx++);
  }

  // add sub header for topic polling
  full_text.emplace_back("");
  selectables.emplace_back(ROSType::LOCAL_SUBSCRIBER, topic->name);
  full_text.emplace_back("Toggle 'rostopic hz '" + topic->name + "' " + ((_local_subscription) ? "OFF" : "ON"));
  selectable_indices.emplace_back(selectable_idx + 1);

  // display rostopic hz info, if available
  float rate, min, max, stddev;
  size_t window;
  if (_local_subscription && _local_subscription->get_statistics(rate, min, max, stddev, window))
    full_text.emplace_back(" rate: " + std::to_string(rate) + " | min: " + std::to_string(min));
  else if (_local_subscription)
    full_text.emplace_back(" Trying to connect...");
  else
    full_text.emplace_back(" Disconnected");
}

ActionPacket TopicInfoPanel::render(const std::optional<ComputationalGraph>& graph)
{
  // start from a blank slate
  redraw();
  mvwaddnstr(_window, 0, 1, "ROS Topic Summary", _cols - 2 * BORDER);

  // if we don't have a graph or topic selected we can't do anything
  if (!graph || !_active_topic || graph->topics().find(*_active_topic) == graph->topics().end())
    return NULL_ACTION;
  
  // otherwise print some info about this topic
  const auto& topic = graph->topics().at(*_active_topic);

  // construct desired textual representation and metadata
  std::vector<std::string> full_text;
  std::vector<std::pair<ROSType, std::string>> selectables;
  std::vector<size_t> selectable_indices;
  construct(topic, full_text, selectables, selectable_indices);

  // get the visible region bounds
  const auto [begin, end, selection_idx, highlight_idx] = _scroll.update(selectables, selectable_indices, full_text.size());
  _sub_selection = selectables[selection_idx];

  // iterate through and display the visible section of text
  for (size_t i = begin; i != end; ++i)
    print_line(BORDER + i - begin, full_text[i], format(i == highlight_idx));

  // add a little blurb if we're scrolling
  if (_scroll.scroll_required(full_text.size()))
    print_line_center(BORDER + end - begin, (end < full_text.size()) ? "-- more --" : "-- end --");

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

  // handle a switch
  if (_sub_selection->first == ROSType::PUBLISHER || _sub_selection->first == ROSType::SUBSCRIBER)
    return {DISPLAY_NODE, _sub_selection->second};

  // check if user requested a local subscription
  if (_sub_selection->first == ROSType::LOCAL_SUBSCRIBER)
  {
    // toggle on or off
    if (_local_subscription)
      _local_subscription.reset();
    else
      _local_subscription = std::make_unique<ros1::AnonymousSubscriber>("ros_curses_hz");
    return NULL_ACTION;
  }

  // no other options... shouldn't get here
  assert(false);
  return NULL_ACTION;
}

} // namespace ros_curses::panels