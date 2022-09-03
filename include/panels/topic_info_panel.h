/*
 *
 *
 * 
 */

#pragma once

// ros_curses
#include "panel_base.h"

namespace ros_curses::panels
{

/* A Panel subclass to display topic information.
 */
class TopicInfoPanel : public PanelBase
{
 private:

  // the topic we're currently highlighting
  std::optional<std::string> _active_topic;

  // currently highlighted information
  std::optional<std::pair<ROSType, std::string>> _sub_selection;

  /* Construct required representations of our desired display.
   *
   * Args:
   *    topic: The currently selected topic, with all ROS primitive information.
   *    full_text: vector of strings we'll populate with the entirety of our desired display
   *    selectables: a vector of _unique_ selectable items
   *    selectable_indices: the corresponding indices of 'selectables' in 'full_text'
   */
  void construct(const auto& topic,
                 std::vector<std::string>& full_text,
                 std::vector<std::pair<ROSType, std::string>>& selectables,
                 std::vector<size_t>& selectable_indices);

 public:

  // constructor
  TopicInfoPanel() : PanelBase(true) {}

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>& graph) override;

  /* Handle 'enter' keystroke.
   */
  virtual ActionPacket handle_enter() override;

  /* Update currently activated topic.
   */
  virtual void select(const std::optional<std::string>& topic) override { _active_topic = topic; };

}; // class TopicInfoPanel

} // namespace ros_curses::panels