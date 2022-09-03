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
  std::optional<std::pair<std::string, std::string>> _sub_selection;

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