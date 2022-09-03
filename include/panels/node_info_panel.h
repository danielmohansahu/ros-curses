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
class NodeInfoPanel : public PanelBase
{
 private:

  // the node we're currently highlighting
  std::optional<std::string> _active_node;

  // currently highlighted information
  std::optional<std::pair<std::string, std::string>> _sub_selection;

 public:

  // constructor
  NodeInfoPanel() : PanelBase(true) {}

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>& graph) override;

  /* Handle 'enter' keystroke.
   */
  virtual ActionPacket handle_enter() override;

  /* Update currently activated node.
   */
  virtual void select(const std::optional<std::string>& node) override { _active_node = node; };

}; // class NodeInfoPanel

} // namespace ros_curses::panels