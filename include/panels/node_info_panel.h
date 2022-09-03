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

}; // class NodeInfoPanel

} // namespace ros_curses::panels