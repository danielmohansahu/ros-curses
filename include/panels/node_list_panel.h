/*
 *
 *
 * 
 */

#pragma once

// STL
#include <optional>

// ros_curses
#include "panel_base.h"

namespace ros_curses::panels
{

/* A Panel to list all available nodes.
 */
class NodeListPanel : public PanelBase
{
 private:

 public:

  NodeListPanel() : PanelBase(true) {}

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>& graph) override;

}; // class NodeListPanel

} // namespace ros_curses::panels