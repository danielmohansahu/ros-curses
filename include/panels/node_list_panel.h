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

  // last index displayed in our visible region, to implement scrolling hysteresis
  size_t _last_start_idx {0};

 public:

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>& graph) override;

  /* Override base class method to reset our state variables.
   */
  void set_visible(const bool visible) override;

}; // class NodeListPanel

} // namespace ros_curses::panels