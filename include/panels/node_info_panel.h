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

  // last index displayed in our visible region, to implement scrolling hysteresis
  size_t _last_start_idx {0};

  // currently highlighted information
  std::optional<std::pair<std::string, std::string>> _sub_selection;

 public:

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>& graph) override;

  /* Handle 'enter' keystroke.
   */
  virtual ActionPacket handle_enter() override;

  /* Override base class method to reset our state variables.
   */
  void set_visible(const bool visible) override;

}; // class NodeInfoPanel

} // namespace ros_curses::panels