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

/* A Panel subclass used as a placeholder.
 */
class TestPanel : public PanelBase
{
 private:

 public:

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>& graph) override;

  /* Handle 'up' keystroke.
   */
  void handle_key_up() override;

  /* Handle 'down' keystroke.
   */
  void handle_key_down() override;

}; // class TestPanel

} // namespace ros_curses::panels