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

  /* Class constructor.
   */
  TestPanel();

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  void render(const std::optional<ComputationalGraph>& graph) override;

  /* Handle 'up' keystroke.
   */
  void handle_key_up() override;

  /* Handle 'down' keystroke.
   */
  void handle_key_down() override;

  /* Handle 'enter' keystroke.
   */
  void handle_enter() override;


}; // class TestPanel

} // namespace ros_curses::panels