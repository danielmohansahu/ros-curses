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

/* A Panel subclass to display service information.
 */
class ServiceInfoPanel : public PanelBase
{
 private:

   // amount to shift to select next node (expected -1,1, but forward compatible)
  int _shift {0};

  // scrolling calculation helper class
  ScrollRegion _scroll;

  // last index displayed in our visible region, to implement scrolling hysteresis
  size_t _last_start_idx {0};

  // currently highlighted information
  std::optional<std::pair<std::string, std::string>> _sub_selection;

 public:

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>& graph) override;

  /* Handle 'up' keystroke.
   */
  void handle_key_up() override { _shift = -1; };

  /* Handle 'down' keystroke.
   */
  void handle_key_down() override { _shift = 1; };

  /* Handle 'enter' keystroke.
   */
  virtual ActionPacket handle_enter() override;

  /* Override base class method to reset our state variables.
   */
  void set_visible(const bool visible) override;

  /* Override base class method to reset our state variables.
   */
  void move_and_resize(const size_t rows, const size_t cols, const size_t y, const size_t x);

}; // class ServiceInfoPanel

} // namespace ros_curses::panels