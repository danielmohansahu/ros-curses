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

/* A Panel subclass used as a placeholder.
 */
class TopicListPanel : public PanelBase
{
 private:

  // currently selected object
  std::optional<std::string> _selection;

  // amount to shift to select next node (expected -1,1, but forward compatible)
  int _shift {0};

  // scrolling calculation helper class
  ScrollRegion _scroll;

  // last index displayed in our visible region, to implement scrolling hysteresis
  size_t _last_start_idx {0};

 public:

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  void render(const std::optional<ComputationalGraph>& graph) override;

  /* Handle 'up' keystroke.
   */
  void handle_key_up() { _shift = -1; };

  /* Handle 'down' keystroke.
   */
  void handle_key_down() { _shift = 1; };

  /* Handle 'enter' keystroke.
   */
  void handle_enter() override {};

  /* Override base class method to reset our state variables.
   */
  void set_visible(const bool visible) override;

  /* Override base class method to reset our state variables.
   */
  void move_and_resize(const size_t rows, const size_t cols, const size_t y, const size_t x);

}; // class TopicListPanel

} // namespace ros_curses::panels