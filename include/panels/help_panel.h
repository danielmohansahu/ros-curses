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
class HelpPanel : public PanelBase
{
 private:

  // hardcoded scrollable section of display
  const std::vector<std::string> _items;

  // scrolling indices calculation helper class
  ScrollRegion _scroll;

  // currently selected index; determines oversized display section
  size_t _scroll_offset;

 public:

  /* Class constructor.
   */
  HelpPanel();

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>&) override;

  /* Handle 'up' keystroke.
   */
  void handle_key_up() override;

  /* Handle 'down' keystroke.
   */
  void handle_key_down() override;

  /* Override base class method to reset our IDX counter.
   */
  void set_visible(const bool visible) override;

  /* Override base class method to update our scroll calculations.
   */
  void move_and_resize(const size_t rows, const size_t cols, const size_t y, const size_t x);

}; // class TestPanel

} // namespace ros_curses::panels