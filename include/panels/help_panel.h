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

  // currently selected index; determines oversized display section
  size_t _scroll_offset;

 public:

  /* Class constructor.
   */
  HelpPanel();

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  void render(const std::optional<ComputationalGraph>&) override;

  /* Handle 'up' keystroke.
   */
  void handle_key_up() override;

  /* Handle 'down' keystroke.
   */
  void handle_key_down() override;

  /* Handle 'enter' keystroke.
   *
   * We don't support 'ENTER'.
   */
  void handle_enter() override {};

  /* Override base class method to reset our IDX counter.
   */
  void set_visible(const bool visible) override;

}; // class TestPanel

} // namespace ros_curses::panels