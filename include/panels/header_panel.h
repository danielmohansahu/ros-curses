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

/* Panel containing header information.
 */
class HeaderPanel : public PanelBase
{
 private:

  // some state variables
  std::string _status {"initializing..."};
  int _active {0};

 public:

  /* Allow external setting of the status message.
   */
  void set_status(const std::string& status) { _status = status; }

  /* Allow external setting of the status message.
   */
  void set_active(const int active) { _active = active; }

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>&) override;

  /* Handle 'up' keystroke.
   *
   * Disabled: we should never be 'active'
   */
  void handle_key_up() override {};

  /* Handle 'down' keystroke.
   *
   * Disabled: we should never be 'active'
   */
  void handle_key_down() override {};

}; // class HeaderPanel

} // namespace ros_curses::panels