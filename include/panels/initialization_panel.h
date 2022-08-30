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
class InitializationPanel : public PanelBase
{
 private:

 public:

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>&) override;

  /* Handle 'up' keystroke.
   */
  void handle_key_up() override {};

  /* Handle 'down' keystroke.
   */
  void handle_key_down() override {};

}; // class HeaderPanel

} // namespace ros_curses::panels