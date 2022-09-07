/*
 *
 *
 *
 */

#pragma once

// STL
#include <optional>

// ros_curses
#include "base/panel_base.h"

namespace ros_curses::panels
{

/* A Panel subclass used as a placeholder.
 */
class HelpPanel : public PanelBase
{
 private:

  // hardcoded scrollable section of display
  const std::vector<std::string> _items;

 public:

  /* Class constructor.
   */
  HelpPanel();

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>&) override;

}; // class TestPanel

} // namespace ros_curses::panels