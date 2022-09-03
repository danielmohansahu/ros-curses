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

/* A Panel subclass to display all available services.
 */
class ServiceListPanel : public PanelBase
{
 private:

 public:

  // constructor
  ServiceListPanel() : PanelBase(true) {}

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>& graph) override;

}; // class ServiceListPanel

} // namespace ros_curses::panels