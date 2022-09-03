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

  // the node we're currently highlighting
  std::optional<std::string> _active_service;

  // currently highlighted information
  std::optional<std::pair<std::string, std::string>> _sub_selection;

 public:

  // constructor
  ServiceInfoPanel() : PanelBase(true) {}

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>& graph) override;

  /* Handle 'enter' keystroke.
   */
  virtual ActionPacket handle_enter() override;

  /* Update currently activated service.
   */
  virtual void select(const std::optional<std::string>& service) override { _active_service = service; };

}; // class ServiceInfoPanel

} // namespace ros_curses::panels