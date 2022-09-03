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
  std::optional<std::pair<ROSType, std::string>> _sub_selection;

  /* Construct required representations of our desired display.
   *
   * Args:
   *    service: The currently selected service, with all ROS primitive information.
   *    full_text: vector of strings we'll populate with the entirety of our desired display
   *    selectables: a vector of _unique_ selectable items
   *    selectable_indices: the corresponding indices of 'selectables' in 'full_text'
   */
  void construct(const auto& service,
                 std::vector<std::string>& full_text,
                 std::vector<std::pair<ROSType, std::string>>& selectables,
                 std::vector<size_t>& selectable_indices);

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