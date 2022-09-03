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

/* A Panel subclass to display topic information.
 */
class NodeInfoPanel : public PanelBase
{
 private:

  // the node we're currently highlighting
  std::optional<std::string> _active_node;

  // currently highlighted information
  std::optional<std::pair<ROSType, std::string>> _sub_selection;

  /* Construct required representations of our desired display.
   *
   * Args:
   *    node: The currently selected node, with all ROS primitive information.
   *    full_text: vector of strings we'll populate with the entirety of our desired display
   *    selectables: a vector of _unique_ selectable items
   *    selectable_indices: the corresponding indices of 'selectables' in 'full_text'
   */
  void construct(const auto& node,
                 std::vector<std::string>& full_text,
                 std::vector<std::pair<ROSType, std::string>>& selectables,
                 std::vector<size_t>& selectable_indices);

 public:

  // constructor
  NodeInfoPanel() : PanelBase(true) {}

  /* Generate complete display from the given fully container ComputationalGraph.
   */
  ActionPacket render(const std::optional<ComputationalGraph>& graph) override;

  /* Handle 'enter' keystroke.
   */
  virtual ActionPacket handle_enter() override;

  /* Update currently activated node.
   */
  virtual void select(const std::optional<std::string>& node) override { _active_node = node; };

}; // class NodeInfoPanel

} // namespace ros_curses::panels