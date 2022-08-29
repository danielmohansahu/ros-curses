/*
 *
 *
 * 
 */

// STL
#include <algorithm>

// ros_curses
#include "panels/help_panel.h"

namespace ros_curses::panels
{

HelpPanel::HelpPanel()
 : _items({
  "       UP : move cursor up",
  "     DOWN : move cursor down",
  "     LEFT : switch active display",
  "    RIGHT : switch active display",
  "      TAB : cycle panels",
  "     h, ? : toggle help menu",
  "        q : quit / exit",
  "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit", "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit",   "        q : quit / exit", "end"
  })
{
}

void HelpPanel::render(const std::optional<ComputationalGraph>&)
{
  // completely redraw, to start with a blank slate
  redraw();

  // help panel header - not scrollable
  mvwaddstr(_window, 0, 1, "ros-curses help");

  // visible region parameters
  const size_t visible_start_idx = 1;
  const size_t visible_end_buffer = 1;
  const size_t visible_region_size = _rows - visible_start_idx - visible_end_buffer;

  // scroll parameters
  const bool scroll_required = visible_region_size < _items.size();
  _scroll_offset = (scroll_required) ? std::clamp(_scroll_offset, 0UL, _items.size() - visible_region_size) : 0;
  
  // loop parameters
  const size_t start_idx = _scroll_offset;
  const size_t end_idx = start_idx + std::min(visible_region_size - 1, _items.size());

  // iterate through visible section of items
  for (size_t i = start_idx; i != end_idx; ++i)
    mvwaddstr(_window, visible_start_idx + i - start_idx, 1, _items[i].c_str());

  // add a little blurb if we're scrolling
  if (scroll_required && _scroll_offset != _items.size() - visible_region_size)
    mvwaddstr(_window, visible_start_idx + end_idx - start_idx, 1, "  -- more --  ");
}

void HelpPanel::handle_key_up()
{
  if (_scroll_offset > 0)
    --_scroll_offset;
}

void HelpPanel::handle_key_down()
{
  ++_scroll_offset;
}

void HelpPanel::set_visible(const bool visible)
{
  // reset our selected index and call parent method
  _scroll_offset = 0;
  PanelBase::set_visible(visible);
}


} // namespace ros_curses::panels