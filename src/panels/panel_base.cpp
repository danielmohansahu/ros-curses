/*
 *
 *
 * 
 */

// ros_curses
#include "panels/panel_base.h"

namespace ros_curses::panels
{


PanelBase::PanelBase(size_t rows, size_t cols, size_t origin_y, size_t origin_x)
 : _window(newwin(rows, cols, origin_y, origin_x)), _panel(new_panel(_window))
{
  // set common curses parameters for this window
  keypad(_window, true);
  nodelay(_window, true);
  leaveok(_window, true);

  // default state is visible, inactive
  show_panel(_panel);
}

void PanelBase::redraw()
{
  // clear contents
  wclear(_window);

  // indicate that we're active via a solid box or inactive via dashed
  if (_active)
    box(_window, ACS_VLINE, ACS_HLINE);
  else
    box(_window, '|', '-');

  // indicate that we should be completely redrawn on next doupdate
  touchwin(_window);
}

void PanelBase::set_active(const bool active)
{
  // put on top of the panel stack, if active
  if (active)
    top_panel(_panel);

  // set state and redraw
  _active = active;
  redraw();
}

void PanelBase::set_visible(const bool visible)
{
  if (visible)
    show_panel(_panel);
  else
    hide_panel(_panel);
}

void PanelBase::move_and_resize(const size_t rows, const size_t cols, const size_t y, const size_t x)
{
  // store size info
  _rows = rows;
  _cols = cols;

  // resize window, move it, and then replace the panel
  wresize(_window, rows, cols);
  mvwin(_window, y, x);
  replace_panel(_panel, _window);
  redraw();
}

} // namespace ros_curses::panels