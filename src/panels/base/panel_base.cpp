/*
 *
 *
 *
 */

// STL
#include <assert.h>

// ros_curses
#include "panels/base/panel_base.h"

namespace ros_curses::panels
{

PanelBase::PanelBase(size_t rows, size_t cols, size_t origin_y, size_t origin_x, bool selectable)
 : _window(newwin(rows, cols, origin_y, origin_x)), _panel(new_panel(_window)), _scroll(selectable)
{
  // set common curses parameters for this window
  keypad(_window, true);
  nodelay(_window, true);
  leaveok(_window, true);
  clearok(_window, true);

  // default state is visible, inactive
  show_panel(_panel);
  redraw();
}

void PanelBase::redraw()
{
  // clear contents
  werase(_window);

  // draw border
  draw_border();

  // indicate that we should be completely redrawn on next doupdate
  touchwin(_window);
}

void PanelBase::draw_border()
{
  // indicate that we're active via a solid box or inactive via dashed
  if (_active)
    box(_window, ACS_VLINE, ACS_HLINE);
  else
    box(_window, '|', '-');
}

int PanelBase::format(const bool selected, const bool zombie) const
{
  // highlight based on available information
  if (!selected && !zombie)
    return A_NORMAL;
  else if (!selected && zombie)
    return A_DIM;
  else if (selected && !_active)
    return A_STANDOUT | A_DIM;
  else if (selected && _active)
    return A_STANDOUT;

  // we made a programming mistake if we get here
  assert(false);
  return A_NORMAL;
}

void PanelBase::set_active(const bool active)
{
  // put on top of the panel stack, if active
  if (active)
    top_panel(_panel);
  else
    clear_filter();

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
  _scroll.reset();
  _visible = visible;
}

bool PanelBase::visible() const
{
  return _visible;
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

  // update scroll region (with an extra line if filtering)
  if (filtering())
    _scroll.resize(_rows - BORDER * 4);
  else
    _scroll.resize(_rows - BORDER * 3);

  // actually redraw representation
  redraw();
}

void PanelBase::print_line(const size_t row, const std::string& str, const int format)
{
  // enable custom formatting
  if (format != A_NORMAL)
    wattron(_window, format);

  // write to line
  mvwaddnstr(_window, row, BORDER, str.c_str(), _cols - BORDER);

  // clear to end of line
  wclrtoeol(_window); // clear residual characters

  // turn off custom formatting
  if (format != A_NORMAL)
    wattroff(_window, format);
}

void PanelBase::print_line_center(const size_t row, const std::string& str, const int format)
{
  // same as 'print_line', but print in the center of the available space

  // handle edge case where the string is too long:
  if (str.size() >= (_cols - 2 * BORDER))
  {
    print_line(row, str, format);
    return;
  }

  // enable custom formatting
  if (format != A_NORMAL)
    wattron(_window, format);

  size_t start_col = (_cols - 2 * BORDER - str.size()) / 2;
  mvwaddstr(_window, row, start_col, str.c_str());

  // clear to end of line
  wclrtoeol(_window); // clear residual characters

  // turn off custom formatting
  if (format != A_NORMAL)
    wattroff(_window, format);
}

} // namespace ros_curses::panels