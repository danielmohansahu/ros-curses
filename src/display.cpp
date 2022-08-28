/*
 *
 *
 */

// ros_curses
#include "display.h"

namespace curses
{

Panel::Panel(size_t rows, size_t cols, size_t origin_y, size_t origin_x)
{
  // create a new window
  _window = std::make_unique<WINDOW>(*newwin(rows, cols, origin_y, origin_x));

  // create a new panel
  _panel = std::make_unique<PANEL>(*new_panel(&(*_window)));
}

Display::Display(const std::shared_ptr<ros_curses::Updater>& updater)
 : _updater(updater)
{
  // curses initialization
  initscr();
  start_color();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);

  // create panels

}


Display::~Display()
{
  // curses shutdown
  endwin();
}

ros_curses::Action Display::process_user_input()
{
  // I am a stub
  return Action::NONE;
}


}; // namespace ros_curses