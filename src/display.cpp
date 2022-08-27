/*
 *
 *
 */

// ros_curses
#include "display.h"

namespace curses
{

Display::Display(const std::shared_ptr<ros_curses::Updater>& updater)
 : _updater(updater)
{
  // curses initialization

  // I am a stub
}


Display::~Display()
{
  // curses shutdown

  // I am a stub
}

ros_curses::Action Display::process_user_input()
{
  // I am a stub
  return Action::NONE;
}


}; // namespace ros_curses