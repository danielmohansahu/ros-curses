/*
 *
 *
 */

// ros_curses
#include "display.h"

namespace curses
{

Panel::Panel(size_t rows, size_t cols, size_t origin_y, size_t origin_x)
 : _window(newwin(rows, cols, origin_y, origin_x)), _panel(new_panel(_window))
{
  keypad(_window, true);
  nodelay(_window, true);
  scrollok(_window, true);
}

void Panel::set_active()
{
  // I am a stub;
}

Display::Display(const std::shared_ptr<ros_curses::Updater>& updater)
 : _updater(updater)
{
  // curses initialization
  initscr();              // initialize screens
  start_color();          // allow color processing
  cbreak();               // ???
  noecho();               // don't echo user input to screen
  keypad(stdscr, true);   // properly interpret keypad / arrow keys
  nodelay(stdscr, true);  // don't block on calls for user input

  // instantiate panels via std::unordered_map's autoconstruction
  _panels[PanelNames::HELP];
  _panels[PanelNames::NODEINFO];
  _panels[PanelNames::NODELIST];
  _panels[PanelNames::TOPICINFO];
  _panels[PanelNames::TOPICLIST];
  _panels[PanelNames::SERVICEINFO];
  _panels[PanelNames::SERVICELIST];
  _panels[PanelNames::PARAMINFO];
  _panels[PanelNames::PARAMLIST];
  _panels[PanelNames::INITIALIZATION];

  // start with only the INITIALIZATION screen active
  _panels.at(_active).set_active();
  _panels.at(_active).show();

  // resize to fit current display
  resize();
}

Display::~Display()
{
  // curses shutdown
  endwin();
}

ros_curses::Action Display::process_user_input()
{
  // handle user input in active display
  const int ch = _panels.at(_active).get_ch();
  switch (ch)
  {
    case ERR:           // no user input; do nothing
      return Action::NONE;
      break;
    case KEY_RESIZE:    // terminal resize event
      resize();
      break;
    case KEY_UP:        // move selection up
      // @TODO
      break;
    case KEY_DOWN:      // move selection down
      // @TODO
      break;
    case KEY_LEFT:      // move selection left
      // @TODO
      break;
    case KEY_RIGHT:     // move selection right
      // @TODO
      break;
    case int('q'):      // user requested exit
      return Action::EXIT;
      break;
    case int('\t'):     // tab between panel display options
      // @TODO
      break;
    case int('h'):      // display help screen
    case int('?'):      // display help screen
      // @TODO
      break;
    default:
      _panels.at(_active).add_str(0, 0, "Unknown command: " + std::to_string(ch));
      break;
  }

  // indicate that no user action is required.
  return Action::NONE;
}

void Display::resize()
{
  
}

} // namespace ros_curses