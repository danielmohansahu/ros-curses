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
  // set common curses parameters for this window
  keypad(_window, true);
  nodelay(_window, true);
  scrollok(_window, true);
  leaveok(_window, true);

  // make sure we're not hidden, although we may be superseded
  show_panel(_panel);
}

void Panel::redraw()
{
  // clear contents
  wclear(_window);

  // indicate that we're active via a box
  if (_active)
    box(_window, ACS_VLINE, ACS_HLINE);

  // indicate that we should be completely redrawn on next doupdate
  touchwin(_window);
}

void Panel::set_active()
{
  _active = true;
  top_panel(_panel);
  wmove(_window, 0, 0);
  redraw();
}

void Panel::set_inactive(const bool hide)
{
  if (hide)
    hide_panel(_panel);
  // clear contents
  _active = false;
  redraw();
}

void Panel::move_and_resize(const size_t rows, const size_t cols, const size_t y, const size_t x)
{
  // resize window, move it, and then replace the panel
  wresize(_window, rows, cols);
  mvwin(_window, y, x);
  replace_panel(_panel, _window);
  redraw();
}

void Panel::write(const std::vector<ros_curses::LineDatum>& lines)
{
  // write the given data directly to our screen
  //  @TODO handle formatting
  for (size_t i = 0; i != lines.size(); ++i)
    mvwaddstr(_window, i + 1, 1, lines[i].text.c_str());
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
  curs_set(0);            // don't show the cursor
  leaveok(stdscr, true);  // don't move the cursor unless explicitly told

  // instantiate panels via std::unordered_map's autoconstruction
  _panels[PanelNames::INITIALIZATION];
  _panels[PanelNames::HELP];
  _panels[PanelNames::NODEINFO];
  _panels[PanelNames::NODELIST];
  _panels[PanelNames::TOPICINFO];
  _panels[PanelNames::TOPICLIST];
  _panels[PanelNames::SERVICEINFO];
  _panels[PanelNames::SERVICELIST];
  _panels[PanelNames::PARAMINFO];
  _panels[PanelNames::PARAMLIST];

  // resize to fit current display
  resize();

  // start with only the INITIALIZATION screen active
  activate(PanelNames::INITIALIZATION);

  // add header information
  update_header("initializing...");
}

Display::~Display()
{
  // curses shutdown
  endwin();
}

void Display::activate(const PanelNames panel, bool hide)
{
  // save last state to support toggling
  _last_active = _active;
  _active = panel;

  // set states of appropriate panels
  _panels.at(_last_active).set_inactive(hide);
  _panels.at(_active).set_active();
}

ros_curses::Action Display::process(const std::unordered_map<ros_curses::PanelNames,std::vector<ros_curses::LineDatum>>& updates)
{
  // core method to be called every loop

  // perform updates for screens handle internally
  update_header();
  update_help_panel();

  // process incoming data
  for (const auto& [key, data] : updates)
    _panels.at(key).write(data);

  // process user input
  const Action action = process_user_input();

  // perform refresh of all windows / panels
  refresh();
  update_panels();
  doupdate();

  // return action
  return action;
}

ros_curses::Action Display::process_user_input()
{
  // initialize action to be NONE
  Action action = Action::NONE;

  // handle user input in active display
  const int ch = _panels.at(_active).get_ch();
  switch (ch)
  {
    case ERR:           // no user input; do nothing
      break;
    case KEY_RESIZE:    // terminal resize event
      resize();
      break;
    case KEY_UP:        // move selection up
      action = Action::DECREMENT;
      break;
    case KEY_DOWN:      // move selection down
      action = Action::INCREMENT;
      break;
    case KEY_LEFT:      // move selection left
    case KEY_RIGHT:     // move selection right
      switch_displays();
      break;
    case int('q'):      // user requested we exit the current screen
      if (_active != PanelNames::HELP)
        return Action::EXIT;
    case int('h'):      // toggle help screen
    case int('?'):      // toggle help screen
      (_active != PanelNames::HELP) ? activate(PanelNames::HELP) : activate(_last_active, true);
      break;
    case int('\t'):     // tab between panel display options
      cycle_displays();
      break;
    default:
      _panels.at(_active).debug("Unknown command: " + std::to_string(ch));
      break;
  }

  // indicate that no user action is required.
  return action;
}

void Display::update_header(const std::string& status)
{
  // latch header information
  if (status != "")
    _header_status = status;

  // use the stdscr for all our header information
  mvaddstr(0, 0, "ros-curses: Command line introspection of the ROS computational graph.");
  mvprintw(1, 0, "  Status: %s", _header_status.c_str());
  mvprintw(2, 0, "  Active: %u", _active);
}

void Display::cycle_displays(const bool reverse)
{
  // cycle through displays
  switch (_active)
  {
    case PanelNames::NODELIST:
    case PanelNames::NODEINFO:
      (reverse) ? activate(PanelNames::PARAMLIST) : activate(PanelNames::TOPICLIST);
      break;
    case PanelNames::TOPICLIST:
    case PanelNames::TOPICINFO:
      (reverse) ? activate(PanelNames::NODELIST) : activate(PanelNames::SERVICELIST);
      break;
    case PanelNames::SERVICELIST:
    case PanelNames::SERVICEINFO:
      (reverse) ? activate(PanelNames::TOPICLIST) : activate(PanelNames::PARAMLIST);
      break;
    case PanelNames::PARAMLIST:
    case PanelNames::PARAMINFO:
      (reverse) ? activate(PanelNames::SERVICELIST) : activate(PanelNames::NODELIST);
      break;
    default:
      // do nothing; these panels can't be cycled!
      break;
  }
}

void Display::switch_displays()
{
  // update to activate the left or right display
  switch(_active)
  {
    case PanelNames::NODELIST: activate(PanelNames::NODEINFO); break;
    case PanelNames::NODEINFO: activate(PanelNames::NODELIST); break;
    case PanelNames::TOPICLIST: activate(PanelNames::TOPICINFO); break;
    case PanelNames::TOPICINFO: activate(PanelNames::TOPICLIST); break;
    case PanelNames::SERVICELIST: activate(PanelNames::SERVICEINFO); break;
    case PanelNames::SERVICEINFO: activate(PanelNames::SERVICELIST); break;
    case PanelNames::PARAMLIST: activate(PanelNames::PARAMINFO); break;
    case PanelNames::PARAMINFO: activate(PanelNames::PARAMLIST); break;
    default: break;
  }
}

void Display::resize()
{
  // get terminal size
  int rows, cols;
  getmaxyx(stdscr, rows, cols);

  // INITIALIZATION screen gets the full size except for a few header rows
  _panels.at(PanelNames::INITIALIZATION).move_and_resize(rows - HEADER_ROWS, cols, HEADER_ROWS, 0);

  // help panel gets a fixed width in the middle of the screen
  _panels.at(PanelNames::HELP).move_and_resize(4 * rows / 5, HELP_COLS, rows / 10, (cols - HELP_COLS) / 2);

  // list displays get the left half of the screen
  _panels.at(PanelNames::NODELIST).move_and_resize(rows - HEADER_ROWS, cols / 2, HEADER_ROWS, 0);
  _panels.at(PanelNames::TOPICLIST).move_and_resize(rows - HEADER_ROWS, cols / 2, HEADER_ROWS, 0);
  _panels.at(PanelNames::SERVICELIST).move_and_resize(rows - HEADER_ROWS, cols / 2, HEADER_ROWS, 0);
  _panels.at(PanelNames::PARAMLIST).move_and_resize(rows - HEADER_ROWS, cols / 2, HEADER_ROWS, 0);

  // info displays get the right half of the screen
  _panels.at(PanelNames::NODEINFO).move_and_resize(rows - HEADER_ROWS, cols / 2, HEADER_ROWS, cols / 2);
  _panels.at(PanelNames::TOPICINFO).move_and_resize(rows - HEADER_ROWS, cols / 2, HEADER_ROWS, cols / 2);
  _panels.at(PanelNames::SERVICEINFO).move_and_resize(rows - HEADER_ROWS, cols / 2, HEADER_ROWS, cols / 2);
  _panels.at(PanelNames::PARAMINFO).move_and_resize(rows - HEADER_ROWS, cols / 2, HEADER_ROWS, cols / 2);
}

void Display::update_help_panel()
{
  // hardcoded information for the help window
  std::vector<ros_curses::LineDatum> message;
  message.emplace_back("ros-curses help");
  message.emplace_back("       UP : move cursor up");
  message.emplace_back("     DOWN : move cursor down");
  message.emplace_back("     LEFT : switch active display");
  message.emplace_back("    RIGHT : switch active display");
  message.emplace_back("      TAB : cycle panels");
  message.emplace_back("     h, ? : toggle help menu");
  message.emplace_back("        q : quit / exit");

  // write message
  _panels.at(PanelNames::HELP).write(message);
}

} // namespace ros_curses