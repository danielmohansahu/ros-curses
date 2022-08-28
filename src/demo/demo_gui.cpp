/* Demonstration of basic curses infrastructure, sans information.
 *
 * 
 * 
 */

// STL
#include <unistd.h>
#include <string>
#include <random>
#include <algorithm>
#include <optional>

// ros_curses
#include "display.h"
#include "parsers/mock.h"

int main(int argv, char** argc)
{
  using PanelNames = ros_curses::PanelNames;
  using LineDatum = ros_curses::LineDatum;

  // construct Mock parser to generate a random graph
  ros_curses::MockGraphParser mock {};

  // construct basic display class
  auto display = curses::Display();

  // activate display
  display.activate(PanelNames::NODELIST);

  // make a dummy empty data payload
  //  @todo generate randomly
  std::unordered_map<PanelNames,std::vector<LineDatum>> dummy_payload {};

  // process until shutdown
  while (display.process(dummy_payload) != ros_curses::Action::EXIT)
    usleep(10000);
}