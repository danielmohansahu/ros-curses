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

using namespace ros_curses;

int main(int argv, char** argc)
{
  // construct Mock parser to generate a random graph
  MockGraphParser mock {};

  // construct basic display class
  auto display = Display();

  // activate display
  display.activate(PanelNames::NODELIST);

  // make a dummy empty data payload
  //  @todo generate randomly
  std::unordered_map<PanelNames,std::vector<LineDatum>> dummy_payload {};

  // process until shutdown
  while (display.process(mock.poll()) != Action::EXIT)
    usleep(10000);
}