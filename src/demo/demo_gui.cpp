/* Demonstration of basic curses infrastructure, sans information.
 *
 * 
 * 
 */

// STL
#include <unistd.h>

// ros_curses
#include "display.h"
#include "updater.h"

/* Mock Updater class to populate the panels.
 */

class MockUpdater : public ros_curses::Updater
{
 public:
  std::optional<std::vector<ros_curses::LineDatum>> update(const std::string& panel) override
  {
    return std::nullopt;
  }

}; // class MockUpdater

int main(int argv, char** argc)
{
  // construct mock updater
  auto updater = std::make_shared<MockUpdater>();

  // construct basic display class
  auto display = curses::Display(updater);

  // process until shutdown
  while (display.process_user_input() != ros_curses::Action::EXIT)
    usleep(10000);
}