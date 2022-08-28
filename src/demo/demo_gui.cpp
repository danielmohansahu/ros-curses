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
#include "updater.h"

/* Mock Updater class to populate the panels.
 */

class MockUpdater : public ros_curses::Updater
{
 public:
  std::optional<std::vector<ros_curses::LineDatum>> update(const std::string& panel) override
  {
    std::vector<ros_curses::LineDatum> result;

    // create a random number of lines of gibberish
    const size_t lines = random_integer(_dist_500);
    for (size_t i = 0; i != lines; ++i)
      result.emplace_back(random_string(random_integer(_dist_500)), random_bool(), random_integer(_dist_16));
    return result;
  }

  MockUpdater()
   : _rng(_dev())
  {
  }

 private:
  // random number generators
  std::random_device _dev;
  std::mt19937 _rng;

  // consistently sized distributions
  std::uniform_int_distribution<std::mt19937::result_type> _dist_2 {0,1};
  std::uniform_int_distribution<std::mt19937::result_type> _dist_16 {0,100};
  std::uniform_int_distribution<std::mt19937::result_type> _dist_500 {0,500};

  bool random_bool()
  {
    return _dist_2(_rng) == 0;
  }

  size_t random_integer(size_t min, size_t max)
  {
    std::uniform_int_distribution<std::mt19937::result_type> dist(min, max);
    return dist(_rng);
  }

  size_t random_integer(std::uniform_int_distribution<std::mt19937::result_type>& dist)
  {
    return dist(_rng);
  }

  std::string random_string(size_t length) const
  {
    // https://stackoverflow.com/a/12468109/2460835
    auto randchar = []() -> char
    {
        const char charset[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz                                        ";
        const size_t max_index = (sizeof(charset) - 1);
        return charset[ rand() % max_index ];
    };
    std::string str(length,0);
    std::generate_n(str.begin(), length, randchar);
    return str;
  }

}; // class MockUpdater

int main(int argv, char** argc)
{
  // construct mock updater
  auto updater = std::make_shared<MockUpdater>();

  // construct basic display class
  auto display = curses::Display(updater);

  // activate display
  display.activate(ros_curses::PanelNames::NODELIST);

  // process until shutdown
  while (display.process_user_input() != ros_curses::Action::EXIT)
    usleep(10000);
}