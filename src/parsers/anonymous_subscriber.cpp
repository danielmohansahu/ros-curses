/* Anonymous Subscriber for minimal ROS Topic Introspection.
 *
 *
 */

// ros_curses
#include "parsers/anonymous_subscriber.h"

namespace ros_curses::ros1
{

// for chrono time literals
using namespace std::chrono_literals;

AnonymousSubscriber::AnonymousSubscriber(const std::string& topic, const size_t window)
 : _topic(topic), _stats(window)
{
  // spin off connection thread
  _thread_future = std::async(std::launch::async, &AnonymousSubscriber::connect_and_poll, this);
}

AnonymousSubscriber::~AnonymousSubscriber()
{
  _exit = true;
  if (_thread_future.valid())
    _thread_future.get();
}

bool AnonymousSubscriber::get_statistics(float& rate, float& min, float& max, float& stddev, size_t& window) const
{
  // make sure we're connected
  if (!connected())
    return false;

  // acquire lock and update stats
  std::lock_guard<std::mutex> lock(_stats_mutex);
  rate =    _stats.rate;
  min =     _stats.min;
  max =     _stats.max;
  stddev =  _stats.stddev;
  window =  _stats.window;

  // indicate success
  return true;
}

bool AnonymousSubscriber::connected() const
{
  // return true only if the thread is still active and first time connection is  true (_connected)
  if (!_connected)
    return false;

  // if the thread has exited we're definitely not connected
  if (!_thread_future.valid())
    return false;

  // see if the thread as _just_ ended
  if (_thread_future.wait_for(0s) == std::future_status::ready)
    return false;

  // otherwise, we're connected
  return true;
}

bool AnonymousSubscriber::connect()
{
  // I am a stub
  return false;
}

void AnonymousSubscriber::connect_and_poll()
{
  // initialize connection
  if (_connected = connect(); !_connected)
    return;

  // process callbacks until shutdown.
  while (!_exit)
    std::this_thread::sleep_for(1s);
}

void AnonymousSubscriber::callback()
{
  // I am a stub
}

} // namespace ros_curses::ros1