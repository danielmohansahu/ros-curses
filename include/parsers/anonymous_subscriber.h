/* Anonymous Subscriber for minimal ROS Topic Introspection.
 *
 *
 */

// STL
#include <chrono>
#include <limits>
#include <future>
#include <atomic>
#include <mutex>

// XMLRPC
#include <xmlrpcpp/XmlRpc.h>

namespace ros_curses::ros1
{

struct Statistics
{
  const size_t window {std::numeric_limits<size_t>::max()};
  size_t count {0};
  float min {0};
  float max {0};
  float stddev {0};
  float rate {0};

  explicit Statistics(const size_t window_) : window(window_) {};
}; // stuct Statistics

/* Lightweight connection to an advertises ROS Topic
 */
class AnonymousSubscriber
{
 private:

  // core thread object
  std::future<void> _thread_future;

  // thread synchronization objects
  std::atomic<bool> _connected {false};
  std::atomic<bool> _exit {false};
  mutable std::mutex _stats_mutex;

  // tracked topic name
  const std::string _topic;

  // statistical information
  Statistics _stats;

 public:
  // no default constructor
  AnonymousSubscriber() = delete;

  // shutdown on destruction
  ~AnonymousSubscriber();

  /* Constructor - begins attempted connection to given topic.
   *
   * Args:
   *    topic:  The string identifier of the topic.
   *    window: Window size for rate calculation.
   */
  explicit AnonymousSubscriber(const std::string& topic, const size_t window = std::numeric_limits<size_t>::max());

  /* Get publishing statistics.
   *
   * Args:
   *    rate:   The average rate (hz) over our window.
   *    min:    The minimum delay between messages received (s).
   *    max:    The maximum delay between messages received (s).
   *    stddev: The standard deviation of results.
   *    window: The window over which we're calculating.
   */
  bool get_statistics(float& rate, float& min, float& max, float& stddev, size_t& window) const;

  /* Poll for connection status.
   */
  bool connected() const;

 private:

  /* Create a connection to the targe topic.
   */
  bool connect();

  /* Initialization and polling thread - this will try to connect asynchronously.
   */
  void connect_and_poll();

  /* Core callback for incoming published data.
   */
  void callback();

}; // class AnonymousSubscriber

} // namespace ros_curses::ros1