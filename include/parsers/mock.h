/*
 *
 *
 * 
 */

#pragma once

// STL
#include <random>
#include <vector>
#include <string>
#include <optional>

// ros_curses
#include "../computational_graph.h"

namespace ros_curses
{

/* A class to generate a semi-random computational graph for visualization testing.
 */
class MockGraphParser
{
 private:
  // internal lists representing our basic system state
  std::vector<std::pair<std::string,std::vector<std::string>>> _publications, _subscriptions, _services;

  // connectedness state information
  bool _connected {false};
  double _connection_change_prob {0.0};

  // variables for random calculations
  std::random_device _dev;
  std::mt19937 _rng;
  const static inline std::vector<std::string> _common_names {
    "foo", "bar", "foobar", "topic_n", "node_i", "FOO", "BAR", "FOOBAR", "soup", "artichoke",
    "val", "IAMALONGNAME", "randomness", "to", "MY_NAME_IS_WAY_TOO_LONG_ITS_CRAZY_CMON_GUYS"
  };

  // common uniform int distributions
  std::uniform_int_distribution<std::mt19937::result_type> _dist_common_names{0, _common_names.size() - 1};
  std::uniform_int_distribution<std::mt19937::result_type> _dist_random_names{0, 50};

 public:
  MockGraphParser()
   : _rng(_dev())
  {
  }

  /* Return whether or not we have a (mock) connection.
   */
  bool connected()
  {
    // simulate a semi-random connection; every call makes this more likely
    if (random_bool(_connection_change_prob))
    {
      _connected = !_connected;
      _connection_change_prob = 0.0;
    }
    _connection_change_prob += 0.001;
    return _connected;
  }

  /* Update our current graph and return rendered CG.
   */
  std::optional<ComputationalGraph> poll()
  {
    // if we're not connected we can't return anything
    if (!connected())
      return std::nullopt;

    // there's a high probability nothing changes
    if (random_bool(0.1))
    {
      // looks like we got a change! update the graph

      // random deletions of topics / services
      for (size_t i = 0; i != _publications.size(); ++i)
        if (random_bool(0.05)) _publications.erase(_publications.begin() + i);
      for (size_t i = 0; i != _subscriptions.size(); ++i)
        if (random_bool(0.05)) _subscriptions.erase(_subscriptions.begin() + i);
      for (size_t i = 0; i != _services.size(); ++i)
        if (random_bool(0.05)) _services.erase(_services.begin() + i);

      // random additions of topics / services
      if (random_bool(0.1))
        _publications.emplace_back(random_name(), random_names());
      if (random_bool(0.1))
        _subscriptions.emplace_back(random_name(), random_names());
      if (random_bool(0.1))
        _services.emplace_back(random_name(), random_names());
    }

    return ComputationalGraph(_publications, _subscriptions, _services);
  }

 private:
  bool random_bool(const double prob)
  {
    // return a random boolean; 'prob' chance of true
    return std::bernoulli_distribution(prob)(_rng);
  }

  std::vector<std::string> random_names()
  {
    // return a randomly sized vector of random names
    std::vector<std::string> result;
    while (random_bool(0.5))
      result.push_back(random_name());
    return result;
  }

  std::string random_name()
  {
    // we have a good chance of choosing from a preselected set of names
    if (random_bool(0.9))
      return _common_names[_dist_common_names(_rng)];

    // otherwise, construct a totally random string
    std::string base("///////////ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz");
    std::shuffle(base.begin(), base.end(), _rng);
    return base.substr(0, _dist_random_names(_rng));
  }

}; // namespace MockGraphParser

} // namespace ros_curses
