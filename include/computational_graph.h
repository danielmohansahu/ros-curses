/*
 *
 *
 * 
 */

#pragma once

// STL
#include <set>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <algorithm>

namespace ros_curses
{

/* Internal representation of the ROS computationl graph.
 */
class ComputationalGraph
{
 public:
  // forward declarations
  struct Node;

  /* Internal representation of a ROS Topic.
   */
  struct Topic
  {
    const std::string name;
    std::string type;
    std::set<std::shared_ptr<Node>> publishers;
    std::set<std::shared_ptr<Node>> subscribers;
    bool active {true};

    // construction by name only
    Topic() = delete;
    explicit Topic(const std::string& name_) : name(name_) {}

    // merge in another Topic's information
    void merge(const Topic& other)
    {
      // add all new incoming data
      publishers.insert(other.publishers.begin(), other.publishers.end());
      subscribers.insert(other.subscribers.begin(), other.subscribers.end());
      // mark us as "active"
      active = true;
    }
  }; // struct Topic

  /* Internal representation of a ROS Service.
   */
  struct Service
  {
    const std::string name;
    std::string type;
    std::set<std::shared_ptr<Node>> advertisers;
    bool active {true};

    // construction by name only
    Service() = delete;
    explicit Service(const std::string& name_) : name(name_) {}

    // merge in another Service's information
    void merge(const Service& other)
    {
      // add all new incoming data
      advertisers.insert(other.advertisers.begin(), other.advertisers.end());
      // mark us as "active"
      active = true;      
    }
  }; // struct Service

  /* Internal representation of a ROS parameter.
   */
  struct Param
  {
    std::string name;
    std::string namespace_;
    std::string type;
    bool dynamic {false};
    bool active {true};
  }; // struct Param

  /* Internal representation of a ROS node.
   */
  struct Node
  {
    const std::string name;
    std::string namespace_;
    std::set<std::shared_ptr<Topic>> publications;
    std::set<std::shared_ptr<Topic>> subscriptions;
    std::set<std::shared_ptr<Service>> services;
    bool active {true};

    // construction by name only
    Node() = delete;
    explicit Node(const std::string& name_) : name(name_) {}

    // merge in another Service's information
    void merge(const Node& other)
    {
      // add all new incoming data
      publications.insert(other.publications.begin(), other.publications.end());
      subscriptions.insert(other.subscriptions.begin(), other.subscriptions.end());
      services.insert(other.services.begin(), other.services.end());
      // mark us as "active"
      active = true;
    }
  }; // struct Node

  // convenience typedefs
  using NodeMap = std::unordered_map<std::string, std::shared_ptr<Node>>;
  using TopicMap = std::unordered_map<std::string, std::shared_ptr<Topic>>;
  using ServiceMap = std::unordered_map<std::string, std::shared_ptr<Service>>;
  using ParamMap = std::unordered_map<std::string, std::shared_ptr<Param>>;

  // all our ROS primitives
  NodeMap _nodes;
  TopicMap _topics;
  ServiceMap _services;
  ParamMap _params;

  // whether or not our last connection succeeded

 public:
  /* Construct a new graph from the given ROS status message
   *
   * Args:
   *    publications:   Mapping from topic -> all publishers
   *    subscriptions:  Mapping from topic -> all publishers
   *    services:       Mapping from service -> all advertisers
   */
  ComputationalGraph(const std::vector<std::pair<std::string, std::vector<std::string>>>& publications,
                     const std::vector<std::pair<std::string, std::vector<std::string>>>& subscriptions,
                     const std::vector<std::pair<std::string, std::vector<std::string>>>& services);

  /* Merge incoming graph with our internal representation, marking nodes as active / inactive.
   */
  void merge(const ComputationalGraph& other);

  /* nodes structure accessor.
   */
  const NodeMap& nodes() const {return _nodes; }

  /* topics structure accessor.
   */
  const TopicMap& topics() const {return _topics; }

  /* services structure accessor.
   */
  const ServiceMap& services() const {return _services; }

  /* params structure accessor.
   */
  const ParamMap& params() const {return _params; }

  /* nodes keys accessor.
   */
  std::vector<std::string> node_list() const { return get_keys(_nodes); };

  /* topics keys accessor.
   */
  std::vector<std::string> topic_list() const { return get_keys(_topics); };

  /* services keys accessor.
   */
  std::vector<std::string> service_list() const { return get_keys(_services); };

  /* params keys accessor.
   */
  std::vector<std::string> param_list() const { return get_keys(_params); };

 private:

  /* convenience method to extract keys from a map.
   */
  template <typename T>
  std::vector<std::string> get_keys(const std::unordered_map<std::string,T> map) const
  {
    // get keys in a vector
    std::vector<std::string> result;
    result.reserve(map.size());
    for (const auto& kv : map)
      result.push_back(kv.first);

    // sort for good measure
    std::sort(result.begin(), result.end());
    return result;
  }

};

} // namespace ros_curses