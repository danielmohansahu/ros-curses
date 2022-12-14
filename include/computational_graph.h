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
#include <optional>

// convenience type for constexpr assertions
template <typename...> inline constexpr bool always_false = false;

namespace ros_curses
{

/* Internal representation of the ROS computationl graph.
 */
class ComputationalGraph
{
 public:

  /* Internal representation of a ROS Topic.
   */
  struct Topic
  {
    const std::string name;
    std::string type;
    std::set<std::string> publishers;
    std::set<std::string> subscribers;
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
    std::set<std::string> advertisers;
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
    std::string value;
    bool dynamic {false};
    bool active {true};

    // construction by name only
    Param() = delete;
    explicit Param(const std::string& name_)
      : name(name_), namespace_(name_.substr(0, name_.find_last_of('/')))
    {}

  }; // struct Param

  /* Internal representation of a ROS node.
   */
  struct Node
  {
    const std::string name;
    std::string namespace_;
    std::set<std::string> publications;
    std::set<std::string> subscriptions;
    std::set<std::string> services;
    std::set<std::string> parameters;
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
  bool _connected {false};

 public:

  /************************ Core Construction API ****************************/

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

  /* Getter for connectivity status.
   */
  bool connected() const { return _connected; }

  /* Setter for connectivity status.
   */
  void set_connected(const bool status) { _connected = status; }

  /* Merge incoming graph with our internal representation, marking nodes as active / inactive.
   */
  void merge(const ComputationalGraph& other);

  /* Merge incoming params with our internal representation.
   */
  void merge_params(const std::unordered_map<std::string, std::string>& params);

  /* Set the given topic types.
   */
  void set_topic_types(const std::vector<std::pair<std::string,std::string>>& types);

  /* Save current state to a YAML file.
   */
  bool save(const std::string& filename) const;

  /* Create a new CG from a saved-state YAML file.
   */
  static std::optional<ComputationalGraph> load(const std::string& filename);

  /***************************** Accessor API ********************************/

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

  /* Map access via type.
   */
  template <typename T>
  std::unordered_map<std::string, std::shared_ptr<T>> map() const
  {
    if constexpr (std::is_same_v<T, Node>)
      return nodes();
    else if constexpr (std::is_same_v<T, Topic>)
      return topics();
    else if constexpr (std::is_same_v<T, Service>)
      return services();
    else if constexpr (std::is_same_v<T, Param>)
      return params();
    else
      static_assert(always_false<T>, "Invalid Map type requested.");
  }

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

  /* Map access via type.
   */
  template <typename T>
  std::vector<std::string> list() const
  {
    if constexpr (std::is_same_v<T, Node>)
      return node_list();
    else if constexpr (std::is_same_v<T, Topic>)
      return topic_list();
    else if constexpr (std::is_same_v<T, Service>)
      return service_list();
    else if constexpr (std::is_same_v<T, Param>)
      return param_list();
    else
      static_assert(always_false<T>, "Invalid List type requested.");
  }

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