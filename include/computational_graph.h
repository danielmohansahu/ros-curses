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

  // all our ROS primitives
  std::unordered_map<std::string, std::shared_ptr<Node>> _nodes;
  std::unordered_map<std::string, std::shared_ptr<Topic>> _topics;
  std::unordered_map<std::string, std::shared_ptr<Service>> _services;
  std::unordered_map<std::string, std::shared_ptr<Param>> _params;

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

};

} // namespace ros_curses