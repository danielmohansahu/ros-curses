/*
 *
 *
 * 
 */

// STL
#include <assert.h>

// ros_curses
#include "computational_graph.h"

namespace ros_curses
{

ComputationalGraph::ComputationalGraph(const std::vector<std::pair<std::string, std::vector<std::string>>>& publications,
                                       const std::vector<std::pair<std::string, std::vector<std::string>>>& subscriptions,
                                       const std::vector<std::pair<std::string, std::vector<std::string>>>& services)
 : _connected(true)
{
  // iterate through publications, updating internal list of topics / nodes
  for (const auto& [topic, publishers] : publications)
    for (const auto& publisher : publishers)
    {
      // construct node / topic if not yet present
      if (_nodes.find(publisher) == _nodes.end())
        _nodes[publisher] = std::make_shared<Node>(publisher);
      if (_topics.find(topic) == _topics.end())
        _topics[topic] = std::make_shared<Topic>(topic);

      _nodes.at(publisher)->publications.insert(_topics.at(topic)->name);
      _topics.at(topic)->publishers.insert(_nodes.at(publisher)->name);
    }

  // iterate through subscriptions, updating internal list of topics / nodes
  for (const auto& [topic, subscribers] : publications)
    for (const auto& subscriber : subscribers)
    {
      // construct node / topic if not yet present
      if (_nodes.find(subscriber) == _nodes.end())
        _nodes[subscriber] = std::make_shared<Node>(subscriber);
      if (_topics.find(topic) == _topics.end())
        _topics[topic] = std::make_shared<Topic>(topic);

      _nodes.at(subscriber)->subscriptions.insert(_topics.at(topic)->name);
      _topics.at(topic)->subscribers.insert(_nodes.at(subscriber)->name);
    }

  // iterate through services, updating internal list of services / nodes
  for (const auto& [service, advertisers] : services)
    for (const auto& advertiser : advertisers)
    {
      // construct node / topic if not yet present
      if (_nodes.find(advertiser) == _nodes.end())
        _nodes[advertiser] = std::make_shared<Node>(advertiser);
      if (_services.find(service) == _services.end())
        _services[service] = std::make_shared<Service>(service);

      _nodes.at(advertiser)->services.insert(_services.at(service)->name);
      _services.at(service)->advertisers.insert(_nodes.at(advertiser)->name);
    }
}

void ComputationalGraph::merge(const ComputationalGraph& other)
{
  // perform a brute force search across all elements of our graph
  // N.B. We can tell if a node / service / topic becomes inactive, but this
  //  implementation doesn't track if a node stops publishing to a live topic, etc.

  // first, mark all elements as inactive
  for (auto& kv : _nodes)
    kv.second->active = false;
  for (auto& kv : _topics)
    kv.second->active = false;
  for (auto& kv : _services)
    kv.second->active = false;

  // merge all subcomponents
  for (const auto& [name, node] : other._nodes)
    if (_nodes.find(name) == _nodes.end())
      _nodes[name] = node;
    else
    _nodes[name]->merge(*node);
  for (const auto& [name, topic] : other._topics)
    if (_topics.find(name) == _topics.end())
      _topics[name] = topic;
    else
      _topics[name]->merge(*topic);
  for (const auto& [name, service] : other._services)
    if (_services.find(name) == _services.end())
      _services[name] = service;
    else
      _services[name]->merge(*service);

  // mark as active
  _connected = true;
}

void ComputationalGraph::merge_params(const std::unordered_map<std::string, std::string>& params)
{
  // mark all current params as inactive (to distinguish params deleted from the server)
  for (auto& kv : _params)
    kv.second->active = false;

  // iterate through all new params, updating values as necessary
  for (const auto& [name, value] : params)
  {
    auto element = _params.find(name);
    if (element == _params.end())
    {
      // construct a new param
      const auto& res = _params.emplace(name, std::make_shared<Param>(name));

      // sanity check this actually updated
      assert(res.second);
      element = res.first;
    }

    // update value and mark as active
    element->second->value = value;
    element->second->active = true;
  }

}

} // namespace ros_curses