/*
 *
 *
 * 
 */

// ros_curses
#include "computational_graph.h"

namespace ros_curses
{

ComputationalGraph::ComputationalGraph(const std::vector<std::pair<std::string, std::vector<std::string>>>& publications,
                                       const std::vector<std::pair<std::string, std::vector<std::string>>>& subscriptions,
                                       const std::vector<std::pair<std::string, std::vector<std::string>>>& services)
{
  // iterate through publications, updating internal list of topics / nodes
  for (const auto& [topic, publishers] : publications)
    for (const auto& publisher : publishers)
    {
      _nodes[publisher]->publications.insert(_topics[topic]);
      _topics.at(topic)->publishers.insert(_nodes.at(publisher));
    }

  // iterate through subscriptions, updating internal list of topics / nodes
  for (const auto& [topic, subscribers] : publications)
    for (const auto& subscriber : subscribers)
    {
      _nodes[subscriber]->subscriptions.insert(_topics[topic]);
      _topics.at(topic)->subscribers.insert(_nodes.at(subscriber));
    }

  // iterate through services, updating internal list of services / nodes
  for (const auto& [service, advertisers] : services)
    for (const auto& advertiser : advertisers)
    {
      _nodes[advertiser]->services.insert(_services[service]);
      _services.at(service)->advertisers.insert(_nodes.at(advertiser));
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
    _nodes[name]->merge(*node);
  for (const auto& [name, topic] : other._topics)
    _topics[name]->merge(*topic);
  for (const auto& [name, service] : other._services)
    _services[name]->merge(*service);
}

} // namespace ros_curses