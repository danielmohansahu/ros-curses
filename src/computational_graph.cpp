/*
 *
 *
 *
 */

// STL
#include <fstream>
#include <iostream>
#include <assert.h>

// YAMLCPP
#include <yaml-cpp/yaml.h>

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
  for (const auto& [topic, subscribers] : subscriptions)
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

    // if this param's namespace is a node, link them
    for (auto& kv : _nodes)
      if (kv.second->name == element->second->namespace_)
        kv.second->parameters.insert(name);
  }
}

void ComputationalGraph::set_topic_types(const std::vector<std::pair<std::string,std::string>>& types)
{
  // set the given topics
  for (const auto& [topic, type_] : types)
  {
    auto element = _topics.find(topic);
    if (element == _topics.end())
    {
      // topic doesn't yet exist - create it
      const auto& res = _topics.emplace(topic, std::make_shared<Topic>(topic));

      // sanity check that this was inserted
      assert(res.second);
      element = res.first;
    }

    // update the topic type
    element->second->type = type_;
    // @TODO what if there's a type mismatch? How to handle?
  }
}

bool ComputationalGraph::save(const std::string& filename) const
{
  // construct a YAML::Node representation of data
  YAML::Node config;

  // populate with publishers, subscribers
  for (const auto& topic : _topics)
  {
    for (const auto& pub : topic.second->publishers)
      if (const auto node = _nodes.find(pub); node != _nodes.end() && node->second->active)
        config["publishers"].push_back(node->first);
    for (const auto& sub : topic.second->subscribers)
      if (const auto node = _nodes.find(sub); node != _nodes.end() && node->second->active)
        config["subscribers"].push_back(node->first);
  }

  // populate with services
  for (const auto& service : _services)
    for (const auto& adv : service.second->advertisers)
      if (const auto node = _nodes.find(adv); node != _nodes.end() && node->second->active)
        config["advertisers"].push_back(node->first);

  // populate with parameters
  for (const auto& param : _params)
    config["parameters"][param.first] = param.second->value;

  // try to dump to file
  std::ofstream ofs(filename.c_str(), std::ofstream::out);
  if (!ofs)
    return false;

  ofs << config;
  return true;
}

bool ComputationalGraph::load(const std::string& filename)
{
  // attempt to load the given file
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(filename);
  }
  catch (const YAML::BadFile& e)
  {
    return false;
  }

  // merge the contents of this file into our data


  // I am a stub
  return false;
}

} // namespace ros_curses
