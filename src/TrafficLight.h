#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include <condition_variable>
#include <deque>
#include <mutex>

#include "TrafficObject.h"

// forward declarations to avoid include cycle
class Vehicle;

enum TrafficLightPhase { red, green };

template <class T>
class MessageQueue {
 public:
  T receive();
  void send(T&& msg);

 private:
  std::deque<T> _queue;
  std::condition_variable _condition;
  std::mutex _mutex;
};

class TrafficLight : public TrafficObject {
 public:
  // constructor / desctructor
  TrafficLight();

  // getters / setters
  TrafficLightPhase getCurrentPhase();

  // typical behaviour methods
  void waitForGreen();
  void simulate();

 private:
  void cycleThroughPhases();

  std::shared_ptr<MessageQueue<TrafficLightPhase>> _phase_queue;

  std::condition_variable _condition;
  std::mutex _mutex;
  TrafficLightPhase _currentPhase;
};

#endif