#include "TrafficLight.h"

#include <future>
#include <iostream>
#include <random>

/* Implementation of class "MessageQueue" */

template <typename T>
T MessageQueue<T>::receive() {
  // do modification under lock with respect to condition variable
  std::unique_lock<std::mutex> uLock(_mutex);
  _condition.wait(uLock, [this] { return !_queue.empty(); });

  // pull the message
  T msg = std::move(_queue.front());
  _queue.pop_front();

  return msg;
}

template <typename T>
void MessageQueue<T>::send(T&& msg) {
  // push new message under lock
  std::lock_guard<std::mutex> uLock(_mutex);

  // add new message to queue
  _queue.emplace_back(std::move(msg));

  // notify client
  _condition.notify_one();
}

/* Implementation of class "TrafficLight" */

TrafficLight::TrafficLight() {
  _type = ObjectType::objectTrafficLight;
  _currentPhase = TrafficLightPhase::red;
  _phase_queue = std::make_shared<MessageQueue<TrafficLightPhase>>();
}

void TrafficLight::waitForGreen() {
  while (true) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(1));  // reduce CPU usage
    if (_phase_queue->receive() == TrafficLightPhase::green) {
      return;
    }
  }
}

TrafficLightPhase TrafficLight::getCurrentPhase() { return _currentPhase; }

void TrafficLight::simulate() {
  threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<long> cycleDistribution(
      4000, 6000);  // between 4 and 6 in ms
  std::chrono::time_point<std::chrono::system_clock> lastUpdate;

  // randomized cycle duration
  long cycleDuration = cycleDistribution(gen);

  // init stopWatch
  lastUpdate = std::chrono::system_clock::now();

  while (true) {
    // sleep at every iteration to reduce CPU usage
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // compute time difference to stop watch
    long timeSinceLastUpdate =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now() - lastUpdate)
            .count();

    if (timeSinceLastUpdate >= cycleDuration) {
      _currentPhase = _currentPhase == TrafficLightPhase::red
                          ? TrafficLightPhase::green
                          : TrafficLightPhase::red;

      // send cycle update (using async)
      auto ftr_phase_update =
          std::async(std::launch::async, &MessageQueue<TrafficLightPhase>::send,
                     _phase_queue, std::move(_currentPhase));
      ftr_phase_update.wait();

      // reset stop watch for next cycle
      lastUpdate = std::chrono::system_clock::now();

      // randomly choose duration for next cycle
      cycleDuration = cycleDistribution(gen);
    }
  }
}