#include "TrafficLight.h"

#include <future>
#include <iostream>
#include <random>

/* Implementation of class "MessageQueue" */

template <typename T>
T MessageQueue<T>::receive()
{
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait()
    // to wait for and receive new messages and pull them from the queue using move semantics.
    // The received object should then be returned by the receive function.

    // do modification under lock with respect to condition variable
    std::unique_lock<std::mutex> uLock(_mutex);
    _condition.wait(uLock, [this] { return !_queue.empty() });

    // pop the latest and remove it from queue
    T msg = std::move(_queue.back());
    _queue.pop_back();

    return msg;
}

template <typename T>
void MessageQueue<T>::send(T&& msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex>
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.

    // push new message under lock
    std::lock_guard<std::mutex> uLock(_mutex);

    // add new message to queue
    _queue.push_back(std::move(msg));

    // notify client
    _condition.notify_one();
}

/* Implementation of class "TrafficLight" */

TrafficLight::TrafficLight()
{
    _type = ObjectType::objectTrafficLight;
    _currentPhase = TrafficLightPhase::red;
}

TrafficLight::~TrafficLight() { }

void TrafficLight::waitForGreen()
{
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop
    // runs and repeatedly calls the receive function on the message queue.
    // Once it receives TrafficLightPhase::green, the method returns.

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // reduce CPU usage
        if (_phase_queue->receive() == TrafficLightPhase::green) {
            return;
        }
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}

void TrafficLight::simulate()
{
    // FP.2b : Finally, the private method „cycleThroughPhases“ should be
    // started in a thread when the public method „simulate“ is called. To do
    // this, use the thread queue in the base class.
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    // FP.2a : Implement the function with an infinite loop that measures the time
    // between two loop cycles and toggles the current phase of the traffic light
    // between red and green and sends an update method to the message queue using
    // move semantics. The cycle duration should be a random value between 4 and 6
    // seconds. Also, the while-loop should use std::this_thread::sleep_for to
    // wait 1ms between two cycles.

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<long> cycleDistribution(4000, 6000); // between 4 and 6 in ms
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;

    // randomized cycle duration
    long cycleDuration = cycleDistribution(gen);

    // init stopWatch
    lastUpdate = std::chrono::system_clock::now();

    while (true) {
        // sleep at every iteration to reduce CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // compute time difference to stop watch
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();

        if (timeSinceLastUpdate >= cycleDuration) {
            _currentPhase = _currentPhase == TrafficLightPhase::red ? TrafficLightPhase::green : TrafficLightPhase::red;

            // send cycle update (using async)
            auto ftr_phase_update = std::async(std::launch::async, &MessageQueue<TrafficLightPhase>::send, _phase_queue, std::move(_currentPhase));
            ftr_phase_update.wait();

            // reset stop watch for next cycle
            lastUpdate = std::chrono::system_clock::now();

            // randomly choose duration for next cycle
            cycleDuration = cycleDistribution(gen);
        }
    }
}