#ifndef C8959F93_13ED_430C_B702_893EC60F12CC
#define C8959F93_13ED_430C_B702_893EC60F12CC

#include <queue>
#include <mutex>
#include <condition_variable>
#include "event_msg.h"

template<typename T>
class EventQueue
{
public:
    static EventQueue<T>& GetInstance()
    {
        static EventQueue<T> instance;
        return instance;
    }

    void push(T&& ref)
    {
        std::lock_guard<std::mutex> lck(mtx);
        eventQueue.emplace(ref);
        cv.notify_all();
    }

    T wait_and_get_front()
    {
        std::unique_lock<std::mutex> lck(mtx);
        cv.wait(lck, [this]{return !eventQueue.empty();});
        T res = eventQueue.front();
        eventQueue.pop();
        // std::cout << "before construct" << std::endl;
        return std::move(res);
    }
private:
    EventQueue() = default;
    ~EventQueue() = default;
    EventQueue(const EventQueue& ref) = delete;
    EventQueue& operator=(const EventQueue& ref) = delete;

private:
    std::mutex mtx;
    std::condition_variable cv;
    std::queue<T> eventQueue;
};

#define CDD_FUSION_EVENT_QUEUE EventQueue<EventMessage>::GetInstance()


#endif /* C8959F93_13ED_430C_B702_893EC60F12CC */
