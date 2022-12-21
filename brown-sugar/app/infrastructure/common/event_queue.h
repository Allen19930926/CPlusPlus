#ifndef C8959F93_13ED_430C_B702_893EC60F12CC
#define C8959F93_13ED_430C_B702_893EC60F12CC

#include <queue>
#include <mutex>
#include <condition_variable>
#include "event_msg.h"
#include <chrono>
#include <glog/logging.h>

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
        bool meet_cond = cv.wait_for(lck, std::chrono::milliseconds(1000), [this]{return !eventQueue.empty();});
        if (meet_cond)
        {
            T res = eventQueue.front();
            eventQueue.pop();
            return std::move(res);
        }
        else
        {
            LOG(ERROR) << "event thread wait time out!!, event queue size is " << eventQueue.size();
            return T(MsgType::INVALID, nullptr, 0);
        }
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
