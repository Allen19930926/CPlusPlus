#include "event_dispatcher.h"
#include "v2x_fusion_algorithm.h"
#include "camera_fusion_algorithm.h"
#include "can_fusion_algorithm.h"
#include "v2x_adas_event_macro.h"
#include "event_queue.h"


void EventDispatcher::ProcessMessage()
{
    EventMessage&& msg = CDD_FUSION_EVENT_QUEUE.wait_and_get_front();
    // printf ("Msgtype = %u\n", msg.msgType);
    switch(msg.msgType)
    {
        case MsgType::V2X :                     V2xFusionAlgo::ProcessRecieveData(msg.data, msg.msglen); break;
        case MsgType::CAMERA :                  CameraFusionAlgo::ProcessRecieveData(msg.data, msg.msglen); break;
        case MsgType::CAN :                     CanFusionAlgo::ProcessRecieveData(msg.data, msg.msglen); break;
        // case MsgType::CAN :                     CanFusionAlgo::ProcessRecieveData(msg.data, msg.msglen); break;
        default: break;
    }
}


