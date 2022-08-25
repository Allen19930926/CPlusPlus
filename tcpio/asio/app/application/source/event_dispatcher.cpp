#include "event_dispatcher.h"
#include "v2x_fusion_algorithm.h"
#include "camera_fusion_algorithm.h"
#include "can_fusion_algorithm.h"
#include "v2x_adas_event_macro.h"
#include <iostream>
#include "proxy_repository.h"
#include "xds_proxy.h"
#include "cds_proxy.h"
#include "hmi_proxy.h"
#include "com_tcp_server.h"

extern ComTcpServer * comTcpServer;

void EventDispatcher::ProcessMessage(const EventMessage& msg)
{
    // printf ("Msgtype = %s\n", msg.data);
    // std::cout << "pop:"  << msg.data << std::endl;
#ifdef __x86_64__
    if (msg.msgType != MsgType::CAMERA)
    {
        comTcpServer->Write(ComTcpMessage(msg));
    }
#endif

    switch(msg.msgType)
    {
        case MsgType::V2X :                     V2xFusionAlgo::ProcessRecieveData(msg.data, msg.msglen); break;
        case MsgType::CAMERA :                  CameraFusionAlgo::ProcessRecieveData(msg.data, msg.msglen); break;
        case MsgType::CAN :                     CanFusionAlgo::ProcessRecieveData(msg.data, msg.msglen); break;
        // case MsgType::HMI :                     HmiFusionAlgo::ProcessRecieveData(msg.data, msg.msglen); break;
        case MsgType::IPC_GNSS_HEADING_PITCH_ROLL:
        case MsgType::IPC_GNSS_LAT_LONG:
        {
            std::shared_ptr<IProxy> ptr;
            if (CDD_FUSION_PROXY_REPO.GetSpecificProxy(MsgType::XDS_DUMMY, ptr))
            {
                auto proxy(std::dynamic_pointer_cast<XdsProxy>(ptr));
                proxy->ProcessIncomingMessage(msg.msgType, msg.data, msg.msglen);
            }
            if (CDD_FUSION_PROXY_REPO.GetSpecificProxy(MsgType::HMI_DUMMY, ptr))
            {
                auto proxy(std::dynamic_pointer_cast<HmiProxy>(ptr));
                proxy->ProcessRecieveIPCData(msg.msgType, msg.data, msg.msglen);
            }
            CanFusionAlgo::ProcessRecieveData(msg.msgType, msg.data, msg.msglen);
        }
        break;
        case MsgType::IPC_GNSS_ACC:
        case MsgType::IPC_GNSS_GYRO:        
        case MsgType::IPC_GNSS_SPEED :
        case MsgType::IPC_GNSS_DATA_INFO :
        case MsgType::IPC_GNSS_STD :
        case MsgType::IPC_GNSS_UTC :
        {
            std::shared_ptr<IProxy> ptr;
            if (CDD_FUSION_PROXY_REPO.GetSpecificProxy(MsgType::XDS_DUMMY, ptr))
            {
                auto proxy(std::dynamic_pointer_cast<XdsProxy>(ptr));
                proxy->ProcessIncomingMessage(msg.msgType, msg.data, msg.msglen);
            }
        };break;
        case MsgType::IPC_GNSS_HEIGHT_TIME :
        {
            std::shared_ptr<IProxy> ptr;
            if (CDD_FUSION_PROXY_REPO.GetSpecificProxy(MsgType::XDS_DUMMY, ptr))
            {
                auto proxy(std::dynamic_pointer_cast<XdsProxy>(ptr));
                proxy->ProcessIncomingMessage(msg.msgType, msg.data, msg.msglen);
            }
            CanFusionAlgo::ProcessRecieveData(msg.msgType, msg.data, msg.msglen);
        };break;

        case MsgType::HMI_PAD:
        {
            std::shared_ptr<IProxy> ptr;
            if (CDD_FUSION_PROXY_REPO.GetSpecificProxy(MsgType::HMI_DUMMY, ptr))
            {
                auto proxy(std::dynamic_pointer_cast<HmiProxy>(ptr));
                proxy->ProcessRecievePadData(msg.msgType, msg.data, msg.msglen);
            }

        };break;

        case MsgType::HMI_GSENTRY:
        {
            std::shared_ptr<IProxy> ptr;
            if (CDD_FUSION_PROXY_REPO.GetSpecificProxy(MsgType::HMI_DUMMY, ptr))
            {
                auto proxy(std::dynamic_pointer_cast<HmiProxy>(ptr));
                proxy->ProcessRecieveGsentryData(msg.msgType, msg.data, msg.msglen);
            }
        };break;
        case MsgType::IPC_EVH:
        case MsgType::IPC_HMI_INFO:
        case MsgType::IPC_SYS_ERROR:
        {
            std::shared_ptr<IProxy> ptr;
            if (CDD_FUSION_PROXY_REPO.GetSpecificProxy(MsgType::HMI_DUMMY, ptr))
            {
                auto proxy(std::dynamic_pointer_cast<HmiProxy>(ptr));
                proxy->ProcessRecieveIPCData(msg.msgType, msg.data, msg.msglen);
            }
        };break;

        case MsgType::IPC_CAN:
        {
            std::shared_ptr<IProxy> ptr;
            if (CDD_FUSION_PROXY_REPO.GetSpecificProxy(MsgType::CDS, ptr))
            {
                auto proxy(std::dynamic_pointer_cast<CdsProxy>(ptr));
                proxy->ProcessIncomingMessage(msg.msgType, msg.data, msg.msglen);
            }
        };break;
        default: break;
    }
}


