#include "proxy_repository.h"
#include "event_msg.h"
#include <algorithm>
#include <mutex>

// namespace
// {
//     std::mutex repoMtx;
// }

ProxyRepo::ProxyRepo()
{
    proxyRepo.reserve(10);
}

void ProxyRepo::AddProxy(std::shared_ptr<IProxy> newProxy)
{
    proxyRepo.push_back(newProxy);
}

void ProxyRepo::Start()
{
    std::for_each(proxyRepo.begin(), proxyRepo.end(), [](std::shared_ptr<IProxy> it) { it->Start(); });
}

bool ProxyRepo::GetSpecificProxy(MsgType type_, std::shared_ptr<IProxy>& res)
{
    if (type_ >= MsgType::INVALID)
    {
        return false;
    }

    for (auto it: proxyRepo)
    {
        if (it->IsSpecificProxyType(type_))
        {
            res = it;
            return true;
        }
    }

    return false;
}

