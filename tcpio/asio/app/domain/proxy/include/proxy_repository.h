#ifndef D295DBC1_8A71_4881_909F_168D1FFE5986
#define D295DBC1_8A71_4881_909F_168D1FFE5986

#include <vector>
#include <memory>
#include "iproxy.h"

class ProxyRepo
{
public:
    static ProxyRepo& GetInstance()
    {
        static ProxyRepo repo;
        return repo;
    }

    void Start();

    void AddProxy(std::shared_ptr<IProxy> newProxy);

    bool GetSpecificProxy(MsgType type_, std::shared_ptr<IProxy>& res);

private:
    ProxyRepo();
    ~ProxyRepo() = default;
    ProxyRepo(const ProxyRepo& ) = delete;
    ProxyRepo& operator=(const ProxyRepo& ) = delete;

private:
    std::vector<std::shared_ptr<IProxy>> proxyRepo;
};

#define CDD_FUSION_PROXY_REPO ProxyRepo::GetInstance()

#endif /* D295DBC1_8A71_4881_909F_168D1FFE5986 */
