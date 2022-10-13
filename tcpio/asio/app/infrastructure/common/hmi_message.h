#ifndef HMI_MESSAGE_H
#define HMI_MESSAGE_H


#include <cstddef>
#include <cstring>

#include "hmi_pad_data.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

class HmiMessage {
public:
    template <typename T> HmiMessage(T obj) {
        j = json(obj);
    }

    HmiMessage(uint8_t const* const data, const int len) {
        try {
            j = json::parse(data, data + len);
        } catch (...) {
            std::cout << "json parse excpetion, data: " << std::string(reinterpret_cast<char const*>(data), len) << std::endl;
        }
    }

public:
    operator std::string const() {
        return j.dump();
    }

    template <typename T> T Deserialize() {
        try {
            return j.get<T>();
        } catch (...) {
            std::cout << "json deserialize excpetion: " << j.dump() << std::endl;
            return T();
        }
    }

    int GetTag() {
        if (!(j.contains("tag")))
        {
            return -1;   
        }
        if(!j["tag"].is_number())
        {
            std::cout<<"tag is not number"<< std::endl;
            return -1;
        };

        return j["tag"];
    }

    std::string Serialize() {
        return j.dump();
    }

private:
    json j;
};


#endif