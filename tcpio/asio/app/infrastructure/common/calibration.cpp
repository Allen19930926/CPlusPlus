#include "calibration.h"
#include <fstream>
#include <glog/logging.h>
#include "nlohmann/json.hpp"
#include "data_base.h"

const std::string calibration_file = "calibration.json";

void Calibration::ReloadCaliJson()
{
    std::string path = GetExecutePath();
    std::string file = path + "/" + calibration_file;
    LOG(INFO) << "calibration file : " << file;
    // parse config
    std::ifstream cali_file(file);

    if (!cali_file.is_open()) {
      LOG(INFO) << "open calibration file " << calibration_file << " failed!"
                << std::endl;
      return ;
    }

    const nlohmann::json &json = nlohmann::json::parse(cali_file);

    if (json.is_null()) {
      LOG(INFO) << "parse calibration file " << calibration_file << " failed!"
                << std::endl;
      return ;
    }

    CalibrateStruct& CalibrateVariable = DataRepo::GetInstance().GetCalibrateVariables();

    CalibrateVariable.dx_gnss_to_rear_center = json.at("dx");
    CalibrateVariable.dy_gnss_to_rear_center = json.at("dy");
    LOG(INFO) << "calibrate dx = " << CalibrateVariable.dx_gnss_to_rear_center;
    LOG(INFO) << "calibrate dy = " << CalibrateVariable.dy_gnss_to_rear_center;
}

std::string Calibration::GetExecutePath()
{
    char szCurWorkPath[256];
    memset(szCurWorkPath,'\0',256);
    int nRet = readlink ("/proc/self/exe", szCurWorkPath , 256);
    if(nRet>256||nRet<0){
    return  "";
    }
    //上面已经获取到了可执行文件的据对路径了(包含文件名)，
    //下面的for循环是为了去除路径中的文件名，如果需要的话
    for(int i=nRet;i>0;i--){
    if(szCurWorkPath[i]=='/' || szCurWorkPath[i]=='\\'){
            szCurWorkPath[i]='\0';
            break;
        }
    }
    //这就是最终的文件路径，例如  "/usr/var"
    std::string szRet = szCurWorkPath;
    return szRet;
}
