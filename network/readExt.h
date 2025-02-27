#ifndef READ_EXT_H
#define READ_EXT_H

// ns3 includes
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/stats-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ptr.h"
#include "ns3/application.h"
#include "ns3/log.h"

#include <nlohmann/json.hpp>

#include "nodeApp.h"

using namespace ns3;

std::ostream& operator<<(std::ostream& os, const NetworkConfig& config) {
    os << "NetworkConfig{\n"
       << "  updateInterval: " << config.updateGranularity << "\n"
       << "  segmentSize: " << config.segmentSize << "\n"
       << "  uavsNames[" << config.uavsNames.size() << "]: ";
    
    for (const auto& name : config.uavsNames) {
        os << name << " ";
    }
    os << "\n";
    
    os << "  uavsPositions[" << config.uavsPositions.size() << "]:\n";
    for (const auto& pos : config.uavsPositions) {
        os << "    (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")\n";
    }
    
    os << "  tcpSendBuffer: " << config.tcpSendBuffer << "\n"
       << "  tcpRecvBuffer: " << config.tcpRecvBuffer << "\n"
       << "  p2pRate: " << config.p2pRate << "\n"
       << "  p2pMtu: " << config.p2pMtu << "\n"
       << "  p2pDelay: " << config.p2pDelay << "\n"
       << "}";
    
    return os;
}


bool readConfig(const std::string& filename, NetworkConfig& config) {

    // 讀取文件
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open configuration file: " << filename << std::endl;
        return false;
    }

    // 解析 JSON
    nlohmann::json j = nlohmann::json::parse(file);

    // 讀取所有配置項
    config.updateGranularity = j["updateGranularity"].get<double>();
    config.segmentSize = j["segmentSize"].get<int>();
    config.uavsNames = j["uavsNames"].get<std::vector<std::string>>();
    config.uavsPositions = j["uavsPositions"].get<std::vector<std::vector<float>>>();
    config.tcpSendBuffer = j["tcpSendBuffer"].get<int>();
    config.tcpRecvBuffer = j["tcpRecvBuffer"].get<int>();
    config.p2pRate = j["p2pRate"].get<std::string>();
    config.p2pMtu = j["p2pMtu"].get<int>();
    config.p2pDelay = j["p2pDelay"].get<double>();

    std::cout << "Configuration file loaded successfully" << std::endl;
    return true;
}

void scheduleMessage(std::unordered_map<std::string, Ptr<NodeApp>> name2nodeApps, messageTransmitLog msg){
    Ptr<NodeApp> sender = name2nodeApps[msg.senderName];
    // Ptr<NodeApp> receiver = name2nodeApps[msg.receiverName];

    Simulator::Schedule (Seconds (msg.timestamp), &NodeApp::SendPacket, sender, msg);

    std::cout << "Schedule " << msg.msg_count <<":"<< msg.msg_type  
            << ", send " << msg.senderName  << " to " << msg.receiverName
            <<  " on " << std::setprecision(6) << msg.timestamp << std::endl;

}

void readMessageTransmitLog(const std::string& filename, const std::unordered_map<std::string, Ptr<NodeApp>>& name2nodeApps) {
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open configuration file: " << filename << std::endl;
        return;
    }


    // 讀取標題行
    std::string line;
    std::getline(file, line);

    // 讀取數據行
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string field;
        messageTransmitLog data;

        // 按順序讀取每個欄位

        std::getline(ss, field, ',');
        data.msg_count = std::stoi(field);

        std::getline(ss, field, ',');
        data.timestamp = std::stod(field);

        std::getline(ss, data.msg_type, ',');
        
        std::getline(ss, data.content, ',');

        std::getline(ss, data.senderName, ',');
        std::getline(ss, data.receiverName, '\r');
        
        if(data.msg_count >= 0){
            scheduleMessage(name2nodeApps, data);
        }
        
        
    }
    std::cout << "Complete Loaded: "<< filename << std::endl;
}

#endif