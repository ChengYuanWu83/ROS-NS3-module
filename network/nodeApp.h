#ifndef NODE_APP_H
#define NODE_APP_H

#include "ns3/application.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"

#include "../c_library_v2/common/mavlink.h"
#include "../c_library_v2/minimal/mavlink_msg_heartbeat.h"

#include <iomanip>

using namespace ns3;

struct NetworkConfig {
    float updateGranularity;              // 更新間隔
    uint32_t segmentSize;             // TCP 分段大小
    std::vector<std::string> uavsNames;      // 節點名稱
    std::vector<std::vector<float>> uavsPositions;  // 節點初始位置

    
    // 網路參數
    uint32_t tcpSendBuffer;
    uint32_t tcpRecvBuffer;
    std::string p2pRate;
    uint32_t p2pMtu;
    double p2pDelay;

};

struct messageTransmitLog {
    double timestamp;
    std::string msg_type;
    int msg_count;
    std::string content;
    std::string senderName;
    std::string receiverName;
};

class NodeApp : public Application {
public:
    static TypeId GetTypeId(void);
    NodeApp();
    virtual ~NodeApp();

    

    // 設置應用程式的基本參數
    virtual void Setup(std::string nodeName, Ptr<Socket> socket, Ipv4Address address, uint32_t port, const std::vector<Address>& peerAddresses, std::string filename);
    virtual void SendPacket (messageTransmitLog logMsg);
    
protected:

	bool m_running;
    

    std::string m_nodeName;
    Ipv4Address m_address;
    uint32_t m_port;
    Ptr<Socket> m_socket; 
	std::vector<Address> m_peerAddresses;

    uint32_t m_timeSlot;
    uint32_t m_round;

    static std::ofstream s_messageDelayLog;  
    // record the packet send time with msg_count in order to get the send time when recv packet
    static std::map<int, Time> s_packetSendTimes; 
    // record the socket2name in order to know sender name by address
    static std::map<Ipv4Address, std::string> s_address2Name; 
    static bool s_isInitialized;

private:    
    virtual void StartApplication(void) ;
    virtual void StopApplication(void);
	virtual void HandleRead(Ptr<Socket> socket);
    virtual void InitTimingFile(std::string filename);

};

#endif // NODE_APP_H