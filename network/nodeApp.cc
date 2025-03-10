#include "nodeApp.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NodeApp");

NS_OBJECT_ENSURE_REGISTERED(NodeApp);

// Initialize static member variables
std::map<int, Time> NodeApp::s_packetSendTimes;
std::map<Ipv4Address, std::string> NodeApp::s_address2Name;
bool NodeApp::s_isInitialized = false;
std::ofstream NodeApp::s_messageDelayLog;

TypeId NodeApp::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::NodeApp")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<NodeApp>();
    return tid;
}

NodeApp::NodeApp() : 
    m_running(false),
    m_port(5000),  // default poer
    m_socket(nullptr),
    m_round(1)
    {
    NS_LOG_FUNCTION(this);

}

NodeApp::~NodeApp() {
    NS_LOG_FUNCTION(this);
}

void NodeApp::Setup(std::string nodeName, Ptr<Socket> socket, Ipv4Address address, uint32_t port, const std::vector<Address>& peerAddresses, std::string filename) {
    m_nodeName = nodeName;
    m_socket = socket;
    m_address = address;
    m_port = port;
    m_peerAddresses = peerAddresses;


    if (!s_isInitialized) {
        s_packetSendTimes.clear();
        s_address2Name.clear();
        s_isInitialized = true;
        InitTimingFile(filename);
        NS_LOG_INFO("Initialized shared data structures");
    }

    s_address2Name[address] = nodeName;
}

void NodeApp::StartApplication(void){
    NS_LOG_FUNCTION(this);

    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
    m_socket->Bind (local);
    // m_socket->Connect (m_peer);
    NS_LOG_DEBUG("Setting up receive callback");
    // m_socket->Listen();
    m_socket->SetRecvCallback (MakeCallback (&NodeApp::HandleRead, this));
    
    

    m_running = true;
}

void NodeApp::StopApplication(void) {
    NS_LOG_FUNCTION(this);
    m_running = false;

    // 關閉所有連接
    if (m_socket) {
        m_socket->Close();
    }

    NS_LOG_INFO("[" << m_nodeName << " stopped]");
}

void NodeApp::InitTimingFile(std::string filename) {
    s_messageDelayLog.open(filename);
    if (s_messageDelayLog.is_open()) {
        s_messageDelayLog << "msg_count,msg_type,sendTime,recvTime,delay,sender,receiver" << std::endl;
        NS_LOG_INFO("Created packet timing file: " << filename);
    } else {
        NS_LOG_ERROR("Failed to create packet timing file: " << filename);
    }
}

void NodeApp::HandleRead(Ptr<Socket> socket) {
    NS_LOG_FUNCTION(this << socket);

    Time receiveTime = Simulator::Now();
    
    if (!socket) {
        NS_LOG_ERROR("Invalid socket in HandleRead");
        return;
    }

    Ptr<Packet> packet;
    Address from;

    // recv packet
    while ((packet = socket->RecvFrom(from))) {
        if (packet->GetSize() == 0) { // 空包檢查
            NS_LOG_WARN("Received empty packet");
            break;
        }

        InetSocketAddress inetFrom = InetSocketAddress::ConvertFrom(from);
        NS_LOG_INFO(m_nodeName << " received packet in " << receiveTime.GetSeconds() 
                    << " from=" << inetFrom.GetIpv4() << ":" << inetFrom.GetPort()
                    << " size=" << packet->GetSize() << " bytes");
        
        // decode MAVLink message
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        packet->CopyData(buf, packet->GetSize());

        mavlink_message_t msg;
        mavlink_status_t status;

        for (uint32_t i = 0; i < packet->GetSize(); ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
            // Handle the received MAVLink message here
            // For now, we'll just print the message ID
            Time sendTime = s_packetSendTimes[msg.seq];
            Time delay = receiveTime - sendTime;

            // store to messageDelayLog
            if (s_messageDelayLog.is_open()) {
                s_messageDelayLog << (int)msg.seq << ","
                            << msg.msgid << ","
                            << std::fixed << std::setprecision(7) << sendTime.GetSeconds() << ","
                            << std::fixed << std::setprecision(7) << receiveTime.GetSeconds() << ","
                            << std::fixed << std::setprecision(7) << delay.GetSeconds() << ","
                            << s_address2Name[inetFrom.GetIpv4()] << "," 
                            << m_nodeName << std::endl;
                
            }
            NS_LOG_INFO("Received MAVLink message with ID: " << msg.msgid);
            }
        }
    }
}



void NodeApp::SendPacket (messageTransmitLog logMsg) {
    if (!m_running) return;

    Time sendTime = Simulator::Now();

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    if(logMsg.msg_type.compare("HEARTBEAT") == 0){
        mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
    }
    else if(logMsg.msg_type.compare("HEARTBEAT_ACK") == 0){
        mavlink_msg_mission_ack_pack(1, 200, &msg, MAV_RESULT_ACCEPTED ,2, 200, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
    }
    // Create a sample MAVLink message (heartbeat in this case)
    
    s_packetSendTimes[msg.seq] = sendTime;

    // Serialize the message
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Create packet
    Ptr<Packet> packet = Create<Packet> (buf, len);

    // Send the packet
    // InetSocketAddress destAddr(m_peerAddresses[0], m_port);
    InetSocketAddress destAddr = InetSocketAddress::ConvertFrom(m_peerAddresses[0]);

    NS_LOG_INFO(m_nodeName << " sending MAVLink heartbeat to " << destAddr.GetIpv4() << ":" << destAddr.GetPort());
        
    int bytes = m_socket->SendTo(packet, 0, m_peerAddresses[0]);
    if (bytes < 0) {
        NS_LOG_ERROR(m_nodeName << " failed to send packet to " << destAddr.GetIpv4() << ":" << destAddr.GetPort());
    } 
    // m_sendEvent = Simulator::ScheduleNow (&MAVLinkApplication::SendPacket, this);
}