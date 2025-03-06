#ifndef TCP_POSITION_SERVER_HPP
#define TCP_POSITION_SERVER_HPP

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuaternion.h"
#include "terrain_system.hpp"

// Include necessary headers
#include <cstdint>
#include <chrono>

// Use packed structures to match the client's expectations
#pragma pack(push, 1)
enum class PacketTypes_t : uint8_t
{
    TwistPacket = 0,
    CreateUnitPacket = 1,
    UpdateUnitPositionPacket = 2,
    DeleteUnitPacket = 3,
};


struct TwistSendable_t
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
};

struct CreateUnitSendable_t
{
    int type;
    TwistSendable_t twist;
};


struct SendablePacket_t
{
    PacketTypes_t type;
    int id;
    uint32_t seq;
    uint32_t size;
    uint64_t stamp;
    uint8_t data[]; // Be cautious with flexible array members
};
#pragma pack(pop)

class TcpPositionServer {
private:
    int server_socket_;
    int client_socket_;
    struct sockaddr_in server_address_, client_address_;
    socklen_t client_addrlen_;
    uint32_t seq_number_;  // Sequence number for packets

public:
    TcpPositionServer(int port);
    ~TcpPositionServer();

    void updatePositionOfUnit(int unit_id,
                              const chrono::ChVector3<double>& position,
                              const chrono::ChQuaternion<double>& rotation, TerrainSystemCoordinates &terrain_system);
};

#endif  // TCP_POSITION_SERVER_HPP