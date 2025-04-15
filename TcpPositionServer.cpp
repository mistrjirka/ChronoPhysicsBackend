#include "TcpPositionServer.hpp"
#include <iostream>
#include <cstring>
#include <vector>
#include <fcntl.h>

TcpPositionServer::TcpPositionServer(int port) : seq_number_(0) {
    server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_ < 0) {
        std::cerr << "Failed to create server socket." << std::endl;
        exit(1);
    }

    int opt = 1;
    if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &opt, sizeof(opt)) < 0) {
        std::cerr << "setsockopt failed" << std::endl;
        exit(1);
    }

    server_address_.sin_family = AF_INET;
    server_address_.sin_addr.s_addr = INADDR_ANY;
    server_address_.sin_port = htons(port);

    if (bind(server_socket_, (struct sockaddr*)&server_address_, sizeof(server_address_)) < 0) {
        std::cerr << "Failed to bind server socket." << std::endl;
        exit(1);
    }

    if (listen(server_socket_, 1) < 0) {
        std::cerr << "Listen failed." << std::endl;
        exit(1);
    }

    client_addrlen_ = sizeof(client_address_);
    client_socket_ = accept(server_socket_, (struct sockaddr*)&client_address_, &client_addrlen_);

    if (client_socket_ < 0) {
        std::cerr << "Failed to accept client connection." << std::endl;
        exit(1);
    }

    // Optionally set the client socket to non-blocking
    int flags = fcntl(client_socket_, F_GETFL, 0);
    fcntl(client_socket_, F_SETFL, flags | O_NONBLOCK);
}

TcpPositionServer::~TcpPositionServer() {
    if (client_socket_ >= 0) {
        close(client_socket_);
    }
    if (server_socket_ >= 0) {
        close(server_socket_);
    }
}

void TcpPositionServer::updatePositionOfUnit(int unit_id,
                              const chrono::ChVector3<double>& position,
                              const chrono::ChQuaternion<double>& rotation, TerrainSystemCoordinates &terrain_system) {
    // Convert rotation quaternion to Euler angles (roll, pitch, yaw)
    double q0 = rotation.e0();  // scalar part
    double q1 = rotation.e1();
    double q2 = rotation.e2();
    double q3 = rotation.e3();

    // Compute roll (x-axis rotation)
    double roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));

    // Compute pitch (y-axis rotation)
    double sinp = 2.0 * (q0 * q2 - q3 * q1);
    double pitch;
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // Compute yaw (z-axis rotation)
    double yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

    // Convert Chrono Euler angles to UE rotation
    ChVector3d euler_angles(roll, pitch, yaw);
    ChVector3d ue_euler_angles = terrain_system.convertChronoToUERotation(euler_angles);
    roll = ue_euler_angles.x();
    pitch = ue_euler_angles.y();
    yaw = ue_euler_angles.z();

    // Prepare the twist data
    ChVector3d terrain_coordinates = terrain_system.convertChronoToUE(position);

    TwistSendable_t twistData;
    twistData.x = static_cast<float>(terrain_coordinates.x());
    twistData.y = static_cast<float>(terrain_coordinates.y());
    twistData.z = static_cast<float>(terrain_coordinates.z());
    twistData.roll = static_cast<float>(roll);
    twistData.pitch = static_cast<float>(pitch);
    twistData.yaw = static_cast<float>(yaw);
    // Get current timestamp in nanoseconds
    auto now = std::chrono::high_resolution_clock::now();
    int64_t timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               now.time_since_epoch())
                               .count();

    // Calculate total packet size
    uint32_t dataSize = sizeof(TwistSendable_t);
    uint32_t packetSize = sizeof(SendablePacket_t) + dataSize;

    // Create a buffer to hold the packet
    std::vector<uint8_t> buffer(packetSize);

    // Fill in the packet header
    SendablePacket_t* packet = reinterpret_cast<SendablePacket_t*>(buffer.data());
    packet->type = PacketTypes_t::UpdateUnitPositionPacket;
    packet->id = unit_id;
    packet->seq = seq_number_++;
    packet->size = dataSize;
    packet->stamp = timestamp_ns;

    // Copy the twist data into the packet data
    memcpy(packet->data, &twistData, dataSize);

    // Send the packet
    ssize_t bytes_sent = send(client_socket_, buffer.data(), buffer.size(), 0);
    if (bytes_sent < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // Non-blocking socket, no data sent this time
            // Handle accordingly if needed
        } else {
            std::cerr << "Failed to send data to client: " << strerror(errno) << std::endl;
            // Optionally handle disconnection
        }
    } else {
        // Data sent successfully
        // Optionally, print debug information
        // std::cout << "Sent " << bytes_sent << " bytes to client." << std::endl;
    }
}