#pragma once

#include "../include/dynamixel_sdk/dynamixel_sdk.h" 
#include <array>

/// @brief Connects to the five dynamixels used by the 3001 arm
/// @param portHandler dynamixel class, handles serial communication
/// @param groupBulkRead dynamixel class, controls simultaneous writing
/// @param groupBulkWrite dyanmixel class, controls simulaneous reading
/// @param baudrate_idx int, index indicating which baudrate to use (9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000)
/// @param primary_dxl_id int, the first dynamixel id in the chain. Must be 11.
/// @return bool, if the function has succeeded or failed
bool initDynamixel( dynamixel::PortHandler* portHandler,
                    dynamixel::GroupBulkRead &groupBulkRead,
                    dynamixel::GroupBulkWrite &groupBulkWrite,
                    const int baudrate_idx,
                    const int primary_dxl_id);     

/// @brief a size 5 int32_t array of joint positions and bool success if the updateJoints function was successful
struct update_joints_struct {
    std::array<int32_t, 5> joints_array;
    bool success;
};


// @brief Uses the dynamixelSDK to send a request for position data and then receives
// the data and sets our 'joints' struct with that data.
// @param groupBulkRead dynamixel class, controls simultaneous reading
// @param packetHandler dynamixel class, handles protocol packet construction
// @param primary_dxl_id int, the first dynamixel id in the chain. must be 11.
// @return update_joints_struct
update_joints_struct updateJoints(  dynamixel::GroupBulkRead &groupBulkRead, 
                                    dynamixel::PacketHandler* packetHandler, 
                                    int primary_dxl_id);

/// @brief Sets each servo to a desired 
/// @param current_joint_array 
/// @param joint_array 
/// @param groupBulkWrite 
/// @param packetHandler 
/// @param primary_dxl_id 
/// @return 
bool setJoints( std::array<int32_t, 5> current_joint_array,
                std::array<int32_t, 5> joint_array, 
                dynamixel::GroupBulkWrite &groupBulkWrite, 
                dynamixel::PacketHandler* packetHandler,
                int primary_dxl_id);

void disableTorque( dynamixel::PortHandler* portHandler,
                    dynamixel::PacketHandler* packetHandler,
                    int primary_dxl_id);

bool enableTorque(  dynamixel::PortHandler* portHandler,
                    dynamixel::PacketHandler* packetHandler,
                    int primary_dxl_id);                             

bool diffWithinLimits( std::array<int32_t, 5>& current, 
                std::array<int32_t, 5>& desired,
                int32_t lower, int32_t upper);

std::array<uint8_t, 5> checkOpMode(  dynamixel::PacketHandler* packetHandler,
                                    dynamixel::PortHandler* portHandler);   

bool setOpMode( dynamixel::PortHandler* portHandler,
                 dynamixel::PacketHandler* packetHandler,
                 int dxl_id,
                 uint8_t opmode);                                    