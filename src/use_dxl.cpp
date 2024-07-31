#include "../include/dynamixel_sdk/dynamixel_sdk.h" 
#include <array>
#include <cstdint>

/*
use_dxl.cpp
Dynamixel-specific functions here. 

Dynamixel has a funky way of manipulating two objects defined in gui.cpp, portHandler and packetHandler, 

*/ 

/*
Dynamixel defines (from https://emanual.robotis.com/docs/en/dxl/x/xm430-w350)
NOTE: Data in EEPROM can only be written when torque_enable = 0!
*/

#define ADDR_TORQUE_ENABLE      64      // RAM torque enable address
#define ADDR_PRESENT_POSITION   132     // RAM present position address
#define ADDR_GOAL_POSITION      116     // RAM goal position address 
#define LEN_PRESENT_POSITION    4       // present position data length (bytes) 
#define LEN_GOAL_POSITION       4       // goal position data length (bytes)

#define ADDR_OP_MODE            11      // EEPROM: operating mode address
#define LEN_OP_MODE             1       // operating mode data length (bytes)

void convert_int32_uint8(int32_t goalPosition, uint8_t* convertedGoalPos) {
    convertedGoalPos[0] = DXL_LOBYTE(DXL_LOWORD(goalPosition));
    convertedGoalPos[1] = DXL_HIBYTE(DXL_LOWORD(goalPosition));
    convertedGoalPos[2] = DXL_LOBYTE(DXL_HIWORD(goalPosition));
    convertedGoalPos[3] = DXL_HIBYTE(DXL_HIWORD(goalPosition));
}

bool initDynamixel( dynamixel::PortHandler* portHandler,
                    dynamixel::GroupBulkRead &groupBulkRead,
                    dynamixel::GroupBulkWrite &groupBulkWrite,
                    const int baudrate_idx,
                    const int primary_dxl_id) {

    const int bauds[] = {9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000};

    // open port
    if (portHandler->openPort()) {
        printf("[initDynamixel]\t port opened!\n");
    } else {
        printf("[dxl]\t port couldn't be opened!\n");
        return false;
    }

    // set baud rate
    if (portHandler->setBaudRate((bauds[baudrate_idx]))) {
        printf("[initDynamixel]\t baudrate set to %d\n", bauds[baudrate_idx]); 
    } else {
        printf("[initDynamixel]\t baudrate couldn't be set!\n");
        fprintf(stderr, "[initDynamixel]\t couldn't set baudrate!\n");
        return false;
    }


    for (int i = 0; i < 5; i++) {
        if (groupBulkRead.addParam(primary_dxl_id + i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
            printf("dynamixel %d present position param added!\n", 11+ i);
        } else {
            printf("dynamixel %d present position param couldn't be added!\n", 11 + i);
            return false;
        }
    }

    return true;

}

std::array<uint8_t, 5> checkOpMode(dynamixel::PacketHandler* packetHandler,
                                   dynamixel::PortHandler* portHandler) {

    std::array<uint8_t, 5> result = {99, 99, 99, 99, 99};
    const int primary_dxl_id = 11;

    uint8_t dxl_error = 0;  
    uint8_t opmode; // data received from read1ByteTxRx

    for (int i = 0; i < 5; i++) {
        packetHandler->read1ByteTxRx(portHandler, primary_dxl_id + i, ADDR_OP_MODE, &opmode, &dxl_error);
        result[i] = opmode;
    }
    
    return result;
}

bool setOpMode(
                dynamixel::PortHandler* portHandler, 
                dynamixel::PacketHandler* packetHandler,
                int dxl_id,
                uint8_t opmode) {

    uint8_t dxl_error = 0;  // Dynamixel error byte
    int dxl_comm_result;    // Communication result

    // Write operation mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_OP_MODE, opmode, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
        printf("Failed to write operation mode to Dynamixel ID %d: %s\n", dxl_id, packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    } else if (dxl_error != 0) {
        printf("Dynamixel ID %d returned error: %s\n", dxl_id, packetHandler->getRxPacketError(dxl_error));
        return false;
    }

    // Operation succeeded
    return true;
}

struct update_joints_struct {
    std::array<int32_t, 5> joints_array;
    bool success;
};

update_joints_struct updateJoints(  dynamixel::GroupBulkRead &groupBulkRead, 
                                    dynamixel::PacketHandler* packetHandler, 
                                    int primary_dxl_id) {
    
    int dxl_comm_result = COMM_TX_FAIL; // Communication result set initially to failed state
    uint8_t dxl_error = 0;              // to store result from packetHandler (communication)
    
    // handle errors
    dxl_comm_result = groupBulkRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      printf("txrxresult error\n");
      return {{0,0,0,0,0}, false};
    } else if (groupBulkRead.getError(primary_dxl_id, &dxl_error)) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      printf("rxpacket error\n");
      return {{0,0,0,0,0}, false};
    }

    std::array<int32_t, 5> joint_array;

    for (int i = 0; i < 5; i++) {
        int32_t encoderval = groupBulkRead.getData(primary_dxl_id + i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        joint_array[i] = encoderval;
    }

    return {joint_array, true};
}

bool enableTorque(  dynamixel::PortHandler* portHandler,
                    dynamixel::PacketHandler* packetHandler,
                    int primary_dxl_id) {

    uint8_t dxl_error = 0; 
    int dxl_comm_result = COMM_TX_FAIL; // Communication result set initially to failed state

    for (int i = 0; i < 5; i++) { // iterate from id 11 -> 15

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, primary_dxl_id + i, ADDR_TORQUE_ENABLE, 1, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            printf("[enableTorque] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        } else if (dxl_error != 0) {
            printf("[enableTorque] %s\n", packetHandler->getRxPacketError(dxl_error));
            return false;
        } else {
            printf("[enableTorque] servo %03d torque has been enabled!\n", primary_dxl_id + i);
        }
    }

    return true;

}

void disableTorque( dynamixel::PortHandler* portHandler,
                    dynamixel::PacketHandler* packetHandler,
                    int primary_dxl_id) {

    uint8_t dxl_error = 0; 
    int dxl_comm_result = COMM_TX_FAIL; // Communication result set initially to failed state

    for (int i=0; i<5; i++) { // iterate form id 11 -> 15

        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, primary_dxl_id + i, ADDR_TORQUE_ENABLE, 0, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        } else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        } else {
            printf("[disableTorque] servo %03d torque has been disabled!\n", primary_dxl_id + i);
        }

    }

    portHandler->closePort();
}

/*
diffWithinLimits() 
compares a requested joint array to the current joint array. 
If the values are too far apart, there's something wrong (like a crash) and new position shouldn't be sent.
Also used for checking if the robot is homed or not.
*/
bool diffWithinLimits(  std::array<int32_t, 5>& current, 
                        std::array<int32_t, 5>& desired,
                        int32_t lower, int32_t upper) {
    
    // a 50 step difference is ~4 deg (360steps/4096deg)*50 steps = 4.39453125 deg
    std::array<int32_t, 5> difference;


    for (int i = 0; i < 5; i++) {
        difference[i] = current[i] - desired[i];
        // printf("[diffWithinLimits] checking difference... current (%d) - desired {%d} = %d\n",current[i], desired[i], difference[i]); // debug
        if (difference[i] < lower || difference[i] > upper) {
       printf("[diffWithinLimits] difference %d is too large!\n", difference[i]); // debug
        return false;
        }
    }

    return true;
}

/*
setJoints() sets each servo to a desired position.
Given five servo positions as passed with "goals" int32 array, use dynamixel sdk 
to add that value to a packet and then send that packet.
*/
bool setJoints( std::array<int32_t, 5> current_joint_array,
                std::array<int32_t, 5> joint_array, 
                dynamixel::GroupBulkWrite &groupBulkWrite, 
                dynamixel::PacketHandler* packetHandler,
                int primary_dxl_id) {

    static bool try_addParam = false;

    static bool dxl_changeparam_result = false; // to store result from changeParam
    static bool dxl_comm_result = COMM_TX_FAIL;

    // check for excessive movement. if true, the movement is good.
    static const int32_t lower = -300;
    static const int32_t upper = 300;
    if (diffWithinLimits(current_joint_array, joint_array, lower, upper)) {     // this should really be a acceleration check not a difference check

        for (int i = 0; i < 5; i++) {
            uint8_t convertedData[4];
            convert_int32_uint8(joint_array[i], convertedData);
            dxl_changeparam_result = groupBulkWrite.changeParam(primary_dxl_id + i,
                                                                ADDR_GOAL_POSITION,
                                                                LEN_GOAL_POSITION,
                                                                convertedData);
            if (dxl_changeparam_result != true) {
                fprintf(stderr, "[setJoints] groupBulkWrite changeparam failed. Did you addParam()?\n");

                if (!try_addParam) {
                    try_addParam = true;
                }
            }
        }

        if (try_addParam) {

            bool dxl_addparam_result = false;   // to store result from addParam
            
            for (int i = 0; i < 5; i++) {
                uint8_t convertedData[4];
                convert_int32_uint8(joint_array[i], convertedData);
                dxl_addparam_result = groupBulkWrite.addParam(  primary_dxl_id + i,
                                                                ADDR_GOAL_POSITION,
                                                                LEN_GOAL_POSITION,
                                                                convertedData);
                if (dxl_addparam_result != true) {
                    fprintf(stderr, "[initDynamixel] groupBulkWrite addparam failed\n");
                } else {
                    printf("[initDynamixel] groupBulkWrite addparam successful!\n");
                }
            }

            try_addParam = false;

        }

        // Send message with txPacket().
        dxl_comm_result = groupBulkWrite.txPacket();

        if (dxl_comm_result != COMM_SUCCESS) {
            printf("[setJoints] %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        } else {
            return true;
        }

    } else {
        printf("[setJoints] difference between current and desired position is too large!\n");
        return false;
    }

}