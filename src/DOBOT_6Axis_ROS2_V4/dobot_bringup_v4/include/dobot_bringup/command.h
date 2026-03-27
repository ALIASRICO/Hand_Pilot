/**
 ***********************************************************************************************************************
 *
 * @author qiufengwu
 * @date   2024/01/23
 *
 *
 ***********************************************************************************************************************
 */

#ifndef COMMANDER_H
#define COMMANDER_H

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <algorithm>
#include <regex>
#include <assert.h>
#include <cstring>
#include <dobot_bringup/tcp_socket.h>

#pragma pack(push, 1)
// data aligned to 8-byte and 48-byte boundaries
// size: 30x8x6 = 30x6xsizeof(double)
typedef struct RealTimeData_t
{
    //
    uint16_t len;                // 0000 ~ 0001  character length
    uint16_t Reserve[3];         // 0002 ~ 0007  placeholder
    uint64_t digital_input_bits; // 0008 ~ 0015  DI calculated by bit
    uint64_t digital_outputs;    // 0016 ~ 0023  DO calculated by bit
    uint64_t robot_mode;         // 0024 ~ 0031  robot mode
    uint64_t controller_timer;   // 0032 ~ 0039  robot timestamp (ms since epoch 1970)
    uint64_t run_time;           // 0040 ~ 0047  robot uptime in ms
    // 0048 ~ 0095                       //
    uint64_t test_value;         // 0048 ~ 0055  memory structure test reference value  0x0123 4567 89AB CDEF
    double safety_mode;          // 0056 ~ 0063  (deprecated field)
    double speed_scaling;        // 0064 ~ 0071  global speed scaling
    double linear_momentum_norm; // 0072 ~ 0079  robot current momentum (not implemented)
    double v_main;               // 0080 ~ 0087  controller board voltage (not implemented)
    double v_robot;              // 0088 ~ 0095  robot voltage (48V)
    // 0096 ~ 0143                       //
    double i_robot;                      // 0096 ~ 0103 robot current
    double program_state;                // 0104 ~ 0111 script execution state
    double safety_status;                // 0112 ~ 0119 safety status (not implemented)
    double tool_accelerometer_values[3]; // 0120 ~ 0143 TCP acceleration (not implemented)
    // 0144 ~ 0191                       //
    double elbow_position[3]; // 0144 ~ 0167 elbow position (not implemented)
    double elbow_velocity[3]; // 0168 ~ 0191 elbow velocity (not implemented)
    // 0192 ~ ...                        //
    double q_target[6];           // 0192 ~ 0239  // target joint position
    double qd_target[6];          // 0240 ~ 0287  // target joint velocity
    double qdd_target[6];         // 0288 ~ 0335  // target joint acceleration
    double i_target[6];           // 0336 ~ 0383  // target joint current
    double m_target[6];           // 0384 ~ 0431  // target joint torque
    double q_actual[6];           // 0432 ~ 0479  // actual joint position
    double qd_actual[6];          // 0480 ~ 0527  // actual joint velocity
    double i_actual[6];           // 0528 ~ 0575  // actual current
    double i_control[6];          // 0576 ~ 0623  // TCP sensor force (not implemented)
    double tool_vector_actual[6]; // 0624 ~ 0671  // TCP actual coordinates (TCP: tool center point)
    double TCP_speed_actual[6];   // 0672 ~ 0719  // TCP velocity
    double TCP_force[6];          // 0720 ~ 0767  // TCP force (current loop calculation)
    double tool_vector_target[6]; // 0768 ~ 0815  // TCP target coordinates
    double TCP_speed_target[6];   // 0816 ~ 0863  // TCP target velocity
    double motor_temperatures[6]; // 0864 ~ 0911  // joint temperature
    double joint_modes[6];        // 0912 ~ 0959  // joint control mode
    double v_actual[6];           // 960  ~ 1007  // joint voltage
    int8_t handtype[4];           // 1008,1009,1010,1011 R, D, N, cfg   handedness info (removed in newer versions)
    int8_t userCoordinate;        // 1012 user coordinate system ID
    int8_t toolCoordinate;        // 1013 tool coordinate system ID
    int8_t isRunQueuedCmd;        // 1014 algorithm queue running flag
    int8_t isPauseCmdFlag;        // 1015 algorithm queue pause flag
    int8_t velocityRatio;         // 1016 joint velocity ratio
    int8_t accelerationRatio;     // 1017 joint acceleration ratio
    int8_t jerkRatio;             // 1018 joint jerk ratio (not implemented)
    int8_t xyzVelocityRatio;      // 1019 Cartesian position velocity ratio (x,y,z in distance/s)
    int8_t rVelocityRatio;        // 1020 Cartesian orientation velocity ratio (rx,ry,rz in deg/s)
    int8_t xyzAccelerationRatio;  // 1021 Cartesian position acceleration ratio
    int8_t rAccelerationRatio;    // 1022 Cartesian orientation acceleration ratio
    int8_t xyzJerkRatio;          // 1023 Cartesian position jerk ratio (not implemented)
    int8_t rJerkRatio;            // 1024 Cartesian orientation jerk ratio (not implemented)
    int8_t BrakeStatus;           // 1025 robot brake status
    int8_t EnableStatus;          // 1026 robot enable status
    int8_t DragStatus;            // 1027 robot drag status
    int8_t RunningStatus;         // 1028 robot running status
    int8_t ErrorStatus;           // 1029 robot alarm status
    int8_t JogStatus;             // 1030 robot jog status
    int8_t RobotType;             // 1031 M1 model handedness
    int8_t DragButtonSignal;      // 1032 button board drag signal
    int8_t EnableButtonSignal;    // 1033 button board enable signal
    int8_t RecordButtonSignal;    // 1034 button board record signal
    int8_t ReappearButtonSignal;  // 1035 button board replay signal
    int8_t JawButtonSignal;       // 1036 button board gripper control signal
    int8_t SixForceOnline;        // 1037 6-axis force online status (not implemented)
    int8_t CollisionStates;       // 1038 collision state
    int8_t ArmApproachState;      // 1039 forearm approach pause state
    int8_t J4ApproachState;       // 1040 J4 approach pause state
    int8_t J5ApproachState;       // 1041 J5 approach pause state
    int8_t J6ApproachState;       // 1042 J6 approach pause state
    int8_t Reserve2[61];          // 1043 ~ 1103   reserved
    double vibrationDisZ;         // 1104 ~ 1111 accelerometer Z-axis vibration displacement
    uint64_t currentCommandId;    // 1112 ~ 1119 current motion queue ID
    double m_actual[6];           // 1120 ~ 1167 actual torque
    double load;                  // 1168 ~ 1175 payload mass
    double centerX;               // 1176 ~ 1183 payload X center-of-mass offset
    double centerY;               // 1184 ~ 1191 payload Y center-of-mass offset
    double centerZ;               // 1192 ~ 1199 payload Z center-of-mass offset
    double user[6];               // 1200 ~ 1247 user coordinate system values
    double tool[6];               // 1248 ~ 1295 tool coordinate system values
    double TraceIndex;            // 1296 ~ 1303 trajectory replay index (not implemented)
    double SixForceValue[6];      // 1304 ~ 1351 6-axis force sensor raw values
    double TargetQuaternion[4];   // 1352 ~ 1383 target quaternion
    double ActualQuaternion[4];   // 1384 ~ 1415 actual quaternion
    uint16_t AutoManualMode;      // 1416 ~ 1417 manual/auto mode: 0=off, 1=manual, 2=auto
    int8_t Reserve3[22];          // 1418 ~ 1439
} RealTimeData;
#pragma pack(pop)

class CRCommanderRos2
{
protected:
    static constexpr double PI = 3.1415926;

private:
    std::mutex mutex_;
    double current_joint_[6];
    double tool_vector_[6];
    std::shared_ptr<RealTimeData> real_time_data_;
    std::atomic<bool> is_running_;
    std::unique_ptr<std::thread> thread_;
    std::shared_ptr<TcpClient> real_time_tcp_;
    std::shared_ptr<TcpClient> dash_board_tcp_;

public:
    explicit CRCommanderRos2(const std::string &ip);

    ~CRCommanderRos2();
    void getCurrentJointStatus(double *joint);
    void getToolVectorActual(double *val);
    void recvTask();
    void init();
    bool callRosService(const std::string cmd, int32_t &err_id);
    bool callRosService_f(const std::string cmd, int32_t &err_id,std::string &mode_id);
    bool callRosService(const std::string cmd, int32_t &err_id, std::vector<std::string> &result_);
    bool isEnable() const;
    bool isConnected() const;
    uint16_t getRobotMode() const;
    std::shared_ptr<RealTimeData> getRealData() const;

private:
    static void doTcpCmd(std::shared_ptr<TcpClient> &tcp, const char *cmd, int32_t &err_id,
                         std::vector<std::string> &result);
    static void doTcpCmd_f(std::shared_ptr<TcpClient> &tcp, const char *cmd, int32_t &err_id,std::string &mode_id,
                         std::vector<std::string> &result);
    static inline double rad2Deg(double rad)
    {
        return rad * 180.0 / PI;
    }

    static inline double deg2Rad(double deg)
    {
        return deg * PI / 180.0;
    }
};

#endif // COMMANDER_H
