#ifndef ROS2_STRUCTS_H
#define ROS2_STRUCTS_H

struct PowerStateMsg {
    float system_voltage_v;
    float system_current_a;
    float motors_voltage_v;
    float motors_current_a;
    bool is_critical;     
};

struct OdometryMsg {
    float left_rpm;       
    float right_rpm;      
    float pitch_angle;    
};

struct LidarPointMsg {
    float angle_deg;    
    float distance_mm; 
    bool is_valid;     
};

#endif
