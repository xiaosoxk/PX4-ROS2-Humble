# AI 唯一合法输出: 期望速度/偏航指令, 必须经 safety_arbiter 校验后才可下发
std_msgs/Header header
float32 vx_body      # 前向 m/s
float32 vy_body      # 右向 m/s
float32 vz_body      # 垂直 m/s
float32 yaw_rate_dps # 偏航角速度 deg/s
int32   source       # 0跟随 1避障规划 2智能拍摄
