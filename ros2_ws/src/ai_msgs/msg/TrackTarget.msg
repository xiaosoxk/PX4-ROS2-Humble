std_msgs/Header header
int32 track_id       # 稳定目标ID, -1=无
int32 cls
float32 bearing_deg  # 目标方位
float32 distance_m   # 估计距离(激光/双目), NaN=未知
float32 conf
bool  lost           # 跟丢标志
