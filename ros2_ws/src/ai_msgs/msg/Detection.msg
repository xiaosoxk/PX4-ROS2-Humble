std_msgs/Header header
int32 cls            # 类别: 0行人 1车辆 2船只 3动物
float32 conf         # 置信度 0~1
float32 x_min
float32 y_min
float32 x_max
float32 y_max        # 像素框
float32 bearing_deg  # 相对机头的方位角(已用内参解算)
