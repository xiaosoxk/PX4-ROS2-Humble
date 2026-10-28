std_msgs/Header header
string node_name     # 检测/跟踪/规划/云台...
float32 latency_ms   # 本节点最近处理延迟
uint8  state         # 0正常 1降级 2故障
string detail
