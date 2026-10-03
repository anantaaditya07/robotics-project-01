# SemanticObstacle.msg
std_msgs/Header header        # frame_id = "map", stamp = source image stamp
uint32 id                     # stable track id
string class_name             # "person", "chair", ...
float32 confidence
geometry_msgs/Point position # in header.frame_id
float32 range                 # LiDAR range used (m), for debugging
float32 radius                # footprint radius (m), from per-class table
builtin_interfaces/Duration ttl
