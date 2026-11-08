uint8 RED = 0
uint8 BLUE = 1
uint8 NONE = 2
uint8 PURPLE = 3

float32[] kpx
float32[] kpy

uint8 color
string number
string type
float32 distance_to_image_center
float64 yaw
geometry_msgs/Pose pose

float32 detect_quality
float32 reprojection_error
uint8   target_range       # 0=CLOSE, 1=MEDIUM, 2=FAR