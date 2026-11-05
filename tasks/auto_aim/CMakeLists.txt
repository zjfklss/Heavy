add_library(auto_aim STATIC
  armor.cpp
  classifier.cpp
  detector.cpp
  solver.cpp
  target.cpp
  tracker.cpp
  yolo.cpp
  yolos/yolov5.cpp
  yolos/yolov8.cpp
  yolos/yolo11.cpp
  multithread/mt_detector.cpp
)
target_link_libraries(auto_aim PUBLIC tools openvino::runtime)
