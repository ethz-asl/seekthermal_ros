uint32 height
uint32 width

# float32 image, raw data
float32[] data_raw # actual matrix data, size is (height*width) col1, col2, ...

# uint8 image, dead pixel mask
uint8[] data_mask # actual matrix data, size is (height*width) col1, col2, ...

# images
sensor_msgs/Image image_gray
sensor_msgs/Image image_colored
