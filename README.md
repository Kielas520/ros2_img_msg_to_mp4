# ros2_img_msg_to_mp4
# 同时录制图像话题和相机信息话题
ros2 bag record -o autoaim_data_bag /image_raw /camera_info /imu/rpy
ros2 bag play autoaim_data_bag
