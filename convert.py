import os
import cv2
import rclpy
from rclpy.serialization import deserialize_message
# 【修正1】在这里导入 StorageFilter
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tqdm import tqdm

# ================= 配置区域 =================
# 注意：这里要是文件夹路径
BAG_PATH = 'autoaim_data_bag'
TOPIC_NAME = '/image_raw'
OUTPUT_VIDEO = 'output_video.mp4'
FPS = 240
# ===========================================

def get_total_message_count(bag_path, topic_name):
    try:
        import yaml
        metadata_path = os.path.join(bag_path, 'metadata.yaml')
        if not os.path.exists(metadata_path):
            return None
        with open(metadata_path, 'r') as f:
            meta = yaml.safe_load(f)
        topics = meta.get('rosbag2_bagfile_information', {}).get('topics_with_message_count', [])
        for t in topics:
            if t['topic_metadata']['name'] == topic_name:
                return t['message_count']
    except Exception:
        pass
    return None

def main():
    if not os.path.exists(BAG_PATH):
        print(f"错误: 找不到路径 {BAG_PATH}")
        return

    storage_options = StorageOptions(uri=BAG_PATH, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # 【修正2】使用正确的类初始化过滤器
    storage_filter = StorageFilter(topics=[TOPIC_NAME])
    reader.set_filter(storage_filter)

    bridge = CvBridge()
    video_writer = None
    frame_count = 0

    total_msgs = get_total_message_count(BAG_PATH, TOPIC_NAME)
    pbar = tqdm(total=total_msgs, unit='frame', desc="Converting")

    print(f"开始读取: {BAG_PATH}")

    while reader.has_next():
        (topic, data, t) = reader.read_next()

        # 虽然加了 filter，还是建议保留这个判断作为双重保险
        if topic == TOPIC_NAME:
            try:
                msg = deserialize_message(data, Image)

                # 图像格式转换逻辑
                if msg.encoding == "bgr8":
                    cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
                elif msg.encoding == "rgb8":
                    cv_img = bridge.imgmsg_to_cv2(msg, "rgb8")
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
                else:
                    try:
                        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
                    except CvBridgeError:
                        cv_img = bridge.imgmsg_to_cv2(msg, "passthrough")
                        if "bayer" in msg.encoding:
                            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BayerBG2BGR)
                        elif len(cv_img.shape) == 2:
                             cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)

                if video_writer is None:
                    height, width = cv_img.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    video_writer = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, FPS, (width, height))

                video_writer.write(cv_img)
                frame_count += 1
                pbar.update(1)

            except Exception as e:
                print(f"跳过一帧: {e}")
                continue

    pbar.close()
    if video_writer:
        video_writer.release()
        print(f"转换完成！共 {frame_count} 帧")

if __name__ == "__main__":
    main()
