#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloStopDetector(Node):
    def __init__(self):
        super().__init__('yolo_stop_detector')
        self.bridge = CvBridge()#用于ros图片和cv图片转换
        self.image_sub = self.create_subscription(Image,'/ascamera/camera_publisher/rgb0/image',self.image_callback,10)#图片订阅
        self.result_image_pub = self.create_publisher(Image,'/yolo_result_image',10)#检测图片发布
        self.stop_pub = self.create_publisher(Bool,'/stop_detected',10)#停止消息发布
        self.model = YOLO('/home/ubuntu/ros2_ws/yolo_models/yolov8n.pt')#加载推理模型
        self.cv_image_count = 0
        self.get_logger().info("YOLO停止标志检测器已启动")
        self.last_stop = False
    def image_callback(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        self.cv_image_count += 1
        # 每5帧检测一次
        if self.cv_image_count % 5 != 0:
            return
        result = self.model.predict(cv_image,imgsz=416,conf=0.4,verbose=False)[0]
        # 可用的属性和方法
        # result.boxes        # 检测框信息
        # result.boxes.xyxy   # 框的坐标 [x1, y1, x2, y2]
        # result.boxes.cls    # 类别 ID（整数）
        # result.boxes.conf   # 置信度（0~1）
        # result.boxes.data   # 全部数据 [x1, y1, x2, y2, conf, cls]
        # result.names        # 类别名字典 {0: 'person', 11: 'stop sign', ...}
        # result.plot()       # 画检测结果的图片
        annotated_image = result.plot()
        self.publish_result_image(annotated_image)#发布检测画面
        stop=False
        for cls, conf, box in zip(result.boxes.cls,result.boxes.conf,result.boxes.xyxy):
            if int(cls.item()) == 11 and conf.item()>0.4: # item() 把 PyTorch 张量（tensor）转换成 Python 数字 
                x1,y1,x2,y2 = map(int,box.tolist())
                area=(x2-x1)*(y2-y1)
                h,w=cv_image.shape[:2]
                if area > 0.02*w*h:     #检测框大于图片一定面积生效
                    stop=True
                    break
        msg=Bool()
        msg.data=stop
        self.stop_pub.publish(msg)
        if stop and not self.last_stop:
            self.get_logger().info("YOLO检测到停止标志")
        self.last_stop = stop
            
    def publish_result_image(self, image):
        result_msg = self.bridge.cv2_to_imgmsg(image,encoding='bgr8')
        self.result_image_pub.publish(result_msg)
def main():
    rclpy.init()
    node=YoloStopDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__=="__main__":
    main()