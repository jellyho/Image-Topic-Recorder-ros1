#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import datetime, os

class ImageToVideoConverter:
    def __init__(self, topic, fps, decoding):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.image_callback)
        self.fps = 24 if fps is None else fps
        self.decoding = decoding
        self.video_writer = None
        self.last_frame_time = None
        self.datetime = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        self.home_dir = os.path.expanduser('~')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.decoding)
        except Exception as e:
            print(e)
            return
        current_frame_time = rospy.Time.now()
        if self.last_frame_time is not None:
            time_diff = (current_frame_time - self.last_frame_time).to_sec()
            if self.video_writer is None:
                height, width, _ = cv_image.shape
                fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Video codec
                self.video_writer = cv2.VideoWriter(self.home_dir+f'/recorded_video_{self.datetime}.avi', fourcc, self.fps, (width, height))
                rospy.loginfo('[Image Recorder] Record Started.')
            
            num_frame = int(time_diff * self.fps)
            for _ in range(num_frame):
                self.video_writer.write(cv_image)
            self.last_frame_time = current_frame_time - rospy.Duration.from_sec(time_diff - (float(num_frame) / self.fps))
        else:
            self.last_frame_time = current_frame_time

    def run(self):
        rospy.spin()
        if self.video_writer is not None:
            self.video_writer.release()
            rospy.loginfo('[Image Recorder] Video saved.')

if __name__ == '__main__':
    rospy.init_node('image_recorder', anonymous=True)
    topic = rospy.get_param('~topic_name')
    fps = rospy.get_param('~fps')
    decoding = rospy.get_param('~decoding')
    converter = ImageToVideoConverter(topic, fps, decoding)
    converter.run()
