# Image-Topic-Recorder-ros1
Image Topic Recorder that preservate frame interval in python


### The recorded video is the same as real time
You don't have to worry about the awkwardness of video recording caused by the jagged spacing (processing time, communication time, etc.) between the input video frame and the input topic. The time is automatically calculated and always a real-time video is created.

As you can see from the video below, it can be seen that the video does not accelerate and maintains real-time even though the interval between input topics has increased by turning the AI model.

https://github.com/jellyho/Image_topic_recorder-ros1/assets/20741606/ea9f0d1b-e048-4220-b69c-e39546339139


#### 1) Installation
```jsx
cd catkin_ws/src
git clone https://github.com/jellyho/Image_topic_recorder-ros1
cd ..
catkin_make
```

#### 2) Run
```jsx
source devel/setup.bash

roslaunch recorder record.launch topic:="/image_topic_name" fps:=24
```

> You should consider rgb encoding [here](https://github.com/jellyho/Image_topic_recorder-ros1/blob/6c64b94fe193cf3d61aae27149717dc4c7f6d959/recorder/scripts/ImageRecorder.py#L21)(rgb, bgr .. etc)
