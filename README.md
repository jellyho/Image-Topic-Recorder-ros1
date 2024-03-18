# Image-Topic-Recorder-ros1
Image Topic Recorder that preservate frame interval in python


### Frames are not skipped, not accelerated. Image Topics are recorded at the same incoming speed.

1) Installation
```jsx
cd catkin_ws/src
git clone https://github.com/jellyho/Image_topic_recorder-ros1
cd ..
catkin_make
```

2) Run
```jsx
source devel/setup.bash

roslaunch recorder record.launch topic:="/image_topic_name" fps:=24
```

> You should consider rgb encoding here(rgb, bgr .. etc)
