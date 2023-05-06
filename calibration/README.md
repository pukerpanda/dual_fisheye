# Canera calibration

## Huawei, Panoramic, Camera CV60, Dual 13 MP

 http://wiki.ros.org/camera_calibration

1. Collect calibration images: To calibrate the camera, you need to
   capture several images of a calibration pattern, such as a
   chessboard or a circle grid. It's important to capture the images
   from different angles and orientations to ensure the calibration is
   accurate.

   FEX: Make short film with checkerboard.

2. Use ROS camera_calibration allows easy calibration of monocular or
   stereo cameras using a checkerboard calibration target.

3. Replay captured sequence to ROS topic:
	: ./scripts/video_replay2ros_topic.py -i data/PIC_20230504_080209.mp4 -t camera/image_raw/compressed -r 10

4. Convert equirect to cubemap and select face to calibrate.
   rosrun camera_split main.py _camera:=front _side:=1024

	Input equirect shape (2688, 5376, 3)
	Output cubemap{face} shape (1024, 1024, 3)
