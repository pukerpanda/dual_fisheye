

# Double fisheye image to panorama (Samsung Gear360/2017)

1. Install Hugin
 ```  
 sudo add-apt-repository ppa:ubuntuhandbook1/apps
 sudo apt install hugin
 ```

3. Setup converter

	https://github.com/ultramango/gear360pano.git

4. Run convertion
  ```
  ls -1 ~/Videos/gear360_nazare/100PHOTO/*MP4 | parallel --load 99% --noswap --memfree 500M --bar   ./gear360video.sh -p {}
  ```

6. Add EXIF tag
  ```
   exiftool -XMP-GSpherical:Spherical="true" file.mp4

   ffprobe -v error -of flat=s=_ -select_streams v:0 -show_entries stream=height,width  data/test.mp4
   ./scripts/video_equirect2cubemap_images.py -i data/test_pano.mp4 -o data/faro/images -n 10 -s 2048
  ```


5. Use spatial audio in 360-degree and VR videos
   
   [https://support.google.com/youtube/answer/6395969]
   
   [https://github.com/google/spatial-media/releases/tag/v2.1]
