

# Double fisheye image to panorama

1. Install Hugin
   : sudo add-apt-repository ppa:ubuntuhandbook1/apps
   : sudo apt install hugin

2. Setup converter

	https://github.com/ultramango/gear360pano.git

3. Run convertion
   : ls -1 ~/Videos/gear360_nazare/100PHOTO/*MP4 | parallel --load 99% --noswap --memfree 500M --bar ./gear360video.sh -p {}

4. Add EXIF tag
   : exiftool -XMP-GSpherical:Spherical="true" file.mp4