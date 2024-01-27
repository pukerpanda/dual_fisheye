
echo -n 'frames: '; ffprobe -v error -select_streams v:0 -count_packets -show_entries stream=nb_read_packets -of csv=p=0  $1
echo -n 'duration: '; ffprobe -v error -show_entries format=duration -of default=noprint_wrappers=1:nokey=1  $1
echo -n 'resolution: '; ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0  $1