

# NERFSTUDIO

https://github.com/nerfstudio-project/nerfstudio

To process your own data run:

## Split video into cubemap sequnce

Script takes equirect frame and split it into face.
To keep


```shell
./scripts/video_equirect2cubemap_images.py -i data/fantana.mp4   -o data/fantana2/images -n 100 -s 820
```

## Use Metashape (or colmap) to recover image poses

Export scene cameras and convert them to Nerfstudio format

```shell
python3 ../../scripts/agi2nerf.py --xml_in transforms_agisoft_opt.xml
```

## Train NeRf network

ns-train nerfacto --data data/CV60/fantana2

## Render trajectory into video
ns-render --load-config outputs/fantana2/nerfacto/2023-05-09_104053/config.yml --traj filename --camera-path-filename data/CV60/fantana2/camera_paths/2023-05-09_104053.json --output-path renders/fantana2/2023-05-09_104053.mp4


## Convert a video to GIF using ffmpeg

HOWTO: https://superuser.com/questions/556029/how-do-i-convert-a-video-to-gif-using-ffmpeg-with-reasonable-quality

```shell
ffmpeg -i renders/fantana2/2023-05-09_104053.mp4 -vf "fps=25,scale=640:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" output.gif
```


![fantana](images/fantana.gif "Fantana")
