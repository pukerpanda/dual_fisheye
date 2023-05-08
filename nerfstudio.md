

# NERFSTUDIO

https://github.com/nerfstudio-project/nerfstudio

To process your own data run:

1. Convert equirect video to poses

``` shell
ns-process-data video --camera-type equirectangular --images-per-equirect 8 --num-frames-target 100 --data data/CV60/bonsai.mp4 --output-dir data/CV60/bonsai --skip-colmap --crop-factor  0.2 0.2 0 0
```

Removes top and bottom views from omni camera
--crop-factor {top bottom left right}


ns-process-data video --camera-type equirectangular --images-per-equirect 8 --num-frames-target 100 --data data/CV60/bonsai.mp4 --output-dir data/CV60/bonsai --skip-image-processing


ns-process-data images --data data/CV60/bonsai/images_mask --output-dir data/CV60/bonsai/colmap/ --skip-image-processing
