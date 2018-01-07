3D Videos with show
===================

To make 3D videos, open `show` for the dataset.
1. Click "Animate Path" and store the images in a folder called "left"
2. Enter Advanced Mode and set "Shift Path for 3D" and the "3D Shift" for baseline
3. Animate Path again and store the images in a folder called "right"

Export for YouTube
-----------------
1. Append left and right images into one, reducing width of original images to half

    for (( c=1; c < 1000; c++ ))
    do
      `printf "convert +append -resize 960x1080! left/animframe%05d.jpg right/animframe%05d.jpg long3D/animframe%05d.jpg" "$c" "$c" "$c"`
    done
    mencoder "mf://long3D/?*.jpg" -mf fps=25 -o test.avi -ovc lavc -lavcopts vcodec=mjpeg:vbitrate=8000

2. Upload the video to YouTube, select "3D Video" in the advanced options. The video is already in 3D format.

Export for mplayer
------------------
    for (( c=1; c < 1000; c++ ))
    do
      `printf "composite -stereo +0 left/animframe%05d.jpg right/animframe%05d.jpg stereo3D/animframe%05d.jpg" "$c" "$c" "$c"`
    done
    mencoder "mf://stereo3D/?*.jpg" -mf fps=25 -o test.avi -ovc lavc -lavcopts vcodec=mjpeg:vbitrate=8000

