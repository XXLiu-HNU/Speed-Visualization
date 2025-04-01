ffmpeg -framerate 30 -i temp_frames/frame_%04d.png -c:v png -pix_fmt rgba output.mov
