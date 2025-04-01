# 🚗 Speedometer Visualization Tool
Dynamically generate speed dashboard animations, support transparent background video export, adapt to video editing software (such as Premiere Pro, Jianying, etc.), add professional speed indication effects for sports scenes, racing games, data visualization, etc.

## 📸 Demo & Results
![动效演示](misc/smooth_speed.gif)
## Add in a Video
Use ffmepg to generate transparent videos in the folder where the generated images are stored, thereby adding speed prompts in video production

`ffmpeg -framerate 30 -i frame_%04d.png -c:v png -pix_fmt rgba output.mov`

Afterwards, use editing software such as PR and JianYing for video production
![image](https://github.com/user-attachments/assets/3e2c3286-a12f-4608-b11f-631d4dcd9359)
