# takes photo from camera
mkdir snap_shot
mkdir snap_shot/L
mkdir snap_shot/R
for i in {1..10}; do
  if [[ ! -f "./snap_shot/L/img$i.jpg" ]]; then
    ffmpeg -f video4linux2 -s 320x240 -i /dev/video0 -frames 1  "./snap_shot/L/img$i".jpg
  fi

  if [[ ! -f "./snap_shot/R/img$i.jpg" ]]; then
    ffmpeg -f video4linux2 -s 320x240 -i /dev/video1 -frames 1 "./snap_shot/R/img$i".jpg
    break
  fi
done
