# takes photo from camera
for i in {1..10}; do
  if [[ ! -f "L/img$i.jpg" ]]; then
    ffmpeg -f video4linux2 -s 320x240 -i /dev/video0 -frames 1  "L/img$i".jpg
  fi

  if [[ ! -f "R/img$i.jpg" ]]; then
    ffmpeg -f video4linux2 -s 320x240 -i /dev/video1 -frames 1 "R/img$i".jpg
    break
  fi
done
