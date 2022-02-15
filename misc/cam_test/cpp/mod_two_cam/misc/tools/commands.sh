// takes photo from camera
for i in {1..10}; do
    if ! test -f "L/img$i.jpg"; then
      ffmpeg -f video4linux2 -s 320x240 -i /dev/video0 -frames 1  "img$i".jpg
    fi

    if ! test -f "R/img$i.jpg"
      ffmpeg -f video4linux2 -s 320x240 -i /dev/video1 -frames 1 "img$i".jpg
      break
    fi
done
