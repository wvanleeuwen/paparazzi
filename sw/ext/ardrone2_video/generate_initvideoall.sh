ip="$(ifconfig | grep  192.168.1 | cut -d ':' -f 2 |  cut -d ' ' -f 1)"
echo "auto configuring ip... using:"
echo $ip
echo $1
echo "# enable the following line to auto start video stream from cam1"
echo "#gst-launch v4l2src device=/dev/video2 ! videorate ! 'video/x-raw-yuv,framerate=15/1' ! videoscale ! video/x-raw-yuv, width=320, height=240 ! example tcp_port=2002 threshtune=99 ! dspmp4venc ! rtpmp4vpay config-interval=2 ! udpsink host=$ip port=5000" >> $1/initvideoall.sh
echo "# or the following line use zbar on cam2: (no ppz communication -> ppz video module will crash if started )"
echo "gst-launch v4l2src device=/dev/video2 ! videorate ! 'video/x-raw-yuv,framerate=15/1' ! videoscale ! video/x-raw-yuv, width=320, height=240 ! ffmpegcolorspace ! zbar ! dspmp4venc ! rtpmp4vpay config-interval=2 ! udpsink host=$ip port=5000" >> $1/initvideoall.sh
echo "or the following line which will run Guido's skysegmentation"
echo "#gst-launch v4l2src device=/dev/video1 ! videorate ! 'video/x-raw-yuv,framerate=15/1' ! videoscale ! video/x-raw-yuv, width=160, height=96 ! mavlab adjust=6 ! queue ! dspmp4venc ! rtpmp4vpay config-interval=2 ! udpsink host=$ip port=5000" >> $1/initvideoall.sh
