echo kevins video initializer!
kill -9 `pidof program.elf`
kill -9 `pidof gst-launch-0.10`
mkdir -p /opt
mount --bind /data/video/raw/opt /opt
export PATH=/opt/arm/gst/bin:$PATH
export DSP_PATH=/opt/arm/tidsp-binaries-23.i3.8/
/bin/dspbridge/cexec.out -T /opt/arm/tidsp-binaries-23.i3.8/baseimage.dof -v
/bin/dspbridge/dynreg.out -r /opt/arm/tidsp-binaries-23.i3.8/m4venc_sn.dll64P -v


#gst-launch v4l2src device=/dev/video2 ! videorate ! 'video/x-raw-yuv,framerate=15/1' ! videoscale ! video/x-raw-yuv, width=320, height=240 ! example tcp_port=2002 threshtune=99 ! dspmp4venc ! rtpmp4vpay config-interval=2 ! udpsink host=192.168.1.3 port=5000
gst-launch v4l2src device=/dev/video2 ! videorate ! 'video/x-raw-yuv,framerate=15/1' ! videoscale ! video/x-raw-yuv, width=320, height=240 ! ffmpegcolorspace ! zbar ! dspmp4venc ! rtpmp4vpay config-interval=2 ! udpsink host=192.168.1.3 port=5000
#gst-launch v4l2src device=/dev/video1 ! videorate ! 'video/x-raw-yuv,framerate=15/1' ! videoscale ! video/x-raw-yuv, width=160, height=96 ! mavlab adjust=6 ! queue ! dspmp4venc ! rtpmp4vpay config-interval=2 ! udpsink host=192.168.1.3 port=5000
