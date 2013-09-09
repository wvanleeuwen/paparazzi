export PKG_CONFIG_PATH=/opt/arm/gst/lib/pkgconfig
cd ~/svn/Aero/code/gst-MAVLab
sudo sb2 make clean install
ftp 192.168.1.1 <<script

cd kevin
cd usb
cd arm
cd gst
cd lib
cd gstreamer-0.10
put libMAVLab.so
mkdir hoi
bye
script


#dit werkt niet:
telnet 192.168.1.1 <<script2

#rm /.gstreamer-0.10/registry.arm.bin # strange -> brakes new file...

cd data/video/kevin/usb/arm/gst/lib/gstreamer-0.10
while [ ! -f hoi ]
do
  sleep 2
done
chmod 777 libMAVLab.so
rm hoi -r
exit
script2
