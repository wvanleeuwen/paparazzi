echo kevins video initializer!
kill -9 `pidof program.elf`
kill -9 `pidof gst-launch-0.10`
mkdir -p /opt
mount --bind /data/video/raw/opt /opt
export PATH=/opt/arm/gst/bin:$PATH
export DSP_PATH=/opt/arm/tidsp-binaries-23.i3.8/
/bin/dspbridge/cexec.out -T /opt/arm/tidsp-binaries-23.i3.8/baseimage.dof -v
/bin/dspbridge/dynreg.out -r /opt/arm/tidsp-binaries-23.i3.8/m4venc_sn.dll64P -v


