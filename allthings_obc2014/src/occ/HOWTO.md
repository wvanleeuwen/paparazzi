# CHDK onto Camera SD

## Get the pre-build CHDK

From the repo server found we install our copy of the CHDK onto an SD card need for in the OCC. get it via:

 $ wget http://www.github.com/openuas/obc2014/occ_chdk_latest.img.xz

Somtimes the SD cannot work with a blocksize of 4M use bs=256k in that case

## Copy OS to SD storage 

Insert youSD card into your PC SD reader. One can directly write the CHDK to SD using xzcat and dd. The xzcat and dd commands can be used to uncompress and write the .xz image to an SD flash device on the fly. Do it via:

 $ xzcat occ_chdk_latest.img.xz | pv | dd of=/dev/sdX bs=4M

Where sdX should be replaced by the device of your SD reader most of the time this is sdb

Always type sync before removing the media to make sure that all data is written. 

 $ sync

WARNING: Ubuntu and some other distributions, auto-mount the media when inserted. This can cause inconsistency and errors! You can umount all using: umount /dev/sdX*

 $ sudo umount /dev/sdb


