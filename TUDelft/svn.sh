echo "Archive Paparazzi -> SVN:SmartUAV"
git archive HEAD | tar -x -C ../svn_pprz
echo "Archive LuftBoot -> SVN"
cd ./sw/ext/luftboot
git archive HEAD | tar -x -C ../../../../svn_pprz/sw/ext/luftboot
echo "Archive LibOpenCM3 -> SVN"
cd ../libopencm3/
git archive HEAD | tar -x -C ../../../../svn_pprz/sw/ext/libopencm3/
cd ../../../

