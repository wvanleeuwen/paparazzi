#echo "Cleanup TUDelft SVN Version"
#cd ../gitsvnpprz/
#make clean 2&>1 > tmp.txt
#rm tmp.txt
#rm -vrf ./conf/conf.xml.*-*
#git svn rebase
#git svn dcommit
#cd ../contribute2pprz/
#git reset --hard 
#git status -s | sed 's/?? /rm -rf /' > rm.sh
#chmod +x ./rm.sh
#./rm.sh
#git fetch paparazzi
#git pull paparazzi master
#cd ../gitsvnpprz/
#echo "Exporting TUDelft SVN To Tudelft git"
#cd ../tudelft
#git checkout master
#cd ../gitsvnpprz/
#git archive master | tar -x -C ../paparazzi/
echo "Exporting TUDelft SVN To Paparazzi"
git archive master | tar -x -C ../paparazzi/ 
cd ../paparazzi/
git status -s | grep 'TUDelft' | sed 's/?? /rm -rf /' > rm.sh
chmod +x ./rm.sh
./rm.sh
rm ./rm.sh
rm -rf ./sw/airborne/modules/onboardcam
rm -rf ./sw/airborne/modules/opticflow
rm -rf ./conf/conf.xml
rm -rf ./conf/controlpanel.xml
#rm -rf ./sw/airborne/boards/tiny_sense.h
#rm -rf ./sw/airborne/booz/aerovinci
#rm -rf ./conf/joysticks
#rm -rf ./conf/simulator/jsbsim/aircraft/LISA_BOOZ_BART.xml
#echo "# Add following SVN files to Paparazzi Github" > ./add.sh
#git status -s | grep -v add.sh | sed 's/?? /git add /' >> add.sh
#chmod +x ./add.sh
#gedit ./add.sh
