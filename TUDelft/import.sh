echo "Updating/Cleaning SVN"
cd ../gitsvnpprz
rm -vrf ./conf/conf.xml.*-*
echo "Make Clean"
make clean 2&>1 > tmp.txt
rm tmp.txt 
echo "Rebase and Dcommit"
git svn rebase
git svn dcommit
echo "GIT VERSION"
cd ../tudelft/
git checkout master
git reset --hard
echo "Remove Untracked Files" 
git status -s | sed 's/??/rm -rf/' > rm.sh
chmod +x ./rm.sh
./rm.sh
echo "Pull latest remote version"
git fetch tudelft
git pull tudelft master
echo "Exporting GIT -> SVN"
git archive master | tar -x -C ../gitsvnpprz
cd ../gitsvnpprz/
echo "Adding new Files"
git status -s | grep -v add.sh | grep '?? ' | sed 's/?? /git add /' > add.sh
chmod +x ./add.sh
gedit add.sh
./add.sh
rm ./add.sh 
#svn remove ./tud.txt --force
# rm -f ./add.sh
#echo "Removing Deleted Files"
#svn export ./ ../paparazzi3/ --force
#cd ../paparazzi3/
#git status -s | grep 'TUDelft' | sed 's/??/rm -rf /' > rm.sh
#chmod +x ./rm.sh
#./rm.sh
#rm -rf ./import.sh
#rm -rf ./export.sh
#rm -rf ./sw/airborne/modules/onboardcam
#rm -rf ./sw/airborne/modules/opticflow
#rm -rf ./sw/airborne/boards/tiny_sense.h
#rm -rf ./sw/airborne/booz/aerovinci
#rm -rf ./conf/simulator/jsbsim/aircraft/LISA_BOOZ_BART.xml
#echo "Check this list manually: what files need to be added to ENAC server?" > ./rm.sh
#svn status | sed 's/?       /svn remove ..\/ppz2svn\//' >> rm.sh
#chmod +x ./rm.sh
#gedit ./rm.sh
#svn status | sed 's/?    /rm -rf /' > ./rm.sh
#cd ../ppz2svn
#svn diff > tud.txt
#gedit tud.txt
