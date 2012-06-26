git status -s | grep '??' | sed s"/?? /rm -rf /" > ./rm.sh
chmod +x ./rm.sh
./rm.sh
