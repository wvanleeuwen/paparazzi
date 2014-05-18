
git submodule sync
git submodule update
rm -rf ./conf/settings/SkySeg.xml
rm -rf ./conf/flight_plans/SkySeg.xml
ln -s ../../sw/ext/ardrone2_vision/modules/ObstacleAvoidSkySegmentation/settings/SkySeg.xml ./conf/settings/SkySeg.xml
ln -s ../../sw/ext/ardrone2_vision/modules/ObstacleAvoidSkySegmentation/flightplan/SkySeg.xml ./conf/flight_plans/SkySeg.xml
make -C ./sw/ext/ardrone2_vision/modules/ObstacleAvoidSkySegmentation/visionresult/

