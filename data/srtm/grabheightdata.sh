#!/bin/bash

# Use freely, OpenUAS 2010
# Script to grab ranges of height data, very handy if you test your UAS in various areas in a continent ;)
# Change, add or delete values in 49 50 51 and 003 004 005 006 007 008 009 to get the ranges you need.

#outer=1
#for lat in 48 49 50 51 52
#do
#  inner=1           # Reset inner loop counter.
#  for long in 001 002 003 004 005 006 007 008 009 010 011 012 013 014
#  do
#    #echo "Pass $inner in inner loop."
#    let "inner+=1"  # Increment inner loop counter.
#     #For testing comment the wget line and use he echo line
#     #echo "http://dds.cr.usgs.gov/srtm/version2_1/SRTM3/Eurasia/N"$lat"E"$long".hgt.zip"
#     wget "http://dds.cr.usgs.gov/srtm/version2_1/SRTM3/Eurasia/N"$lat"E"$long".hgt.zip"
#  done
#  let "outer+=1"    # Increment outer loop counter. 

i=1
for lat in 48 49 50 51 52
do
  j=1
  for long in 001 002 003 004 005 006 007 008 009 010 011 012 013 014
  do
    let "j+=1"
     wget -S -N "http://dds.cr.usgs.gov/srtm/version2_1/SRTM3/Eurasia/N"$lat"E"$long".hgt.zip"
     sleep 2
  done
  let "i+=1" 
done               

exit 0
