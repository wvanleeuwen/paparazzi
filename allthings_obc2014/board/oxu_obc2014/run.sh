#!/bin/sh

screen -mS candy bash -c "cd /root/develop/allthings_obc2014/src/candy; ./candy 2>&1 | tee -a /root/candy.log"
