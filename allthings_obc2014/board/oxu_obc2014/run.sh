#!/bin/sh

cd ../../src/candy; ./candy 2&>1 | tee -a /root/candy.log &
