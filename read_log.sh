#!/bin/bash
touch .cpu_load.log
diff cpu_load.log .cpu_load.log
cp cpu_load.log .cpu_load.log

#  while sleep 1; do tput clear; less +F test.txt; done
