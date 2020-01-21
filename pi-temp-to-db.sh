
#!/bin/bash
# Script: pi-temp-to-db.sh
# Purpose: Posted the Raspberry Pi  CPU & GPU internal temperature to
# envirostation influxDB database. Data ROW = pitemperature, CPU GPU
# Author Declan Heard Janurary 2020
#-----------------------------------------------------
cpuTemp0=$(cat /sys/class/thermal/thermal_zone0/temp)
cpuTemp1=$(($cpuTemp0/1000))
cpuTemp2=$(($cpuTemp0/100))
cpuTempM=$(($cpuTemp2 % $cpuTemp1))
CPU=$cpuTemp1"."$cpuTempM
GPU=$(/opt/vc/bin/vcgencmd measure_temp | tr -cd '0-9.')

curl -i -XPOST 'http://localhost:8086/write?db=envirostation&u=USERNAME&p=PASSWORD' --data-binary 'pitemperature,location=southdarenth CPU='$CPU',GPU='$GPU''
