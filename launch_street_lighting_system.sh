#!/bin/bash

#write here all watchtower that you are using
#possible update: control if all WT are on and else say which WT wasn't reacting
watchtowers=(watchtower01 watchtower02 watchtower03 watchtower04)

for index in ${!watchtowers[*]}
do
    docker -H ${watchtowers[$index]}.local stop demo_dt_street_light_controller &
done
wait

#Decission if we build all the container again
echo "Do you want to build all controllers? y or n"
read built

#built the container
if  [ "$built" = "y" ]
then
    for index in ${!watchtowers[*]}
    do
        printf "\nSetting up %s\n" ${watchtowers[$index]}
        dts devel watchtower stop -H ${watchtowers[$index]}.local &
    done
    wait
    
    
    for index in ${!watchtowers[*]}
    do
        dts devel build -f --arch arm32v7 -H ${watchtowers[$index]}.local &
    done
    wait
fi

#launch the led control
for index in ${!watchtowers[*]}
do
    printf "\nSetting up %s\n" ${watchtowers[$index]}
    dts duckiebot demo --duckiebot_name ${watchtowers[$index]} --demo_name dt_led_control --package_name dt_led_control --image duckietown/dt_street_light_controller:v1-arm32v7 &
done
wait

#launch the calibration
for index in ${!watchtowers[*]}
do
    printf "\nSetting up %s\n" ${watchtowers[$index]}
    dts duckiebot demo --duckiebot_name ${watchtowers[$index]} --demo_name dt_calibration --package_name dt_calibration --image duckietown/dt_street_light_controller:v1-arm32v7 &
done
wait

sleep 20

#lauch all controller
for index in ${!watchtowers[*]}
do
    printf "\nSetting up %s\n" ${watchtowers[$index]}
    dts duckiebot demo --duckiebot_name ${watchtowers[$index]} --demo_name dt_street_light_controller --package_name dt_street_light_controller --image duckietown/dt_street_light_controller:v1-arm32v7 &
done
wait

echo "The street lighting system is ready"

sleep 4253513513513513513513513513513513513  
