The street lighting system can be strated by executig the shell script "launch_street_lighting_system.sh". The first step is to update the shell script to the used setup. Change the names of the watchtowers in the array "watchtowers" at line five. To execute the shell script run the following comand in the Terminal:
'./launch_street_lighting_system.sh'

The nodes of the street lighting system can also be started individualy.

If you want to launch the callibration:

'dts duckiebot demo --duckiebot_name watchtowerXX --demo_name dt_calibration --package_name dt_calibration --image duckietown/dt_street_light_controller:v1-arm32v7'

If you want to launch the controller so you can launch this command:

'dts duckiebot demo --duckiebot_name watchtowerXX --demo_name dt_street_light_controller --package_name dt_street_light_controller --image duckietown/dt_street_light_controller:v1-arm32v7 --debug'

If you want to launch the LED control:

'dts duckiebot demo --duckiebot_name watchtowerXX --demo_name dt_led_control --package_name dt_led_control --image duckietown/dt_street_light_controller:v1-arm32v7'

