#!/bin/bash

echo "which WT do you want to flash?"
read watchtowernumber

#copy folder for flashing and second script
scp -r software\ -\ WT mom@watchtower$watchtowernumber:~
scp MCflashing_PI.sh mom@watchtower$watchtowernumber:~
scp LED_control.py mom@watchtower$watchtowernumber:~

#ssh into watchtower
ssh mom@watchtower$watchtowernumber chmod +x MCflashing_PI.sh
ssh mom@watchtower$watchtowernumber ./MCflashing_PI.sh





