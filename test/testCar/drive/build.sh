#!/bin/bash

sudo docker build -t robocar-drive:latest .

echo "Build terminé."

while true; do
  tput bel
  read -n1 -s -t 0.5 key
  if [[ "$key" == "k" ]]; then
    break
  fi
done

echo "Bip arrêté."
