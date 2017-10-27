#!/bin/bash
if [ \( -L /dev/ES522_1 \) -o \( -L /dev/ES522_2 \) ];
then
   echo "[SCRIPT] Stopping existing serial client link"
   sudo cyclades-serial-client stop
fi
echo "[SCRIPT] Starting new serial client link"
sudo cyclades-serial-client start
sudo chmod ga+rw /dev/ES522_*
