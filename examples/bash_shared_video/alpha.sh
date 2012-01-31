#!/bin/bash
# set -o verbose
SCENEID=aalex
PORT=54324
SENDPREFIX="osc-send osc.udp://127.0.0.1:${PORT} /SPIN/${SCENEID}"

${SENDPREFIX}/shape0 ,sffff setColor 1.0 1.0 1.0 $1
echo ${SENDPREFIX}/shape0 ,sffff setColor 1.0 1.0 1.0 $1
${SENDPREFIX} ,s debug
