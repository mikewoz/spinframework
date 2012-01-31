#!/bin/bash
SCENEID=aalex
PORT=54324
SENDPREFIX="osc-send osc.udp://127.0.0.1:${PORT} /SPIN/${SCENEID}"

${SENDPREFIX}$@
