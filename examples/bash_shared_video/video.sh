#!/bin/bash
set -o verbose
SCENEID=default
PORT=54324
SENDPREFIX="osc-send osc.udp://127.0.0.1:${PORT} /SPIN/${SCENEID}"
 ,s debug
${SENDPREFIX} ,sss createNode grid GridNode
${SENDPREFIX} ,sss createNode shape0 ShapeNode
${SENDPREFIX}/shape0 ,si setShape 6 # plane
${SENDPREFIX} ,sss createStateSet cam VideoTexture
${SENDPREFIX}/cam ,ss setPath /dev/video1
${SENDPREFIX}/shape0 ,ss setStateSet cam
${SENDPREFIX}/shape0 ,sfff setScale 1.33333 0 1
${SENDPREFIX}/shape0 ,sfff setTranslation 0 0 0




