#!/bin/bash
set -o verbose
SCENEID=aalex
PORT=54324
SENDPREFIX="osc-send osc.udp://127.0.0.1:${PORT} /SPIN/${SCENEID}"
 ,s debug

${SENDPREFIX} ,sss createNode shape0 ShapeNode
${SENDPREFIX}/shape0 ,si setShape 6 # plane
${SENDPREFIX} ,sss createStateSet state0 SharedVideoTexture
${SENDPREFIX}/state0 ,ss setTextureID scenic_tex0 # /dev/shm/scenic_tex0 must exist
${SENDPREFIX}/shape0 ,ss setStateSet state0
${SENDPREFIX}/shape0 ,sfff setScale 1.33333 0 1
${SENDPREFIX}/shape0 ,sfff setTranslation 0.6666 0 0.6666



${SENDPREFIX} ,sss createNode shape1 ShapeNode
${SENDPREFIX}/shape1 ,si setShape 6 # plane
${SENDPREFIX} ,sss createStateSet state1 SharedVideoTexture
${SENDPREFIX}/state1 ,ss setTextureID scenic_tex1 # /dev/shm/scenic_tex1 must exist
${SENDPREFIX}/shape1 ,ss setStateSet state1
${SENDPREFIX}/shape1 ,sfff setScale 1.33333 0 1
${SENDPREFIX}/shape1 ,sfff setTranslation -0.6666 0 0

i=0
while true
do
    sleep 5
    # Set the render bin for this texture. The higher the number, the later it gets processed (ie, it appears on top). Default renderBin = 11
    ${SENDPREFIX}/state1 ,si setRenderBin `python -c "print((${i} % 2) * 2 + 10)"`
    # `expr ${i} % 2 + 10`
    i=`expr ${i} + 1`
    ${SENDPREFIX} ,s debug
done



