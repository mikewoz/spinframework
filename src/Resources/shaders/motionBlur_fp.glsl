/*
 * Motion blur, with the possibility to specify a blur value to each object
 * through vFactorMap. Otherwise it will use the vMotionBlurFactor
 * Author: Emmanuel Durand - 2012
*/

#version 120

uniform sampler2D vCurrentMap;
uniform sampler2D vPreviousMap;

uniform int vPass;
uniform float vMotionBlurFactor;

varying vec2 texcoord;

void main()
{
    // Bypass pass
    if (vPass == 1)
    {
        gl_FragData[0] = texture2D(vCurrentMap, texcoord.st);
    }
    // second pass: do the blur
    else if (vPass == 2)
    {
        vec4 prevColor = texture2D(vPreviousMap, texcoord.st);
        vec4 currentColor = texture2D(vCurrentMap, texcoord.st);

        gl_FragData[0] = mix(currentColor, prevColor, vMotionBlurFactor);
        //gl_FragData[0] = vec4(vec3(motionFactor.r), 1.0);
        //gl_FragData[0] = currentColor * (1 - motionFactor.r) + prevColor * motionFactor.r;
    }
}
