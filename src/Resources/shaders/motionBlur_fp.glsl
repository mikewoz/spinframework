/*
 * Motion blur, with the possibility to specify a blur value to each object
 * through vFactorMap. Otherwise it will use the vMotionBlurFactor
 * Author: Emmanuel Durand - 2012
*/

#version 120

uniform sampler2D vCurrentMap;
uniform sampler2D vPreviousMap;
uniform sampler2D vFactorMap;

uniform int vPass;
uniform float vMotionBlurFactor;

varying vec2 texcoord;

void main()
{
    // first pass: compute the current blur
    if(vPass == 1)
    {
        // currentBlur = same as motionFactor in pass 1
        vec4 currentBlur = texture2D(vFactorMap, texcoord.st);
        // prevBlur: still the same ...
        vec4 prevBlur = texture2D(vPreviousMap, texcoord.st);

        float lCurrentFactor;
        if(currentBlur.g == 0)
            lCurrentFactor = vMotionBlurFactor;
        else
            lCurrentFactor = currentBlur.r;

        float lFactor = mix(lCurrentFactor, prevBlur.r, 0.5);
        float lMask = step(0.033, lFactor);
        lMask = max(lMask, currentBlur.g);

        gl_FragData[0] = vec4(lFactor, lMask, 0.0, 1.0);
    }
    // second pass: effectively do the blur
    else if(vPass == 2)
    {
        vec4 prevColor = texture2D(vPreviousMap, texcoord.st);
        vec4 currentColor = texture2D(vCurrentMap, texcoord.st);
        // The factor map contains the factor in r,
        // and usage of the non global value or not in g
        vec2 motionFactor = texture2D(vFactorMap, texcoord.st).rg;

        gl_FragData[0] = mix(currentColor, prevColor, motionFactor.r);
        //gl_FragData[0] = vec4(vec3(motionFactor.r), 1.0);
    }
}
