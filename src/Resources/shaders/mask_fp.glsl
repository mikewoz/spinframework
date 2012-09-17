#version 120

uniform sampler2D uColorMap;
uniform sampler2D uDepthMap;
uniform sampler2D uMaskMap;
uniform sampler2D uMaskDepthMap;

varying vec2 texcoord;

void main()
{
    vec4 lColor = texture2D(uColorMap, texcoord.st);
    float lDepth = texture2D(uDepthMap, texcoord.st).r;
    vec4 lMask = texture2D(uMaskMap, texcoord.st);
    float lMaskDepth = texture2D(uMaskDepthMap, texcoord.st).r;

    gl_FragData[0] = lColor*lMask.r;

    /*if(texcoord.s < 0.5)
        gl_FragData[0] = vec4(lMask, 0.0, 0.0, 1.0);
    else
        gl_FragData[0] = vec4(0.0, lColor.r, 0.0, 1.0);*/

    //gl_FragData[0] = lColor;

    /*if(texcoord.t < 0.5)
        if(texcoord.s < 0.5)
            gl_FragData[0] = vec4(vec3(lDepth), 1.0);
        else
            gl_FragData[0] = lColor;
    else
        if(texcoord.s < 0.5)
            gl_FragData[0] = vec4(vec3(lMaskDepth), 1.0);
        else
            gl_FragData[0] = lMask;*/
}
