#version 120

uniform sampler2D vCurrentMap;
uniform sampler2D vPreviousMap;

uniform float vMotionBlurFactor;

varying vec2 texcoord;

void main()
{
    vec4 prevColor = texture2D(vPreviousMap, texcoord.st);
    vec4 currentColor = texture2D(vCurrentMap, texcoord.st);

    gl_FragColor = mix(currentColor, prevColor, vMotionBlurFactor);
    //gl_FragColor = abs(prevColor - currentColor);
    //gl_FragColor.r = vMotionBlurFactor;
}
