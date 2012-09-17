#version 120

uniform sampler2D uColorMap;
uniform sampler2D uDepthMap;
uniform sampler2D uMaskMap;
uniform sampler2D uMaskDepthMap;

uniform float uTransparency;

varying vec2 texcoord;

void main()
{
    const float epsilon = 0.0001;

    vec4 lColor = texture2D(uColorMap, texcoord.st);
    float lDepth = texture2D(uDepthMap, texcoord.st).r;
    vec4 lMask = texture2D(uMaskMap, texcoord.st);
    float lMaskDepth = texture2D(uMaskDepthMap, texcoord.st).r;

    float lVisible = 1.0 - step(1.0, lMask.a);
    float lObject = 1.0 - step(1.0, lDepth);
    gl_FragData[0].rgb = lVisible*lObject*mix(lColor.rgb, lMask.rgb, uTransparency)
        + (1.0-lObject)*lMask.rgb;

}
