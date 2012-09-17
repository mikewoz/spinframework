#version 120

uniform sampler2D uColorMap;
uniform sampler2D uDepthMap;
uniform sampler2D uMaskMap;
uniform sampler2D uMaskDepthMap;

uniform float uTransparency;
uniform float uLightingDistance;
uniform float uNear, uFar;

varying vec2 texcoord;

float smoothStep(float edge0, float edge1, float x)
{
    float t;
    t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

float linearDepth(const float d)
{
    float a = uFar / (uFar-uNear);
    float b = uFar*uNear / (uNear-uFar);
    return b / ((d-a)*uFar);
}

void main()
{
    const float epsilon = 0.0001;

    vec4 lColor = texture2D(uColorMap, texcoord.st);
    float lDepth = texture2D(uDepthMap, texcoord.st).r;
    vec4 lMask = texture2D(uMaskMap, texcoord.st);
    float lMaskDepth = texture2D(uMaskDepthMap, texcoord.st).r;

    float lVisible = 1.0 - step(1.0, lMask.a);
    float lObject = 1.0 - step(1.0, lDepth);
    float lDist = (linearDepth(lDepth) - linearDepth(lMaskDepth))*(uFar-uNear);

    if(lDist >= 0.0)
    {
        if(uLightingDistance > 0.0)
        {
            float lLighten = 1.0 - smoothstep(0.0, uLightingDistance, lDist);
            gl_FragData[0].rgb = lVisible*lObject*lLighten*mix(lColor.rgb, lMask.rgb, uTransparency)
                + (1.0-lLighten)*lMask.rgb;
        }
        else
        {
            gl_FragData[0].rgb = lVisible*lObject*mix(lColor.rgb, lMask.rgb, uTransparency)
                + (1.0-lObject)*lMask.rgb;
        }
    }
}
