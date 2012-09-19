/*
 * Shader de mask, à utiliser avec le PPU mask. Permet de spécifier la distance
 * d'éclairage des objets masqués par les objets masquants
 * Auteur : Emmanuel Durand - 2012
 */

#version 120

uniform sampler2D uColorMap;
uniform sampler2D uDepthMap;
uniform sampler2D uMaskMap;
uniform sampler2D uMaskDepthMap;

uniform int uPass;
uniform float uTransparency;
uniform float uLightingDistance;
uniform float uMaxLightSearch, uLightSearchStep;
uniform float uNear, uFar, uLeft, uRight, uTop, uBottom;

uniform float osgppu_ViewportWidth, osgppu_ViewportHeight;

varying vec2 texcoord;
// Precomputed values
varying float uA, uB;

const float epsilon = 0.0001;

// This function is not available in GLSL 1.2 ...
float smoothStep(float edge0, float edge1, float x)
{
    float t;
    t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// Convert depth value to linear depth
float linearDepth(const float d)
{
    //float a = uFar / (uFar-uNear);
    //float b = uFar*uNear / (uNear-uFar);
    return uB / ((d-uA)*uFar);
}

// Calculate the position of the fragment in view coordinates
vec3 getViewPos(const float s, const float t, const float depth)
{
    float z = linearDepth(depth)*(uFar-uNear);
        
    float x = (1.0-step(0.5, s))*abs(s-0.5)*uLeft + step(0.5, s)*abs(s-0.5)*uRight;
    x = x*z;

    float y = (1.0-step(0.5, t))*abs(t-0.5)*uBottom + step(0.5, t)*abs(t-0.5)*uTop;
    y = y*z;

    return vec3(x,y,z);
}

void main()
{
    if(uPass == 1)
    {
        float lMaskDepth = texture2D(uMaskDepthMap, texcoord.st).r;

        // Calculate the position of the fragment in view coordinates
        float z = linearDepth(lMaskDepth)*(uFar-uNear);
        
        float x = (1.0-step(0.5, texcoord.s))*abs(texcoord.s-0.5)*uLeft
            + step(0.5, texcoord.s)*abs(texcoord.s-0.5)*uRight;
        x = x*z;

        float y = (1.0-step(0.5, texcoord.t))*abs(texcoord.t-0.5)*uBottom
            + step(0.5, texcoord.t)*abs(texcoord.t-0.5)*uTop;
        y = y*z;
    }
    else if(uPass == 2)
    {
        vec4 lColor = texture2D(uColorMap, texcoord.st);
        float lDepth = texture2D(uDepthMap, texcoord.st).r;
        vec4 lMask = texture2D(uMaskMap, texcoord.st);
        float lMaskDepth = texture2D(uMaskDepthMap, texcoord.st).r;

        float lVisible = 1.0 - step(1.0, lMask.a);
        float lObject = 1.0 - step(1.0, lDepth);
        float lDist = (linearDepth(lDepth) - linearDepth(lMaskDepth))*(uFar-uNear);

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

        // This only applies if the objects are behind the masking object
        gl_FragData[0].rgb *= step(0.0, lDist);

        /*gl_FragData[0].rgb = vec3(linearDepth(lMaskDepth));
        gl_FragData[0].a = 1.0;*/
    }
}
