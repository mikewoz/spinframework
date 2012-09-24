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
varying float invViewportHeight, invViewportWidth;

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

// Calculate the distance between two points (WOW!)
float getSquareDist(const vec3 p1, const vec3 p2)
{
    return pow(p1.x-p2.x, 2.0) + pow(p1.y-p2.y, 2.0) + pow(p1.z-p2.z, 2.0);
}

void main()
{
    if(uPass == 1)
    {
        float lMaskDepth = texture2D(uMaskDepthMap, texcoord.st).r;

        if(lMaskDepth == 1.0)
        {
            vec2 lObjFrag;
            float lObjDepth = 1.0;

            // In the neighbourhood, we look for the fragment which is the nearest to the camera
            for(float i=-uMaxLightSearch; i<uMaxLightSearch; i+=uLightSearchStep)
                for(float j=-uMaxLightSearch; j<uMaxLightSearch; j+=uLightSearchStep)
                {
                    vec2 lFrag = vec2(texcoord.s+i*invViewportWidth, texcoord.t+j*invViewportHeight);
                    float lDepth = texture2D(uMaskDepthMap, lFrag.st).r;
                    
                    if(lDepth < lObjDepth)
                    {
                        lObjFrag = lFrag;
                        lObjDepth = lDepth;
                    }
                }

            // We compute the distance between the object's fragment, and the current one
            vec3 lPos = getViewPos(texcoord.s, texcoord.t, lObjDepth);
            vec3 lObjPos = getViewPos(lObjFrag.s, lObjFrag.t, lObjDepth);
            float lDist = getSquareDist(lPos, lObjPos);
            
            float lLightFactor = smoothStep(pow(uLightingDistance, 2.0), 0.0, lDist);
            gl_FragData[0].rgb = vec3(lObjDepth, lLightFactor, 0.0);
            gl_FragData[0].a = 1.0;
        }
        else
        {
            gl_FragData[0].rgb = vec3(lMaskDepth, 1.0, 0.0);
            gl_FragData[0].a = 1.0;
        }
    }
    else if(uPass == 2)
    {
        vec4 lColor = texture2D(uColorMap, texcoord.st);
        float lDepth = texture2D(uDepthMap, texcoord.st).r;
        vec4 lMask = texture2D(uMaskMap, texcoord.st);
        vec2 lMaskDepth = texture2D(uMaskDepthMap, texcoord.st).rg;

        float lVisible = 1.0 - step(1.0, lMask.a);
        float lObject = 1.0 - step(1.0, lDepth);
        float lDist = (linearDepth(lDepth) - linearDepth(lMaskDepth.r))*(uFar-uNear);
        float lLightFactor = lMaskDepth.g;

        if(uLightingDistance > 0.0)
        {
            float lLighten = 1.0 - smoothstep(0.0, uLightingDistance, lDist);
            gl_FragData[0].rgb = lVisible*lObject*lLighten*mix(lColor.rgb, lMask.rgb, uTransparency)
                + lObject*lLighten*lLightFactor*lColor.rgb
                + (1.0-lLighten)*lMask.rgb;
        }
        else
        {
            gl_FragData[0].rgb = lVisible*lObject*mix(lColor.rgb, lMask.rgb, uTransparency)
                + (1.0-lObject)*lMask.rgb;
        }

        // This only applies if the objects are behind the masking object
        gl_FragData[0].rgb *= step(0.0, lDist);

        //gl_FragData[0].rgb = vec3(lMaskDepth.g);
    }
}