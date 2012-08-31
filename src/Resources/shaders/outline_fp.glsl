/*
 * An outline effect, in two pass to combine with the objects color
 * Emmanuel Durand - 2012
*/

#version 120

uniform sampler2D uColorMap;
uniform sampler2D uDepthMap;
uniform sampler2D uOutlineMap;
uniform sampler2D uGlowMap;

uniform int uPass;
uniform float osgppu_ViewportWidth;
uniform float osgppu_ViewportHeight;
uniform vec4 uOutlineColor;

varying vec2 texcoord;

// Small function which returns the depth value of the
// fragment offset by (u,v)
float fetchDepth(const int u, const int v)
{
    float lXRatio = 1.0 / osgppu_ViewportWidth;
    float lYRatio = 1.0 / osgppu_ViewportHeight;

    return texture2D(uDepthMap, texcoord.st + vec2(u*lXRatio, v*lYRatio)).r;
}

// Applies the Sobel operator along the X axis
float sobel()
{
    mat3 lD;
    lD[0][0] = fetchDepth(-1, -1);
    lD[1][0] = fetchDepth(0, -1);
    lD[2][0] = fetchDepth(1, -1);
    lD[0][1] = fetchDepth(-1, 0);
    lD[2][1] = fetchDepth(1, 0);
    lD[0][2] = fetchDepth(-1, 1);
    lD[1][2] = fetchDepth(0, 1);
    lD[2][2] = fetchDepth(1, 1);

    float gx, gy;
    gx = -1*lD[0][0]-2*lD[1][0]-1*lD[2][0]
         +1*lD[0][2]+2*lD[1][2]+1*lD[2][2];
    gy = -1*lD[0][0]-2*lD[0][1]-1*lD[0][2]
         +1*lD[2][0]+2*lD[2][1]+1*lD[2][2];

    return sqrt(pow(gx,2)+pow(gy,2));
}

void main()
{
    // first pass: detect the edges on the depth map
    if(uPass == 1)
    {
        gl_FragData[0].rgb = vec3(sobel());
        gl_FragData[0].a = 1.0;
    }
    // second pass: composite the color and the outline
    else if(uPass == 2)
    {
        float lOutline = texture2D(uOutlineMap, texcoord.st).r;
        float lGlow = texture2D(uGlowMap, texcoord.st).r;
        vec4 lColor = texture2D(uColorMap, texcoord.st);

        lOutline = step(0.1, lOutline);
        gl_FragData[0] = (1.0-lOutline)*lColor + lOutline*uOutlineColor;
        gl_FragData[0].rgb += vec3(lGlow);
        //gl_FragData[0].rgb = vec3(lOutline);
    }
}
