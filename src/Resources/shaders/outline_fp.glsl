/*
 * An outline effect, in two pass to combine with the objects color
 * Emmanuel Durand - 2012
*/

#version 120

#define SQRT2 1.414213562

uniform sampler2D uColorMap;
uniform sampler2D uDepthMap;
uniform sampler2D uOutlineMap;
uniform sampler2D uGlowMap;

uniform float uNear;
uniform float uFar;
uniform int uPass;
uniform float osgppu_ViewportWidth;
uniform float osgppu_ViewportHeight;
uniform vec4 uOutlineColor;
uniform float uGlowPower;

varying vec2 texcoord;

float linearDepth(const float d)
{
    float a = uFar / (uFar-uNear);
    float b = uFar*uNear / (uNear-uFar);
    return b / (d-a);
}

// Small function which returns the depth value of the
// fragment offset by (u,v)
float fetchDepth(const int u, const int v)
{
    float lXRatio = 1.0 / osgppu_ViewportWidth;
    float lYRatio = 1.0 / osgppu_ViewportHeight;

    return texture2D(uDepthMap, texcoord.st + vec2(u*lXRatio, v*lYRatio)).r;
}

mat3 fetchSquare()
{
    mat3 lD;
    lD[0][0] = fetchDepth(-1, -1);
    lD[1][0] = fetchDepth(0, -1);
    lD[2][0] = fetchDepth(1, -1);
    lD[0][1] = fetchDepth(-1, 0);
    lD[1][1] = fetchDepth(0, 0);
    lD[2][1] = fetchDepth(1, 0);
    lD[0][2] = fetchDepth(-1, 1);
    lD[1][2] = fetchDepth(0, 1);
    lD[2][2] = fetchDepth(1, 1);

    return lD;
}

mat3 fetchSquareLinear()
{
    mat3 lD;
    lD[0][0] = linearDepth(fetchDepth(-1, -1));
    lD[1][0] = linearDepth(fetchDepth(0, -1));
    lD[2][0] = linearDepth(fetchDepth(1, -1));
    lD[0][1] = linearDepth(fetchDepth(-1, 0));
    lD[1][1] = linearDepth(fetchDepth(0, 0));
    lD[2][1] = linearDepth(fetchDepth(1, 0));
    lD[0][2] = linearDepth(fetchDepth(-1, 1));
    lD[1][2] = linearDepth(fetchDepth(0, 1));
    lD[2][2] = linearDepth(fetchDepth(1, 1));

    return lD;
}

// Applies the Sobel operator
float sobel()
{
    mat3 lD = fetchSquare();

    float gx, gy;
    gx = -1*lD[0][0]-2*lD[1][0]-1*lD[2][0]
         +1*lD[0][2]+2*lD[1][2]+1*lD[2][2];
    gy = -1*lD[0][0]-2*lD[0][1]-1*lD[0][2]
         +1*lD[2][0]+2*lD[2][1]+1*lD[2][2];

    return sqrt(pow(gx,2)+pow(gy,2));
}

// Applies the Frei-chen operator
float freiChen()
{
    mat3 lD = fetchSquareLinear();

    float g1, g2, g3, g4, g5, g6, g7, g8, g9;
    float denom = 1/(2*SQRT2);

    g1 = 1*lD[0][0]+SQRT2*lD[1][0]+1*lD[2][0]
         -1*lD[0][2]-SQRT2*lD[1][2]-1*lD[2][2]
         *denom;
    g2 = 1*lD[0][0]+SQRT2*lD[0][1]+1*lD[0][2]
         -1*lD[2][0]-SQRT2*lD[2][1]-1*lD[2][2]
         *denom;
    g3 = 1*lD[0][1]+SQRT2*lD[0][2]+1*lD[1][2]
         -1*lD[1][0]-SQRT2*lD[2][0]-1*lD[2][1]
         *denom;
    g4 = -1*lD[1][0]-SQRT2*lD[0][0]-1*lD[0][1]
         +1*lD[1][2]+SQRT2*lD[2][2]+1*lD[2][1]
         *denom;
    g5 = 1*lD[1][0]+lD[0][1]-1*lD[2][1]+1*lD[1][2]
         /2;
    g6 = -1*lD[0][0]+lD[2][0]+1*lD[0][2]-1*lD[2][2]
         /2;
    g7 = 1*lD[0][0]-2*lD[1][0]+1*lD[2][0]
         -2*lD[0][1]+4*lD[1][1]-2*lD[2][1]
         +1*lD[0][2]-2*lD[1][2]+1*lD[2][2]
         /6;
    g8 = -2*lD[0][0]+1*lD[1][0]-2*lD[2][0]
         +1*lD[0][1]+4*lD[1][1]+1*lD[2][1]
         -2*lD[0][2]+1*lD[1][2]-2*lD[2][2]
         /6;
    g9 = lD[0][0]+lD[1][0]+lD[2][0]
         +lD[0][1]+lD[1][1]+lD[2][1]
         +lD[0][2]+lD[1][2]+lD[2][2]
         /3;

    float M = g1*g1 + g2*g2 + g3*g3 + g4*g4;
    float S = M + g5*g5 + g6*g6 + g7*g7 + g8*g8 + g9*g9;
    
    return sqrt(M/S);
}

void main()
{
    // first pass: detect the edges on the depth map
    if(uPass == 1)
    {
        gl_FragData[0].rgb = vec3(sobel());
        //gl_FragData[0].rgb = vec3(freiChen());
        gl_FragData[0].a = 1.0;
    }
    // second pass: composite the color and the outline
    else if(uPass == 2)
    {
        float lOutline = texture2D(uOutlineMap, texcoord.st).r;
        float lGlow = texture2D(uGlowMap, texcoord.st).r;
        vec4 lColor = texture2D(uColorMap, texcoord.st);

        lOutline = step(0.1, lOutline);
        gl_FragData[0] = (1.0-lOutline)*lColor;
        gl_FragData[0] += lGlow*uOutlineColor*uGlowPower*uOutlineColor.a;
        gl_FragData[0] += lOutline*uOutlineColor*uOutlineColor.a;
    }
}
