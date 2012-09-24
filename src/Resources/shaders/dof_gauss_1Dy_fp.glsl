/*
 * Applies the blur value contained in the texBlurMap to the
 * color map, along y axis
 * Author: Emmanuel Durand - 2012
*/

#version 120

uniform sampler2D texBlurMap;
uniform sampler2D texColorMap;

uniform float osgppu_ViewportWidth;
uniform float osgppu_ViewportHeight;

uniform float radius;

varying float invSigma2;
varying float c;
varying float inputTexelHeight;

const float epsilon = 0.0001;

void main(void)
{
    vec4 lColor = vec4(0.0);
    float lTotalWeight = 0.0;

    // Get the blur value at the pixel level
    float lBlur = texture2D(texBlurMap, gl_TexCoord[0].st).r;
    // The real radius is dependant of this value
    float lRadius = lBlur * radius;

    if(lRadius < 0.5)
        lColor = texture2D(texColorMap, gl_TexCoord[0].st);
    else
    {
        for(float i=-lRadius; i<lRadius; i+=1.0)
        {
            float lWeight = c*exp(-(i*i)*invSigma2);
            vec2 lPixCoords = gl_TexCoord[0].st + vec2(0, i*inputTexelHeight);
            
            // Radius of the blur created by the current offset pixel
            float lPixBlurRadius = radius * texture2D(texBlurMap, lPixCoords).r;

            float lIsInBlur = step(abs(i), lPixBlurRadius);
            lTotalWeight += lWeight * lIsInBlur;
            lColor += lIsInBlur * texture2D(texColorMap, lPixCoords) * lWeight;
        }
        lColor /= lTotalWeight;
    }

    gl_FragColor = lColor;
}
