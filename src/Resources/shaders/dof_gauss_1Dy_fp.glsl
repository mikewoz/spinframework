#version 120

uniform sampler2D texBlurMap;
uniform sampler2D texColorMap;

uniform float osgppu_ViewportWidth;
uniform float osgppu_ViewportHeight;

uniform float radius;

varying float sigma2;
varying float c;

void main(void)
{
    vec4 lColor = vec4(0.0);
    float lTotalWeight = 0.0;
    float lInputTexTexelWidth = 1.0 / osgppu_ViewportHeight;

    // Get the blur value at the pixel level
    float lBlur = texture2D(texBlurMap, gl_TexCoord[0].st).r;
    // The real radius is dependant of this value
    float lRadius = lBlur * radius;

    for(float i=-lRadius; i<lRadius; i+=1.0)
    {
        float lWeight = c*exp(-(i*i)/sigma2);
        
        // Radius of the blur created by the current offset pixel
        //float lPixBlurRadius = radius * texture2D(texBlurMap, gl_TexCoord[0].st + vec2(0, i*lInputTexTexelWidth)).r;

        //float lIsInBlur = 1.0; //step(lPixBlurRadius, lRadius);
        lTotalWeight += lWeight;
        lColor += texture2D(texColorMap, gl_TexCoord[0].st + vec2(0, i*lInputTexTexelWidth)) * lWeight;
    }
    lColor /= lTotalWeight;

    gl_FragColor = lColor;
}
