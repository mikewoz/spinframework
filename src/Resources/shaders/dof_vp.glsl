#version 120

uniform float sigma;

const float PI = 3.1415926535897;

varying float sigma2;
varying float c;

void main(void)
{
    gl_TexCoord[0] = gl_MultiTexCoord0;

    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;

    gl_FrontColor = gl_Color;

    sigma2 = 2.0 * sigma * sigma;
    c = sqrt((1.0 / (sigma2*PI)));
}
