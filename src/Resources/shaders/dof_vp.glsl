/*
 * Prepares the data for the blurring due to depth of field
 * Author: Emmanuel Durand - 2012
*/

#version 120

// zNear and zFar values 
uniform float zNear, zFar;
// Focal range of the camera 
uniform float focalRange;
// Parameter for the blur
uniform float sigma;

uniform float osgppu_ViewportHeight;
uniform float osgppu_ViewportWidth;

const float PI = 3.1415926535897;

varying float invSigma2;
varying float c;
varying float a, b;
varying float invFocalRange;
varying float inputTexelHeight, inputTexelWidth;

void main(void)
{
    gl_TexCoord[0] = gl_MultiTexCoord0;
    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;
    gl_FrontColor = gl_Color;

    // Some precomputed values used in the fragment shaders, which do not
    // change
    float sigma2 = 2.0 * sigma * sigma;
    c = sqrt((1.0 / (sigma2*PI)));
    invSigma2 = 1.0/sigma2;

    a = zFar / (zFar - zNear);
    b = zFar*zNear / (zNear-zFar);
    invFocalRange = 1.0 / focalRange;

    inputTexelHeight = 1.0 / osgppu_ViewportHeight;
    inputTexelWidth = 1.0 / osgppu_ViewportWidth;
}
