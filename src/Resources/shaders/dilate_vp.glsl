#version 120

uniform float osgppu_ViewportHeight;
uniform float osgppu_ViewportWidth;

varying vec2 texcoord;
// Precomputed values
varying float invViewportHeight;
varying float invViewportWidth;

void main()
{
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    texcoord = gl_MultiTexCoord0.st;

    // Precompute values
    invViewportHeight = 1.0 / osgppu_ViewportHeight;
    invViewportWidth = 1.0 / osgppu_ViewportWidth;
}
