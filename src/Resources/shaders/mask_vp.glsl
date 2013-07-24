/*
 * A simple passthrough shader
*/

#version 120

uniform float uNear, uFar;
uniform float osgppu_ViewportHeight;
uniform float osgppu_ViewportWidth;

varying vec2 texcoord;
// Precomputed values
varying float uA, uB;
varying float invViewportHeight, invViewportWidth;

void main()
{
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    texcoord = gl_MultiTexCoord0.st;

    // Precomputed values
    uA = uFar / (uFar - uNear);
    uB = uFar*uNear / (uNear-uFar);
    invViewportHeight = 1.0 / osgppu_ViewportWidth;
    invViewportWidth = 1.0 / osgppu_ViewportHeight;
}
