/*
 * An outline effect, in two pass to combine with the objects color
 * Emmanuel Durand - 2012
*/

#version 120

uniform float uNear;
uniform float uFar;
uniform float osgppu_ViewportWidth;
uniform float osgppu_ViewportHeight;

varying vec2 texcoord;
// Precomputed values
varying float invViewportWidth, invViewportHeight;
varying float uA, uB;

void main()
{
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    texcoord = gl_MultiTexCoord0.st;

    // Precomputed values
    invViewportWidth = 1.0/osgppu_ViewportWidth;
    invViewportHeight = 1.0/osgppu_ViewportHeight;
    uA = uFar / (uFar - uNear);
    uB = uFar*uNear / (uNear-uFar);
}
