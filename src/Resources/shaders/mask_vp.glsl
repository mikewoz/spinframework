/*
 * A simple passthrough shader
*/

#version 120

uniform float uNear, uFar;

varying vec2 texcoord;
// Precomputed values
varying float uA, uB;

void main()
{
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    texcoord = gl_MultiTexCoord0.st;

    // Precomputed values
    uA = uFar / (uFar - uNear);
    uB = uFar*uNear / (uNear-uFar);
}
