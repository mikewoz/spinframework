/*
 * A simple passthrough shader for the motion blur effect
 * Emmanuel Durand - 2012
*/

#version 120

varying vec2 texcoord;

void main()
{
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    texcoord = gl_MultiTexCoord0.st;
}
