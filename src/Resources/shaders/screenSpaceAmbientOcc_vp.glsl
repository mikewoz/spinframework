/*
 * Screen space ambient occlusion shader - Vertex pass(-through)
 * Author: Emmanuel Durand - 2012
*/

#version 120

varying vec2 texcoord;

void main(void)
{
    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;
    texcoord = gl_MultiTexCoord0.st;
}
