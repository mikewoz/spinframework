#version 120

varying vec2 texcoord;

void main(void)
{
    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;
    texcoord = gl_MultiTexCoord0.st;
}
