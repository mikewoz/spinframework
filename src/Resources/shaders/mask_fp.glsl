/*
 * An outline effect, in two pass to combine with the objects color
 * Emmanuel Durand - 2012
*/

#version 120

uniform sampler2D uColorMap;
uniform sampler2D uMaskMap;

varying vec2 texcoord;

void main()
{
    float mask = texture2D(uMaskMap, texcoord.st).r;
    vec4 col = texture2D(uColorMap, texcoord.st);
    gl_FragData[0] = vec4( col.r, mask, 0, 1 );
    
}