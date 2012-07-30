#version 120

uniform sampler2D vSSAO;
uniform sampler2D vColor;

varying vec2 texcoord;

void main(void)
{
    vec4 ssao = texture2D(vSSAO, texcoord);
    vec4 color = texture2D(vColor, texcoord);

    gl_FragColor.rgb = color.rgb * ssao.r;
    gl_FragColor.a = 1.0;
}
