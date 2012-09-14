#version 120

uniform sampler2D uColorMap;
uniform sampler2D uMaskMap;

varying vec2 texcoord;

void main()
{
    float mask = texture2D(uMaskMap, texcoord.st).r;
    vec4 col = texture2D(uColorMap, texcoord.st);
    //gl_FragData[0] = vec4( col.r, mask, 0.0, 1.0 );

    if(mask > 0.0)
        gl_FragData[0].rgb = vec3(step(0.001, col.r));
    else
        gl_FragData[0] = vec4(col.rgb/8.0, 1.0);

    /*if(texcoord.s < 0.5)
        gl_FragData[0] = vec4(mask, 0.0, 0.0, 1.0);
    else
        gl_FragData[0] = vec4(0.0, col.r, 0.0, 1.0);*/
}
