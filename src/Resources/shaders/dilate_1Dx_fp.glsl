/*
 * Dilate along the x axis
 * Very simple squared dilatation
*/

uniform sampler2D texUnit0;

uniform float radius;
uniform float osgppu_ViewportWidth;
uniform float osgppu_ViewportHeight;

varying vec2 texcoord;

void main()
{
    vec4 lColor = vec4(0.0);
	float inputTexTexelWidth = 1.0 / osgppu_ViewportWidth;

    for(float i=-radius; i<radius; i+=1.0)
    {
        vec4 lTmp = texture2D(texUnit0, texcoord.st + vec2(i*inputTexTexelWidth, 0));
        lColor = max(lColor, lTmp);
    }

    gl_FragColor = lColor;
    gl_FragColor.a = 1.0;
}
