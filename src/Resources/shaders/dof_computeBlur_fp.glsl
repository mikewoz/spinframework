#version 120

uniform sampler2D texDepthMap;

// Focal length of the camear
uniform float focalLength;

// Focal range of the camera 
uniform float focalRange;

// zNear and zFar values 
uniform float zNear;
uniform float zFar;


void main(void)
{
    vec2 inTex = gl_TexCoord[0].st;

    float a = zFar / (zFar - zNear);
    float b = zFar*zNear / (zNear-zFar);

    float depth = texture2D(texDepthMap, inTex).x;
    float dist = b / (depth-a);

    float blur = clamp(abs(dist-focalLength) / focalRange, 0.0, 1.0);

    gl_FragColor.rgb = vec3(blur);
    gl_FragColor.a = 1.0;
}
