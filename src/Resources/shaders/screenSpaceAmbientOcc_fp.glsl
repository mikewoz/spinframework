#version 120

//uniform sampler2D vColor1;
uniform sampler2D vColor2;
uniform sampler2D vColor3;
uniform sampler2D vNoiseMap;
uniform sampler2D vDepth;

uniform float vPixelSize;
uniform float vNear;
uniform float vFar;
uniform mat4 vProjectionMatrix;
uniform mat4 vInvProjectionMatrix;

uniform mat4 osg_ProjectionMatrixInverse;

varying vec2 texcoord;

/**********************************/
float linearizeDepth(in float pDepth)
{
    return (pDepth-vNear)/(vFar-vNear);
}

/**********************************/
vec3 GetViewPos (in vec2 texCoord)
{
    float depth = texture2D(vColor3, texCoord.st).x;
    vec4 pos = vec4(texCoord.x, texCoord.y, depth, 1.0);
    pos.xyz = pos.xyz * 2.0 - 1.0;
    pos = osg_ProjectionMatrixInverse * pos;
    return pos.xyz / pos.w;
}

/**********************************/
void main(void)
{
    // Settings
    const float ssaoFocus = 0.5;
    const float ssaoPower = 2.0;
    const int ssaoLoops = 32;

    // Depth from the real depth buffer
    float lDepth = texture2D(vDepth, texcoord).r;

    // View space normal
    vec3 lNormal = normalize(texture2D(vColor2, texcoord.st).xyz * 2.0 - 1.0);

    // View space position of the pixel
    vec3 lPos = texture2D(vColor3, texcoord.st).xyz; //GetViewPos(texcoord.st);

    float lOcclusion, lWeight;

    if(lDepth == 1.0)
        lOcclusion = 1.0;
    else
    {
        lPos.z = 1.f/lPos.z; 

        // Random value from the noise map
        vec2 modifier = texture2D(vNoiseMap, texcoord.st).xy + (lPos.xy + lPos.z);
        
        float dist, visibility = 0.f;
        vec4 random, screenPos = vec4(1.f);

        for(int i=0; i<ssaoLoops; i++)
        {
            // Retrieve a new random vector from the texture
            random = texture2D(vNoiseMap, modifier);
            random.xyz = normalize(random.xyz * 2.0 - 1.0);

            // Randomize the modifier for the next loop
            modifier += random.xy;

            // Flip the random vector if it's below the plane
            if(dot(random.xyz, lNormal) < 0.0)
                random.xyz = -random.xyz;

            // Calculate the randomly offset position's screen space coordinates -- second most expensive operation
            screenPos = vProjectionMatrix * vec4(lPos, 1.0); 
            // Offset the screen position
            screenPos.xyz = screenPos.xyz + random.xyz*ssaoFocus;
            // Store the depth of the new position
            float lScreenDepth = (vInvProjectionMatrix*screenPos).z;
            screenPos.xyz /= -screenPos.w;

            // Get the depth at the screen space coordinates -- this is the most expensive operation
            /*vec4 lFragPos = texture2D(vColor3, screenPos.xy * 0.5 + 0.5);
            lFragPos.z = 1.0/lFragPos.z;
            lFragPos = vProjectionMatrix * lFragPos;
            lFragPos.z /= -lFragPos.w;*/

            dist = texture2D(vColor3, screenPos.xy * 0.5 + 0.5).z;

            /*vec3 lFragNorm = texture2D(vColor2, screenPos.xy * 0.5 + 0.5).xyz * 2.0 - 1.0;
            lWeight = 1.0-dot(lFragNorm,lNormal);*/


            dist = 1.f/dist;

            float lDiff = linearizeDepth(lScreenDepth) - linearizeDepth(dist);
            // If the ray hits behind the fragment (from the camera position)

            if(lDiff > 0 && lDiff < ssaoFocus/(vFar-vNear))
            {
                visibility += lDiff/(ssaoFocus/(vFar-vNear));
            }
            // else, if the ray hits in front of the fragment
            else
            {
                visibility += 1.0;
            }
        }

        // Final occlusion factor
        //visibility = max(0.0, min(1.0, (visibility/float(ssaoLoops) - ssaoRescale)*1.5+ssaoRescale));
        lOcclusion = 1.0 - pow(1.0 - visibility/float(ssaoLoops), ssaoPower);
    }

    //gl_FragColor.rgb = texture2D(vColor1, texcoord.st).rgb;
    //gl_FragColor.rgb = texture2D(vColor2, texcoord.st).rgb*2.0-1.0;
    //gl_FragColor.rgb = vec3(texture2D(vColor3, texcoord.st).rgb);
    //gl_FragColor.rgb = texture2D(vNoiseMap, texcoord*2).rgb;

    gl_FragColor.rgb = vec3(lOcclusion);

    gl_FragColor.a = 1.f;
}
