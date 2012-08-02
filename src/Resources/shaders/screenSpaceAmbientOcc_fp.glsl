#version 120

uniform sampler2D vNormal;
uniform sampler2D vPosition;
uniform sampler2D vNoiseMap;
uniform sampler2D vDepth;

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
    float depth = texture2D(vPosition, texCoord.st).x;
    vec4 pos = vec4(texCoord.x, texCoord.y, depth, 1.0);
    pos.xyz = pos.xyz * 2.0 - 1.0;
    pos = osg_ProjectionMatrixInverse * pos;
    return pos.xyz / pos.w;
}

/**********************************/
float smoothStep(in float edge0, in float edge1, in float x)
{
    float t = clamp((x-edge0)/(edge1-edge0), 0.0, 1.0);
    return t*t*(3.0-2.0*t);
}

/**********************************/
void main(void)
{
    // Settings
    const float ssaoFocus = 6.0;
    const float ssaoPower = 3.0;
    const int ssaoLoops = 32;
    const float invSamples = 1.0/float(ssaoLoops);

    vec3 lBufferPos, lBufferNorm;

    // View space normal
    lBufferNorm = texture2D(vNormal, texcoord.st).xyz;
    vec3 lNormal = normalize(lBufferNorm * 2.0 - 1.0);

    // View space position of the pixel
    vec3 lPos = texture2D(vPosition, texcoord.st).xyz; //GetViewPos(texcoord.st);

    // These values shouldn't be equal. If they are, it means
    // that the vNormal and vPosition textures were not rendered for
    // this pixel
    if(lBufferNorm == lPos)
        gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0);
    else
    {
        float lOcclusion, lWeight;

        lPos.z = 1.f/lPos.z;

        // Random value from the noise map
        vec2 modifier = texture2D(vNoiseMap, texcoord.st).xy + (lPos.xy + lPos.z);
        
        float dist, visibility = 0.f;
        vec4 random, screenPos = vec4(1.f);

        for(int i=0; i<ssaoLoops; i++)
        {
            // Retrieve a new random vector from the texture
            random = texture2D(vNoiseMap, modifier);
            random.xyz = random.xyz * 2.0 - 1.0;

            // Randomize the modifier for the next loop
            modifier += random.xy;

            // Flip the random vector if it's below the plane
            if(dot(random.xyz, lNormal) < 0.0)
                random.xyz = -random.xyz;

            // Calculate the randomly offset position's screen space coordinates -- second most expensive operation
            screenPos = vProjectionMatrix * vec4(lPos, 1.0); 
            // Offset the screen position
            screenPos.xyz = screenPos.xyz + random.xyz*ssaoFocus*(noise1(lPos.z)*0.5+0.5);
            // Store the depth of the new position
            float lScreenDepth = (vInvProjectionMatrix*screenPos).z;
            screenPos.xyz /= -screenPos.w;

            // Get the depth at the screen space coordinates -- this is the most expensive operation
            /*vec4 lFragPos = texture2D(vPosition, screenPos.xy * 0.5 + 0.5);
            lFragPos.z = 1.0/lFragPos.z;
            lFragPos = vProjectionMatrix * lFragPos;
            lFragPos.z /= -lFragPos.w;*/
            screenPos.xy = screenPos.xy*0.5+0.5;

            lBufferPos = texture2D(vPosition, screenPos.xy).xyz;
            dist = lBufferPos.z;
            dist = 1.0/dist;

            lBufferNorm = texture2D(vNormal, screenPos.xy).xyz;
            vec3 lFragNorm = normalize(lBufferNorm * 2.0 - 1.0);

            // Same as before, we detect if the shader should use the data from
            // this pixel
            if(lBufferPos == lBufferNorm)
                visibility += 1.0;
            else
            {

                lWeight = smoothStep(-0.5, 1.0, dot(lFragNorm,lNormal));     

                float lDiff = linearizeDepth(lScreenDepth) - linearizeDepth(dist);
                // If the ray hits behind the fragment (from the camera position)

                //if(abs(linearizeDepth(lPos.z) - linearizeDepth(dist)) < ssaoFocus/(vFar-vNear))
                //    visibility += (linearizeDepth(dist) >= linearizeDepth(lScreenDepth)) ? 1.0 : 0.0;

                visibility += min(abs(lDiff/(ssaoFocus/(vFar-vNear))) + lWeight, 1.0);
                
                /*if(lDiff > 0 && lDiff < 2.0*ssaoFocus/(vFar-vNear))
                {
                    visibility += min(lDiff/(ssaoFocus/(vFar-vNear)) + lWeight, 1.0);
                }
                // else, if the ray hits in front of the fragment
                else
                {
                    visibility += 1.0;
                }*/
            }
        }

        // Final occlusion factor
        //visibility = max(0.0, min(1.0, (visibility/float(ssaoLoops) - ssaoRescale)*1.5+ssaoRescale));
        lOcclusion = pow(visibility*invSamples, ssaoPower);

        //gl_FragColor.rgb = texture2D(vColor1, texcoord.st).rgb;
        //gl_FragColor.rgb = texture2D(vNormal, texcoord.st).rgb;
        //gl_FragColor.rgb = vec3(texture2D(vPosition, texcoord.st).rgb);
        //gl_FragColor.rgb = texture2D(vNoiseMap, texcoord*2).rgb;

        gl_FragColor.rgb = vec3(lOcclusion);

        gl_FragColor.a = 1.f;
    }
}
