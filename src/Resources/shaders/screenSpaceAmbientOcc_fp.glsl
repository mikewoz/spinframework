#version 120

uniform sampler2D vNormal;
uniform sampler2D vPosition;
uniform sampler2D vNoiseMap;
uniform sampler2D vDepth;

uniform float vNear;
uniform float vFar;
uniform mat4 vProjectionMatrix;
uniform mat4 vInvProjectionMatrix;

uniform float vSsaoPower;
uniform float vSsaoFocus;
uniform int vSsaoSamples;

varying vec2 texcoord;

/**********************************/
float linearizeDepth(in float pDepth)
{
    return (pDepth-vNear)/(vFar-vNear);
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
    float invSamples = 1.0/float(vSsaoSamples);

    vec3 lBufferPos, lBufferNorm;

    // View space normal
    lBufferNorm = texture2D(vNormal, texcoord.st).xyz;
    vec3 lNormal = normalize(lBufferNorm * 2.0 - 1.0);

    // View space position of the pixel
    vec3 lPos = texture2D(vPosition, texcoord.st).xyz;

    // These values shouldn't be equal. If they are, it means
    // that the vNormal and vPosition textures were not rendered for
    // this pixel
    if(lBufferNorm == lPos)
        gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0);
    else
    {
        float lOcclusion, lWeight;

        //lPos.z = 1.f/lPos.z;

        // Random value from the noise map
        vec2 modifier = texture2D(vNoiseMap, texcoord.st).xy + (lPos.xy + lPos.z);
        
        float dist, visibility = 0.f;
        vec4 random, screenPos = vec4(1.f);

        for(int i=0; i<vSsaoSamples; i++)
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
            screenPos = vProjectionMatrix * vec4(lPos.x, lPos.y, -lPos.z, 1.0); 
            // Offset the screen position
            screenPos.xyz = screenPos.xyz + random.xyz*vSsaoFocus*(noise1(lPos.z)*0.5+0.5);
            // Store the depth of the new position
            float lScreenDepth = -(vInvProjectionMatrix*screenPos).z;
            screenPos.xyz /= screenPos.w;

            // Get the depth at the screen space coordinates -- this is the most expensive operation
            /*vec4 lFragPos = texture2D(vPosition, screenPos.xy * 0.5 + 0.5);
            lFragPos.z = 1.0/lFragPos.z;
            lFragPos = vProjectionMatrix * lFragPos;
            lFragPos.z /= -lFragPos.w;*/
            screenPos.xy = screenPos.xy*0.5+0.5;

            lBufferPos = texture2D(vPosition, screenPos.xy).xyz;
            dist = lBufferPos.z;
            //dist = 1.0/dist;

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

                if(lDiff > 0)
                    visibility += min(abs(lDiff/(vSsaoFocus/(vFar-vNear))) + lWeight, 1.0);
                else
                    visibility += 1.0;
            }
        }

        // Final occlusion factor
        //visibility = max(0.0, min(1.0, (visibility/float(vSsaoSamples) - ssaoRescale)*1.5+ssaoRescale));
        lOcclusion = pow(visibility*invSamples, vSsaoPower);

        //gl_FragColor.rgb = texture2D(vColor1, texcoord.st).rgb;

        // Normal
        //gl_FragColor.rgb = texture2D(vNormal, texcoord.st).rgb;
        // Position in camera space
        //gl_FragColor.rgb = vec3(texture2D(vPosition, texcoord.st).rg, 0.0) + vec3(1.5, 0.0, 0.0);

        // Position in screen space
        /*vec4 lScreenPos = vProjectionMatrix * vec4(lPos.x, lPos.y, -lPos.z, 1.0);
        lScreenPos.xyz = lScreenPos.xyz / lScreenPos.w;
        gl_FragColor.rgb = lScreenPos.xyz;*/

        //gl_FragColor.rgb = texture2D(vNoiseMap, texcoord*2).rgb; 
        //gl_FragColor.rgb = vec3(texcoord.xy, 0.0);

        gl_FragColor.rgb = vec3(lOcclusion);
        gl_FragColor.a = 1.f;
    }
}
