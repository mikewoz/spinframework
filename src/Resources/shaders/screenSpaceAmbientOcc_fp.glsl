#version 120

uniform sampler2D vColor1;
uniform sampler2D vColor2;
uniform sampler2D vColor3;
uniform sampler2D vNoiseMap;
uniform sampler2D vDepth;

uniform float vPixelSize;
uniform float vNear;
uniform float vFar;
uniform mat4 vProjectionMatrix;

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
    const float ssaoFocus = 0.04;
    const float ssaoPower = 2.0;
    const int ssaoLoops = 32;

    // Depth from the real depth buffer
    float lDepth = texture2D(vDepth, texcoord).r;

    // View space normal
    vec3 lNormal = texture2D(vColor2, texcoord.st).xyz * 2.0 - 1.0;

    // View space position of the pixel
    vec3 lPos = texture2D(vColor3, texcoord.st).xyz; //GetViewPos(texcoord.st);

    float lOcclusion;

    if(lDepth == 1.0)
        lOcclusion = 1.0;
    else
    {
        lPos.z = 1.f/lPos.z; 

        // Random value from the noise map
        vec2 modifier = texture2D(vNoiseMap, texcoord / vPixelSize / 32.f).xy + (lPos.xy + lPos.z);
        
        float dist, visibility = 0.f;
        vec4 random, screenPos, viewPos = vec4(1.f);

        for(int i=0; i<ssaoLoops; i++)
        {
            // Retrieve a new random vector from the texture
            random = texture2D(vNoiseMap, modifier);

            // Not much point in normalizing -- no visual difference
            random.xyz = normalize(random.xyz * 2.0 - 1.0);

            // Randomize the modifier for the next loop
            modifier += random.xy;

            // Flip the random vector if it's below the plane
            if(dot(random.xyz, lNormal) < 0.0) 
                random.xyz = -random.xyz;


            // Randomly offset view-space position
            viewPos.xyz = random.xyz * (ssaoFocus) + lPos;
            viewPos.w = 1.f;

            // Calculate the randomly offset position's screen space coordinates -- second most expensive operation
            screenPos = vProjectionMatrix * viewPos;
            screenPos.xyz /= -screenPos.w;

            // Get the depth at the screen space coordinates -- this is the most expensive operation
            dist = texture2D(vColor3, screenPos.xy * 0.5 + 0.5).z;
            lDepth = texture2D(vDepth, screenPos.xy * 0.5 + 0.5).r;
            if(lDepth == 1.0)
            {
                visibility += 1.0; 
            }
            else
            {
                dist = 1.f/dist;
                //dist = GetDistance(screenpos.xy / screenpos.w * 0.5 + 0.5);

                // Visibility is linearly scaled, depending on how far the distance is from the focus range
                //visibility += min(abs((linearizeDepth(viewPos.z) - linearizeDepth(dist)) / ssaoFocus + 1.0), 1.0);
                
                float lDiff = linearizeDepth(viewPos.z) - linearizeDepth(dist);
                // If the ray hits behind the fragment (from the camera position)
                if(lDiff > 0)
                {
                    visibility += min(lDiff/ssaoFocus, 1.0);
                }
                // else, if the ray hits in front of the fragment
                else
                {
                    visibility += 1.0; //visibility += max(1.0 + lDiff/ssaoFocus, 0.0);
                }
            }
        }

        // Final occlusion factor
        lOcclusion = pow(visibility / float(ssaoLoops), ssaoPower);
    }

    //gl_FragColor.rgb = texture2D(vColor1, texcoord.st).rgb;
    //gl_FragColor.rgb = texture2D(vColor2, texcoord.st).rgb;
    //gl_FragColor.rgb = vec3(texture2D(vColor3, texcoord.st).rgb);
    //gl_FragColor.rgb = texture2D(vNoiseMap, texcoord*2).rgb;

    //gl_FragColor.rgb = lNormal;
    
    //gl_FragColor.rgb = vec3(random);

    //gl_FragColor.rgb = vec3(texture2D(vColor3, texcoord).z);
    //gl_FragColor.rgb = vec3(lPos.z/8.f);
    //gl_FragColor.rgb = vec3(abs(lPos.xy), 0.f);
    //gl_FragColor.rgb = vec3(viewPos.z/8.f);
    //gl_FragColor.rgb = vec3(viewPos.xy*0.5+0.5, 0.f);
    //gl_FragColor.rgb = vec3(dist/8.f);
    //gl_FragColor.rgb = vec3(screenPos.xy*0.5+0.5, 0.f);
    //gl_FragColor.rgb = vec3(abs(linearizeDepth(viewPos.z) - linearizeDepth(dist))*16384.f);
    //gl_FragColor.rgb = vec3(visibility/float(ssaoLoops)/2.f);
    gl_FragColor.rgb = vec3(lOcclusion)*texture2D(vColor1, texcoord).rgb;
    

    //gl_FragColor.rgb = vec3(lOcclusion, lOcclusion, lPos.z/8.f);

    //gl_FragColor.rgb = vec3(texcoord, 0.f);
    
    /*if(lDepth == 1.0)
        gl_FragColor.rgb = vec3(1.0, 0.0, 0.0);
    else
        gl_FragColor.rgb = vec3(0.0, 1.0, 0.0);*/

    gl_FragColor.a = 1.f;
}
