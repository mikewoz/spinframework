// glslf output by Cg compiler
// cgc version 2.2.0010, build date Sep 29 2009
// command line args: -profile glslf
// source file: post_glowBalloon.cgfx
//vendor NVIDIA Corporation
//version 2.2.0.10
//profile glslf
//program gloBalloon_PS
//semantic gloBalloon_PS.GlowColor
//semantic gloBalloon_PS.GlowExpon
//semantic Script : STANDARDSGLOBAL
//semantic gWorldXf : World
//semantic gWorldITXf : WorldInverseTranspose
//semantic gWvpXf : WorldViewProjection
//semantic gViewIXf : ViewInverse
//semantic gClearColor
//semantic gClearDepth
//semantic gInflate
//semantic gGlowColor
//semantic gGlowExpon
//var float3 GlowColor :  : _GlowColor1 : 1 : 1
//var float GlowExpon :  : _GlowExpon1 : 2 : 1
//var float Script : STANDARDSGLOBAL :  : -1 : 0
//var float4x4 gWorldXf : World : , 4 : -1 : 0
//var float4x4 gWorldITXf : WorldInverseTranspose : , 4 : -1 : 0
//var float4x4 gWvpXf : WorldViewProjection : , 4 : -1 : 0
//var float4x4 gViewIXf : ViewInverse : , 4 : -1 : 0
//var float4 gClearColor :  :  : -1 : 0
//var float gClearDepth :  :  : -1 : 0
//var float gInflate :  :  : -1 : 0
//var float3 gGlowColor :  :  : -1 : 0
//var float gGlowExpon :  :  : -1 : 0
//var float3 IN.WorldNormal : $vin.TEXCOORD0 : TEXCOORD0 : 0 : 1
//var float3 IN.WorldView : $vin.TEXCOORD1 : TEXCOORD1 : 0 : 1
//var float4 gloBalloon_PS : $vout.COLOR : COLOR : -1 : 1
//default Script = 0.8
//default gClearColor = 0 0 0 0
//default gClearDepth = 1
//default gInflate = 0.06
//default gGlowColor = 1 0.9 0.3
//default gGlowExpon = 1.3

struct appdata {
    vec4 _UV;
    vec4 _Normal;
    vec4 _Tangent;
    vec4 _Binormal;
};

struct gloVertOut {
    vec3 _WorldNormal;
    vec3 _WorldView;
};

vec4 _ret_0;
uniform vec3 _GlowColor1;
uniform float _GlowExpon1;
vec3 _TMP14;
float _x0019;
vec3 _TMP20;
float _x0025;

 // main procedure, the original name was gloBalloon_PS
void main()
{

    float _edge;
    vec3 _result;

    _x0019 = dot(gl_TexCoord[0].xyz, gl_TexCoord[0].xyz);
    _TMP14 = inversesqrt(_x0019)*gl_TexCoord[0].xyz;
    _x0025 = dot(gl_TexCoord[1].xyz, gl_TexCoord[1].xyz);
    _TMP20 = inversesqrt(_x0025)*gl_TexCoord[1].xyz;
    _edge = 1.00000000E+00 - dot(_TMP14, _TMP20);
    _edge = pow(_edge, _GlowExpon1);
    _result = _edge*_GlowColor1.xyz;
    _ret_0 = vec4(_result.x, _result.y, _result.z, _edge);
    gl_FragColor = _ret_0;
    return;
} // main end
