// glslv output by Cg compiler
// cgc version 2.2.0010, build date Sep 29 2009
// command line args: -profile glslv
// source file: post_glowBalloon.cgfx
//vendor NVIDIA Corporation
//version 2.2.0.10
//profile glslv
//program gloBalloon_VS
//semantic gloBalloon_VS.Inflate
//semantic gloBalloon_VS.WorldITXf
//semantic gloBalloon_VS.WorldXf
//semantic gloBalloon_VS.ViewIXf
//semantic gloBalloon_VS.WvpXf
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
//var float Inflate :  : _Inflate1 : 1 : 1
//var float4x4 WorldITXf :  : _WorldITXf1[0], 4 : 2 : 1
//var float4x4 WorldXf :  : _WorldXf1[0], 4 : 3 : 1
//var float4x4 ViewIXf :  : _ViewIXf1[0], 4 : 4 : 1
//var float4x4 WvpXf :  : _WvpXf1[0], 4 : 5 : 1
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
//var float3 IN.Position : $vin.POSITION : POSITION : 0 : 1
//var float4 IN.UV : $vin.TEXCOORD0 :  : 0 : 0
//var float4 IN.Normal : $vin.NORMAL : NORMAL : 0 : 1
//var float4 IN.Tangent : $vin.TANGENT0 :  : 0 : 0
//var float4 IN.Binormal : $vin.BINORMAL0 :  : 0 : 0
//var float4 gloBalloon_VS.HPosition : $vout.POSITION : POSITION : -1 : 1
//var float3 gloBalloon_VS.WorldNormal : $vout.TEXCOORD0 : TEXCOORD0 : -1 : 1
//var float3 gloBalloon_VS.WorldView : $vout.TEXCOORD1 : TEXCOORD1 : -1 : 1
//default Script = 0.8
//default gClearColor = 0 0 0 0
//default gClearDepth = 1
//default gInflate = 0.06
//default gGlowColor = 1 0.9 0.3
//default gGlowExpon = 1.3

struct appdata {
    vec3 _Position;
    vec4 _UV;
    vec4 _Normal;
    vec4 _Tangent;
    vec4 _Binormal;
};

struct gloVertOut {
    vec4 _HPosition;
    vec3 _WorldNormal;
    vec3 _WorldView;
};

gloVertOut _ret_0;
appdata _IN1;
uniform float _Inflate1;
uniform vec4 _WorldITXf1[4];
uniform vec4 _WorldXf1[4];
uniform vec4 _ViewIXf1[4];
uniform vec4 _WvpXf1[4];
vec4 _r0022;
vec4 _TMP31;
vec4 _v0032;
float _x0036;
vec4 _r0038;
vec3 _TMP47;
vec3 _v0048;
float _x0052;
vec4 _r0054;

 // main procedure, the original name was gloBalloon_VS
void main()
{

    vec4 _Po;

    _IN1._Normal.xyz = gl_Normal;
    _r0022.x = dot(_WorldITXf1[0], _IN1._Normal);
    _r0022.y = dot(_WorldITXf1[1], _IN1._Normal);
    _r0022.z = dot(_WorldITXf1[2], _IN1._Normal);
    _r0022.xyz;
    _Po = vec4(gl_Vertex.x, gl_Vertex.y, gl_Vertex.z, 1.00000000E+00);
    _v0032 = vec4(gl_Normal.x, gl_Normal.y, gl_Normal.z, 0.00000000E+00);
    _x0036 = dot(_v0032, _v0032);
    _TMP31 = inversesqrt(_x0036)*_v0032;
    _Po = _Po + _Inflate1*_TMP31;
    _r0038.x = dot(_WorldXf1[0], _Po);
    _r0038.y = dot(_WorldXf1[1], _Po);
    _r0038.z = dot(_WorldXf1[2], _Po);
    _v0048 = vec3(_ViewIXf1[0].w, _ViewIXf1[1].w, _ViewIXf1[2].w) - _r0038.xyz;
    _x0052 = dot(_v0048, _v0048);
    _TMP47 = inversesqrt(_x0052)*_v0048;
    _r0054.x = dot(_WvpXf1[0], _Po);
    _r0054.y = dot(_WvpXf1[1], _Po);
    _r0054.z = dot(_WvpXf1[2], _Po);
    _r0054.w = dot(_WvpXf1[3], _Po);
    _ret_0._HPosition = _r0054;
    _ret_0._WorldNormal = _r0022.xyz;
    _ret_0._WorldView = _TMP47;
    gl_TexCoord[1].xyz = _TMP47;
    gl_Position = _r0054;
    gl_TexCoord[0].xyz = _r0022.xyz;
    return;
} // main end
