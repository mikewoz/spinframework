// glslv output by Cg compiler
// cgc version 2.2.0010, build date Sep 29 2009
// command line args: -profile glslv
// source file: test.cgfx
//vendor NVIDIA Corporation
//version 2.2.0.10
//profile glslv
//program v
//semantic v.lightPos
//semantic v.lightType
//semantic v.lightDir
//semantic ParamID
//semantic UIColor_5866
//semantic light1Dir : Direction
//semantic light1Pos : POSITION
//semantic light1Color : LIGHTCOLOR
//semantic light1Attenuation : Attenuation
//semantic light1Hotspot : HotSpot
//semantic light1Falloff : FallOff
//semantic wvp : WorldViewProjection
//semantic worldI : WorldInverse
//semantic worldIT : WorldInverseTranspose
//semantic viewInv : ViewInverse
//semantic world : World
//var float3 lightPos :  : _lightPos1 : 1 : 1
//var int lightType :  :  : 2 : 0
//var float3 lightDir :  :  : 3 : 0
//var string ParamID :  :  : -1 : 0
//var float3 UIColor_5866 :  :  : -1 : 0
//var float3 light1Dir : Direction :  : -1 : 0
//var float3 light1Pos : POSITION :  : -1 : 0
//var float4 light1Color : LIGHTCOLOR :  : -1 : 0
//var float4 light1Attenuation : Attenuation :  : -1 : 0
//var float light1Hotspot : HotSpot :  : -1 : 0
//var float light1Falloff : FallOff :  : -1 : 0
//var float4x4 wvp : WorldViewProjection : _wvp[0], 4 : -1 : 1
//var float4x4 worldI : WorldInverse : , 4 : -1 : 0
//var float4x4 worldIT : WorldInverseTranspose : , 4 : -1 : 0
//var float4x4 viewInv : ViewInverse : , 4 : -1 : 0
//var float4x4 world : World : _world[0], 4 : -1 : 1
//var float4 In.position : $vin.POSITION : POSITION : 0 : 1
//var float4 In.tangent : $vin.TANGENT : $TANGENT : 0 : 1
//var float4 In.binormal : $vin.BINORMAL : $BINORMAL : 0 : 1
//var float4 In.normal : $vin.NORMAL : NORMAL : 0 : 1
//var float4 v.position : $vout.POSITION : POSITION : -1 : 1
//var float3 v.lightVec : $vout.TEXCOORD0 : TEXCOORD0 : -1 : 1
//default ParamID = "0x002"
//default UIColor_5866 = 0.841726 0.345098 0.345098
//default light1Dir = 100 100 100
//default light1Pos = 100 100 100
//default light1Color = 1 1 1 1
//default light1Attenuation = 20 30 0 100
//default light1Hotspot = 43
//default light1Falloff = 45

struct a2v {
    vec4 _position1;
    vec4 _tangent;
    vec4 _binormal;
    vec4 _normal;
};

struct v2f {
    vec4 _position;
    vec3 _lightVec1;
};

v2f _ret_0;
uniform vec3 _lightPos1;
vec4 _r0022;
vec3 _TMP20032;
vec4 _r0034;
vec4 _v0034;
vec3 _r0036;
vec4 _r0044;
attribute vec4 TANGENT;
attribute vec4 BINORMAL;
uniform vec4 _wvp[4];
uniform vec4 _world[4];

 // main procedure, the original name was v
void main()
{

    vec3 _objTangentXf1[3];

    _objTangentXf1[1] = -TANGENT.xyz;
    _r0022.x = dot(_world[0], gl_Vertex);
    _r0022.y = dot(_world[1], gl_Vertex);
    _r0022.z = dot(_world[2], gl_Vertex);
    _TMP20032 = _lightPos1 - _r0022.xyz;
    _v0034 = vec4(_TMP20032.x, _TMP20032.y, _TMP20032.z, 1.00000000E+00);
    _r0034 = _v0034.x*_world[0];
    _r0034 = _r0034 + _v0034.y*_world[1];
    _r0034 = _r0034 + _v0034.z*_world[2];
    _r0034 = _r0034 + _v0034.w*_world[3];
    _r0036.x = dot(BINORMAL.xyz, _r0034.xyz);
    _r0036.y = dot(_objTangentXf1[1], _r0034.xyz);
    _r0036.z = dot(gl_Normal, _r0034.xyz);
    _r0044.x = dot(_wvp[0], gl_Vertex);
    _r0044.y = dot(_wvp[1], gl_Vertex);
    _r0044.z = dot(_wvp[2], gl_Vertex);
    _r0044.w = dot(_wvp[3], gl_Vertex);
    _ret_0._position = _r0044;
    _ret_0._lightVec1 = _r0036;
    gl_Position = _r0044;
    gl_TexCoord[0].xyz = _r0036;
    return;
} // main end
