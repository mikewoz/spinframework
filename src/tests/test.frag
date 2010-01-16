// glslf output by Cg compiler
// cgc version 2.2.0010, build date Sep 29 2009
// command line args: -profile glslf
// source file: test.cgfx
//vendor NVIDIA Corporation
//version 2.2.0.10
//profile glslf
//program f
//semantic f.lightDir
//semantic f.lightColor
//semantic f.lightAttenuation
//semantic f.lightHotspot
//semantic f.lightFalloff
//semantic f.lightType
//semantic f.lightattenType
//semantic f.lightconeType
//semantic f.lightCastShadows
//semantic f.shadowPassCount
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
//var float3 lightDir :  :  : 1 : 0
//var float4 lightColor :  : _lightColor1 : 2 : 1
//var float4 lightAttenuation :  :  : 3 : 0
//var float lightHotspot :  :  : 4 : 0
//var float lightFalloff :  :  : 5 : 0
//var int lightType :  :  : 6 : 0
//var int lightattenType :  :  : 7 : 0
//var int lightconeType :  :  : 8 : 0
//var bool lightCastShadows :  :  : 9 : 0
//var int shadowPassCount :  :  : 10 : 0
//var string ParamID :  :  : -1 : 0
//var float3 UIColor_5866 :  : _UIColor_5866 : -1 : 1
//var float3 light1Dir : Direction :  : -1 : 0
//var float3 light1Pos : POSITION :  : -1 : 0
//var float4 light1Color : LIGHTCOLOR :  : -1 : 0
//var float4 light1Attenuation : Attenuation :  : -1 : 0
//var float light1Hotspot : HotSpot :  : -1 : 0
//var float light1Falloff : FallOff :  : -1 : 0
//var float4x4 wvp : WorldViewProjection : , 4 : -1 : 0
//var float4x4 worldI : WorldInverse : , 4 : -1 : 0
//var float4x4 worldIT : WorldInverseTranspose : , 4 : -1 : 0
//var float4x4 viewInv : ViewInverse : , 4 : -1 : 0
//var float4x4 world : World : , 4 : -1 : 0
//var float3 In.lightVec : $vin.TEXCOORD0 : TEXCOORD0 : 0 : 1
//var float4 f : $vout.COLOR : COLOR : -1 : 1
//default ParamID = "0x002"
//default UIColor_5866 = 0.841726 0.345098 0.345098
//default light1Dir = 100 100 100
//default light1Pos = 100 100 100
//default light1Color = 1 1 1 1
//default light1Attenuation = 20 30 0 100
//default light1Hotspot = 43
//default light1Falloff = 45

struct a2v {
    vec4 _tangent;
    vec4 _binormal;
    vec4 _normal;
};

struct v2f {
    vec3 _lightVec2;
};

vec4 _ret_0;
uniform vec4 _lightColor1;
vec3 _TMP26;
float _x0031;
float _TMP36;
float _b0041;
uniform vec3 _UIColor_5866;

 // main procedure, the original name was f
void main()
{

    vec3 _ret;
    vec3 _diffuseColor;
    float _NdotL;
    vec4 _done;

    _x0031 = dot(gl_TexCoord[0].xyz, gl_TexCoord[0].xyz);
    _TMP26 = inversesqrt(_x0031)*gl_TexCoord[0].xyz;
    _NdotL = dot(vec3( 0.00000000E+00, 0.00000000E+00, 1.00000000E+00), _TMP26);
    _b0041 = min(1.00000000E+00, _NdotL);
    _TMP36 = max(0.00000000E+00, _b0041);
    _diffuseColor = _UIColor_5866.xyz*vec3(_TMP36, _TMP36, _TMP36);
    _ret = _diffuseColor*_lightColor1.xyz;
    _done = vec4(_ret.x, _ret.y, _ret.z, 1.00000000E+00);
    _ret_0 = _done;
    gl_FragColor = _done;
    return;
} // main end
