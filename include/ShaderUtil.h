#ifndef __ShaderUtil_h__
#define __ShaderUtil_h__

#include <osg/Shader>

namespace spin
{

typedef std::map<std::string, std::string> ParsedUniforms; // map of name, type


bool loadShaderSource(osg::Shader* obj, const std::string& fileName );

void replaceSampler2DRect(osg::Shader* obj);

ParsedUniforms parseUniformsFromShader(osg::Shader *shader);


static const char *microshaderVertSource = {
    "#version 120\n"
    "// microshader - colors a fragment based on its position\n"
    "varying vec4 color;\n"
    "void main(void)\n"
    "{\n"
    "    color = gl_Vertex;\n"
    "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
    "}\n"
};

static const char *microshaderFragSource = {
    "#version 120\n"
    "varying vec4 color;\n"
    "void main(void)\n"
    "{\n"
    "    gl_FragColor = clamp( color, 0.0, 1.0 );\n"
    "}\n"
};
}

#endif
