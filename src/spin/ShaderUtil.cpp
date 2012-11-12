#include <stdlib.h>
#include <stdio.h>
#include <osg/Shader>
#include <osgDB/FileUtils>


#include "shaderutil.h"

namespace spin
{

bool loadShaderSource(osg::Shader* obj, const std::string& fileName )
{
    std::string fqFileName = osgDB::findDataFile(fileName);
    if( fqFileName.length() == 0 )
    {
        std::cout << "Warning: Shader file \"" << fileName << "\" not found." << std::endl;
        return false;
    }
    bool success = obj->loadShaderSourceFromFile( fqFileName.c_str());
    if ( !success  )
    {
        std::cout << "Warning: Shader could not be loaded: " << fileName << std::endl;
        return false;
    }
    else
    {
        replaceSampler2DRect(obj);
        return true;
    }
}

void replaceSampler2DRect(osg::Shader* obj)
{
    size_t pos;
    int count;
    std::string src = obj->getShaderSource();
    std::string replaceThis;
    std::string replaceWith;
    
    replaceThis="sampler2DRect"; replaceWith="sampler2D";
    pos = 0;
    while ((pos = src.find(replaceThis, pos)) != std::string::npos)
    {
        src.replace(pos, replaceThis.length(), replaceWith);
        pos += replaceWith.length();
    }

    replaceThis="texture2DRect"; replaceWith="texture2D";
    pos = 0;
    while ((pos = src.find(replaceThis, pos)) != std::string::npos)
    {
        src.replace(pos, replaceThis.length(), replaceWith);
        pos += replaceWith.length();
    }

    obj->setShaderSource(src);
}



ParsedUniforms parseUniformsFromShader(osg::Shader *shader)
{
    char name[100];
    char type[100];
    
    int pos=0;
    int pend=0;
    
    ParsedUniforms uniforms;
    
    std::string s = shader->getShaderSource();    
    while ( (pos=s.find("uniform", pend))!=std::string::npos )
    {
        if ( (pend=s.find(";",pos))==std::string::npos || pend-pos>100 )
            break;
        if ( sscanf(s.c_str()+pos," uniform %[^ ] %[^ ;] ;",type,name)!=2 )
        {
            printf("Unable to parse pos %d to %d\n",pos,pend);
            break;
        }
        //printf("Found 'uniform' at pos %d to pos %d, type='%s' name='%s'\n",pos,pend,type,name);

        uniforms.insert(std::pair<std::string, std::string>(name,type));
    }
    
    return uniforms;
}


}
