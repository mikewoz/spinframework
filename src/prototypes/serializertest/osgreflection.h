/* OpenSceneGraph example, osgreflection.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

#ifndef __osgreflection_h
#define __osgreflection_h

#include <osgDB/OutputStream>
#include <osgDB/InputStream>
#include <osgDB/Registry>

// Not a good including operation here, will be modified after enough tests
//#include "../../src/osgPlugins/osg/BinaryStreamOperator.h"
//#include "BinaryStreamOperator.h"

class ClassInfo;
class ClassInstance;
class Method;

class ReflectionManager : public osg::Referenced
{
public:
    static ReflectionManager* instance();

    ClassInfo* getClassInfo( const std::string& );

    osgDB::OutputStream& getOutputStream() { return *_outputStream; }
    osgDB::InputStream& getInputStream() { return *_inputStream; }
    std::stringstream& getSource() { return _source; }

    ClassInfo* getClassInfo( osgDB::ObjectWrapper* );
    Method* getMethod( osgDB::BaseSerializer* );

protected:
    ReflectionManager();
    virtual ~ReflectionManager();

    typedef std::map<osgDB::ObjectWrapper*, osg::ref_ptr<ClassInfo> > ClassInfoMap;
    ClassInfoMap _classInfoMap;

    typedef std::map<osgDB::BaseSerializer*, osg::ref_ptr<Method> > MethodMap;
    MethodMap _methodMap;

    osgDB::OutputStream* _outputStream;
    osgDB::InputStream* _inputStream;
    std::stringstream _source;
};

class ClassInfo : public osg::Referenced
{
public:
    ClassInfo( osgDB::ObjectWrapper* w=0 ) : _wrapper(w) {}

    ClassInstance* createInstance();
    Method* getMethod( const std::string& );

protected:
    virtual ~ClassInfo() {}

    osgDB::ObjectWrapper* _wrapper;
};

class ClassInstance : public osg::Referenced
{
public:
    ClassInstance( osg::Object* obj ) : _object(obj) {}

    void setObject( osg::Object* obj ) { _object = obj; }
    osg::Object* getObject() { return _object.get(); }
    const osg::Object* getObject() const { return _object.get(); }

protected:
    virtual ~ClassInstance() {}

    osg::ref_ptr<osg::Object> _object;
};

class Method : public osg::Referenced
{
public:
    Method( osgDB::BaseSerializer* s=0 ) : _serializer(s) {}

    template<typename T> bool set( ClassInstance*, T );
    template<typename T> bool get( ClassInstance*, T& );

    bool set( ClassInstance*, const ClassInstance& );
    bool get( ClassInstance*, ClassInstance& );

protected:
    virtual ~Method() {}

    osgDB::BaseSerializer* _serializer;
};

#endif
