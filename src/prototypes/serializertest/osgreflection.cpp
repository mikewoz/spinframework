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

#include "osgreflection.h"

// *****************************************************************************
// ReflectionManager implementation

ReflectionManager* ReflectionManager::instance()
{
    static osg::ref_ptr<ReflectionManager> s_manager = new ReflectionManager;
    return s_manager.get();
}

ReflectionManager::ReflectionManager()
{
/*
	osgDB::Options *options = new osgDB::Options("Ascii");
	_outputStream = new osgDB::OutputStream(options);
	_inputStream = new osgDB::InputStream(options);
*/
	_outputStream = new osgDB::OutputStream(0);
	_inputStream = new osgDB::InputStream(0);

    //_outputStream->setOutputIterator( new BinaryOutputIterator(&_source) );
    //_inputStream->setInputIterator( new BinaryInputIterator(&_source) );
    //_outputStream->setOutputIterator( new osgDB::OutputIterator(&_source) );
    //_inputStream->setInputIterator( new osgDB::OutputIterator(&_source) );
}

ReflectionManager::~ReflectionManager()
{
}

ClassInfo* ReflectionManager::getClassInfo( const std::string& name )
{
    osgDB::ObjectWrapperManager* wrapperManager = osgDB::Registry::instance()->getObjectWrapperManager();
    if ( wrapperManager )
    {
        osgDB::ObjectWrapper* wrapper = wrapperManager->findWrapper(name);
        if ( wrapper ) return getClassInfo(wrapper);
    }
    return NULL;
}

ClassInfo* ReflectionManager::getClassInfo( osgDB::ObjectWrapper* wrapper )
{
    ClassInfoMap::iterator itr = _classInfoMap.find(wrapper);
    if ( itr!=_classInfoMap.end() ) return itr->second.get();
    
    ClassInfo* info = new ClassInfo( wrapper );
    _classInfoMap[wrapper] = info;
    return info;
}

Method* ReflectionManager::getMethod( osgDB::BaseSerializer* serializer )
{
    MethodMap::iterator itr = _methodMap.find(serializer);
    if ( itr!=_methodMap.end() ) return itr->second.get();
    
    Method* method = new Method( serializer );
    _methodMap[serializer] = method;
    return method;
}

// *****************************************************************************
// ClassInfo implementation

ClassInstance* ClassInfo::createInstance()
{
    if ( _wrapper )
    {
        const osg::Object* obj = _wrapper->getProto();
        if ( obj ) return new ClassInstance( obj->cloneType() );
    }
    return NULL;
}

Method* ClassInfo::getMethod( const std::string& name )
{
    if ( _wrapper )
    {
        osgDB::BaseSerializer* serializer = _wrapper->getSerializer(name);
        if ( serializer ) return ReflectionManager::instance()->getMethod(serializer);
    }
    return NULL;
}

// *****************************************************************************
// Method implementation

template<typename T>
bool Method::set( ClassInstance* clsObject, T value )
{
    bool ok = false;
    ReflectionManager::instance()->getOutputStream() << value;
    if ( _serializer )
    {
        ok = _serializer->read( ReflectionManager::instance()->getInputStream(), *(clsObject->getObject()) );
    }
    ReflectionManager::instance()->getSource().clear();
    return ok;
}

template<typename T>
bool Method::get( ClassInstance* clsObject, T& value )
{
    bool ok = false;
    if ( _serializer )
    {
        ok = _serializer->write( ReflectionManager::instance()->getOutputStream(), *(clsObject->getObject()) );
    }
    if ( ok ) ReflectionManager::instance()->getInputStream() >> value;
    ReflectionManager::instance()->getSource().clear();
    return ok;
}

bool Method::set( ClassInstance* clsObject, const ClassInstance& instance )
{
    bool ok = false;
    ReflectionManager::instance()->getOutputStream() << (instance.getObject()!=NULL) << instance.getObject();
    if ( _serializer )
    {
        ok = _serializer->read( ReflectionManager::instance()->getInputStream(), *(clsObject->getObject()) );
    }
    ReflectionManager::instance()->getSource().clear();
    return ok;
}

bool Method::get( ClassInstance* clsObject, ClassInstance& instance )
{
    bool ok = false;
    if ( _serializer )
    {
        ok = _serializer->write( ReflectionManager::instance()->getOutputStream(), *(clsObject->getObject()) );
    }
    if ( ok )
    {
        osg::Object* obj = ReflectionManager::instance()->getInputStream().readObject();
        instance.setObject( obj );
    }
    ReflectionManager::instance()->getSource().clear();
    return ok;
}

// *****************************************************************************

/*
int main( int argc, char **argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    
    ClassInfo* boxInfo = ReflectionManager::instance()->getClassInfo("osg::Box");
    Method* boxMethod = boxInfo->getMethod("HalfLengths");
    ClassInstance* box = boxInfo->createInstance();
    boxMethod->set( box, osg::Vec3(5.0f, 5.0f, 2.0f) );
    
    ClassInfo* drawableInfo = ReflectionManager::instance()->getClassInfo("osg::ShapeDrawable");
    Method* drawableMethod = drawableInfo->getMethod("Shape");
    ClassInstance* drawable = drawableInfo->createInstance();
    drawableMethod->set( drawable, *box );
    
    // FIXME..
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( dynamic_cast<osg::Drawable*>(drawable->getObject()) );
    
    osgViewer::Viewer viewer;
    viewer.setSceneData( geode );
    return viewer.run();
}
*/
