#include <iostream>
#include <cstdlib>
#include <cmath>
#include <osgViewer/Viewer>
#include <osg/Group>
#include <osgDB/OutputStream>
#include <osgDB/InputStream>
#include <osgDB/Registry>
#include <osgDB/ReadFile>

#include "myshape.h"
#include "osgreflection.h"

typedef std::map<osgDB::BaseSerializer*, osg::ref_ptr<Method> > MethodMap;


/*
class myObjectWrapper : public osgDB::ObjectWrapper
{
public:
	ObjectWrapper::SerializerList getSerializers() {return _serializers;}
};
*/

// need this if we hava a dynamic serializer library?:
//USE_SERIALIZER_WRAPPER_LIBRARY(mytest)
// need this if we're linking statically:
USE_SERIALIZER_WRAPPER(MyShape)

int main(int argc, char **argv)
{

	mytest::MyShape *sphere = new mytest::MyShape(mytest::MyShape::SPHERE);
	mytest::MyShape *box = new mytest::MyShape(mytest::MyShape::BOX);

	sphere->setTranslation(osg::Vec3(-1, 0, 0));
	box->setTranslation(osg::Vec3(1, 0, 0));
	//box->setNote("test box");

	osg::Group *grp = new osg::Group();
	grp->addChild(sphere);
	grp->addChild(box);


    osgDB::ObjectWrapperManager* wrapperManager = osgDB::Registry::instance()->getObjectWrapperManager();
    osgDB::ObjectWrapper* wrapper = wrapperManager->findWrapper("mytest::MyShape");

    if ( wrapperManager )
    {
        if ( wrapper )
        {
        	std::cout << "woohoo. Got wrapper: " << wrapper->getName() << std::endl;

        	// (note: need to modify osg, to make serializers_ public)
        	/*
        	std::cout << "serializer list:" << std::endl;
        	for ( osgDB::ObjectWrapper::SerializerList::iterator itr=wrapper->_serializers.begin(); itr!=wrapper->_serializers.end(); ++itr )
        	{
        		std::cout << "  " << (*itr)->getName() << std::endl;
        	}
        	 */

        	osgDB::StringList assoc = wrapper->getAssociates();
        	std::cout << wrapper->getName() << " has " << assoc.size() << " associates: " << std::endl;
        	for (unsigned int i=0; i<assoc.size(); i++)
        	{
        		std::cout << "  associate: " << assoc[i] << std::endl;
        	}

        	/*
        	osgDB::StringList props;
        	std::vector<int> types;
        	wrapper->readSchema(props, types);
        	std::cout << wrapper->getName() << " has " << props.size() << " properties: " << std::endl;
        	for (unsigned int i=0; i<props.size(); i++)
        	{
        		std::cout << "  prop: " << props[i] << std::endl;
        	}
        	*/

        	const std::string serializerName = "Num";
        	osgDB::BaseSerializer* serializer = wrapper->getSerializer("Num");
        	if ( serializer )
        		std::cout << "got serializer: " << serializer->getName() << std::endl;
        	else
        		std::cout << "oops. Couldn't find serializer for '"<<serializerName<<"'" << std::endl;

        }
        else
        {
        	std::cout << "oops. Couldn't find wrapper for mytest::MyShape" << std::endl;
        	return 1;
        }
    }


    // test osgreflection:
    ClassInfo* MyShapeInfo = ReflectionManager::instance()->getClassInfo("mytest::MyShape");
    ClassInfo* intInfo = ReflectionManager::instance()->getClassInfo("int");

    // try to create a new object:
    ClassInstance* newShape = MyShapeInfo->createInstance();

    // THE ABOVE WORKS. Now, HOW TO CALL METHODS?!


    // direct way:
    osgDB::BaseSerializer* serializer = wrapper->getSerializer("Num");
    if ( serializer )
    {
		ReflectionManager::instance()->getOutputStream() << 4;
		serializer->read( ReflectionManager::instance()->getInputStream(), *(newShape->getObject()) );
    } else std::cout << "ERROR" << std::endl;


    // try to set the 'Num':
    //Method* MyShapeNumMethod = MyShapeInfo->getMethod("Num");
    //MyShapeNumMethod->set( newShape, 4 );


    // try to set the 'Note' string:
    //Method* MyShapeNoteMethod = MyShapeInfo->getMethod("Note");
    //MyShapeNoteMethod->set( newShape, std::string("foo") );

    // try to set the 'Translation' (osg x,y,z vector):
    //Method* MyShapeMethod = MyShapeInfo->getMethod("Translation");
    //MyShapeMethod->set( newShape, osg::Vec3(0.0,0.0,1.0) );


	// view our scene:
    /*
	osgViewer::Viewer viewer;
	viewer.setSceneData( grp );
    return viewer.run();
	*/
}
