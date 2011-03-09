#include <osgViewer/Viewer>
#include <osg/Group>

#include <iostream>
#include <cstdlib>
#include <cmath>


#include <osgDB/OutputStream>
#include <osgDB/InputStream>
#include <osgDB/Registry>
#include <osgDB/ReadFile>

#include "myshape.h"
#include "osgreflection.h"

typedef std::map<osgDB::BaseSerializer*, osg::ref_ptr<Method> > MethodMap;

int main(int argc, char **argv)
{
	spinframework::myshape *sphere = new spinframework::myshape(spinframework::myshape::SPHERE);
	spinframework::myshape *box = new spinframework::myshape(spinframework::myshape::BOX);

	sphere->setTranslation(osg::Vec3(-1, 0, 0));
	box->setTranslation(osg::Vec3(1, 0, 0));

	//sphere->setNote("test string");

	osg::Group *grp = new osg::Group();
	grp->addChild(sphere);
	grp->addChild(box);


    osgDB::ObjectWrapperManager* wrapperManager = osgDB::Registry::instance()->getObjectWrapperManager();
    osgDB::ObjectWrapper* wrapper = wrapperManager->findWrapper("spinframework::myshape");

    if ( wrapperManager )
    {
        if ( wrapper )
        {
        	std::cout << "woohoo. Got wrapper: " << wrapper->getName() << std::endl;

        	osgDB::StringList assoc = wrapper->getAssociates();
        	std::cout << wrapper->getName() << " has " << assoc.size() << " associates: " << std::endl;
        	for (unsigned int i=0; i<assoc.size(); i++)
        	{
        		std::cout << "  associate: " << assoc[i] << std::endl;
        	}

        	osgDB::StringList props;
        	std::vector<int> types;
        	wrapper->readSchema(props, types);
        	std::cout << wrapper->getName() << " has " << props.size() << " properties: " << std::endl;
        	for (unsigned int i=0; i<props.size(); i++)
        	{
        		std::cout << "  prop: " << props[i] << std::endl;
        	}

        	std::string serializerName = "Num";
        	osgDB::BaseSerializer* serializer = wrapper->getSerializer(serializerName);
        	if ( serializer )
        		std::cout << "got serializer: " << serializer->getName() << std::endl;
        	else
        		std::cout << "oops. Couldn't find serializer for '"<<serializerName<<"'" << std::endl;




        }
        else
        {
        	std::cout << "oops. Couldn't find wrapper for spinframework::myshape" << std::endl;
        	return 1;
        }
    }


    // test osgreflection:
    ClassInfo* myshapeInfo = ReflectionManager::instance()->getClassInfo("spinframework::myshape");

    // try to create a new object:
    ClassInstance* newShape = myshapeInfo->createInstance();

    // try to set the 'Num':
    //Method* myshapeNumMethod = myshapeInfo->getMethod("Num");
    //myshapeNumMethod->set( newShape, 4 );

    // another way:
    /*
    osgDB::BaseSerializer* serializer = wrapper->getSerializer("Num");
    if ( serializer )
    {
		ReflectionManager::instance()->getOutputStream() << 4;
		serializer->read( ReflectionManager::instance()->getInputStream(), *(newShape->getObject()) );
    } else std::cout << "ERROR" << std::endl;
	*/

    // try to set the 'Note' string:
    //Method* myshapeNoteMethod = myshapeInfo->getMethod("Note");
    //myshapeNoteMethod->set( newShape, std::string("foo") );

    // try to set the 'Translation' (osg x,y,z vector):
    //Method* myshapeMethod = myshapeInfo->getMethod("Translation");
    //myshapeMethod->set( newShape, osg::Vec3(0.0,0.0,1.0) );


	// view our scene:
    /*
	osgViewer::Viewer viewer;
	viewer.setSceneData( grp );
    return viewer.run();
	*/
}
