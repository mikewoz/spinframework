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

	sphere->setNote("test string");

	osg::Group *grp = new osg::Group();
	grp->addChild(sphere);
	grp->addChild(box);


    osgDB::ObjectWrapperManager* wrapperManager = osgDB::Registry::instance()->getObjectWrapperManager();
    if ( wrapperManager )
    {
        osgDB::ObjectWrapper* wrapper = wrapperManager->findWrapper("spinframework::myshape");
        if ( wrapper )
        {
        	std::cout << "got wrapper: " << wrapper->getName() << std::endl;
        	osgDB::BaseSerializer* serializer = wrapper->getSerializer("note");
        	if ( serializer )
        	{
        		std::cout << "got serializer for note: " << serializer->getName() << std::endl;
        	}
        	else
        	{
        		std::cout << "oops. Couldn't find serializer for 'note'" << std::endl;
        	}

        }
        else
        {
        	std::cout << "oops. Couldn't find wrapper for " << name << std::endl;
        }
    }


    // test osgreflection:
    ClassInfo* myshapeInfo = ReflectionManager::instance()->getClassInfo("spinframework::myshape");
    Method* myshapeMethod = myshapeInfo->getMethod("note");
    ClassInstance* newShape = myshapeInfo->createInstance();
    myshapeMethod->set( newShape, "foo" );



	/*
	osgViewer::Viewer viewer;
	viewer.setSceneData( grp );
    return viewer.run();
	*/
}
