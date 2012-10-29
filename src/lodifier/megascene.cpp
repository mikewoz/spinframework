#include <osg/Group>
#include <osg/Material>
#include <osg/LightModel>
#include <osg/Geometry>
#include <osg/ArgumentParser>
#include <osg/ApplicationUsage>
#include <osg/Texture2D>
#include <osg/PolygonMode>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PagedLOD>
#include <osg/MatrixTransform>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgViewer/Viewer>
#include <osgUtil/Optimizer>

#include <boost/lexical_cast.hpp>

#include <stdio.h>
#include <stdarg.h>

#include <iostream>
#include <sstream>

#define MAX_BOX 100
#define MIN_BOX -MAX_BOX

int g_tab = 0;

void print_with_indent( int indent, char * s, ... )
{
    va_list argptr;
    va_start(argptr, s);
    //printf( "%*s" "%s", indent, " ", s, argptr );
    printf( "%*s", indent, " " );
    printf( s, argptr );
    //vfprintf(stderr, format, argptr);
    va_end(argptr);


}

void indent( int i )
{
    printf( "%*s", i, " " );
}

void indent()
{
    printf( "%*s", g_tab, " " );
}

/*******************************************************************************/
osg::Texture2D* createTexFromImg( osg::Image* img )
{

  osg::Texture2D* tex = new osg::Texture2D;

  tex->setImage( img );
  tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
  tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
  tex->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_BORDER);
  tex->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_BORDER);

  return tex;

}


double modnar( double min = 0.0, double max = 1.0 )
{
    return (((double)rand() / (double)RAND_MAX) + min ) * (max-min);
}



/*******************************************************************************/

int main( int argc, char **argv )
{

    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    // set up the usage document, in case we need to print out how to use this program.
    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" creates a hierarchy of files for paging which can be later loaded by viewers.");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename ...");
    arguments.getApplicationUsage()->addCommandLineOption("-h or --help","Display this information");
    arguments.getApplicationUsage()->addCommandLineOption("-s","set the LOD Scale factor.  should be the intended display resolution");
    arguments.getApplicationUsage()->addCommandLineOption("-r","scene bounding sphere radius [optional]");
    arguments.getApplicationUsage()->addCommandLineOption("-i","set the input file");
    arguments.getApplicationUsage()->addCommandLineOption("-o","set the output file (defaults to output.ive)");
    arguments.getApplicationUsage()->addCommandLineOption("--makeAllChildrenPaged","Force all children of LOD to be written out as external PagedLOD children");

    // if user request help write it out to cout.
    if (arguments.read("-h") || arguments.read("--help")) {
        arguments.getApplicationUsage()->write(std::cout);
        return 1;
    }
    int nb = 100;
    bool display = false;
    std::string dummyImagePath = "indian.jpg";
    while (arguments.read("-d")) { display = true;}
    while (arguments.read("-nb", nb)) {}
    while (arguments.read("-img", dummyImagePath)) {}

    osg::Image* dummyImg = osgDB::readImageFile( dummyImagePath );
    if ( !dummyImg ) {
        printf("no image\n");
        exit(0);
    }

    printf( "using [%s] as dummy image\n", dummyImagePath.c_str() );

    srand ( time(NULL) );
    char buf[100];
    osg::Image* tmpImg = 0;

    osg::Group* root = new osg::Group();

    /* if ( !osgDB::makeDirectory( "images" ) ) {
        printf( "could not create directory images\n");
    }

    */

    osg::Material* mat = new osg::Material;
    mat->setColorMode(osg::Material::DIFFUSE);
    mat->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 1, 1, 1));
    mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 1, 1, 1));
    mat->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 1, 1, 1));
    mat->setShininess(osg::Material::FRONT_AND_BACK, 128.0f);

    osg::LightModel* ltModel = new osg::LightModel;
    ltModel->setTwoSided(true);

    for ( int i = 0; i < nb; i++ ) {

        osg::Geode* gd = new osg::Geode();
        osg::Geometry* gm = new osg::Geometry();

        osg::Vec3Array* vertices = new osg::Vec3Array;
        osg::Vec2Array* texcoords = new osg::Vec2Array;
        osg::Vec4Array* colors = new osg::Vec4Array;

        (*vertices).push_back( osg::Vec3(0, 0, 0) );
        (*vertices).push_back( osg::Vec3(1, 0, 0) );
        (*vertices).push_back( osg::Vec3(1, 1, 0) );
        (*vertices).push_back( osg::Vec3(0, 1, 0) );

        (*texcoords).push_back( osg::Vec2(0, 0) );
        (*texcoords).push_back( osg::Vec2(1, 0) );
        (*texcoords).push_back( osg::Vec2(1, 1) );
        (*texcoords).push_back( osg::Vec2(0, 1) );

        (*colors).push_back( osg::Vec4(1,1,1,1) );

        gm->setVertexArray(vertices);
        gm->setTexCoordArray(0,texcoords);
        gm->setColorArray(colors);
        gm->setColorBinding(osg::Geometry::BIND_OVERALL);

        osg::ref_ptr<osg::DrawArrays> da=new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4);
        gm->addPrimitiveSet(da.get());

        gd->addDrawable( gm );

        tmpImg = new osg::Image( *dummyImg );
        sprintf( buf,  "images/dummy%04i.png", i );
        tmpImg->setFileName( buf );
        //////osgDB::writeImageFile( *tmpImg, buf);

        osg::Texture2D *tex = createTexFromImg( tmpImg );
        osg::StateSet* ss = gd->getOrCreateStateSet();
        ss->setTextureAttributeAndModes( 0, tex, osg::StateAttribute::ON );

        ss->setAttributeAndModes( mat, osg::StateAttribute::ON );
        ss->setAttribute( ltModel );
        ss->setMode( GL_CULL_FACE, osg::StateAttribute::OFF );
        ss->setMode( GL_LIGHTING, osg::StateAttribute::ON );

        // osg::PolygonMode* pm = new osg::PolygonMode;
        // pm->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL );
        // ss->setAttributeAndModes( pm, osg::StateAttribute::ON );

        osg::MatrixTransform* mt = new osg::MatrixTransform();
        osg::Vec3 axis( modnar(-1,1), modnar(-1,1), modnar(-1,1) );
        axis.normalize();
        mt->setMatrix( osg::Matrix::rotate( modnar(0, 2*M_PI), axis ) *
                       osg::Matrix::translate( modnar(MIN_BOX, MAX_BOX), modnar(MIN_BOX, MAX_BOX), modnar(MIN_BOX, MAX_BOX) ) );

        mt->addChild( gd );
        root->addChild( mt );

        printf( "%4i of %i done       \r", i, nb );
        fflush( stdout );
    }

    printf("saving... (this may take a while)\n");

    osgDB::writeNodeFile( *root, "megascene.dae" );
    osgDB::writeNodeFile( *root, "megascene.osgt" );

    if ( !display ) return 0;
    /********************************************/

    osgViewer::Viewer viewer;

    viewer.setSceneData( root );

    viewer.realize();

    viewer.run();


}
