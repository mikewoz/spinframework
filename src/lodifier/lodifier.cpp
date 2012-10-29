#include <osg/Group>
#include <osg/Notify>
#include <osg/Geometry>
#include <osg/ArgumentParser>
#include <osg/ApplicationUsage>
#include <osg/Texture2D>
#include <osg/TextureRectangle>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PagedLOD>
#include <osg/ProxyNode>

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

#define DEBUG_VIEW


#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define CLAMP(x, l, h) (((x) > (h)) ? (h) : (((x) < (l)) ? (l) : (x)))

std::string g_outputDir = "";
std::string g_extension = ".osg";
int g_tab = 0;
float g_lodScale = 1.0f;
float g_sceneRadius = -1;

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

double logb( double b, double x )
{
    return log(x)/log(b);
}


typedef std::vector< osg::ref_ptr<osg::StateSet> > StateSetList;

/*******************************************************************************/


bool lodifyImage( osg::Image* image, std::deque<osg::Image*>& imgLOD )
{

    if ( !image ) return false;

    osg::ref_ptr<osg::Image> img = image;

    int s = img->s();
    int t = img->t();

    if ( !s || !t || !img->data() ) {
        indent(); printf( "image not loaded!  loading it.\n" );
        std::string fileName = img->getFileName();
        img = osgDB::readImageFile( fileName );
    }

    //indent(); printf("img file name: %s\n", img->getFileName().c_str() );
    indent(); printf("img is %i x %i\n",img->s(), img->t() );

    std::string ext = osgDB::getFileExtensionIncludingDot( img->getFileName() );
    std::string prefix = osgDB::getNameLessExtension( osgDB::getSimpleFileName(img->getFileName()) );
    //prefix = "balls";
    // prefix = g_outputDir + prefix;
    //osg::Image* tmp = new osg::Image( *img );

    osg::ref_ptr<osg::Image> tmp = new osg::Image( *img );

    indent(); printf("ensureValidSizeForTexturing...\n");
    tmp->ensureValidSizeForTexturing( 4096 );
    indent(); printf("ensureValidSizeForTexturing... done\n");
    s = tmp->s();
    t = tmp->t();

    tmp->setFileName( prefix + boost::lexical_cast<std::string>(s) + "x" + boost::lexical_cast<std::string>(t) + ext );
    osgDB::writeImageFile( *tmp, g_outputDir + tmp->getFileName() );

#ifdef DEBUG_VIEW
    //f ( tmp->r() == 1 ) {
    bool grayscale = false;
    if ( osg::Image::computeNumComponents(tmp->getPixelFormat()) == 1 ) {
        indent(); printf("grayscale!\n");
        grayscale = true;
    }
    int nblod = logb( 2, MAX(s, t) ) - 2;
    indent(); printf( "nblod = %i\n", nblod);
    int blood = nblod;
#endif



    osg::Image* dummy = new osg::Image();
    dummy->setImage( tmp->s(), tmp->t(), tmp->r(),
                     tmp->getInternalTextureFormat(),
                     tmp->getPixelFormat(), tmp->getDataType(),
                     0, tmp->getAllocationMode() );
    dummy->setFileName( tmp->getFileName() );
    imgLOD.push_front( dummy );
    indent(); printf("tmp   is %i x %i\n", tmp->s(), tmp->t() );
    indent(); printf("dummy is %i x %i\n", dummy->s(), dummy->t() );

#ifdef DEBUG_VIEW
    tmp.release();
#endif
    //imgLOD.push_front( tmp );




    while ( s > 2 && t > 2 ) {
        s /= 2;
        t /= 2;

#ifdef DEBUG_VIEW
        indent(); printf("NEW! lod image %i x %i\n", s, t );
        tmp = new osg::Image( *img );
#endif

        tmp->scaleImage( s, t, tmp->r() );
        tmp->setFileName( prefix + boost::lexical_cast<std::string>(s) + "x" + boost::lexical_cast<std::string>(t) + ext );

#ifdef DEBUG_VIEW
        if ( !grayscale ) {
            if ( nblod ) {

                double w = (double)nblod / (double)blood;
                indent(); printf("make it %f blue green!\n", w );
                for ( int i = 0; i < tmp->s(); i++ ) {
                    for ( int j = 0; j < tmp->t(); j++ ) {
                        //tmp->data(i,j)[0] = 0;
                        tmp->data(i,j)[1] = (unsigned char)( (1.0-w)*255.0 + w*tmp->data(i,j)[1] );
                        tmp->data(i,j)[2] = (unsigned char)( w*255.0 + (1.0-w)*tmp->data(i,j)[2] );
                    }
                }

            } else {
                indent(); printf("make it red!\n" );
                for ( int i = 0; i < tmp->s(); i++ ) {
                    for ( int j = 0; j < tmp->t(); j++ ) {
                        tmp->data(i,j)[0] = 255;
                        //tmp->data(i,j)[1] = 0;
                        //tmp->data(i,j)[2] = 0;
                    }
                }
            }
        }
        nblod--;
#endif

        osgDB::writeImageFile( *tmp, g_outputDir + tmp->getFileName() );
        dummy = new osg::Image();
        dummy->setImage( tmp->s(), tmp->t(), tmp->r(),
                         tmp->getInternalTextureFormat(),
                         tmp->getPixelFormat(), tmp->getDataType(),
                         0, tmp->getAllocationMode() );
        dummy->setFileName( tmp->getFileName() );
        imgLOD.push_front( dummy );
        indent(); printf("tmp   is %i x %i\n", tmp->s(), tmp->t() );
        indent(); printf("dummy is %i x %i\n", dummy->s(), dummy->t() );
#ifdef DEBUG_VIEW
        tmp.release();
#endif
        //tmp.release();
        // imgLOD.push_front( tmp );
        printf("\n");
    }

    if ( image != img.get() ) {
        //delete( img );
        indent(); printf( "freeing temp image...\n" );
        img.release();
        indent(); printf( "freeing temp image... done\n" );
    }

#ifndef DEBUG_VIEW
    tmp.release();
#endif // DEBUG_VIEW


    indent(); printf("image lodifying done\n");
    return true;
}

/*******************************************************************************/

bool isTextureAttribute( osg::StateSet* stateset )
{
    if ( !stateset ) return false;

    osg::StateSet::TextureAttributeList tal = stateset->getTextureAttributeList();
    if ( tal.size() == 0 ) return false;

    for( size_t i = 0; i > tal.size(); i++ ) {

        osg::StateSet::AttributeList& al = tal[i];
        for(osg::StateSet::AttributeList::iterator itr=al.begin();
            itr != al.end(); ++itr) {
            if ( itr->second.first ) {

            }
        }

    }





    /*



    if ( tal.size() == 0 ) return false;

    for ( size_t i = 0; i < tal.size(); i++ ) {

    }

    osg::StateAttribute* attr = stateset->getTextureAttribute(0,osg::StateAttribute::TEXTURE);
    if (attr) {
        osg::Texture2D* texture2D = dynamic_cast<osg::Texture2D*>(attr);
        if (texture2D)  return true;

        osg::TextureRectangle* textureRec = dynamic_cast<osg::TextureRectangle*>(attr);
        if (textureRec) return true;
        }*/
    return false;
}

/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/

class ConvertToPageLODVistor : public osg::NodeVisitor
{
public:
    ConvertToPageLODVistor( bool doTex, bool doGeom, osg::Group* root ): // const std::string& basename,
        osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
        //   _basename( basename ),
        _doTextures( doTex ),
        _doGeometry( doGeom ),
        _root( root )
    {
        _nbPlods = 0;
        _nbGroups = 0;
        //_baseGroup = 0;
    }

    virtual ~ConvertToPageLODVistor()
    {
    }

    /***************************************************************************/
    virtual void apply(osg::Transform& t)
    {
        indent(g_tab); printf( "traversing transform ...\n");
        g_tab += 4;
        traverse( t );
        g_tab -= 4;
        indent(g_tab); printf( "traversing transform done\n");
    }
    /***************************************************************************/
    virtual void apply(osg::Group& g)
    {
        indent(g_tab); printf( "traversing group ...\n");
        g_tab += 4;
        traverse(g);

        if ( _root == &g ) {
            indent(); printf( "is root!\n" );
            g_tab -= 4;
            indent(); printf( "traversing ROOT done\n" );
            return;
        }

        if ( g.getNumChildren() == 0 ) {
            indent(); printf( "empty group!!!!\n" );
            g_tab -= 4;
            indent(g_tab); printf( "traversing group done\n");
            return;
        }

        if ( g.getNumChildren() == 1 ) {
            //osg::PagedLOD* plod = dynamic_cast<osg::PagedLOD*>( g.getChild(0) );
            osg::Node* n = g.getChild(0);
            //if ( plod ) {
            if ( n ) {
                indent(); printf( "useless group!!!!\n" );
                //////osg::ref_ptr<osg::Group> gref( &g );
                osg::Node::ParentList pl = g.getParents();
                for ( size_t i = 0; i < pl.size(); i++ ) {
                    //pl[i]->replaceChild( &g, plod );
                    pl[i]->replaceChild( &g, n );
                }
                g_tab -= 4;
                indent(g_tab); printf( "traversing group done\n");
                return;
            }
        }


        osg::PagedLOD* plod = new osg::PagedLOD();
        plod->setRangeMode( osg::LOD::PIXEL_SIZE_ON_SCREEN ); // here too?
        //plod->setRangeMode( osg::LOD:: DISTANCE_FROM_EYE_POINT );
        const osg::BoundingSphere& bs = g.getBound();
        plod->setCenter( bs.center() );
        plod->setRadius( bs.radius() );

        bool foundLOD = false;
        osg::Group* lowDef = new osg::Group();
        for ( size_t i = 0; i < g.getNumChildren(); i++ ) {
            osg::LOD* lod = dynamic_cast<osg::LOD*>( g.getChild(i) );
            if ( lod ) {
                lowDef->addChild( lod->getChild(0) );
                /////plod->setRadius( lod->getChild(0)->getBound().radius() ); not with PIXEL_SIZE_ON_SCREEN
                indent(); printf("added pagedlod child 0 to lowdef group\n");
                foundLOD = true;
            } else {
                lowDef->addChild( g.getChild(i) );
                indent(); printf("added something to lowdef group ******************************************\n");
            }
        }

        if ( foundLOD ) {

#ifdef DEBUG_VIEW_SPHERES
            osg::ShapeDrawable* sd = new osg::ShapeDrawable( new osg::Sphere( bs.center(), bs.radius() ) );
            sd->setColor( osg::Vec4(1, 0, 1, 0.3) );
            osg::Geode* sg = new osg::Geode();
            sg->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);
            sg->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            sg->addDrawable( sd );
            lowDef->addChild( sg );
#endif

            float cutoff = bs.radius() / g_sceneRadius;// _root->getBound().radius();
            indent(); printf( "cutoff = %f\n", cutoff );

            //plod->addChild( lowDef, 70, 1000, "" );
            plod->addChild( lowDef, 0, cutoff * g_lodScale, "" );

            osg::ref_ptr<osg::Group> gref( &g );

            osg::Node::ParentList pl = g.getParents();
            for ( size_t i = 0; i < pl.size(); i++ ) {
                pl[i]->replaceChild( &g, plod );
            }

            indent(); printf("asdf %i\n", g.getNumChildren() );
            std::string hidefFile = "group_" + boost::lexical_cast<std::string>(_nbGroups) + g_extension;
            //plod->addChild( &g, 0, 70, hidefFile ); //high lod
            plod->addChild( &g, cutoff * g_lodScale, 1000.0f * g_lodScale, hidefFile ); //high lod.. in pixels

            indent(); printf( "GROUP: added LOD range %f to %f\n", 0.0, cutoff * g_lodScale );
            indent(); printf( "GROUP: added LOD range %f to %f\n", cutoff * g_lodScale, 1000.0 * g_lodScale );


            osgDB::writeNodeFile( g, g_outputDir + hidefFile );
            _nbGroups++;

        } else {

            osg::Node::ParentList pl = g.getParents();
            for ( size_t i = 0; i < pl.size(); i++ ) {
                pl[i]->replaceChild( &g, lowDef );
            }
        }



        //osgDB::writeNodeFile( *plod, "grouplod.osg" );
        //abort();
        ///_baseGroup = plod;

        g_tab -= 4;
        indent(g_tab); printf( "traversing group done\n");
    }


    /***************************************************************************/
    typedef std::deque<osg::Image*> ImageLOD;
    typedef std::map<osg::StateSet*, ImageLOD> ImageLodList;

    virtual void apply(osg::Geode& g)
    {
        indent(g_tab); printf( "traversing geode ...\n");
        g_tab += 4;
        // traverse(g);


        ImageLOD imgLOD;
        std::deque<osg::Geode*> geoLOD;
        osg::Geode* gtmp;// = new osg::Geode( g );
        //geoLOD.push_back( gtmp );

        if ( _doGeometry ) {
            indent(); printf("not doing geometry yet\n");
        }


        if ( _doTextures ) {
            StateSetList ssl;
            osg::StateSet* ss;

            ss = g.getStateSet();

            if ( ss && isTextureAttribute(ss) ) ssl.push_back( ss );

            for( size_t i = 0; i < g.getNumDrawables(); i++ ) {
                osg::StateSet *ss = g.getDrawable(i)->getStateSet();
                //if ( isTextureAttribute(ss) )
                if ( ss->getNumTextureAttributeLists() > 0 )
                    ssl.push_back( ss );

            }

            int nbTextures = ssl.size();
            if ( !nbTextures ) {
                g_tab -= 4;
                indent(g_tab); printf( "NO TEXTURES\n");
                indent(g_tab); printf( "traversing geode done\n");
                return;
            }

            for (StateSetList::iterator itr=ssl.begin(); itr!=ssl.end(); ++itr) {
                // check if this stateset has a special texture
                indent(); printf( "getNumTextureAttributeLists = %u\n", (*itr)->getNumTextureAttributeLists() );

                osg::StateAttribute *attr;
                size_t texunit = 0;
                for ( texunit = 0; texunit < (*itr)->getNumTextureAttributeLists(); texunit++ ) {
                    attr = (*itr)->getTextureAttribute( texunit, osg::StateAttribute::TEXTURE );
                    if (attr) break;
                }

                //osg::StateAttribute *attr = (*itr)->getTextureAttribute(0,osg::StateAttribute::TEXTURE);
                if (attr) {
                    std::string imageFile = attr->asTexture()->getImage(0)->getFileName();
                    //indent(g_tab); printf( "LODIFING IMAGE [%s]\n", imageFile.c_str() );
                    //std::deque<osg::Image*> imgLOD;

                    lodifyImage( attr->asTexture()->getImage(0), imgLOD );

                    (*itr)->removeAssociatedTextureModes( texunit, attr );
                    (*itr)->removeTextureAttribute( texunit, attr );//osg::StateAttribute::TEXTURE );


                    for ( size_t i = 0; i < imgLOD.size(); i++ ) {
                        gtmp = new osg::Geode( g );
                        //ss = gtmp->getOrCreateStateSet();

                        osg::Texture2D* tex = new osg::Texture2D;
                        //tex->setDataVariance(osg::Object::DYNAMIC); // protect from being optimized away as static state.
                        tex->setImage( imgLOD[i] );

                        //printf("ss.getName() = %s\n", g.getStateSet()->getName().c_str() );
                        ss = new osg::StateSet( *(*itr) );//*g.getStateSet() );
                        ss->setTextureAttributeAndModes( texunit, tex, osg::StateAttribute::ON );
                        gtmp->setStateSet( ss );

                        //if ( !_doGeometry )
                        /////osgDB::writeNodeFile( *gtmp, "blarg" + boost::lexical_cast<std::string>(i) + ".osg" );
                        geoLOD.push_back( gtmp );
                    }

                    break; // only get one !!!!!!!!!!!!!!!
                } else {
                    indent(); printf("Texture attribute fetch FAILED!\n");
                }
            }
        }

        osg::PagedLOD* plod = new osg::PagedLOD();
        plod->setRangeMode( osg::LOD::PIXEL_SIZE_ON_SCREEN );

        float f = MAX( imgLOD[imgLOD.size()-1]->s(), imgLOD[imgLOD.size()-1]->t() );// / g_lodScale; uhhhh/// ( 2.0 * g.getBound().radius() );
        float minSize = 0;
        float cutoff = 2.0 * sqrt(2.0); //2.0 * g.getBound().radius() / g_sceneRadius;// _root->getBound().radius();
        cutoff /= g_lodScale;
        float maxSize = cutoff;
        float incr = (1.0f - cutoff) / pow(2.0f, geoLOD.size()-3.0f);
        indent(); printf( "cutoff = %f, incr = %f\n", cutoff, incr );

        for ( size_t i = 0; i < geoLOD.size(); i++ ) {
            //plod->addChild( geoLOD[i], minSize * g_lodScale, maxSize * g_lodScale,
            plod->addChild( geoLOD[i], minSize * f, maxSize * f,
                            "geode_" + boost::lexical_cast<std::string>(_nbPlods) +
                            "_" + boost::lexical_cast<std::string>(i) + g_extension );

            indent(); printf( "GEODE: added LOD range %f to %f\n", minSize * f, maxSize * f );
            fflush(stdout);
            osgDB::writeNodeFile( *plod->getChild(i), g_outputDir + plod->getFileName(i) );
            //indent(); printf( "GEODE: added LOD range %f to %f\n", minSize * g_lodScale, maxSize * g_lodScale );

            minSize = maxSize;
            maxSize = cutoff + incr*pow(2,i);

        }

        indent(); printf("plod->getNumChildren = %u\n", plod->getNumChildren() );

        plod->setRange( plod->getNumChildren()-1, plod->getMinRange(plod->getNumChildren()-1), 1000.0f * g_lodScale );

        const osg::BoundingSphere& bs = geoLOD[geoLOD.size()-1]->getBound();
        plod->setCenter( bs.center() );
        plod->setRadius( bs.radius() );

        _nbPlods++;

        osg::Node::ParentList pl = g.getParents();
        for ( size_t i = 0; i < pl.size(); i++ ) {
            pl[i]->replaceChild( &g, plod );
        }

        g_tab -= 4;
        indent(g_tab); printf( "traversing geode done\n");
    }

    /***************************************************************************/

    virtual void apply(osg::PagedLOD& plod)
    {
        traverse(plod);
    }
    virtual void apply(osg::LOD& lod)
    {
        traverse(lod);
    }

    virtual void apply(osg::Node& n)
    {
        indent(g_tab); printf( "traversing a node ...FLKJADSFLKJSAGDLJADSLGJASDGJADSFLGJADSLGJADSFLJGAD\n");
        g_tab += 4;
        traverse(n);
        g_tab -= 4;
        indent(g_tab); printf( "traversing node done\n");
    }
    /***************************************************************************/


    bool _doTextures;
    bool _doGeometry;

    osg::Group* _root;
    int _nbPlods;
    int _nbGroups;

};


/*******************************************************************************/


class ReadImageCallback : public osgDB::Registry::ReadFileCallback
{
public:
    virtual osgDB::ReaderWriter::ReadResult readImage(const std::string& fileName,
                                                     const osgDB::ReaderWriter::Options* options)
    {
        std::cout<<"before readImage : " << fileName <<std::endl;
        // note when calling the Registry to do the read you have to call readNodeImplementation NOT readNode, as this will
        // cause on infinite recusive loop.
        osgDB::ReaderWriter::ReadResult result = osgDB::Registry::instance()->readImageImplementation(fileName,options);
        std::cout<<"after readImage"<<std::endl;

        if ( !osgDB::fileExists( fileName ) )
            return osgDB::ReaderWriter::ReadResult( osgDB::ReaderWriter::ReadResult::FILE_NOT_FOUND );

        osg::Image* img = new osg::Image();
        img->setFileName( fileName );

        return osgDB::ReaderWriter::ReadResult( img, osgDB::ReaderWriter::ReadResult::FILE_LOADED ); // that's a lie!

    }
};

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

    std::string inputFile = "";
    std::string outputFile = "";
    bool display = false;
    while (arguments.read("-s",g_lodScale)) {}
    while (arguments.read("-r",g_sceneRadius)) {}
    while (arguments.read("-i",inputFile)) {}
    while (arguments.read("-o",outputFile)) {}
    while (arguments.read("-d")) { display = true;}



    if ( outputFile.empty() ) outputFile = "output.osgt";
    std::string outbase( osgDB::getNameLessExtension(outputFile) );
    g_extension = osgDB::getFileExtensionIncludingDot( outputFile );
    g_outputDir = outbase + "_data/";

    if ( !osgDB::makeDirectory( g_outputDir ) ) {
        printf( "could not create directory %s\n", g_outputDir.c_str() );
    }

    //#define TEST
    //#ifdef TEST
    osgDB::Registry::instance()->setReadFileCallback(new ReadImageCallback());

    osgDB::Options* opt = new osgDB::Options();
    opt->setObjectCacheHint( osgDB::Options::CACHE_NONE );


    std::vector< osg::ref_ptr<osg::Node> > nodeList;
    for( int pos = 1; pos < arguments.argc(); pos++ ) {
        if ( !arguments.isOption(pos) ) {
            // not an option so assume string is a filename.
            osg::Node *node = osgDB::readNodeFile( arguments[pos], opt );
            if( node )   {
                osgDB::getDataFilePathList().push_back( osgDB::getFilePath( arguments[pos]) );
                if ( node->getName().empty() ) node->setName( arguments[pos] );
                nodeList.push_back( node );
            }
        }
    }

    printf( "read all node files\n" );
    osgDB::Registry::instance()->setReadFileCallback( NULL );

    osg::ref_ptr<osg::Group> group = 0;

    if ( nodeList.size() == 0 ) {
        printf( "no model loaded.\n");
        return 0;
    } else if ( nodeList.size() == 1 ) {

        if ( !nodeList[0]->asGroup() ) {
            group = new osg::Group();
            group->addChild( nodeList[0].get() );
        } else {
            group = nodeList[0]->asGroup();
        }


    } else {
        //for(NodeList::iterator itr=nodeList.begin();itr!=nodeList.end();++itr)
        //group->addChild((*itr).get());
        group = new osg::Group();
        for ( size_t i = 0; i < nodeList.size(); i++ )
            group->addChild( nodeList[i].get());
    }

    /*#else // test

    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile( inputFile );

    if (!model) {
        printf("No model loaded.\n");
        return 1;
    }
    //osgDB::Registry::instance()->getDataFilePathList().push_back( osgDB::getFilePath(inputFile) );
    osgDB::getDataFilePathList().push_back( osgDB::getFilePath(inputFile) ); // w/FileUtils

    osg::Group* group = model->asGroup();

    if ( !group ) {
        printf("model is not a group.  putting it in a group.\n");
        group = new osg::Group();
        group->addChild( model );
    } else {
        printf( "model is a group!\n" );
    }

#endif // TEST
    */
    if ( g_sceneRadius <= 0.0f ) g_sceneRadius = group->getBound().radius();

    printf( "scene radius = %f\n", g_sceneRadius );
    //exit(0);

    ConvertToPageLODVistor converter(true, false, group); // ewww
    group->accept(converter);

    osgDB::writeNodeFile( *group, g_outputDir + "base" + g_extension );

    osg::ProxyNode* pn = new osg::ProxyNode();
    pn->addChild( group, g_outputDir + "base" + g_extension  );
    pn->setRadius( group->getBound().radius() );
    pn->setCenter( group->getBound().center() );
    pn->setLoadingExternalReferenceMode( osg::ProxyNode::LOAD_IMMEDIATELY );

    //osg::ref_ptr<osgDB::Options> options = new osgDB::Options;
    //options->setDatabasePath( "" );
    //pn->setDatabaseOptions( options.get() );
    //pn->setDatabasePath(".");

    osgDB::writeNodeFile( *pn, outputFile );


    if ( !display ) return 0;
    /********************************************/

    osgViewer::Viewer viewer;

    osg::Camera* cam = viewer.getCamera();
    if (g_lodScale == 1.0f) cam->setLODScale( 1920.0 );

    //cam->addChild( pn );
    viewer.setSceneData( pn );

    viewer.realize();

    viewer.run();


}
