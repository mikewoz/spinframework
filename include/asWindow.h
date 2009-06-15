#ifndef ASWINDOW_H_
#define ASWINDOW_H_

#include <osg/Referenced>
#include <osg/GraphicsContext>

/**
 * \brief For managing OSG viewer windows in the case of multiple outputs.
 * 
 * For example, we can tell the window which screen to display on, and what 
 * resolution to use
 */
class asWindow : public osg::Referenced
{
	
public:
	asWindow(int argid);
	~asWindow();
	
	void update();
	void setResolution(std::string resolutionString);

	int getID() { return id; }
	osg::GraphicsContext::Traits *getGfxTraits() { return gfxTraits.get(); }
	osg::GraphicsContext *getGfxContext() { return gfxContext.get(); }
	
private:
	int id;
	osg::ref_ptr<osg::GraphicsContext::Traits> gfxTraits;
	osg::ref_ptr<osg::GraphicsContext> gfxContext;

};

#endif
