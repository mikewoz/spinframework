#include <iostream>
#include <string>

#include <osg/GraphicsContext>


#include "asUtil.h"
#include "asWindow.h"

using namespace std;

asWindow::asWindow(int argid)
{	
	id = argid;
	
	// create a GraphicsContext::Traits for this window and initialize with some defaults:
	gfxTraits = new osg::GraphicsContext::Traits;
	
	gfxTraits->x = 50;
	gfxTraits->y = 50;
	gfxTraits->width = 320;
	gfxTraits->height = 240;
	gfxTraits->windowDecoration = true;
	gfxTraits->doubleBuffer = true;
	gfxTraits->useCursor = true;
	gfxTraits->supportsResize = true;
	gfxTraits->sharedContext = 0;
	
	gfxTraits->displayNum = 0;
	gfxTraits->screenNum = 0;
	
	
	gfxContext = osg::GraphicsContext::createGraphicsContext(gfxTraits.get());
	if (gfxContext.valid())
	{
		gfxContext->setClearColor(osg::Vec4f(0.0f,0.0f,0.0f,1.0f));
		gfxContext->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//std::cout << "created GraphicsContext for window: " << id << std::endl;
	} else {
		std::cout << "ERROR: (asWindow) Could not create GraphicsContext." << std::endl;
	}
	
}

asWindow::~asWindow()
{
	
}

// *****************************************************************************



void asWindow::setResolution(std::string resolutionString)
{
	// The resolutionString comes in the format WIDTHxHEIGHT. If values are in
	// the range [0-1], then these are assumed to be ratios of the screen.
	// Otherwise, they are absolute pixel values.
	

	unsigned int maxWidth, maxHeight;
	
	resolutionString.replace(resolutionString.find("x"), 1, " ");
	std::vector<float> v = floatsFromString(resolutionString);

	if (v.size()==2)
	{
		// first check if the wsi is valid:
		osg::GraphicsContext::WindowingSystemInterface* wsi = gfxContext->getWindowingSystemInterface();
		//osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
		if (!wsi)
		{
			std::cout << "ERROR: No WindowSystemInterface available. Cannot set resoution." << std::endl;
			return;
		}

		if ((v[0]>1) && (v[1]>1))
		{
			// if the values are above 1, then just use them as absolute pixel
			// values:
			gfxTraits->width = (int) v[0];
			gfxTraits->height = (int) v[1];
			
		} else if ((v[0]>0) && (v[1]>0)) {
			
			// otherwise, consider them as ratios of the max screen resolution:
	
			wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(gfxTraits->screenNum), maxWidth, maxHeight);

			gfxTraits->width =  (int) (maxWidth * v[0]);
			gfxTraits->height = (int) (maxHeight * v[1]);
		}

	
	
	
	} else {
		std::cout << "ERROR: Could not set resolution. The value provided is invalid: " << resolutionString << std::endl;
		return;
	}
	
}
