// -----------------------------------------------------------------------------
// |    ___  ___  _  _ _     ___                                        _      |
// |   / __>| . \| || \ |   | __>_ _  ___ ._ _ _  ___  _ _ _  ___  _ _ | |__   |
// |   \__ \|  _/| ||   |   | _>| '_><_> || ' ' |/ ._>| | | |/ . \| '_>| / /   |
// |   <___/|_|  |_||_\_|   |_| |_|  <___||_|_|_|\___.|__/_/ \___/|_|  |_\_\   |
// |                                                                           |
// |---------------------------------------------------------------------------|
//
// http://spinframework.sourceforge.net
// Copyright (C) 2009 Mike Wozniewski, Zack Settel
//
// Developed/Maintained by:
//    Mike Wozniewski (http://www.mikewoz.com)
//    Zack Settel (http://www.sheefa.net/zack)
// 
// Principle Partners:
//    Shared Reality Lab, McGill University (http://www.cim.mcgill.ca/sre)
//    La Societe des Arts Technologiques (http://www.sat.qc.ca)
//
// Funding by:
//    NSERC/Canada Council for the Arts - New Media Initiative
//    Heritage Canada
//    Ministere du Developpement economique, de l'Innovation et de l'Exportation
//
// -----------------------------------------------------------------------------
//  This file is part of the SPIN Framework.
//
//  SPIN Framework is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  SPIN Framework is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the Lesser GNU General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#include "wxVessRenderer.h"

//(*InternalHeaders(wxVessRenderer)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
//*)

#include <wx/image.h>
//#include "wx/cursor.h"
#include "wx/glcanvas.h"
#include <wx/dcclient.h>

#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers>

#include "vessThreads.h"
extern vessThread *vess;
extern pthread_mutex_t pthreadLock;

//(*IdInit(wxVessRenderer)
const long wxVessRenderer::vessRenderer_grid = wxNewId();
const long wxVessRenderer::vessRenderer_trackNode = wxNewId();
const long wxVessRenderer::ID_TOOLBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(wxVessRenderer,wxFrame)
	//(*EventTable(wxVessRenderer)
	//*)

    EVT_IDLE(wxVessRenderer::OnIdle)

END_EVENT_TABLE()

wxVessRenderer::wxVessRenderer(wxWindow* parent,wxWindowID id,const wxPoint& pos,const wxSize& size)
{
	//(*Initialize(wxVessRenderer)
	Create(parent, id, _("SPIN :: Renderer"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("id"));
	SetClientSize(wxSize(720,480));
	Move(wxDefaultPosition);
	wxVessRenderer_ToolBar = new wxToolBar(this, ID_TOOLBAR1, wxDefaultPosition, wxDefaultSize, wxTB_HORIZONTAL|wxNO_BORDER, _T("ID_TOOLBAR1"));
	ToolBarItem1 = wxVessRenderer_ToolBar->AddTool(vessRenderer_grid, _("Grid"), wxBitmap(wxImage(_T("../images/grid.gif"))), wxBitmap(wxImage(_T("../images/grid.gif"))), wxITEM_CHECK, _("Enable/Disable Grid"), _("Enable/Disable Grid"));
	ToolBarItem2 = wxVessRenderer_ToolBar->AddTool(vessRenderer_trackNode, _("trackNode"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_GO_TO_PARENT")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_GO_TO_PARENT")),wxART_TOOLBAR), wxITEM_NORMAL, _("Choose a node to track"), _("The camera will attach to this node and show the view from the local perspective. This allows for dynamic camera control via any node in the scene."));
	wxVessRenderer_ToolBar->Realize();
	SetToolBar(wxVessRenderer_ToolBar);
	
	Connect(vessRenderer_grid,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&wxVessRenderer::OnGridToggle);
	//*)


    int *attributes = new int[7];
    attributes[0] = int(WX_GL_DOUBLEBUFFER);
    attributes[1] = WX_GL_RGBA;
    attributes[2] = WX_GL_DEPTH_SIZE;
    attributes[3] = 8;
    attributes[4] = WX_GL_STENCIL_SIZE;
    attributes[5] = 8;
    attributes[6] = 0;


    canvas = new OSGCanvas(this, wxID_ANY, wxDefaultPosition, size, wxSUNKEN_BORDER, wxT("OSGCanvas"), attributes);

    GraphicsWindowWX* gw = new GraphicsWindowWX(canvas);

    canvas->SetGraphicsWindow(gw);



	// *************************************************************************
    // in case of regular osg::Viewer:
/*
    viewer = new osgViewer::Viewer;
    viewer->getCamera()->setGraphicsContext(gw);
    viewer->getCamera()->setViewport(0,0,size.GetWidth(),size.GetHeight());
    viewer->setSceneData(vess->sceneManager->rootNode.get());
    viewer->addEventHandler(new osgViewer::StatsHandler);
    viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
    viewer->setCameraManipulator(new osgGA::TrackballManipulator);
*/


	// *************************************************************************
	// create osgViewer (compositeViewer):
    viewer = new osgViewer::CompositeViewer;
    viewer->setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);

    // create at least one view:
    osgViewer::View *view = new osgViewer::View;
    view->getCamera()->setGraphicsContext(gw);
    view->getCamera()->setViewport(0,0,200,200);//size.GetWidth(),size.GetHeight());
    view->getCamera()->setClearColor(osg::Vec4(0,0,0,0));
    view->setSceneData(vess->sceneManager->rootNode.get());


    // add the view:
    viewer->addView(view);


	// *************************************************************************
	// set up the Manipulator:

	//osgGA::TrackballManipulator *manipulator = new osgGA::TrackballManipulator();
	osgGA::NodeTrackerManipulator *manipulator = new osgGA::NodeTrackerManipulator();


	manipulator->setTrackerMode( osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION );
	//manipulator->setTrackerMode( osgGA::NodeTrackerManipulator::NODE_CENTER_AND_AZIM );
	//manipulator->setRotationMode( osgGA::NodeTrackerManipulator::ELEVATION_AZIM );
	//manipulator->setRotationMode( osgGA::NodeTrackerManipulator::TRACKBALL );

	manipulator->setTrackNode(vess->sceneManager->rootNode.get());


	view->setCameraManipulator(manipulator);


	// *************************************************************************
	// set up some event (GUIEventHandler) handlers:

	//view->addEventHandler(new osgViewer::StatsHandler);

	//view->addEventHandler(new osgViewer::ThreadingHandler);
	//view->addEventHandler(new osgViewer::WindowSizeHandler);

	//view->addEventHandler(new osgViewer::KeyboardHandler);
	//view->addEventHandler(new osgViewer::MouseHandler);
	//view->addEventHandler(new osgViewer::ResizeHandler);

	//view->addEventHandler(new osgViewer::RecordCameraPathHandler);



	//view->setLightingMode(osg::View::NO_LIGHT);
	view->setLightingMode(osg::View::HEADLIGHT);
	//view->setLightingMode(osg::View::SKY_LIGHT);



}

wxVessRenderer::~wxVessRenderer()
{
	//(*Destroy(wxVessRenderer)
	//*)
}

void wxVessRenderer::OnIdle(wxIdleEvent &event)
{
    // We now have to go through all the nodes, and check if we need to update the
    // graph. Note: this cannot be done as a callback in a traversal - dangerous.
    // In the callback, we have simply flagged what needs to be done (eg, set the
    // newParent symbol).

/*
    pthread_mutex_lock(&pthreadLock);
    vess->sceneManager->updateGraph();
    pthread_mutex_unlock(&pthreadLock);
*/


    pthread_mutex_lock(&pthreadLock);
    viewer->frame();
    pthread_mutex_unlock(&pthreadLock);

    #ifdef __WXMAC__
    //canvas->SwapBuffers();
    #endif

    event.RequestMore();
}

// *****************************************************************************

BEGIN_EVENT_TABLE(OSGCanvas, wxGLCanvas)
    EVT_SIZE                (OSGCanvas::OnSize)
    EVT_PAINT               (OSGCanvas::OnPaint)
    EVT_ERASE_BACKGROUND    (OSGCanvas::OnEraseBackground)

    EVT_CHAR                (OSGCanvas::OnChar)
    EVT_KEY_UP              (OSGCanvas::OnKeyUp)

    EVT_ENTER_WINDOW        (OSGCanvas::OnMouseEnter)
    EVT_LEFT_DOWN           (OSGCanvas::OnMouseDown)
    EVT_MIDDLE_DOWN         (OSGCanvas::OnMouseDown)
    EVT_RIGHT_DOWN          (OSGCanvas::OnMouseDown)
    EVT_LEFT_UP             (OSGCanvas::OnMouseUp)
    EVT_MIDDLE_UP           (OSGCanvas::OnMouseUp)
    EVT_RIGHT_UP            (OSGCanvas::OnMouseUp)
    EVT_MOTION              (OSGCanvas::OnMouseMotion)
END_EVENT_TABLE()

OSGCanvas::OSGCanvas(wxWindow *parent, wxWindowID id,
    const wxPoint& pos, const wxSize& size, long style, const wxString& name, int *attributes)
    : wxGLCanvas(parent, id, pos, size, style|wxFULL_REPAINT_ON_RESIZE, name, attributes)
{
    // default cursor to standard
    _oldCursor = *wxSTANDARD_CURSOR;
}

OSGCanvas::~OSGCanvas()
{
}

void OSGCanvas::OnPaint( wxPaintEvent& WXUNUSED(event) )
{
    /* must always be here */
    wxPaintDC dc(this);
}

void OSGCanvas::OnSize(wxSizeEvent& event)
{
    // this is also necessary to update the context on some platforms
    wxGLCanvas::OnSize(event);

    // set GL viewport (not called by wxGLCanvas::OnSize on all platforms...)
    int width, height;
    GetClientSize(&width, &height);

    if (_graphics_window.valid())
    {
        // update the window dimensions, in case the window has been resized.
        _graphics_window->getEventQueue()->windowResize(0, 0, width, height);
        _graphics_window->resized(0,0,width,height);
    }
}

void OSGCanvas::OnEraseBackground(wxEraseEvent& WXUNUSED(event))
{
    /* Do nothing, to avoid flashing on MSW */
}

void OSGCanvas::OnChar(wxKeyEvent &event)
{
#if wxUSE_UNICODE
    int key = event.GetUnicodeKey();
#else
    int key = event.GetKeyCode();
#endif

    if (_graphics_window.valid())
        _graphics_window->getEventQueue()->keyPress(key);

    // If this key event is not processed here, we should call
    // event.Skip() to allow processing to continue.
}

void OSGCanvas::OnKeyUp(wxKeyEvent &event)
{
#if wxUSE_UNICODE
    int key = event.GetUnicodeKey();
#else
    int key = event.GetKeyCode();
#endif

    if (_graphics_window.valid())
        _graphics_window->getEventQueue()->keyRelease(key);

    // If this key event is not processed here, we should call
    // event.Skip() to allow processing to continue.
}

void OSGCanvas::OnMouseEnter(wxMouseEvent &event)
{
    // Set focus to ourselves, so keyboard events get directed to us
    SetFocus();
}

void OSGCanvas::OnMouseDown(wxMouseEvent &event)
{
    if (_graphics_window.valid())
    {
        _graphics_window->getEventQueue()->mouseButtonPress(event.GetX(), event.GetY(),
            event.GetButton());
    }
}

void OSGCanvas::OnMouseUp(wxMouseEvent &event)
{
    if (_graphics_window.valid())
    {
        _graphics_window->getEventQueue()->mouseButtonRelease(event.GetX(), event.GetY(),
            event.GetButton());
    }
}

void OSGCanvas::OnMouseMotion(wxMouseEvent &event)
{
    if (_graphics_window.valid())
        _graphics_window->getEventQueue()->mouseMotion(event.GetX(), event.GetY());
}

void OSGCanvas::UseCursor(bool value)
{
    if (value)
    {
        // show the old cursor
        SetCursor(_oldCursor);
    }
    else
    {
        // remember the old cursor
        _oldCursor = GetCursor();

        // hide the cursor
        //    - can't find a way to do this neatly, so create a 1x1, transparent image
        wxImage image(1,1);
        image.SetMask(true);
        image.SetMaskColour(0, 0, 0);
        wxCursor cursor(image);
        SetCursor(cursor);

        // On wxGTK, only works as of version 2.7.0
        // (http://trac.wxwidgets.org/ticket/2946)
        // SetCursor( wxStockCursor( wxCURSOR_BLANK ) );
    }
}

GraphicsWindowWX::GraphicsWindowWX(OSGCanvas *canvas)
{
    _canvas = canvas;

    _traits = new GraphicsContext::Traits;

    wxPoint pos = _canvas->GetPosition();
    wxSize  size = _canvas->GetSize();

    _traits->x = pos.x;
    _traits->y = pos.y;
    _traits->width = size.x;
    _traits->height = size.y;

    init();
}

GraphicsWindowWX::~GraphicsWindowWX()
{
}

void GraphicsWindowWX::init()
{
    if (valid())
    {
        setState( new osg::State );
        getState()->setGraphicsContext(this);

        if (_traits.valid() && _traits->sharedContext)
        {
            getState()->setContextID( _traits->sharedContext->getState()->getContextID() );
            incrementContextIDUsageCount( getState()->getContextID() );
        }
        else
        {
            getState()->setContextID( osg::GraphicsContext::createNewContextID() );
        }
    }
}

void GraphicsWindowWX::grabFocus()
{
    // focus the canvas
    _canvas->SetFocus();
}

void GraphicsWindowWX::grabFocusIfPointerInWindow()
{
    // focus this window, if the pointer is in the window
    wxPoint pos = wxGetMousePosition();
    if (wxFindWindowAtPoint(pos) == _canvas)
        _canvas->SetFocus();
}

void GraphicsWindowWX::useCursor(bool cursorOn)
{
    _canvas->UseCursor(cursorOn);
}

bool GraphicsWindowWX::makeCurrentImplementation()
{
    _canvas->SetCurrent();
    return true;
}

void GraphicsWindowWX::swapBuffersImplementation()
{
    _canvas->SwapBuffers();
}

void wxVessRenderer::OnGridToggle(wxCommandEvent& event)
{
    if (event.IsChecked())
    {
        std::cout << "enabled grid" << std::endl;
        vess->sceneManager->setGrid(10);
    } else {
        std::cout << "disabled grid" << std::endl;
        vess->sceneManager->setGrid(0);
    }
}
