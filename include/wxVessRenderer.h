#ifndef WXVESSRENDERER_H
#define WXVESSRENDERER_H

//(*Headers(wxVessRenderer)
#include <wx/toolbar.h>
#include <wx/frame.h>
//*)

#include "wx/cursor.h"
#include "wx/glcanvas.h"

/*
#ifdef __WXMAC__
#include "GLUT/glut.h"
#else
#include "GL/glut.h"
#endif
*/

#include <osgViewer/CompositeViewer>
#include <osgViewer/Viewer>
#include <string>

// these classes are also declared below:
class GraphicsWindowWX;
class OSGCanvas;


class wxVessRenderer: public wxFrame
{
	public:

		wxVessRenderer(wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize);
		virtual ~wxVessRenderer();

        void OnIdle(wxIdleEvent& event);

	private:

		//(*Declarations(wxVessRenderer)
		wxToolBarToolBase* ToolBarItem1;
		wxToolBar* wxVessRenderer_ToolBar;
		wxToolBarToolBase* ToolBarItem2;
		//*)

		//(*Identifiers(wxVessRenderer)
		static const long vessRenderer_grid;
		static const long vessRenderer_trackNode;
		static const long ID_TOOLBAR1;
		//*)

		//(*Handlers(wxVessRenderer)
		void OnGridToggle(wxCommandEvent& event);
		//*)

        OSGCanvas *canvas;

        osg::ref_ptr<osgViewer::CompositeViewer> viewer;
        //osg::ref_ptr<osgViewer::Viewer> viewer;

		DECLARE_EVENT_TABLE()
};


class OSGCanvas : public wxGLCanvas
{
public:
    OSGCanvas(wxWindow *parent, wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize, long style = 0,
        const wxString& name = wxT("TestGLCanvas"),
        int *attributes = 0);

    virtual ~OSGCanvas();

    void SetGraphicsWindow(osgViewer::GraphicsWindow *gw)   { _graphics_window = gw; }

    void OnPaint(wxPaintEvent& event);
    void OnSize(wxSizeEvent& event);
    void OnEraseBackground(wxEraseEvent& event);

    void OnChar(wxKeyEvent &event);
    void OnKeyUp(wxKeyEvent &event);

    void OnMouseEnter(wxMouseEvent &event);
    void OnMouseDown(wxMouseEvent &event);
    void OnMouseUp(wxMouseEvent &event);
    void OnMouseMotion(wxMouseEvent &event);

    void UseCursor(bool value);

private:
    DECLARE_EVENT_TABLE()

    osg::ref_ptr<osgViewer::GraphicsWindow> _graphics_window;

    wxCursor _oldCursor;
};

class GraphicsWindowWX : public osgViewer::GraphicsWindow
{
public:
    GraphicsWindowWX(OSGCanvas *canvas);
    ~GraphicsWindowWX();

    void init();

    //
    // GraphicsWindow interface
    //
    void grabFocus();
    void grabFocusIfPointerInWindow();
    void useCursor(bool cursorOn);

    bool makeCurrentImplementation();
    void swapBuffersImplementation();

    // not implemented yet...just use dummy implementation to get working.
    virtual bool valid() const { return true; }
    virtual bool realizeImplementation() { return true; }
    virtual bool isRealizedImplementation() const  { return true; }
    virtual void closeImplementation() {}
    virtual bool releaseContextImplementation() { return true; }

private:
    // XXX need to set _canvas to NULL when the canvas is deleted by
    // its parent. for this, need to add event handler in OSGCanvas
    OSGCanvas*  _canvas;
};



#endif
