///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Sep 12 2010)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __GUIFrame__
#define __GUIFrame__

/**
 * \file
 * The GuiFrame class.
 */

#include <wx/string.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/menu.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/statusbr.h>
#include <wx/treectrl.h>
#include <wx/textctrl.h>
#include <wx/sizer.h>
#include <wx/frame.h>

#define idMenuQuit 1000
#define idMenuAbout 1001

/**
 * Base class for the GUI window.
 */
class GuiFrame : public wxFrame 
{
	public:
		
		GuiFrame(wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("SPIN Editor"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize(481, 466), long style = wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL );
		~GuiFrame();

	protected:
		wxMenuBar* mbar;
		wxMenu* fileMenu;
		wxMenu* helpMenu;
		wxStatusBar* statusBar;
		wxTreeCtrl* m_treeCtrl2;
		wxTextCtrl* m_textCtrl1;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnClose(wxCloseEvent& event) { event.Skip(); }
		virtual void OnQuit(wxCommandEvent& event) { event.Skip(); }
		virtual void OnAbout(wxCommandEvent& event) { event.Skip(); }
};

#endif //__GUIFrame__

