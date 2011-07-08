///////////////////////////////////////////////////////////////////////////// C++ code generated with wxFormBuilder (version Sep 12 2010)// http://www.wxformbuilder.org///// PLEASE DO "NOT" EDIT THIS FILE!///////////////////////////////////////////////////////////////////////////#ifndef __MainFrame_base__#define __MainFrame_base__#include <wx/intl.h>class wxSpinPropGrid;class wxSpinTreeCtrl;#include <wx/treectrl.h>#include <wx/gdicmn.h>#include <wx/font.h>#include <wx/colour.h>#include <wx/settings.h>#include <wx/string.h>#include <wx/panel.h>#include <wx/sizer.h>#include <wx/bitmap.h>#include <wx/image.h>#include <wx/icon.h>#include <wx/toolbar.h>#include <wx/textctrl.h>#include <wx/scrolwin.h>#include <wx/splitter.h>#include <wx/stattext.h>#include <wx/choice.h>#include <wx/menu.h>#include <wx/frame.h>///////////////////////////////////////////////////////////////////////////namespace spineditor{	#define wxID_TOOLBAR_CLEARLOG 1000	#define wxID_TOOLBAR_SAVELOG 1001	#define wxID_TOOLBAR_DEBUG 1002	#define wxID_TOOLBAR_ADD 1003	#define wxID_TOOLBAR_DELETE 1004	#define wxID_TOOLBAR_REFRESH 1005	#define wxID_TOOLBAR_GRID 1006	#define wxID_TOOLBAR_VIEWER 1007		///////////////////////////////////////////////////////////////////////////////	/// Class MainFrame_base	///////////////////////////////////////////////////////////////////////////////	class MainFrame_base : public wxFrame 	{		private:				protected:			wxSplitterWindow* mainSplitter;			wxPanel* upperPanel;			wxSpinTreeCtrl* spinTreeCtrl;			wxSpinPropGrid* spinPropGrid;			wxPanel* lowerPanel;			wxToolBar* m_toolBar2;			wxScrolledWindow* logScrolledWindow;			wxTextCtrl* logTextCtrl;			wxToolBar* m_toolBar1;			wxStaticText* m_staticText1;			wxChoice* serverChooser;			wxMenuBar* menu;			wxMenu* menuFile;			wxMenu* menuHelp;						// Virtual event handlers, overide them in your derived class			virtual void OnClose( wxCloseEvent& event ) { event.Skip(); }			virtual void OnSceneDebug( wxCommandEvent& event ) { event.Skip(); }			virtual void OnNewNode( wxCommandEvent& event ) { event.Skip(); }			virtual void OnDeleteNode( wxCommandEvent& event ) { event.Skip(); }			virtual void OnRefreshScene( wxCommandEvent& event ) { event.Skip(); }			virtual void OnToggleGrid( wxCommandEvent& event ) { event.Skip(); }			virtual void OnToggleViewer( wxCommandEvent& event ) { event.Skip(); }			virtual void OnServerChange( wxCommandEvent& event ) { event.Skip(); }			virtual void OnClearScene( wxCommandEvent& event ) { event.Skip(); }			virtual void OnQuit( wxCommandEvent& event ) { event.Skip(); }			virtual void OnHelp( wxCommandEvent& event ) { event.Skip(); }			virtual void OnAbout( wxCommandEvent& event ) { event.Skip(); }							public:						MainFrame_base( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = _("SPIN Editor"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 500,600 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );			~MainFrame_base();						void mainSplitterOnIdle( wxIdleEvent& )			{				mainSplitter->SetSashPosition( 0 );				mainSplitter->Disconnect( wxEVT_IDLE, wxIdleEventHandler( MainFrame_base::mainSplitterOnIdle ), NULL, this );			}			};	} // namespace spineditor#endif //__MainFrame_base__