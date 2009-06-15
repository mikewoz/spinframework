#ifndef WXVESSEDITOR_H
#define WXVESSEDITOR_H

//(*Headers(wxVessEditor)
#include <wx/sizer.h>
#include <wx/splitter.h>
#include <wx/toolbar.h>
#include <wx/panel.h>
#include "wxVessPropGrid.h"
#include "wxVessTreeCtrl.h"
#include <wx/frame.h>
//*)

class wxVessEditor: public wxFrame
{
	public:

		wxVessEditor(wxWindow* parent,wxWindowID id=wxID_ANY);
		virtual ~wxVessEditor();

		//(*Declarations(wxVessEditor)
		wxToolBarToolBase* ToolBarItem4;
		wxToolBarToolBase* ToolBarItem3;
		wxVessPropGrid* VessPropGrid;
		wxToolBar* wxVessEditor_ToolBar;
		wxToolBarToolBase* ToolBarItem1;
		wxVessTreeCtrl* vessTree;
		wxPanel* editorPanel;
		wxPanel* treePanel;
		wxSplitterWindow* vessEditor_splitter;
		wxToolBarToolBase* ToolBarItem2;
		//*)

        void OnVessSelectionChange(wxTreeEvent &event);
        void OnDragBegin(wxTreeEvent &event);
        void OnDragEnd(wxTreeEvent &event);

	protected:

		//(*Identifiers(wxVessEditor)
		static const long ID_VESS_TREE;
		static const long ID_PANEL1;
		static const long ID_CUSTOM1;
		static const long ID_PANEL2;
		static const long ID_SPLITTERWINDOW1;
		static const long vessEditor_newNode;
		static const long vessEditor_clear;
		static const long vessEditor_refresh;
		static const long vessEditor_debugPrint;
		static const long ID_TOOLBAR1;
		//*)

	private:

		//(*Handlers(wxVessEditor)
		void OnNewNode(wxCommandEvent& event);
		void OnRefresh(wxCommandEvent& event);
		void OnvessTreePaint(wxPaintEvent& event);
		void OnVessTreeLeftDown(wxMouseEvent& event);
		void OnvessTreePaint1(wxPaintEvent& event);
		void OnvessTreePaint2(wxPaintEvent& event);
		void OnVessTreeLeftDClick(wxMouseEvent& event);
		void OnCustom1Paint(wxPaintEvent& event);
		void OnvessTreePaint3(wxPaintEvent& event);
		void OnDebugPrint(wxCommandEvent& event);
		void OnClear(wxCommandEvent& event);
		//*)

		DECLARE_EVENT_TABLE()
};


#endif
