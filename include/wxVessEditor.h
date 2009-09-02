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
//  You should have received a copy of the GNU Lesser General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

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
		wxToolBarToolBase* ToolBarItem5;
		wxPanel* treePanel;
		wxSplitterWindow* vessEditor_splitter;
		wxToolBarToolBase* ToolBarItem2;
		//*)

	protected:

		//(*Identifiers(wxVessEditor)
		static const long ID_VESS_TREE;
		static const long ID_treePanel;
		static const long ID_CUSTOM1;
		static const long ID_PANEL2;
		static const long ID_SPLITTERWINDOW1;
		static const long vessEditor_clear;
		static const long vessEditor_refresh;
		static const long vessEditor_debugPrint;
		static const long vessEditor_newNode;
		static const long vessEditor_deleteNode;
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
		void OnDeleteNode(wxCommandEvent& event);
		//*)

		DECLARE_EVENT_TABLE()

		lo_server_thread listeningServer;
};


#endif
