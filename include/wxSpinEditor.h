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

#ifndef WXSPINEDITOR_H
#define WXSPINEDITOR_H

//(*Headers(wxSpinEditor)
#include "wxSpinPropGrid.h"
#include <wx/sizer.h>
#include <wx/splitter.h>
#include <wx/toolbar.h>
#include <wx/panel.h>
#include "wxSpinTreeCtrl.h"
#include <wx/frame.h>
//*)

class wxSpinEditor: public wxFrame
{
	public:

		wxSpinEditor(wxWindow* parent,wxWindowID id=wxID_ANY);
		virtual ~wxSpinEditor();

		//(*Declarations(wxSpinEditor)
		wxToolBarToolBase* ToolBarItem4;
		wxToolBarToolBase* ToolBarItem3;
		wxSplitterWindow* spinEditor_splitter;
		wxSpinTreeCtrl* spinTree;
		wxToolBarToolBase* ToolBarItem1;
		wxToolBar* wxSpinEditor_ToolBar;
		wxPanel* editorPanel;
		wxToolBarToolBase* ToolBarItem5;
		wxPanel* treePanel;
		wxSpinPropGrid* SpinPropGrid;
		wxToolBarToolBase* ToolBarItem2;
		//*)

	protected:

		//(*Identifiers(wxSpinEditor)
		static const long ID_SPIN_TREE;
		static const long ID_treePanel;
		static const long ID_CUSTOM1;
		static const long ID_PANEL2;
		static const long ID_SPLITTERWINDOW1;
		static const long spinEditor_clear;
		static const long spinEditor_refresh;
		static const long spinEditor_debugPrint;
		static const long spinEditor_newNode;
		static const long spinEditor_deleteNode;
		static const long ID_TOOLBAR1;
		//*)

	private:

		//(*Handlers(wxSpinEditor)
		void OnNewNode(wxCommandEvent& event);
		void OnRefresh(wxCommandEvent& event);
		void OnspinTreePaint(wxPaintEvent& event);
		void OnSpinTreeLeftDown(wxMouseEvent& event);
		void OnspinTreePaint1(wxPaintEvent& event);
		void OnspinTreePaint2(wxPaintEvent& event);
		void OnSpinTreeLeftDClick(wxMouseEvent& event);
		void OnCustom1Paint(wxPaintEvent& event);
		void OnspinTreePaint3(wxPaintEvent& event);
		void OnDebugPrint(wxCommandEvent& event);
		void OnClear(wxCommandEvent& event);
		void OnDeleteNode(wxCommandEvent& event);
		//*)

		DECLARE_EVENT_TABLE()

		lo_server_thread listeningServer;
};


#endif
