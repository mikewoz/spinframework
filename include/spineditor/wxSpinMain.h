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

#ifndef WXSPINMAIN_H
#define WXSPINMAIN_H

//(*Headers(wxSpinMain)
#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/menu.h>
#include <wx/textctrl.h>
#include <wx/splitter.h>
#include <wx/tglbtn.h>
#include <wx/radiobut.h>
#include <wx/toolbar.h>
#include <wx/panel.h>
#include <wx/frame.h>
#include <wx/statusbr.h>
//*)

#include "wxSpinSettings.h"
#include "wxSpinEditor.h"
#include <streambuf>

class wxSpinMain: public wxFrame
{
    public:

        wxSpinMain(wxWindow* parent,wxWindowID id = -1);
        virtual ~wxSpinMain();

    private:

        //(*Handlers(wxSpinMain)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnStartButtonClick(wxCommandEvent& event);
        void OnStartStopToggle(wxCommandEvent& event);
        void OnLoadScene(wxCommandEvent& event);
        void OnSaveScene(wxCommandEvent& event);
        void OnShowConfig(wxCommandEvent& event);
        void OnShowEditor(wxCommandEvent& event);
        void OnShowRenderer(wxCommandEvent& event);
        void OnChangeSpinType(wxCommandEvent& event);
        void OnSpinMaster(wxCommandEvent& event);
        void OnSpinSlave(wxCommandEvent& event);
        void OnClose(wxCloseEvent& event);
        void OnSpinModeChange(wxCommandEvent& event);
        //*)

        //(*Identifiers(wxSpinMain)
        static const long ID_STATICTEXT1;
        static const long ID_RADIOBUTTON1;
        static const long ID_RADIOBUTTON2;
        static const long ID_TOGGLEBUTTON2;
        static const long ID_PANEL1;
        static const long ID_TEXTCTRL6;
        static const long ID_SCROLLEDWINDOW1;
        static const long ID_SPLITTERWINDOW1;
        static const long idMenuOpen;
        static const long idMenuSave;
        static const long idMenuQuit;
        static const long idMenuShowConfig;
        static const long idMenuShowEditor;
        static const long idMenuShowRenderer;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        static const long wxSpin_load;
        static const long wxSpin_Save;
        static const long wxSpin_showConfig;
        static const long wxSpin_showRenderer;
        static const long wxSpin_showEditor;
        static const long ID_TOOLBAR1;
        //*)

        //(*Declarations(wxSpinMain)
        wxSplitterWindow* mainSplitter;
        wxToolBarToolBase* ToolBarItem4;
        wxPanel* mainPanel;
        wxScrolledWindow* logPanel;
        wxMenuItem* MenuItem7;
        wxToolBarToolBase* ToolBarItem3;
        wxMenuItem* MenuItem5;
        wxMenu* Menu3;
        wxMenuItem* MenuItem4;
        wxRadioButton* spinRadio_slave;
        wxStaticText* StaticText1;
        wxToolBarToolBase* ToolBarItem1;
        wxMenuItem* MenuItem3;
        wxTextCtrl* logTextCtrl;
        wxMenuItem* MenuItem6;
        wxToolBarToolBase* ToolBarItem5;
        wxMenuItem* wxSpinMenu_About;
        wxRadioButton* spinRadio_master;
        wxToolBar* wxSpin_ToolBar;
        wxToolBarToolBase* ToolBarItem2;
        wxStatusBar* wxSpin_StatusBar;
        wxToggleButton* StartStop;
        //*)

        wxSpinSettings *spinSettingsFrame;

#if wxUSE_STD_IOSTREAM
        //wxStreamToTextRedirector *redirector;
        std::streambuf *oldstdout;
#endif


        DECLARE_EVENT_TABLE()
};

#endif
