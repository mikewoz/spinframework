/***************************************************************
 * Name:      wxVessMain.h
 * Purpose:   Defines Application Frame
 * Author:    Mike Wozniewski (mike@mikewoz.com)
 * Created:   2009-03-11
 * Copyright: Mike Wozniewski (www.mikewoz.com)
 * License:
 **************************************************************/

#ifndef WXVESSMAIN_H
#define WXVESSMAIN_H

//(*Headers(wxVessMain)
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

#include "wxVessConfig.h"
#include <streambuf>

class wxVessMain: public wxFrame
{
    public:

        wxVessMain(wxWindow* parent,wxWindowID id = -1);
        virtual ~wxVessMain();

    private:

        //(*Handlers(wxVessMain)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnStartButtonClick(wxCommandEvent& event);
        void OnStartStopToggle(wxCommandEvent& event);
        void OnLoadScene(wxCommandEvent& event);
        void OnSaveScene(wxCommandEvent& event);
        void OnShowConfig(wxCommandEvent& event);
        void OnShowEditor(wxCommandEvent& event);
        void OnShowRenderer(wxCommandEvent& event);
        void OnChangeVessType(wxCommandEvent& event);
        void OnVessMaster(wxCommandEvent& event);
        void OnVessSlave(wxCommandEvent& event);
        //*)

        //(*Identifiers(wxVessMain)
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
        static const long wxVess_load;
        static const long wxVess_Save;
        static const long wxVess_showConfig;
        static const long wxVess_showEditor;
        static const long wxVess_showRenderer;
        static const long ID_TOOLBAR1;
        //*)

        //(*Declarations(wxVessMain)
        wxToolBar* wxVess_ToolBar;
        wxSplitterWindow* mainSplitter;
        wxToolBarToolBase* ToolBarItem4;
        wxPanel* mainPanel;
        wxScrolledWindow* logPanel;
        wxMenuItem* MenuItem7;
        wxToolBarToolBase* ToolBarItem3;
        wxMenuItem* MenuItem5;
        wxMenu* Menu3;
        wxMenuItem* MenuItem4;
        wxRadioButton* vessRadio_slave;
        wxStatusBar* wxVess_StatusBar;
        wxStaticText* StaticText1;
        wxRadioButton* vessRadio_master;
        wxToolBarToolBase* ToolBarItem1;
        wxMenuItem* MenuItem3;
        wxTextCtrl* logTextCtrl;
        wxMenuItem* MenuItem6;
        wxToolBarToolBase* ToolBarItem5;
        wxMenuItem* wxVessMenu_About;
        wxToolBarToolBase* ToolBarItem2;
        wxToggleButton* StartStop;
        //*)

        wxVessConfig *vessConfigFrame;

#if wxUSE_STD_IOSTREAM
        //wxStreamToTextRedirector *redirector;
        std::streambuf *oldstdout;
#endif

        DECLARE_EVENT_TABLE()
};

#endif
