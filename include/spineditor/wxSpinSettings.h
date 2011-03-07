#ifndef WXSPINSETTINGS_H
#define WXSPINSETTINGS_H

//(*Headers(wxSpinSettings)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/panel.h>
#include <wx/frame.h>
//*)

class wxSpinSettings: public wxFrame
{
    public:

        wxSpinSettings(wxWindow* parent=0,wxWindowID id=wxID_ANY,const wxPoint& pos=wxDefaultPosition,const wxSize& size=wxDefaultSize);
        virtual ~wxSpinSettings();


        //(*Declarations(wxSpinSettings)
        wxTextCtrl* rxPort;
        wxTextCtrl* rxAddr;
        wxStaticText* StaticText2;
        wxTextCtrl* txAddr;
        wxStaticText* StaticText1;
        wxStaticText* StaticText3;
        wxStaticText* StaticText5;
        wxTextCtrl* spinID;
        wxTextCtrl* txPort;
        wxPanel* spinSettings_panel;
        wxStaticText* StaticText4;
        //*)

    private:


        //(*Identifiers(wxSpinSettings)
        static const long ID_STATICTEXT5;
        static const long ID_TEXTCTRL5;
        static const long ID_STATICTEXT3;
        static const long ID_TEXTCTRL3;
        static const long ID_STATICTEXT4;
        static const long ID_TEXTCTRL4;
        static const long ID_STATICTEXT1;
        static const long ID_TEXTCTRL1;
        static const long ID_STATICTEXT2;
        static const long ID_TEXTCTRL2;
        static const long ID_PANEL1;
        //*)

        //(*Handlers(wxSpinSettings)
        void OnClose(wxCloseEvent& event);
        //*)

        DECLARE_EVENT_TABLE()
};

#endif
