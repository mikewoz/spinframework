#ifndef WXVESSSETTINGS_H
#define WXVESSSETTINGS_H

//(*Headers(wxVessSettings)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/panel.h>
#include <wx/frame.h>
//*)

class wxVessSettings: public wxFrame
{
	public:

		wxVessSettings(wxWindow* parent=0,wxWindowID id=wxID_ANY,const wxPoint& pos=wxDefaultPosition,const wxSize& size=wxDefaultSize);
		virtual ~wxVessSettings();


		//(*Declarations(wxVessSettings)
		wxTextCtrl* rxPort;
		wxTextCtrl* rxAddr;
		wxStaticText* StaticText2;
		wxTextCtrl* txAddr;
		wxStaticText* StaticText1;
		wxStaticText* StaticText3;
		wxStaticText* StaticText5;
		wxTextCtrl* vessID;
		wxTextCtrl* txPort;
		wxPanel* vessSettings_panel;
		wxStaticText* StaticText4;
		//*)

	private:


		//(*Identifiers(wxVessSettings)
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

		//(*Handlers(wxVessSettings)
		void OnClose(wxCloseEvent& event);
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
