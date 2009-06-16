#ifndef WXVESSCONFIG_H
#define WXVESSCONFIG_H

//(*Headers(wxVessConfig)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/panel.h>
//*)

class wxVessConfig: public wxPanel
{
	public:

		wxVessConfig(wxWindow* parent,wxWindowID id=wxID_ANY,const wxPoint& pos=wxDefaultPosition,const wxSize& size=wxDefaultSize);
		virtual ~wxVessConfig();

		//(*Declarations(wxVessConfig)
		wxTextCtrl* rxPort;
		wxTextCtrl* rxAddr;
		wxTextCtrl* txAddr;
		wxStaticText* StaticText2;
		wxStaticText* StaticText1;
		wxStaticText* StaticText3;
		wxStaticText* StaticText5;
		wxTextCtrl* vessID;
		wxTextCtrl* txPort;
		wxStaticText* StaticText4;
		wxPanel* vessConfigPanel;
		//*)

	private:

		//(*Identifiers(wxVessConfig)
		static const long ID_STATICTEXT1;
		static const long ID_TEXTCTRL1;
		static const long ID_STATICTEXT2;
		static const long ID_TEXTCTRL2;
		static const long ID_STATICTEXT3;
		static const long ID_TEXTCTRL3;
		static const long ID_STATICTEXT4;
		static const long ID_TEXTCTRL4;
		static const long ID_STATICTEXT5;
		static const long ID_TEXTCTRL5;
		static const long ID_PANEL1;
		//*)

		//(*Handlers(wxVessConfig)
		void OnClose(wxCloseEvent& event);
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
