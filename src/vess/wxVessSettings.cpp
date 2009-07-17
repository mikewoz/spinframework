#include "wxVessSettings.h"

//(*InternalHeaders(wxVessSettings)
#include <wx/settings.h>
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/string.h>
//*)

//(*IdInit(wxVessSettings)
const long wxVessSettings::ID_STATICTEXT5 = wxNewId();
const long wxVessSettings::ID_TEXTCTRL5 = wxNewId();
const long wxVessSettings::ID_STATICTEXT3 = wxNewId();
const long wxVessSettings::ID_TEXTCTRL3 = wxNewId();
const long wxVessSettings::ID_STATICTEXT4 = wxNewId();
const long wxVessSettings::ID_TEXTCTRL4 = wxNewId();
const long wxVessSettings::ID_STATICTEXT1 = wxNewId();
const long wxVessSettings::ID_TEXTCTRL1 = wxNewId();
const long wxVessSettings::ID_STATICTEXT2 = wxNewId();
const long wxVessSettings::ID_TEXTCTRL2 = wxNewId();
const long wxVessSettings::ID_PANEL1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(wxVessSettings,wxFrame)
	//(*EventTable(wxVessSettings)
	//*)
END_EVENT_TABLE()

wxVessSettings::wxVessSettings(wxWindow* parent,wxWindowID id,const wxPoint& pos,const wxSize& size)
{
	//(*Initialize(wxVessSettings)
	wxStaticBoxSizer* StaticBoxSizer2;
	wxFlexGridSizer* FlexGridSizer3;
	wxFlexGridSizer* FlexGridSizer2;
	wxBoxSizer* BoxSizer1;
	wxStaticBoxSizer* StaticBoxSizer1;
	wxFlexGridSizer* FlexGridSizer1;

	Create(parent, wxID_ANY, _("SPIN :: Settings"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
	vessSettings_panel = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
	BoxSizer1 = new wxBoxSizer(wxVERTICAL);
	FlexGridSizer1 = new wxFlexGridSizer(0, 3, 0, 0);
	StaticText5 = new wxStaticText(vessSettings_panel, ID_STATICTEXT5, _("VESS Session ID:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
	FlexGridSizer1->Add(StaticText5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	vessID = new wxTextCtrl(vessSettings_panel, ID_TEXTCTRL5, _("Default"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL5"));
	FlexGridSizer1->Add(vessID, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	BoxSizer1->Add(FlexGridSizer1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, vessSettings_panel, _("VESS Location (unicast):"));
	FlexGridSizer3 = new wxFlexGridSizer(1, 4, 0, 0);
	StaticText3 = new wxStaticText(vessSettings_panel, ID_STATICTEXT3, _("Addr:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
	wxFont StaticText3Font = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
	if ( !StaticText3Font.Ok() ) StaticText3Font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
	StaticText3Font.SetPointSize(9);
	StaticText3->SetFont(StaticText3Font);
	FlexGridSizer3->Add(StaticText3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	rxAddr = new wxTextCtrl(vessSettings_panel, ID_TEXTCTRL3, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
	FlexGridSizer3->Add(rxAddr, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText4 = new wxStaticText(vessSettings_panel, ID_STATICTEXT4, _("Port:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
	wxFont StaticText4Font = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
	if ( !StaticText4Font.Ok() ) StaticText4Font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
	StaticText4Font.SetPointSize(9);
	StaticText4->SetFont(StaticText4Font);
	FlexGridSizer3->Add(StaticText4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	rxPort = new wxTextCtrl(vessSettings_panel, ID_TEXTCTRL4, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL4"));
	FlexGridSizer3->Add(rxPort, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer1->Add(FlexGridSizer3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	BoxSizer1->Add(StaticBoxSizer1, 2, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer2 = new wxStaticBoxSizer(wxHORIZONTAL, vessSettings_panel, _("Broadcast/Multicast Channel:"));
	FlexGridSizer2 = new wxFlexGridSizer(1, 4, 0, 0);
	StaticText1 = new wxStaticText(vessSettings_panel, ID_STATICTEXT1, _("Addr:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	wxFont StaticText1Font = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
	if ( !StaticText1Font.Ok() ) StaticText1Font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
	StaticText1Font.SetPointSize(9);
	StaticText1->SetFont(StaticText1Font);
	FlexGridSizer2->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	txAddr = new wxTextCtrl(vessSettings_panel, ID_TEXTCTRL1, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL1"));
	FlexGridSizer2->Add(txAddr, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText2 = new wxStaticText(vessSettings_panel, ID_STATICTEXT2, _("Port:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
	wxFont StaticText2Font = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
	if ( !StaticText2Font.Ok() ) StaticText2Font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
	StaticText2Font.SetPointSize(9);
	StaticText2->SetFont(StaticText2Font);
	FlexGridSizer2->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	txPort = new wxTextCtrl(vessSettings_panel, ID_TEXTCTRL2, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL2"));
	FlexGridSizer2->Add(txPort, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer2->Add(FlexGridSizer2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	BoxSizer1->Add(StaticBoxSizer2, 2, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	vessSettings_panel->SetSizer(BoxSizer1);
	BoxSizer1->Fit(vessSettings_panel);
	BoxSizer1->SetSizeHints(vessSettings_panel);

	Connect(wxID_ANY,wxEVT_CLOSE_WINDOW,(wxObjectEventFunction)&wxVessSettings::OnClose);
	//*)
}

wxVessSettings::~wxVessSettings()
{
	//(*Destroy(wxVessSettings)
	//*)
}


void wxVessSettings::OnClose(wxCloseEvent& event)
{
    this->Show(false);
    event.Veto();

}
