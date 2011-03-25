#ifndef __MainFrame__
#define __MainFrame__

#include "MainFrame_base.h"

class MainFrame : public MainFrame_base
{
public:
    MainFrame( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("SPIN Editor"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 400,500 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
    ~MainFrame();



    private:

    protected:

        virtual void OnNewNode( wxCommandEvent& event );
};


#endif
