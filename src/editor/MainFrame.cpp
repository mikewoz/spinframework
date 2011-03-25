#include <iostream>
#include "MainFrame.h"


MainFrame::MainFrame( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : MainFrame_base( parent, id, title, pos, size, style )
{



}
MainFrame::~MainFrame()
{

}

void MainFrame::OnNewNode( wxCommandEvent& event )
{
    std::cout << "Got OnNewNode" << std::endl;
    event.Skip();
}
