#ifndef __MainFrame__
#define __MainFrame__

#include "main.h"
#include "MainFrame_base.h"
#include <streambuf>

// use DECLARE_APP so we can have the wxGetApp function (which returns a
// reference to our application object) to be visible to other files:
DECLARE_APP(spineditor::SpinEditorApp)

namespace spineditor
{

class MainFrame : public MainFrame_base
{
public:
    MainFrame( wxWindow* parent );
    ~MainFrame();

protected:
    virtual void OnQuit(wxCommandEvent& event);
    virtual void OnClose(wxCloseEvent& event);
    virtual void OnNewNode( wxCommandEvent& event );
    virtual void OnDeleteNode( wxCommandEvent& event );
    virtual void OnRefreshScene( wxCommandEvent& event );
    virtual void OnClearScene( wxCommandEvent& event );
    virtual void OnToggleGrid( wxCommandEvent& event );
    virtual void OnToggleViewer( wxCommandEvent& event );
    virtual void OnHelp( wxCommandEvent& event );
    virtual void OnAbout( wxCommandEvent& event );
    virtual void OnSceneDebug( wxCommandEvent& event );

private:

    #if wxUSE_STD_IOSTREAM
        wxStreamToTextRedirector *redirector;
    #endif

};

} // end namespace

#endif
