#ifndef __MainFrame__
#define __MainFrame__

#include "main.h"
#include "MainFrame_base.h"
#include "EventHandler.h"
#include <streambuf>

// use DECLARE_APP so we can have the wxGetApp function (which returns a
// reference to our application object) to be visible to other files:
DECLARE_APP(spineditor::SpinEditorApp)

namespace spineditor
{

class MainFrame : public MainFrame_base, public spin::EventHandler
{
public:
    MainFrame( wxWindow* parent );
    ~MainFrame();

    virtual void onInfoMessage(spin::InfoMessage *msg);
    virtual void onServerChange(std::vector<spin::InfoMessage*> serverList);

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
    virtual void OnServerChange( wxCommandEvent& event );

private:

//#ifndef __WXMAC__
    #if wxUSE_STD_IOSTREAM
        wxStreamToTextRedirector *redirector;
    #endif
//#endif

};

/**
 * We use a callback to listen to INFO channel OSC messages (ie, to determine
 * what servers and/or clients are out there).
 */
int spin_info_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);


} // end namespace

#endif
