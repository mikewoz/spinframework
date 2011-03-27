#ifndef __MainFrame__
#define __MainFrame__

#include "MainFrame_base.h"

namespace spineditor
{

class MainFrame : public MainFrame_base
{
public:
    MainFrame( wxWindow* parent );
    ~MainFrame();

protected:
    virtual void OnNewNode( wxCommandEvent& event );
    virtual void OnDeleteNode( wxCommandEvent& event );
    virtual void OnRefresh( wxCommandEvent& event );
    virtual void OnToggleGrid( wxCommandEvent& event );
    virtual void OnToggleViewer( wxCommandEvent& event );
    virtual void OnHelp( wxCommandEvent& event );
    virtual void OnAbout( wxCommandEvent& event );
    virtual void OnSceneDebug( wxCommandEvent& event );
};

} // end namespace

#endif
