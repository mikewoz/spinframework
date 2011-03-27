#ifndef __dialog_NewNode__
#define __dialog_NewNode__

#include "dialog_NewNode_base.h";

namespace spineditor {

class dialog_NewNode : public dialog_NewNode_base
{

public:
    dialog_NewNode( wxWindow* parent);
    ~dialog_NewNode();

protected:
    virtual void OnCancel( wxCommandEvent& event );
    virtual void OnOK( wxCommandEvent& event );

};

} // end namespace

#endif
