#include <iostream>
#include "spinApp.h"
#include "SceneManager.h"
#include "dialog_NewNode.h"

using namespace spineditor;

dialog_NewNode::dialog_NewNode(wxWindow* parent) : dialog_NewNode_base(parent)
{
    // fill the wxChoice control with current node types
    std::vector<std::string> nodeTypes = spin::spinApp::Instance().sceneManager->getAllNodeTypes();
    for (std::vector<std::string>::iterator it = nodeTypes.begin(); it != nodeTypes.end(); ++it)
    {
        m_nodeType->Append(wxString(*it));

    }
}

dialog_NewNode::~dialog_NewNode() {}

void dialog_NewNode::OnCancel( wxCommandEvent& event )
{
    std::cout << "got OnCancel" << std::endl;
    event.Skip();
}

void dialog_NewNode::OnOK( wxCommandEvent& event )
{
    //std::string nodeID = m_nodeID->GetLineText(0).ToStdString();
    //std::string nodeType = m_nodeType->GetString(m_nodeType->GetSelection()).ToStdString();
    wxString nodeID = m_nodeID->GetLineText(0);
    wxString nodeType = m_nodeType->GetString(m_nodeType->GetSelection());

    // TODO: validate that nodeID has valid characters

    std::cout << "Trying to create node: '" << (char*)nodeID.char_str() << "' of type: " << (char*)nodeType.char_str() << std::endl;
    spin::spinApp::Instance().SceneMessage("sss", "createNode", (char*)nodeID.char_str(), (char*)nodeType.char_str(), SPIN_ARGS_END);
    //spin::spinApp::Instance().SceneMessage("sss", "createNode", nodeID.ToStdString(), nodeType.ToStdString(), SPIN_ARGS_END);

    event.Skip();
}
