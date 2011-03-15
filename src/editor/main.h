/***************************************************************
 * Name:      spineditorMain.h
 * Purpose:   Defines Application Frame
 * Author:    Mike Wozniewski (mike@mikewoz.com)
 * Created:   2011-03-15
 * Copyright: Mike Wozniewski (http://www.mikewoz.com)
 * License:
 **************************************************************/

#ifndef SPINEDITORMAIN_H
#define SPINEDITORMAIN_H



#include "spineditorApp.h"


#include "GUIFrame.h"

class spineditorFrame: public GUIFrame
{
    public:
        spineditorFrame(wxFrame *frame);
        ~spineditorFrame();
    private:
        virtual void OnClose(wxCloseEvent& event);
        virtual void OnQuit(wxCommandEvent& event);
        virtual void OnAbout(wxCommandEvent& event);
};

#endif // SPINEDITORMAIN_H
