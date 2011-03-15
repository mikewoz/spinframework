/***************************************************************
 * Name:      spineditorApp.cpp
 * Purpose:   Code for Application Class
 * Author:    Mike Wozniewski (mike@mikewoz.com)
 * Created:   2011-03-15
 * Copyright: Mike Wozniewski (http://www.mikewoz.com)
 * License:
 **************************************************************/

#ifdef WX_PRECOMP
#include "wx_pch.h"
#endif

#ifdef __BORLANDC__
#pragma hdrstop
#endif //__BORLANDC__

#include "spineditorApp.h"
#include "spineditorMain.h"

IMPLEMENT_APP(spineditorApp);

bool spineditorApp::OnInit()
{
    spineditorFrame* frame = new spineditorFrame(0L);
    
    frame->Show();
    
    return true;
}
