/***************************************************************
 * Name:      vessWX.h
 * Purpose:   Defines Application Class
 * Author:    Mike Wozniewski (mike@mikewoz.com)
 * Created:   2009-03-11
 * Copyright: Mike Wozniewski (www.mikewoz.com)
 * License:
 **************************************************************/

#ifndef VESSWX_H
#define VESSWX_H

// For compilers that support precompilation, includes <wx/wx.h>.
#include <wx/wxprec.h>

#ifdef __BORLANDC__
    #pragma hdrstop
#endif

// for all others, include the necessary headers (this file is usually all you
// need because it includes almost all "standard" wxWidgets headers)
#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif



#include <wx/app.h>


class vessWX : public wxApp
{
    public:
        virtual bool OnInit();

};

#endif
