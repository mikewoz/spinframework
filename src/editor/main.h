#ifndef _main_H_
#define _main_H_

#include <wx/wx.h>
#include <wx/cmdline.h>
#include <wx/timer.h>
#include "spinClientContext.h"


namespace spineditor
{

enum {
    SpinPollTimer_ID  = wxID_HIGHEST
};

static const wxCmdLineEntryDesc g_cmdLineDesc[] =
{
     { wxCMD_LINE_SWITCH,
             "h",
             "help",
             "displays command-line help",
             wxCMD_LINE_VAL_NONE, wxCMD_LINE_OPTION_HELP
     },
     { wxCMD_LINE_OPTION,
             "s",
             "scene-id",
             "Specify the scene ID to listen to",
             wxCMD_LINE_VAL_STRING, wxCMD_LINE_PARAM_OPTIONAL | wxCMD_LINE_NEEDS_SEPARATOR
     },
     { wxCMD_LINE_OPTION,
             "r",
             "server-addr",
             "Specify the remote server address (Default is osc.udp://239.0.0.1:54323)",
             wxCMD_LINE_VAL_STRING, wxCMD_LINE_PARAM_OPTIONAL | wxCMD_LINE_NEEDS_SEPARATOR
     },
     { wxCMD_LINE_OPTION,
             "u",
             "user-id",
             "Specify a user ID for this editor (Default is the local hostname)",
             wxCMD_LINE_VAL_STRING, wxCMD_LINE_PARAM_OPTIONAL | wxCMD_LINE_NEEDS_SEPARATOR
     },
     { wxCMD_LINE_SWITCH,
             "v",
             "version",
             "Display the version number and exit",
             wxCMD_LINE_VAL_NONE, wxCMD_LINE_PARAM_OPTIONAL
     },
     { wxCMD_LINE_NONE }
};



/**
 * The Spin Editor application.
 */
class SpinEditorApp : public wxApp
{
    public:
        /**
         * Constructor. We make it explicit.
         */
        SpinEditorApp() :
            spinListener()
        {
        }

        /**
         * Called when this application is launched.
         *
         * Creates the MainWindow instance.
         * We also make sure there is a SPIN context going on.
         * We ask for a refresh.
         */
        virtual bool OnInit();
        virtual int OnExit();

        /**
         * Starts spinListener threads (can possibly be changed in the future
         * to start either a client or server context, depending on choices
         * made in the GUI)
         */
        void start();

        void OnSpinPollTimer(wxTimerEvent& event);

        /**
         * Initializes commandline arguments
         */
        virtual void OnInitCmdLine(wxCmdLineParser& parser);

        /**
         * Does the actual parsing of commandline arguments
         */
        virtual bool OnCmdLineParsed(wxCmdLineParser& parser);

        /**
         * If the user provides bad arguments, show the usage and exit
         */
        virtual bool OnCmdLineError(wxCmdLineParser & parser);

        DECLARE_EVENT_TABLE()

    private:
        spin::spinClientContext spinListener;
        wxTimer *spinPollTimer_;


};


} // end namespace

#endif
