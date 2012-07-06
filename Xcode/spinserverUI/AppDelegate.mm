// -----------------------------------------------------------------------------
// |    ___  ___  _  _ _     ___                                        _      |
// |   / __>| . \| || \ |   | __>_ _  ___ ._ _ _  ___  _ _ _  ___  _ _ | |__   |
// |   \__ \|  _/| ||   |   | _>| '_><_> || ' ' |/ ._>| | | |/ . \| '_>| / /   |
// |   <___/|_|  |_||_\_|   |_| |_|  <___||_|_|_|\___.|__/_/ \___/|_|  |_\_\   |
// |                                                                           |
// |---------------------------------------------------------------------------|
//
// http://spinframework.sourceforge.net
// Copyright (C) 2009 Mike Wozniewski, Zack Settel
//
// Developed/Maintained by:
//    Mike Wozniewski (http://www.mikewoz.com)
//    Zack Settel (http://www.sheefa.net/zack)
//
// Principle Partners:
//    Shared Reality Lab, McGill University (http://www.cim.mcgill.ca/sre)
//    La Societe des Arts Technologiques (http://www.sat.qc.ca)
//
// Funding by:
//    NSERC/Canada Council for the Arts - New Media Initiative
//    Heritage Canada
//    Ministere du Developpement economique, de l'Innovation et de l'Exportation
//
// -----------------------------------------------------------------------------
//  This file is part of the SPIN Framework.
//
//  SPIN Framework is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  SPIN Framework is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------


#import "AppDelegate.h"
#include "spinApp.h"
#include "spinUtil.h"
#include "SceneManager.h"
#include "spinServerContext.h"
#ifdef WITH_SPATOSC
#include <spatosc/spatosc.h>
#endif

@implementation AppDelegate

//@synthesize window = _window;
//@synthesize window = window;

- (void)dealloc
{
    
    [super dealloc];
}

- (void)applicationWillTerminate:(NSNotification *)notification
{
    server.stop();
}

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification
{
    //[textView setFont:[NSFont fo
    
    pipe = [NSPipe pipe];
    pipeReadHandle = [pipe fileHandleForReading];
    dup2([[pipe fileHandleForWriting] fileDescriptor], fileno(stdout));
    
    
    [[NSNotificationCenter defaultCenter] addObserver: self selector: @selector(handleLog:) name: NSFileHandleReadCompletionNotification object: pipeReadHandle];
    [pipeReadHandle readInBackgroundAndNotify];
     
    
	// *************************************************************************
    // If no command line arguments were passed, check if there is an args file
    // at ~/.spinFramework/args and override argc and argv with those:
    std::vector<char*> newArgs = spin::getUserArgs();
    if (newArgs.size() > 1)
    {
        int argc;
        char **argv;

        newArgs.insert(newArgs.begin(), "fake");
        argc = (int)newArgs.size()-1;
        argv = &newArgs[0];         
            
        // PARSE ARGUMENTS::
            
        osg::ArgumentParser arguments(&argc,argv);
            
        if (!server.parseCommandLineOptions(&arguments))
            std::cout << "spinserver: error parsing arguments" << std::endl;
            
        // remaining arguments are assumed to be scene elements to be loaded (.xml)
        std::vector<std::string>::iterator arg;
        for (int i=1; i<argc; i++) //arg=arguments.begin(); arg!=arguments.end(); ++arg)
        {
            if (arguments[i][0]!='-')
            {
                // not an option so assume string is a filename
                spin::spinApp::Instance().sceneManager_->loadXML(arguments[i]);
            } else i++;
        }
        
        arguments.reportRemainingOptionsAsUnrecognized();
        if (arguments.errors()) arguments.writeErrorMessages(std::cout);

    }

    // *************************************************************************
    // for now, hardcode one spatosc translator in the scene:
#ifdef WITH_SPATOSC
    //spin::spinApp::Instance().audioScene->addTranslator("spatosc01", "BasicTranslator", "127.0.0.1", "18032");
    spin::spinApp::Instance().audioScene->addTranslator("spatosc01", "BasicTranslator", "osc.tcp://localhost:18033");
    spin::spinApp::Instance().hasAudioRenderer = true;
    //spin::spinApp::Instance().audioScene->debugPrint();

#endif
    
    server.start();
    
}

- (void) handleLog:(NSNotification *)notification
{
    NSString *str = [[NSString alloc] initWithData: [[notification userInfo] objectForKey: NSFileHandleNotificationDataItem] encoding: NSASCIIStringEncoding] ;

    //[[[textView textStorage] mutableString] appendString: str];
    
    [textView setString:[[textView string] stringByAppendingString:str]];
    [str release];
    
    [[notification object] readInBackgroundAndNotify];
}

- (IBAction)clearLog:(id)sender
{
    [textView setString:@""];
}



- (IBAction)refresh:(id)sender
{
    spin::spinApp::Instance().sceneManager_->refreshAll();
}
- (IBAction)refreshSubscribers:(id)sender
{
    if (spin::spinApp::Instance().getContext()->isServer())
    {
        spin::spinServerContext *s = dynamic_cast<spin::spinServerContext*>(spin::spinApp::Instance().getContext());
        s->refreshSubscribers();
    }
    else
    {
        spin::spinApp::Instance().SceneMessage("s", "refreshSubscribers", SPIN_ARGS_END);
    }
}

- (IBAction)clearScene:(id)sender
{
    int result = NSRunInformationalAlertPanel(@"Warning", @"You are about to clear the entire scene. Proceed?", @"OK", @"Cancel",nil);
    
    switch(result)
    {
        case NSAlertDefaultReturn: // OK
            spin::spinApp::Instance().sceneManager_->clear();
            spin::spinApp::Instance().sceneManager_->clearUsers();
            spin::spinApp::Instance().sceneManager_->clearStates();
            break;
        case NSAlertAlternateReturn: // Cancel
            break;
    }
}
- (IBAction)toggleGrid:(id)sender
{
    if ([sender isSelected])
    {
        spin::spinApp::Instance().sceneManager_->deleteNode("grid");
        //[sender setSelected:FALSE];
    }
    else {
        spin::spinApp::Instance().sceneManager_->getOrCreateNode("grid","GridNode");
        //[sender setSelected:TRUE];       
    }
}
- (IBAction)debugContext:(id)sender
{
    spin::spinApp::Instance().sceneManager_->debugContext(); 
}
- (IBAction)debugNodes:(id)sender
{
    spin::spinApp::Instance().sceneManager_->debugNodes(); 
}
- (IBAction)debugStateSets:(id)sender
{
    spin::spinApp::Instance().sceneManager_->debugStateSets(); 
}
- (IBAction)debugSceneGraph:(id)sender
{
    spin::spinApp::Instance().sceneManager_->debugSceneGraph(); 
}


@end
