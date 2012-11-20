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

#import <Cocoa/Cocoa.h>
#include <AppKit/AppKit.h>
#import "SpinViewerAppDelegate.h"
#include "spinApp.h"
#include <dlfcn.h>

int main(int argc, char *argv[])
{
    
    NSAutoreleasePool *pool = [NSAutoreleasePool new];
     
    // Manually load libSPIN in order to have the RTLD_GLOBAL flag for the
    // externs and singleton? Do we need to do this?
    NSString *dylibPath = [[[NSBundle mainBundle] privateFrameworksPath] stringByAppendingPathComponent:@"libSPIN.dylib"];
    const char *libPath = [[dylibPath stringByResolvingSymlinksInPath] cStringUsingEncoding:NSUTF8StringEncoding];
    void *dl = dlopen(libPath, RTLD_NOW|RTLD_GLOBAL);
    const char *dlErr = dlerror();
    if (dlErr) {
        NSLog(@"dlerror() =  %s", dlErr);
    } else {
        NSLog(@"dylib path is %s, dl = %p", libPath, dl);
    }
    
    spin::spinApp &spin = spin::spinApp::Instance();
    
    SpinViewerAppDelegate *delegate = [[SpinViewerAppDelegate alloc] init];
    [NSApplication sharedApplication];
    [NSApp setDelegate:delegate];

    
    
    [pool release];
    
    
    return NSApplicationMain(argc, (const char **)argv);
}
