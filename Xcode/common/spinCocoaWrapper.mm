//
//  spinCocoaWrapper.m
//  spinframework
//
//  Created by Mike Wozniewski on 12-01-21.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#import "spinCocoaWrapper.h"
#include "spinApp.h"
#include "SceneManager.h"
#include "spinServerContext.h"

@implementation spinCocoaWrapper



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
    spin::spinApp::Instance().sceneManager_->clear();
    spin::spinApp::Instance().sceneManager_->clearUsers();
    spin::spinApp::Instance().sceneManager_->clearStates();
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
