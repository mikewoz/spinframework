//
//  spinCocoaWrapper.h
//  spinframework
//
//  Created by Mike Wozniewski on 12-01-21.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#import <Foundation/Foundation.h>

@interface spinCocoaWrapper : NSObject


- (IBAction)refresh:(id)sender;
- (IBAction)refreshSubscribers:(id)sender;
- (IBAction)clearScene:(id)sender;
- (IBAction)debugContext:(id)sender;
- (IBAction)debugNodes:(id)sender;
- (IBAction)debugStateSets:(id)sender;
- (IBAction)debugSceneGraph:(id)sender;

@end
