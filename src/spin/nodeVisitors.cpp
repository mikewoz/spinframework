#include "nodeVisitors.h"

#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/Switch>

#include <iostream>

void UpdateSceneVisitor::apply(osg::Node &node) {
    traverse(node);
}


void UpdateSceneVisitor::apply(osg::Group &node)  {
    ReferencedNode *n;
    if (n=dynamic_cast<ReferencedNode*>(&node)) {
        //printf("UpdateSceneVisitor: we got ourselves a node here....\n");
        n->callbackUpdate();
    }
    traverse(node);
}
