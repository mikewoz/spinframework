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

#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>

#include "ConstraintsNode.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "osgUtil.h"




using namespace std;



// ***********************************************************
// constructor:
ConstraintsNode::ConstraintsNode (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{
    nodeType = "ConstraintsNode";
    this->setName(string(id->s_name) + ".ConstraintsNode");

    _target= gensym("NULL");
    _mode = BASIC;

    _cubeSize = osg::Vec3(0,0,0);
    _cubeOffset = osg::Vec3(0,0,0);

}

// ***********************************************************
// destructor
ConstraintsNode::~ConstraintsNode()
{

}


void ConstraintsNode::callbackUpdate()
{
    GroupNode::callbackUpdate();
}

// *****************************************************************************
// *****************************************************************************


osg::Vec3 computeDropIntersection(osg::Node* subgraph,float x,float y)
{
    const osg::BoundingSphere& bs = subgraph->getBound();
    float zMax = bs.center().z()+bs.radius();
    float zMin = bs.center().z()-bs.radius();

    osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
        new osgUtil::LineSegmentIntersector(osg::Vec3(x,y,zMin),osg::Vec3(x,y,zMax));

    osgUtil::IntersectionVisitor iv(intersector.get());

    subgraph->accept(iv);

    if (intersector->containsIntersections())
    {
        return intersector->getFirstIntersection().getWorldIntersectPoint();
    }

    return osg::Vec3(x,y,0.0f);
}


// *****************************************************************************
// *****************************************************************************

void ConstraintsNode::setTarget(const char *id)
{
	// we don't check here if the node actually exists... just set the param
	this->_target = gensym(id);
	BROADCAST(this, "ss", "setTarget", getTarget());
}

void ConstraintsNode::setConstraintMode (constraintMode m)
{
	this->_mode = m;
	BROADCAST(this, "si", "setConstraintMode", (int)_mode);
}

void ConstraintsNode::setCubeSize(float xScale, float yScale, float zScale)
{
	_cubeSize = osg::Vec3(xScale, yScale, zScale);
	BROADCAST(this, "sfff", "setCubeSize", xScale, yScale, zScale);
}

void ConstraintsNode::setCubeOffset(float x, float y, float z)
{
	_cubeOffset = osg::Vec3(x, y, z);
	BROADCAST(this, "sfff", "setCubeOffset", x, y, z);
}

void ConstraintsNode::setTranslation (float x, float y, float z)
{
    osg::Vec3 v = osg::Vec3(x,y,z);

    if (!sceneManager->isGraphical())
	{

		if (_mode == DROP)
		{
			osg::ref_ptr<ReferencedNode> targetNode = dynamic_cast<ReferencedNode*>(_target->s_thing);
			if (targetNode.valid())
			{
				v = computeDropIntersection(targetNode.get(), x, y);
			}
		}

		if (_cubeSize.length())
		{
			//std::cout << "v=("<<v.x()<<","<<v.y()<<","<<v.z()<<"), cubeOffset="<<_cubeOffset.x()<<","<<_cubeOffset.y()<<","<<_cubeOffset.z()<<"), cubeSize="<<_cubeSize.x()<<","<<_cubeSize.y()<<","<<_cubeSize.z()<<")" << std::endl;

			float epsilon = 0.0001;
			if (v.x() > _cubeOffset.x() + _cubeSize.x()/2)
				v.x() = _cubeOffset.x() + _cubeSize.x()/2 - epsilon;
			if (v.y() > _cubeOffset.y() + _cubeSize.y()/2)
				v.y() = _cubeOffset.y() + _cubeSize.y()/2 - epsilon;
			if (v.z() > _cubeOffset.z() + _cubeSize.z()/2)
				v.z() = _cubeOffset.z() + _cubeSize.z()/2 - epsilon;

			if (v.x() < _cubeOffset.x() - _cubeSize.x()/2)
				v.x() = _cubeOffset.x() - _cubeSize.x()/2 + epsilon;
			if (v.y() < _cubeOffset.y() - _cubeSize.y()/2)
				v.y() = _cubeOffset.y() - _cubeSize.y()/2 + epsilon;
			if (v.z() < _cubeOffset.z() - _cubeSize.z()/2)
				v.z() = _cubeOffset.z() - _cubeSize.z()/2 + epsilon;
		}
	}

    GroupNode::setTranslation(v.x(), v.y(), v.z());
}


void ConstraintsNode::translate (float x, float y, float z)
{

    if ( !sceneManager->isGraphical() )
    {
    	if ((_mode==BOUNCE)||(_mode==COLLIDE))
    	{
    		// reset the lastDrawable
    		lastDrawable = 0;
    		lastPrimitiveIndex = -1;
    		applyConstrainedTranslation(osg::Vec3(x,y,z));
    	}
    	else {
    		osg::Vec3 newPos = mainTransform->getPosition() + osg::Vec3(x,y,z);
    		setTranslation(newPos.x(),newPos.y(),newPos.z());
    	}
    }
}


void ConstraintsNode::move (float x, float y, float z)
{
    if ( !sceneManager->isGraphical() )
    {
    	osg::Vec3 v = mainTransform->getAttitude() * osg::Vec3(x,y,z);

    	if ((_mode==BOUNCE)||(_mode==COLLIDE))
    	{
    		lastDrawable = 0;
    		lastPrimitiveIndex = -1;
    		applyConstrainedTranslation(v);
    	}
    	else {
    		osg::Vec3 newPos = mainTransform->getPosition() + v;
    		setTranslation(newPos.x(),newPos.y(),newPos.z());
    	}
    }
}


void ConstraintsNode::applyConstrainedTranslation(osg::Vec3 v)
{
	osg::Vec3 localPos = this->getTranslation();
	osg::ref_ptr<ReferencedNode> hitNode;
	ReferencedNode *testNode;

	/*
	std::cout << std::endl << "checking for collisions" << std::endl;
	std::cout << "start =  " << localPos.x()<<","<<localPos.y()<<","<<localPos.z() << std::endl;
	std::cout << "v =      " << v.x()<<","<<v.y()<<","<<v.z() << std::endl;
	*/

	osg::ref_ptr<ReferencedNode> targetNode = dynamic_cast<ReferencedNode*>(_target->s_thing);

	if (targetNode.valid())
	{
		// get current position (including offset from previous bounces)
		osg::Matrix thisMatrix = osg::computeLocalToWorld(this->currentNodePath);
		osg::Vec3 worldPos = thisMatrix.getTrans();

		// set up intersector:
		osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(worldPos, worldPos + v);
		osgUtil::IntersectionVisitor iv(intersector.get());

		// apply intersector:
		targetNode->accept(iv);

		if (intersector->containsIntersections())
		{
			osgUtil::LineSegmentIntersector::Intersections intersections;
			osgUtil::LineSegmentIntersector::Intersections::iterator itr;

			intersections = intersector->getIntersections();

			for (itr = intersections.begin(); itr != intersections.end(); ++itr)
			{
				//std::cout << "testing intersection with " << (*itr).nodePath[0]->getName() << std::endl;

				// first check if the hit is with our target (should be first in
				// the nodepath):

				testNode = dynamic_cast<ReferencedNode*>((*itr).nodePath[0]);
				if (testNode!=targetNode) continue;

				// The intersection has a nodepath that we have to walk down to
				// see which exact node has been hit. It's possible that there
				// are other SPIN nodes in the targetNode's subgraph that are
				// the real source of intersection:

				hitNode = 0; testNode = 0;
				for (unsigned int i=0; i<(*itr).nodePath.size(); i++)
				{
					testNode = dynamic_cast<ReferencedNode*>((*itr).nodePath[i]);
					if (testNode) hitNode = testNode;
				}

				if (hitNode.valid())
				{

					//std::cout << id->s_name << " collision!!! with " << (*itr).drawable->getName() << "[" << (*itr).primitiveIndex << "] @ " << std::endl;
					//std::cout << "localHitPoint:\t" << localHitPoint.x()<<","<<localHitPoint.y()<<","<<localHitPoint.z() << std::endl;
					//std::cout << "localHitNormal:\t" << localHitNormal.x()<<","<<localHitNormal.y()<<","<<localHitNormal.z() << std::endl;


					// For BOUNCE mode, we need to check if we've intersected
					// with the same primitive again. This may occur since we
					// do recursive translations after repositioning the node at
					// the bounce point (and there may be numerical imprecision)
					// If so, we skip this intersection:
					if ((_mode==BOUNCE) &&
					    (lastDrawable.get()==(*itr).drawable.get()) &&
						    (lastPrimitiveIndex==(*itr).primitiveIndex))
					{
						continue;
					}
					else
					{
						lastDrawable=(*itr).drawable;
						lastPrimitiveIndex=(*itr).primitiveIndex;
					}


					osg::Vec3 localHitPoint = (*itr).getWorldIntersectPoint();
					osg::Vec3 localHitNormal = (*itr).getWorldIntersectNormal();
					localHitNormal.normalize();


					// current direction vector:
					osg::Vec3 dirVec = v;
					dirVec.normalize();

					// Find the rotation between the direction vector and the
					// surface normal at the hit point:
					osg::Quat rot;
					rot.makeRotate(dirVec, localHitNormal);
					//std::cout << "rot =    " << rot.x()<<","<<rot.y()<<","<<rot.z()<<","<<rot.w() << " ... angle=" << acos(rot.w())*2 << ", indegrees=" << osg::RadiansToDegrees(acos(rot.w()))*2 << std::endl;

					// the surface normal may be in the opposite direction
					// from us. If so, we need to flip it:
					if (acos(rot.w()) * 2 > osg::PI_2)
					{
						localHitNormal *= -1;
						rot.makeRotate(dirVec, localHitNormal);
					}

					//std::cout << "newHitNormal:\t" << localHitNormal.x()<<","<<localHitNormal.y()<<","<<localHitNormal.z() << std::endl;

					osg::Vec3 rotEulers = QuatToEuler(rot*2);
					//std::cout << "rot_eu = " << osg::RadiansToDegrees(rotEulers.x())<<","<<osg::RadiansToDegrees(rotEulers.y())<<","<<osg::RadiansToDegrees(rotEulers.z()) << std::endl;



					if (_mode==COLLIDE)
					{
						// just set translation to the hitpoint, but back
						// along the normal by a bit

						osg::Vec3 collisionPoint = localHitPoint - (localHitNormal * 0.01);
						setTranslation(collisionPoint.x(), collisionPoint.y(), collisionPoint.z());

						BROADCAST(this, "ssfff", "collide", hitNode->id->s_name, osg::RadiansToDegrees(rotEulers.x()), osg::RadiansToDegrees(rotEulers.y()), osg::RadiansToDegrees(rotEulers.z()));

						return;
					}


					else if (_mode==BOUNCE)
					{
						// bounce returns a translation mirrored about the hit
						// normal to the surface



						// the new direction vector is a rotated version
						// of the original, about the localHitNormal:
						osg::Vec3 newDir = (rot * 2.0) * -dirVec;
						newDir.normalize();
						//std::cout << "newDir = " << newDir.x()<<","<<newDir.y()<<","<<newDir.z() << std::endl;


						// amount of distance still to travel after bounce:
						double dist = v.length() - (localHitPoint-localPos).length();
						//std::cout << "dist =   " << dist << std::endl;



						// in node is translating (ie, changing position
						// independently from it's orientatino), then we need to
						// flip the velocity vector.
						if (_velocityMode == GroupNode::TRANSLATE)
						{
							osg::Vec3 newVel = newDir * _velocity.length();
							setVelocity(newVel.x(), newVel.y(), newVel.z());
						}

						// if the node is moving along it's orientation vector,
						// we need to update the orientation of so that it
						// points in 'bounced' direction
						else if (_velocityMode == GroupNode::MOVE)
						{
							osg::Quat newQuat;
							newQuat.makeRotate(osg::Vec3(0,1,0),  newDir);
							osg::Vec3 newRot = Vec3inDegrees(QuatToEuler(newQuat));
							//std::cout << "newrot = " << newRot.x()<<","<<newRot.y()<<","<<newRot.z() << std::endl;
							setOrientation(newRot.x(), newRot.y(), newRot.z());

						}

						// the new position will be just at the hitpoint (plus
						// a little bit, so that it doesn't intersect with the
						// same surface again)
						//osg::Vec3 hitPoint_adj = localHitPoint + (newDir*0.00000001*dist);
						setTranslation(localHitPoint.x(), localHitPoint.y(), localHitPoint.z());

						// pseudo-recursively apply remainder of bounce:
						applyConstrainedTranslation(newDir*dist);

						BROADCAST(this, "ssfff", "bounce", hitNode->id->s_name, osg::RadiansToDegrees(rotEulers.x()), osg::RadiansToDegrees(rotEulers.y()), osg::RadiansToDegrees(rotEulers.z()));

						return;

					}
				}
			}
		} //else std::cout << "no intersections" << std::endl;
	}


	// no intersections, so just do regular translation:
	osg::Vec3 newPos = localPos + v;
	setTranslation(newPos.x(), newPos.y(), newPos.z());
}



// *****************************************************************************
std::vector<lo_message> ConstraintsNode::getState ()
{
    // inherit state from base class
    std::vector<lo_message> ret = GroupNode::getState();

    lo_message msg;
    osg::Vec3 v;

    msg = lo_message_new();
    lo_message_add(msg, "ss", "setTarget", this->getTarget());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "setConstraintMode", this->getConstraintMode());
    ret.push_back(msg);

    msg = lo_message_new();
    v = getCubeSize();
    lo_message_add(msg, "sfff", "setCubeSize", v.x(), v.y(), v.z());
    ret.push_back(msg);

    msg = lo_message_new();
    v = getCubeOffset();
    lo_message_add(msg, "sfff", "setCubeOffset", v.x(), v.y(), v.z());
    ret.push_back(msg);

    return ret;
}
