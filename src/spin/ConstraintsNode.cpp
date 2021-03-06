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

#include "constraintsnode.h"
#include "scenemanager.h"
#include "spinapp.h"
#include "spinbasecontext.h"
#include "osgutil.h"

namespace spin
{


// ***********************************************************
// constructor:
ConstraintsNode::ConstraintsNode (SceneManager *sceneManager, const char* initID) : GroupNode(sceneManager, initID)
{
    this->setNodeType("ConstraintsNode");
    this->setName(this->getID() + ".ConstraintsNode");

    target_= gensym("NULL");
    constraintMode_ = BASIC;

    cubeSize_ = osg::Vec3(0,0,0);
    cubeOffset_ = osg::Vec3(0,0,0);

}

// ***********************************************************
// destructor
ConstraintsNode::~ConstraintsNode()
{

}


void ConstraintsNode::callbackUpdate(osg::NodeVisitor* nv)
{
    GroupNode::callbackUpdate(nv);
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
	this->target_ = gensym(id);
	BROADCAST(this, "ss", "setTarget", getTarget());
}

void ConstraintsNode::setConstraintMode (ConstraintMode m)
{
	this->constraintMode_ = m;
	BROADCAST(this, "si", "setConstraintMode", (int)constraintMode_);
}

void ConstraintsNode::setCubeSize(float xScale, float yScale, float zScale)
{
	cubeSize_ = osg::Vec3(xScale, yScale, zScale);
	BROADCAST(this, "sfff", "setCubeSize", xScale, yScale, zScale);
}

void ConstraintsNode::setCubeOffset(float x, float y, float z)
{
	cubeOffset_ = osg::Vec3(x, y, z);
	BROADCAST(this, "sfff", "setCubeOffset", x, y, z);
}

void ConstraintsNode::setTranslation (float x, float y, float z)
{
    osg::Vec3 v = osg::Vec3(x,y,z);

    if (spinApp::Instance().getContext()->isServer() || (!spinApp::Instance().getContext()->isServer() && (computationMode_==CLIENT_SIDE)))
	{

		if (constraintMode_ == DROP)
		{
			osg::ref_ptr<ReferencedNode> targetNode = dynamic_cast<ReferencedNode*>(target_->s_thing);
			if (targetNode.valid())
			{
				v = computeDropIntersection(targetNode.get(), x, y);
			}
		}

		if (cubeSize_.length())
		{
			//std::cout << "v=("<<v.x()<<","<<v.y()<<","<<v.z()<<"), cubeOffset="<<cubeOffset_.x()<<","<<cubeOffset_.y()<<","<<cubeOffset_.z()<<"), cubeSize="<<cubeSize_.x()<<","<<cubeSize_.y()<<","<<cubeSize_.z()<<")" << std::endl;

			float epsilon = 0.0001;
			if (v.x() > cubeOffset_.x() + cubeSize_.x()/2)
				v.x() = cubeOffset_.x() + cubeSize_.x()/2 - epsilon;
			if (v.y() > cubeOffset_.y() + cubeSize_.y()/2)
				v.y() = cubeOffset_.y() + cubeSize_.y()/2 - epsilon;
			if (v.z() > cubeOffset_.z() + cubeSize_.z()/2)
				v.z() = cubeOffset_.z() + cubeSize_.z()/2 - epsilon;

			if (v.x() < cubeOffset_.x() - cubeSize_.x()/2)
				v.x() = cubeOffset_.x() - cubeSize_.x()/2 + epsilon;
			if (v.y() < cubeOffset_.y() - cubeSize_.y()/2)
				v.y() = cubeOffset_.y() - cubeSize_.y()/2 + epsilon;
			if (v.z() < cubeOffset_.z() - cubeSize_.z()/2)
				v.z() = cubeOffset_.z() - cubeSize_.z()/2 + epsilon;
		}
	}

    GroupNode::setTranslation(v.x(), v.y(), v.z());
}


void ConstraintsNode::translate (float x, float y, float z)
{

    if (spinApp::Instance().getContext()->isServer() || (!spinApp::Instance().getContext()->isServer() && (computationMode_==CLIENT_SIDE)))
    {
    	if ((constraintMode_==BOUNCE)||(constraintMode_==COLLIDE)||(constraintMode_==COLLIDE_THRU)||(constraintMode_==STICK))
    	{
    		// reset the lastDrawable
    		lastDrawable = 0;
    		lastPrimitiveIndex = -1;
    		recursionCounter = 0;
    		applyConstrainedTranslation(osg::Vec3(x,y,z));
    	}
    	else {
    		osg::Vec3 newPos = this->getTranslation() + osg::Vec3(x,y,z);
    		setTranslation(newPos.x(),newPos.y(),newPos.z());
    	}
    }
}


void ConstraintsNode::move (float x, float y, float z)
{
    if (spinApp::Instance().getContext()->isServer() || (!spinApp::Instance().getContext()->isServer() && (computationMode_==CLIENT_SIDE)))
    {
    	osg::Vec3 v = this->getOrientationQuat() * osg::Vec3(x,y,z);

    	if ((constraintMode_==BOUNCE)||(constraintMode_==COLLIDE)||(constraintMode_==COLLIDE_THRU)||(constraintMode_==STICK))
    	{
    		lastDrawable = 0;
    		lastPrimitiveIndex = -1;
    		recursionCounter = 0;
    		applyConstrainedTranslation(v);
    	}
    	else {
    		osg::Vec3 newPos = this->getTranslation() + v;
    		setTranslation(newPos.x(),newPos.y(),newPos.z());
    	}
    }
}

/*
// attempt at this: http://forum.openscenegraph.org/viewtopic.php?p=23242
osg::Polytope ConstraintsNode::getPolytope()
{
    osg::Matrix thisMatrix = osg::computeLocalToWorld(this->currentNodePath_);
    
    osg::Vec3 local
    
    Pos = this->getTranslation();

    const osg::BoundingSphere& bs = this->getBound();
    float zMax = bs.center().z()+bs.radius();
    float zMin = bs.center().z()-bs.radius();
    
    osg::Plane plane;

    // RIGHT
    {
        osg::Vec3 start = this->getTranslation();
        osg::Vec3 end = start + osg::Vec3(bs.radius()+1,0,0);
        osgUtil::LineSegmentIntersector *intersector = new osgUtil::LineSegmentIntersector(start,end);
        osgUtil::IntersectionVisitor iv(intersector);
        this->accept(iv);

        if (intersector->containsIntersections())
        {
            // get last (furthest) intersection
            const osgUtil::LineSegmentIntersector::Intersection& intersection = *(--intersector->getIntersections().end());
            plane.set( (start-end)/bs.radius()+1, 
            
            
            
        }
}
*/

void ConstraintsNode::applyConstrainedTranslation(osg::Vec3 v)
{
	osg::Vec3 localPos = this->getTranslation();
	osg::ref_ptr<ReferencedNode> hitNode;
	ReferencedNode *testNode;

	// with really big velocities (or small enclosures), and if the surface
	// doesn't damp the velocity, it's possible to get see an infinite recursion
	// occur. So, we keep a recursionCounter, and stop prevent this occurrence:
	if (++recursionCounter > 10) 
    {
        //std::cout << "RECURSION" << std::endl;
        return;
    }
    
	/*
	std::cout << std::endl << "checking for collisions" << std::endl;
	std::cout << "start =  " << localPos.x()<<","<<localPos.y()<<","<<localPos.z() << std::endl;
	std::cout << "v =      " << v.x()<<","<<v.y()<<","<<v.z() << std::endl;
	*/

	osg::ref_ptr<ReferencedNode> targetNode = dynamic_cast<ReferencedNode*>(target_->s_thing);

	if (targetNode.valid())
	{
		// get current position (including offset from previous bounces)
		osg::Matrix thisMatrix = osg::computeLocalToWorld(currentNodePath_);

        
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

					//std::cout << this->getID() << " collision!!! with " << (*itr).drawable->getName() << "[" << (*itr).primitiveIndex << "] @ " << std::endl;
					//std::cout << "localHitPoint:\t" << localHitPoint.x()<<","<<localHitPoint.y()<<","<<localHitPoint.z() << std::endl;
					//std::cout << "localHitNormal:\t" << localHitNormal.x()<<","<<localHitNormal.y()<<","<<localHitNormal.z() << std::endl;


					// For BOUNCE mode, we need to check if we've intersected
					// with the same primitive again. This may occur since we
					// do recursive translations after repositioning the node at
					// the bounce point (and there may be numerical imprecision)
					// If so, we skip this intersection:
					if ((constraintMode_==BOUNCE) &&
					    (lastDrawable.get()==(*itr).drawable.get()) &&
						    (lastPrimitiveIndex==(int)(*itr).primitiveIndex))
					{
						//std::cout << "... skipping" << std::endl;
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
					//std::cout << "rot =     " << rot.x()<<","<<rot.y()<<","<<rot.z()<<","<<rot.w() << " ... angle=" << acos(rot.w())*2 << ", indegrees=" << osg::RadiansToDegrees(acos(rot.w()))*2 << std::endl;

					// the surface normal may be in the opposite direction
					// from us. If so, we need to flip it:
					if (acos(rot.w()) * 2 > osg::PI_2)
					{
						localHitNormal *= -1;
						rot.makeRotate(dirVec, localHitNormal);
						//std::cout << "flipped = " << rot.x()<<","<<rot.y()<<","<<rot.z()<<","<<rot.w() << " ... angle=" << acos(rot.w())*2 << ", indegrees=" << osg::RadiansToDegrees(acos(rot.w()))*2 << std::endl;
					}


					osg::Vec3 rotEulers = QuatToEuler(rot);
					//std::cout << "newHitNormal:\t" << localHitNormal.x()<<","<<localHitNormal.y()<<","<<localHitNormal.z() << std::endl;


					if ((constraintMode_==COLLIDE)||(constraintMode_==COLLIDE_THRU)||(constraintMode_==STICK))
					{
						// Let the collisionPoint be just a bit before the real
						// hitpoint (to avoid numerical imprecision placing the
						// node behind the plane):
						osg::Vec3 collisionPoint = localHitPoint - (localHitNormal * 0.01);

						// place the node at the collision point
						setTranslation(collisionPoint.x(), collisionPoint.y(), collisionPoint.z());

						BROADCAST(this, "ssfff", "collide", hitNode->getID().c_str(), osg::RadiansToDegrees(rotEulers.x()), osg::RadiansToDegrees(rotEulers.y()), osg::RadiansToDegrees(rotEulers.z()));


						if (constraintMode_==COLLIDE)
						{
                            // SLIDE along the hit plane with the left over energy:
                            // ie, project the remaining vector onto the surface we
                            // just intersected with.
                            //
                            // using:
                            //   cos(theta) = distToSurface / remainderVector length
                            //
                            double cosTheta = (dirVec * -localHitNormal) ; // dot product
                            osg::Vec3 remainderVector = (localPos + v) - localHitPoint;
                            double distToSurface = cosTheta * remainderVector.length();
                            osg::Vec3 slideVector = remainderVector + (localHitNormal * distToSurface);

                            // pseudo-recursively apply remainder of bounce:
                            applyConstrainedTranslation( slideVector );
						}
                        else if (constraintMode_==COLLIDE_THRU)
						{
                            // allow the node to pass thru (ie, just apply the
                            // translation):
                            //setTranslation(v.x(), v.y(), v.z());
                            osg::Vec3 newPos = localPos + v;
	                        setTranslation(newPos.x(), newPos.y(), newPos.z());
                        }

						return;
					}


					else if (constraintMode_==BOUNCE)
					{
						// bounce returns a translation mirrored about the hit
						// normal to the surface



						// the new direction vector is a rotated version
						// of the original, about the localHitNormal:
						//osg::Vec3 newDir = (rot * 2.0) * -dirVec;
						osg::Vec3 newDir = (rot * (rot * -dirVec));
						newDir.normalize();
						//std::cout << "newDir = " << newDir.x()<<","<<newDir.y()<<","<<newDir.z() << std::endl;


						// amount of distance still to travel after bounce:
						double dist = v.length() - (localHitPoint-localPos).length();
						//std::cout << "dist =   " << dist << std::endl;



						// in node is translating (ie, changing position
						// independently from it's orientatino), then we need to
						// flip the velocity vector.
						if (velocityMode_ == GroupNode::TRANSLATE)
						{
							osg::Vec3 newVel = newDir * velocity_.length();
							setVelocity(newVel.x(), newVel.y(), newVel.z());
						}

						// if the node is moving along it's orientation vector,
						// we need to update the orientation of so that it
						// points in 'bounced' direction
						else if (velocityMode_ == GroupNode::MOVE)
						{
							osg::Quat newQuat;
							newQuat.makeRotate(osg::Vec3(0,1,0),  newDir);
							osg::Vec3 newRot = Vec3inDegrees(QuatToEuler(newQuat));
							//std::cout << "newrot = " << newRot.x()<<","<<newRot.y()<<","<<newRot.z() << std::endl;
							setOrientation(newRot.x(), newRot.y(), newRot.z());

						}

						// the new position will be just a hair before hitpoint
						// (because numerical imprecision may place the hitpoint
						// slightly beyond the surface, and when we bounce the
						// node, we don't want to  intersect with the same
						// surface again)
						//double HAIR = 0.0000001;
						double HAIR = 0.00001;
						setTranslation(localHitPoint.x()-dirVec.x()*HAIR, localHitPoint.y()-dirVec.y()*HAIR, localHitPoint.z()-dirVec.z()*HAIR);
						//setTranslation(localHitPoint.x(), localHitPoint.y(), localHitPoint.z());

                        //std::cout << "rotEulers = " << osg::RadiansToDegrees(rotEulers.x())<<","<<osg::RadiansToDegrees(rotEulers.y())<<","<<osg::RadiansToDegrees(rotEulers.z()) << std::endl;
                        BROADCAST(this, "ssfff", "bounce", hitNode->getID().c_str(), osg::RadiansToDegrees(rotEulers.x()), osg::RadiansToDegrees(rotEulers.y()), osg::RadiansToDegrees(rotEulers.z()));

						// pseudo-recursively apply remainder of bounce:
						applyConstrainedTranslation(newDir*dist);

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
std::vector<lo_message> ConstraintsNode::getState () const
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

} // end of namespace spin

