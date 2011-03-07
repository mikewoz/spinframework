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

#include <osg/Vec4>
#include <osg/Array>
#include <osg/ShapeDrawable>
#include <osgUtil/Optimizer>
#include <osgUtil/SmoothingVisitor>
#include <osg/PositionAttitudeTransform> 

#include "osgUtil.h"

/**
 * Returns an absolute angle difference between v1 and v2 (with no notion of
 * which is ahead or behind the other). Returned angle is from 0 to PI
 */
double AngleBetweenVectors(osg::Vec3 v1, osg::Vec3 v2)
{
	// normalize vectors (note: this must be done alone, not within any vector arithmetic. why?!)
	v1.normalize();
	v2.normalize();
		
	// Get the dot product of the vectors
	double dotProduct = v1 * v2;
	
	// for acos, the value has to be between -1.0 and 1.0, but due to numerical imprecisions it sometimes comes outside this range
	if (dotProduct > 1.0) dotProduct = 1.0;
	if (dotProduct < -1.0) dotProduct = -1.0;

	// Get the angle in radians between the 2 vectors (should this be -acos ? ie, negative?)
	double angle = acos( dotProduct );
		
	// Here we make sure that the angle is not a -1.#IND0000000 number, which means indefinite
	if (isnan(angle))  //__isnand(x)
		return 0;

	// Return the angle in radians
	return( angle );
}

/**
 * Returns a signed angle of rotation that describes the rotation from v1 to v2,
 * assuming that one axis is null. 1=X_AXIS, 2=Y_AXIS, 3=Z_AXIS
 */
double AngleBetweenVectors(osg::Vec3 v1, osg::Vec3 v2, int nullAxis)
{
	// normalize vectors (note: this must be done alone, not within any vector arithmetic. why?!)
	v1.normalize();
	v2.normalize();

	double angle = 0;
	switch (nullAxis)
	{
		case 1: // X_AXIS is ignored
			angle = atan2(v2.z(),v2.y()) - atan2(v1.z(),v1.y());
			break;
		case 2: // Y_AXIS is ignored
			angle = atan2(v2.z(),v2.x()) - atan2(v1.z(),v1.x());
			break;
		case 3: // Z_AXIS is ignored
			angle = atan2(v2.y(),v2.x()) - atan2(v1.y(),v1.x());
			break;
	}

	// angle will be from -90 to 270 for some reason, so convert to -PI,PI
	if (angle>osg::PI) angle -= 2 * osg::PI;
	if (angle<-osg::PI) angle += 2 * osg::PI;
	
	return(angle);
}
		
/**
 * Returns a quaternion that represents the rotation from v1 to v2
 */
osg::Quat RotationBetweenVectors(osg::Vec3 v1, osg::Vec3 v2)
{
	v1.normalize();
	v2.normalize();
	
	// URGH. THIS SHOULD BE DEPRECATED. USE osg::Quat::makeRotate INSTEAD!!

	// Get the dot product of the vectors
	double dotProduct = v1 * v2;
	osg::Vec3 crossProduct = v1 ^ v2;
	
	/*
    double qw = (double) sqrt(v1.length2()*v2.length2()) + dotProduct;
	
	if (qw < 0.0001) { // vectors are 180 degrees apart
		return osg::Quat(0,-v1.z(),v1.y(),v1.x());
	}
	
    return osg::Quat(qw, crossProduct);
    */

/*
	crossProduct.normalize();
	double angle = acos(dotProduct);
	return osg::Quat(cos(angle/2), crossProduct*sin(angle/2));
*/

	crossProduct.normalize();
	return osg::Quat(acos(dotProduct), crossProduct);

} 


osg::Vec3 rotateAroundAxis(osg::Vec3 v, osg::Vec3 axis, float angle)
{
	float c, s, t;
	c = cos(angle);
	s = sin(angle);
	t = 1 - c;

	axis.normalize();
	float w1 = (t * axis.x() * axis.x() +          c  ) * v.x()
			 + (t * axis.x() * axis.y() + s * axis.z()) * v.y()
			 + (t * axis.x() * axis.z() - s * axis.y()) * v.z();
	
	float w2 = (t * axis.x() * axis.y() - s * axis.z()) * v.x()
			 + (t * axis.y() * axis.y() +          c  ) * v.y()
			 + (t * axis.y() * axis.z() + s * axis.x()) * v.z();
	
	float w3 = (t * axis.x() * axis.z() + s * axis.y()) * v.x()
			 + (t * axis.y() * axis.z() - s * axis.x()) * v.y()
			 + (t * axis.z() * axis.z() +          c  ) * v.z();
	
	osg::Vec3 w = osg::Vec3(w1,w2,w3);
	w.normalize();
	
	return w;
}

osg::Quat EulerToQuat (float roll, float pitch, float yaw)
{
	float cr, cp, cy, sr, sp, sy, cpcy, spsy;
	
	pitch = osg::DegreesToRadians(pitch);
	roll = osg::DegreesToRadians(roll);
	yaw = osg::DegreesToRadians(yaw);

	// calculate trig identities
	cr = cos(roll/2);
	cp = cos(pitch/2);
	cy = cos(yaw/2);
	
	sr = sin(roll/2);
	sp = sin(pitch/2);
	sy = sin(yaw/2);
	
	cpcy = cp * cy;
	spsy = sp * sy;
	
	return osg::Quat( sr * cpcy - cr * spsy,
							cr * sp * cy + sr * cp * sy,
							cr * cp * sy - sr * sp * cy,
							cr * cpcy + sr * spsy );
}

osg::Vec3 QuatToEuler(osg::Quat q)
{
	// based on:
	// en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	
	double pitch, roll, yaw;
	
	roll = asin( 2* (q.x()*q.z() - q.w()*q.y()) );
	yaw = atan2( 2* (q.x()*q.y() + q.z()*q.w()) , 1 - (2* (q.y()*q.y() + q.z()*q.z())) );
	pitch =  atan2( 2* (q.x()*q.w() + q.y()*q.z()) , 1 - (2* (q.z()*q.z() + q.w()*q.w())) );
	
	// ignore isnan:
	if (isnan(roll)) roll = 0.0;
	if (isnan(yaw)) yaw = 0.0;
	if (isnan(pitch)) pitch = 0.0;

	// correct to OSG's coordinate system:
	pitch = osg::PI-pitch;
	roll = -roll;

	// ensure in range of [-PI,PI]
	if (roll>osg::PI) roll -= 2 * osg::PI;
	else if (roll<-osg::PI) roll += 2 * osg::PI;
	if (yaw>osg::PI) yaw -= 2 * osg::PI;
	else if (yaw<-osg::PI) yaw += 2 * osg::PI;
	if (pitch>osg::PI) pitch -= 2 * osg::PI;
	else if (pitch<-osg::PI) pitch += 2 * osg::PI;

	return osg::Vec3(pitch,roll,yaw);
}

osg::Vec3 QuatToEuler_old(osg::Quat q)
{
	double pitch, roll, yaw;
	
	double test = q.x()*q.y() + q.z()*q.w();
	
	if (test > 0.499) { // singularity at north pole
		yaw = 2 * atan2(q.x(),q.w());
		pitch = osg::PI/2;
		roll = 0;
	}
	
	else if (test < -0.499) { // singularity at south pole
		yaw = -2 * atan2(q.x(),q.w());
		pitch = - osg::PI/2;
		roll = 0;
	}
	
	else {
	    double sqx = q.x()*q.x();
	    double sqy = q.y()*q.y();
	    double sqz = q.z()*q.z();
	    yaw =  atan2(2*q.y()*q.w()-2*q.x()*q.z() , 1 - 2*sqy - 2*sqz);
	    pitch = asin(2*test);
		roll = atan2(2*q.x()*q.w()-2*q.y()*q.z() , 1 - 2*sqx - 2*sqz);
	}

	return osg::Vec3(yaw,pitch,roll); // note order is messed up on purpose
}

/*
osg::Geode* createGrid(int radius, osg::Vec4 color)
{
	int i,j;

	int numVertices = 8 + (radius*8);
	
	osg::Geode* gridGeode = new osg::Geode();
	
	osg::Geometry* gridLines = new osg::Geometry();
	
	osg::Vec3Array* normals = new osg::Vec3Array;
	osg::Vec3 myCoords[numVertices];
	osg::Vec4Array* colors = new osg::Vec4Array;
	
	for (i = 0; i <= radius; i++)
	{
		myCoords[i+0+(i*7)] = osg::Vec3(-radius,  i, 0.0);
		myCoords[i+1+(i*7)] = osg::Vec3( radius,  i, 0.0);
		myCoords[i+2+(i*7)] = osg::Vec3(-radius, -i, 0.0);
		myCoords[i+3+(i*7)] = osg::Vec3( radius, -i, 0.0);

		myCoords[i+4+(i*7)] = osg::Vec3( i, -radius, 0.0);
		myCoords[i+5+(i*7)] = osg::Vec3( i,  radius, 0.0);
		myCoords[i+6+(i*7)] = osg::Vec3(-i, -radius, 0.0);
		myCoords[i+7+(i*7)] = osg::Vec3(-i,  radius, 0.0);

		for (j=0; j<8; j++)
		{
			if (i==0) colors->push_back(osg::Vec4(1.0, 1.0, 1.0, 0.0));
			else colors->push_back(color);
			normals->push_back(osg::Vec3(0.0, 0.0, 1.0));
		}
	}

	//for (i = 0; i < numVertices; i++)
	//	std::cout << i << ": (" << myCoords[i].x() << "," << myCoords[i].y() << "," << myCoords[i].z() << ")" << std::endl;
	
	
	osg::Vec3Array* vertices = new osg::Vec3Array(numVertices,myCoords);
	
	// pass the created vertex array to the points geometry object.
	gridLines->setVertexArray(vertices);
	
	gridLines->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, numVertices));
	
	// pass the color arry to points geometry, note the binding to tell the geometry
	// that only use one color for the whole object.
	gridLines->setColorArray(colors);
	//gridLines->setColorBinding(osg::Geometry::BIND_OVERALL);
	
	// create the normals
	//gridLines->setNormalArray(normals);
	//gridLines->setNormalBinding(osg::Geometry::BIND_OVERALL);
	osgUtil::SmoothingVisitor::smooth(*gridLines);
	
	gridGeode->addDrawable(gridLines);
	
	osgUtil::Optimizer optimizer;
	optimizer.optimize(gridGeode);

	osg::StateSet* stateset = new osg::StateSet;
	stateset->setMode( GL_LIGHTING, osg::StateAttribute::OFF ); //(GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF)
	gridGeode->setStateSet( stateset );
	
	gridGeode->setName("worldGrid");

	
	return gridGeode;
}
*/

osg::Geometry* createPlane(float halfLength, osg::Vec4 color)
{
	// create Geometry object to store all the vetices and lines primtive.
	osg::Geometry* planeGeom = new osg::Geometry();
		
	// note, anticlockwsie ordering.
	osg::Vec3 myCoords[] =
	{
		osg::Vec3( -halfLength, 0,  halfLength ),
		osg::Vec3( -halfLength, 0, -halfLength ),
		osg::Vec3(  halfLength, 0, -halfLength ),
		osg::Vec3(  halfLength, 0,  halfLength )
	};
	int numCoords = sizeof(myCoords)/sizeof(osg::Vec3);
		
	// pass the created vertex array to the points geometry object.
	planeGeom->setVertexArray(new osg::Vec3Array(numCoords,myCoords));
		
	// set the normal in the same way color.
	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
	planeGeom->setNormalArray(normals);
	planeGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

	osg::Vec4Array* colors = new osg::Vec4Array(1);
	//(*colors)[0].set(1.0f,1.0f,1.0f,1.0f);
	(*colors)[0].set(color.x(), color.y(), color.z(), color.w());
	planeGeom->setColorArray(colors);
	planeGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

	osg::Vec2Array* texcoords = new osg::Vec2Array(4);
	(*texcoords)[0].set(0.0f,1.0f);
	(*texcoords)[1].set(0.0f,0.0f);
	(*texcoords)[2].set(1.0f,0.0f);
	(*texcoords)[3].set(1.0f,1.0f);
	planeGeom->setTexCoordArray(0,texcoords);

	planeGeom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));
	
	return planeGeom;
}


osg::Geode* createHollowSphere(float radius, osg::Vec4 color)
{
	osg::Geode* sphereGeode = new osg::Geode;
	
	osg::Sphere* s = new osg::Sphere( osg::Vec3(0.0, 0.0, 0.0), radius);
	osg::ShapeDrawable* sDrawable = new osg::ShapeDrawable( s );
	sDrawable->setColor( color );
	sphereGeode->addDrawable( sDrawable );
	
	return sphereGeode;
}



osg::Geode* createWireframeRolloff(int /*rolloff*/, float /*distortion*/, float /*scale*/, osg::Vec4 /*color*/)
{
	//int samples = 30; // the number of times we sample the rolloff table
	//int resolution = 10; // level of detail
	//int j,currIndex;
	
	
	//float tableIndex, angle, tableValue, step;
	
	osg::Geode* triGeode = new osg::Geode();

	// TODO: given the rolloff id, get the full table from the database and draw it in 3D:
	/*	
	osg::Geometry* triFan = new osg::Geometry();
	osg::Vec3Array* normals = new osg::Vec3Array;
	osg::Vec3 myCoords[ (samples*resolution)+2 ];
	osg::Vec4Array* colors = new osg::Vec4Array;	
	

	myCoords[0] = osg::Vec3(0.0, 0.0, 0.0);
	colors->push_back(color);
	normals->push_back(osg::Vec3(0.0, 1.0, 0.0) );

	//std::cout << "creating new radiationGeode from rolloffTable: " << rolloff->tableName->s_name << ", which has " << rolloff->tableSize << " entries. Distortion=" << distortion << std::endl;


	for (i=0; i<samples; i++)
	{

		tableIndex = (float) (i+1)/samples;
		angle = osg::DegreesToRadians(tableIndex*180);
		tableValue = rolloff->getValue(tableIndex * distortion); // length of vector
		
		//std::cout << "Sample " << i << ": tableIndex=" << tableIndex << " [angle=" << angle << ", cos()="<< cos(angle) << ", sin()=" << sin(angle) << "], tableValue=" << tableValue << std::endl;
		
		for (j=0; j<resolution; j++)
		{
			currIndex = (i*resolution) + j;

			// j goes from 0 -> resolution
			// therefore, angular step = cos((j/resolution)*2*osg::PI) ... which goes from cos(0) -> cos(2PI)
			
			step = ((float) j/resolution) * 2*osg::PI;
			myCoords[currIndex+1] = osg::Vec3(tableValue*cos(angle)*cos(step), tableValue*cos(angle), tableValue*sin(angle)*sin(step));

			//std::cout << "  vector " << j << ": step=" << step << ", cos(step)="<<cos(step) << ", sin(step)="<<sin(step) << " ... (" << myCoords[currIndex+1].x() << "," << myCoords[currIndex+1].y() << "," << myCoords[currIndex+1].z() << ")" << std::endl;

      normals->push_back(myCoords[currIndex]^myCoords[currIndex+1]);
      colors->push_back(color);
		}
		
	}


	osg::Vec3Array* vertices = new osg::Vec3Array( (samples*resolution)+2, myCoords );

	// pass the created vertex array to the points geometry object.
	triFan->setVertexArray(vertices);
	
	triFan->addPrimitiveSet(new osg::DrawArrays( osg::PrimitiveSet::TRIANGLE_FAN,0, (samples*resolution)+2 ));
	
	// pass the color arry to points geometry, note the binding to tell the geometry
	// that only use one color for the whole object.
	triFan->setColorArray(colors);
	triFan->setColorBinding(osg::Geometry::BIND_OVERALL);
	
	// create the normals
	triFan->setNormalArray(normals);
	triFan->setNormalBinding(osg::Geometry::BIND_OVERALL);
	osgUtil::SmoothingVisitor::smooth(*triFan);
	
	triGeode->addDrawable(triFan);
	
	osgUtil::Optimizer optimizer;
	optimizer.optimize(triGeode); */

	return triGeode;
}


osg::Geode* createHollowCone(float length, float radius, osg::Vec4 color)
{

	int grain = 50; // level of detail
	
	osg::Geode* triGeode = new osg::Geode();
	osg::Geometry* triFan = new osg::Geometry();
	osg::Vec3Array* normals = new osg::Vec3Array;
	osg::Vec3 myCoords[grain + 2];
	osg::Vec4Array* colors = new osg::Vec4Array;

	// tip
	myCoords[0] = osg::Vec3(0.0, 0.0, 0.0);
	colors->push_back(color);
	normals->push_back(osg::Vec3(0.0, 1.0, 0.0) );
	
	for (int i = 0; i <= grain; i++)
	{
		myCoords[i+1] = osg::Vec3(radius*cos(((float)i / (float)grain)*2*osg::PI), length, radius*sin(((float)i / (float)grain)*2*osg::PI));
		normals->push_back(myCoords[i]^myCoords[i+1]);
		colors->push_back(color);
	}
	
	colors->push_back(color);
	
	osg::Vec3Array* vertices = new osg::Vec3Array(grain + 2,myCoords);
	
	// pass the created vertex array to the points geometry object.
	triFan->setVertexArray(vertices);
	
	triFan->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_FAN,0, grain + 2));
	
	// pass the color arry to points geometry, note the binding to tell the geometry
	// that only use one color for the whole object.
	triFan->setColorArray(colors);
	triFan->setColorBinding(osg::Geometry::BIND_OVERALL);
	
	// create the normals
	triFan->setNormalArray(normals);
	triFan->setNormalBinding(osg::Geometry::BIND_OVERALL);
	osgUtil::SmoothingVisitor::smooth(*triFan);
	
	triGeode->addDrawable(triFan);
	
	osgUtil::Optimizer optimizer;
	optimizer.optimize(triGeode);
	
	return triGeode;

}

