/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/
//osgManipulator - Copyright (C) 2007 Fugro-Jason B.V.

#ifndef DraggerWith3Axes_h
#define DraggerWith3Axes_h 1

#include <osgManipulator/Translate1DDragger>

namespace osgManipulator {

/**
 * Dragger for performing translation in all three axes.
 */
class OSGMANIPULATOR_EXPORT DraggerWith3Axes : public CompositeDragger
{
    public:

        DraggerWith3Axes();

        META_OSGMANIPULATOR_Object(osgManipulator,DraggerWith3Axes)

        /** Setup default geometry for dragger. */
        void setupDefaultGeometry();

    protected:

        virtual ~DraggerWith3Axes();

        osg::ref_ptr< Translate1DDragger >  _xDragger;
        osg::ref_ptr< Translate1DDragger >  _yDragger;
        osg::ref_ptr< Translate1DDragger >  _zDragger;
};


}

#endif
