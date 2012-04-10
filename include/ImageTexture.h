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

#ifndef __ImageTexture_H
#define __ImageTexture_H

class SceneManager;

namespace osg {
    class Image;
}

#include <string>
#include <lo/lo_types.h>
#include <vector>
#include <osg/ref_ptr>
#include "ReferencedStateSet.h"

namespace spin
{

/**
 * \brief A texture state that holds a static image
 *
 */
class ImageTexture : public ReferencedStateSet
{

public:

    ImageTexture(SceneManager *sceneManager, const char *initID);
    ~ImageTexture();

    /**
     * TO_BE_DONE
     */

    virtual void debug();
    
    /**
     * Returns whether there is a currently valid image texture
     */
    bool isValid() const;

    /**
     * Creates a texture from a path on disk.
     */
    void setPath (const char* newPath);
    const char *getPath() const { return _path.c_str(); }

    // must reimplement
    virtual std::vector<lo_message> getState() const;

    
private:
    
    osg::ref_ptr<osg::Image> _image;
    std::string _path;

    bool _useTextureRectangle;
};

} // end of namespace spin

#endif
