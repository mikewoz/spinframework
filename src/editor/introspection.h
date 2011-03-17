/*
* This file is part of the SPIN Framework.
*
* Copyright (c) 2009 Mike Wozniewski
* Copyright (c) 2009 Zack Settel
* Copyright (c) 2011 Alexandre Quessy
*
* SPIN Framework is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* SPIN Framework is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
*/

/** \file
 * SPIN Introspection utilities.
 */

#ifndef __INTROSPECTION_H__
#define __INTROSPECTION_H__

#include <vector>
#include <string>

namespace spin
{
namespace editor
{
namespace introspection
{

std::vector<std::string> listSpinNodeTypes();

} // end of namespace introspection
} // end of namespace editor
} // end of namespace spin

#endif
