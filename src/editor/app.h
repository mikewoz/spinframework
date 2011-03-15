/*
* This file is part of the SPIN framework.
*
* Copyright (c) 2009 Mike Wozniewski
* Copyright (c) 2009 Zack Settel
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

#ifndef SPINEDITORAPP_H
#define SPINEDITORAPP_H

#include <wx/app.h>

class spineditorApp: public wxApp {
public:
	virtual bool OnInit();
};

#endif // SPINEDITORAPP_H
