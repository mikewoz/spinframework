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

#include "libloUtil.h"
#include <string.h>
#include <string>

void lo_server_del_method_with_userdata(lo_server lo_serv, const char *path, const char *typespec, void *userdata)
{
    internal_lo_server s = (internal_lo_server) (lo_serv);
    int pattern = 0;

    if (path)
    {
        if (strpbrk(path, " #*,?[]{}") != 0)
            pattern = 1;
        else
            pattern = 0;
    }

    if (! s->first)
    {
    // The server has no registered callbacks, so nothing to be done
        return;
    }

    internal_lo_method it, prev, next;
    it = (internal_lo_method) s->first;
    prev = it;

    while (it)
    {
        // in case we free it:
        next = it->next;

        // If paths match or handler is wildcard:
        if ((it->path == path) ||
            (path && it->path && !strcmp(path, it->path)) ||
            (pattern && it->path && lo_pattern_match(it->path, path))) {

            // If types match or handler is wildcard:
            if ((it->typespec == typespec) || (typespec && it->typespec && !strcmp(typespec, it->typespec)))
            {
                // If the user_data points to the user data provided:
                if (it->user_data == (char*)userdata)
                {
                    // Take care when removing the head:
                    if (it == s->first)
                    {
                        s->first = it->next;
                    }
                    else
                    {
                        prev->next = it->next;
                    }
                    next = it->next;
                    free((char *)it->path);
                    free((char *)it->typespec);
                    free(it);
                    it = prev;
                }
            }
        }
        prev = it;
        if (it)
            it = next;
    }

    // if no remaining methods are registered, we could return a message or
    // destroy the server... TODO?
    if (! s->first)
    {
        // TODO
    }
}

