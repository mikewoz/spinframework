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

#ifndef LIBLOUTIL_H
#define LIBLOUTIL_H

#include "lo/lo.h"
#include "stdlib.h"
#include "string.h"

typedef struct _lo_method {
        const char        *path;
        const char        *typespec;
        lo_method_handler  handler;
        char              *user_data;
        struct _lo_method *next;
} *internal_lo_method;

#ifdef LO_VERSION_25
typedef struct _lo_server {
        int                      socket;
        struct addrinfo         *ai;
        lo_method                first;
        lo_err_handler           err_h;
        int                      port;
        char                    *hostname;
        char                    *path;
        int                      protocol;
        void                    *queued;
        struct sockaddr_storage  addr;
        socklen_t                addr_len;
} *internal_lo_server;
#else
typedef struct _lo_server {
        struct addrinfo         *ai;
        lo_method                first;
        lo_err_handler           err_h;
        int                      port;
        char                    *hostname;
        char                    *path;
        int                      protocol;
        void                    *queued;
        struct sockaddr_storage  addr;
        socklen_t                addr_len;
        int                  sockets_len;
        int                  sockets_alloc;
#ifdef HAVE_POLL
        struct pollfd        *sockets;
#else
        struct { int fd; }   *sockets;
#endif
} *internal_lo_server;
#endif


/**
 * This function allows for the deletion of a method from a liblo server based
 * on the userdata pointer.
 *
 * Unfortunately, the liblo library doesn't provide the interface to do this;
 * A method can only be removed like this:
 *
 * lo_server_del_method(lo_server s, const char *path, const char *typespec)
 *
 * For SPIN, several methods can be registered with the same path (and typespec
 * is usually NULL), so ALL methods will be deleted using this function call!
 *
 * Thus, we've copied the lo_server struct (and named it internal_lo_server)
 * so that we can remove methods if their user_data matches
 */
void lo_server_del_method_with_userdata(lo_server lo_serv, const char *path, const char *typespec, void *userdata);



#endif
