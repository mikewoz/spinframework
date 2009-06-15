// ===================================================================
// Audioscape library for PureData
// Copyright (c) 2007
//
// Collaborators:
//    Shared Reality Lab (SRE), McGill University Centre for Intelligent Machines (CIM)
//       www.cim.mcgill.ca/sre
//    La Société des Arts Technologiques (SAT)
//       www.sat.qc.ca
//
// Project Directors:
//    Science - Jeremy R. Cooperstock (SRE/CIM)
//    Arts - Zack Settel
//
// Conception:
//    Zack Settel
//
// Development Team:
//    Mike Wozniewski (SRE/CIM): Researcher, Head Developer
//    Zack Settel: Artist, Researcher, Audio/DSP programming
//    Jean-Michel Dumas (SAT): Assistant Researcher
//    Mitchel Benovoy (SRE/CIM): Video Texture Programming
//    Stéphane Pelletier (SRE/CIM): Video Texture Programming
//    Pierre-Olivier Charlebois (SRE/CIM): Former Developer
//
// Funding by / Souventionné par:
//    Natural Sciences and Engineering Research Council of Canada (NSERC)
//    Canada Council for the Arts
//    NSERC/Canada Council for the Arts - New Media Initiative
//
// ===================================================================
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
// ===================================================================

//#include "m_pd.h"
#include "pdUtil.h"

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define HASHSIZE 1024
static t_symbol *symhash[HASHSIZE];

void *getbytes(size_t nbytes)
{
    void *ret;
    if (nbytes < 1) nbytes = 1;
    ret = (void *)calloc(nbytes, 1);
    //if (!ret) post("pd: getbytes() failed -- out of memory");
    return (ret);
}

void *copybytes(void *src, size_t nbytes)
{
    void *ret;
    ret = getbytes(nbytes);
    if (nbytes)
        memcpy(ret, src, nbytes);
    return (ret);
}

void freebytes(void *fatso, size_t nbytes)
{
    if (nbytes == 0)
        nbytes = 1;
    free(fatso);
}

t_symbol *dogensym(const char *s, t_symbol *oldsym)
{
    t_symbol **sym1, *sym2;
    unsigned int hash1 = 0,  hash2 = 0;
    int length = 0;
    const char *s2 = s;
    while (*s2)
    {
        hash1 += *s2;
        hash2 += hash1;
        length++;
        s2++;
    }
    sym1 = symhash + (hash2 & (HASHSIZE-1));
    while (sym2 = *sym1)
    {
        if (!strcmp(sym2->s_name, s)) return(sym2);
        sym1 = &sym2->s_next;
    }
    if (oldsym) sym2 = oldsym;
    else
    {
        sym2 = (t_symbol *)getbytes(sizeof(*sym2));
        sym2->s_name = (char*) getbytes(length+1);
        sym2->s_next = 0;
        sym2->s_thing = 0;
        strcpy(sym2->s_name, s);
    }
    *sym1 = sym2;
    return (sym2);
}


t_symbol *gensym(const char *s)
{
    return(dogensym(s, 0));
}
