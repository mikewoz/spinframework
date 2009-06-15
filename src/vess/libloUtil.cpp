#include "libloUtil.h"
#include <string>

void lo_server_del_method_with_userdata(lo_server lo_serv, const char *path, const char *typespec, void *userdata)
{

	internal_lo_server s = (internal_lo_server) (lo_serv);

    int pattern = 0;

    if (path) pattern = strpbrk(path, " #*,?[]{}") != NULL;


    if (!s->first)
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
                    if (it == s->first) {
                        s->first = it->next;
                    } else {
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
	    if (it) it = next;

    }

	// if no remaining methods are registered, we could return a message or
    // destroy the server... TODO?
    if (!s->first)
    {
    }

}
