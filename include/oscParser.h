#ifndef OSCPARSER_H_
#define OSCPARSER_H_

#include <string>
#include "lo/lo.h"

int oscCallback_node(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
//int oscCallback_createNode(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);


void oscParser_create(const char* port);
void oscParser_destroy();

void oscParser_error(int num, const char *m, const char *path);


#endif
