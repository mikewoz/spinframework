// ***************************************************************************
//
//   Generated automatically by genwrapper.
//   Please DO NOT EDIT this file!
//
// ***************************************************************************

#include <cppintrospection/ReflectionMacros>
#include <cppintrospection/TypedMethodInfo>
#include <cppintrospection/StaticMethodInfo>
#include <cppintrospection/Attributes>

#include <ReferencedNode.h>
#include <SceneManager.h>
#include <spinUtil.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_VALUE_REFLECTOR(spin::CronScript)
	I_DeclaringFile("ReferencedNode.h");
	I_Constructor0(____CronScript,
	               "",
	               "");
	I_PublicMemberProperty(boost::python::object, run);
	I_PublicMemberProperty(double, freq);
	I_PublicMemberProperty(double, lastRun);
	I_PublicMemberProperty(bool, enabled);
	I_PublicMemberProperty(bool, serverSide);
	I_PublicMemberProperty(std::string, path);
	I_PublicMemberProperty(std::string, params);
	I_PublicMemberProperty(std::string, pyScript);
	I_PublicMemberProperty(std::string, pyModule);
END_REFLECTOR

BEGIN_VALUE_REFLECTOR(spin::EventScript)
	I_DeclaringFile("ReferencedNode.h");
	I_Constructor0(____EventScript,
	               "",
	               "");
	I_PublicMemberProperty(boost::python::object, run);
	I_PublicMemberProperty(std::string, eventName);
	I_PublicMemberProperty(bool, enabled);
	I_PublicMemberProperty(bool, serverSide);
	I_PublicMemberProperty(std::string, path);
	I_PublicMemberProperty(std::string, params);
	I_PublicMemberProperty(std::string, pyScript);
	I_PublicMemberProperty(std::string, pyModule);
END_REFLECTOR

BEGIN_VALUE_REFLECTOR(boost::python::object)
	I_DeclaringFile("ReferencedNode.h");
	I_Constructor0(____object,
	               "",
	               "");
END_REFLECTOR

BEGIN_VALUE_REFLECTOR(spin::ReferencedNode)
	I_DeclaringFile("ReferencedNode.h");
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, const char *, initID,
	               ____ReferencedNode__SceneManager_P1__C5_char_P1,
	               "",
	               "");
	I_Method1(void, registerNode, IN, spin::SceneManager *, s,
	          Properties::VIRTUAL,
	          __void__registerNode__SceneManager_P1,
	          "",
	          "");
	I_Method1(void, callbackUpdate, IN, osg::NodeVisitor *, nv,
	          Properties::VIRTUAL,
	          __void__callbackUpdate__osg_NodeVisitor_P1,
	          "",
	          "For nodes that require regular programmatic control, there is a callback that is evaluated with every refresh. This function can thus be used for animations, or any other periodic updates.Note that changes to the scene graph structure (eg, moving/deleting nodes should NOT be done within this callback because traversals stacks will become corrupted. The technique is rather to enable a flag and then do the actual change in the SceneManager::updateGraph() method. ");
	I_MethodWithDefaults1(void, updateNodePath, IN, bool, updateChildren, true,
	                      Properties::VIRTUAL,
	                      __void__updateNodePath__bool,
	                      "",
	                      "IMPORTANT: subclasses of ReferencedNode are allowed to contain complicated subgraphs, and can also change their attachmentNode so that children are attached anywhere in that subgraph. If that is the case, the updateNodePath() function MUST be overridden, and extra nodes must be manually pushed onto currentNodePath_. ");
	I_Method1(int, setAttachmentNode, IN, osg::Group *, n,
	          Properties::NON_VIRTUAL,
	          __int__setAttachmentNode__osg_Group_P1,
	          "",
	          "");
	I_Method0(void, updateChildNodePaths,
	          Properties::NON_VIRTUAL,
	          __void__updateChildNodePaths,
	          "",
	          "An internal method that keeps track of the nodepath (for efficient computation of global position, etc. ");
	I_Method1(void, setParent, IN, const char *, newvalue,
	          Properties::NON_VIRTUAL,
	          __void__setParent__C5_char_P1,
	          "",
	          "This method schedules a change in parent for this node. The setParent() does not immediately change the scenegraph, since it can be called at any time, even while in a traversal. The graph is updated later using the attach() method, which is called by SceneManager->updateGraph() when there is a legal time to re-order the scenegraph.Internally, this method just sets the newParent property. ");
	I_Method1(void, attachTo, IN, const char *, parentID,
	          Properties::NON_VIRTUAL,
	          __void__attachTo__C5_char_P1,
	          "",
	          "Attach this node to the node with parentID (if found). ");
	I_Method1(void, detachFrom, IN, const char *, parentID,
	          Properties::NON_VIRTUAL,
	          __void__detachFrom__C5_char_P1,
	          "",
	          "Detaches the node from the parentID (if found). Note: you can pass \"*\" for the parentID and the node will be detached from ALL parents. ");
	I_Method0(bool, inGraph,
	          Properties::NON_VIRTUAL,
	          __bool__inGraph,
	          "",
	          "The inGraph method checks if the node is actually attached to the scene graph. There are cases (eg, SwitchNode or using detachFrom) that may cause a node to be orphaned (not attached anywhere). In these cases, reporters and pointers and anything that maintains a list of targets must check if the node is inGraph(). ");
	I_Method0(unsigned int, getNumParents,
	          Properties::NON_VIRTUAL,
	          __unsigned_int__getNumParents,
	          "",
	          "");
	I_Method1(std::string, getParentID, IN, int, i,
	          Properties::NON_VIRTUAL,
	          __std_string__getParentID__int,
	          "",
	          "Returns the parent id (string) ");
	I_Method1(spin::ReferencedNode *, getParentNode, IN, int, i,
	          Properties::NON_VIRTUAL,
	          __ReferencedNode_P1__getParentNode__int,
	          "",
	          "Returns the current parent as an osg::Group ");
	I_Method0(std::vector< spin::ReferencedNode * >, getChildren,
	          Properties::NON_VIRTUAL,
	          __std_vectorT1_ReferencedNode_P1___getChildren,
	          "",
	          "");
	I_Method1(void, setContext, IN, const char *, newvalue,
	          Properties::VIRTUAL,
	          __void__setContext__C5_char_P1,
	          "",
	          "A node can 'belong' to a certain host machine, allowing it to be rendered or behave differently than on other machines.NOTE: the \"NULL\" string means that it belongs to no specific context.NOTE: a scene operating in SERVER_MODE will always create the node, so this feature is only really relevant for clients applications. ");
	I_Method1(void, setAlpha, IN, float, alpha,
	          Properties::NON_VIRTUAL,
	          __void__setAlpha__float,
	          "",
	          "");
	I_Method0(float, getAlpha,
	          Properties::NON_VIRTUAL,
	          __float__getAlpha,
	          "",
	          "");
	I_Method0(const char *, getContext,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getContext,
	          "",
	          "Returns the current host ");
	I_Method0(std::string, getContextString,
	          Properties::NON_VIRTUAL,
	          __std_string__getContextString,
	          "",
	          "");
	I_Method2(void, setParam, IN, const char *, paramName, IN, const char *, paramValue,
	          Properties::VIRTUAL,
	          __void__setParam__C5_char_P1__C5_char_P1,
	          "",
	          "");
	I_Method2(void, setParam, IN, const char *, paramName, IN, float, paramValue,
	          Properties::VIRTUAL,
	          __void__setParam__C5_char_P1__float,
	          "",
	          "");
	I_Method1(void, setStateSetFromFile, IN, const char *, filename,
	          Properties::NON_VIRTUAL,
	          __void__setStateSetFromFile__C5_char_P1,
	          "",
	          "");
	I_Method1(void, setStateSet, IN, const char *, s,
	          Properties::NON_VIRTUAL,
	          __void__setStateSet__C5_char_P1,
	          "",
	          "");
	I_Method0(const char *, getStateSet,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getStateSet,
	          "",
	          "");
	I_Method0(void, updateStateSet,
	          Properties::VIRTUAL,
	          __void__updateStateSet,
	          "",
	          "In derived classes, you can handle how a stateset gets applied to a node (eg, which part of the subgraph it is attached by overriding the updateStateSet method. ");
	I_Method0(osg::Group *, getAttachmentNode,
	          Properties::NON_VIRTUAL,
	          __osg_Group_P1__getAttachmentNode,
	          "",
	          "subclasses of ReferencedNode may contain complicated subgraphs, and any children get attached not to the node pointer itself, but to an attachmentNode. This attachmentNode essentially defines the origin of the local coordinate system of this node (according to the subgraph). This function returns a pointer to this node. ");
	I_Method0(void, debug,
	          Properties::VIRTUAL,
	          __void__debug,
	          "",
	          "Debug print (to log/console) ");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node. ");
	I_Method0(void, stateDump,
	          Properties::VIRTUAL,
	          __void__stateDump,
	          "",
	          "Request to broadcast the node state via SceneManager. ");
	I_Method1(void, stateDump, IN, lo_address, addr,
	          Properties::VIRTUAL,
	          __void__stateDump__lo_address,
	          "",
	          "Request to send the node state to one address ");
	I_Method0(std::string, getID,
	          Properties::NON_VIRTUAL,
	          __std_string__getID,
	          "",
	          "Return the string id for this node ");
	I_Method0(std::string, getNodeType,
	          Properties::NON_VIRTUAL,
	          __std_string__getNodeType,
	          "",
	          "");
	I_Method0(spin::t_symbol *, getNodeSymbol,
	          Properties::NON_VIRTUAL,
	          __t_symbol_P1__getNodeSymbol,
	          "",
	          "");
	I_Method5(bool, addCronScript, IN, bool, serverSide, IN, const std::string &, label, IN, const std::string &, scriptPath, IN, double, freq, IN, const std::string &, params,
	          Properties::NON_VIRTUAL,
	          __bool__addCronScript__bool__C5_std_string_R1__C5_std_string_R1__double__C5_std_string_R1,
	          "",
	          "");
	I_Method0(bool, callCronScripts,
	          Properties::NON_VIRTUAL,
	          __bool__callCronScripts,
	          "",
	          "");
	I_Method2(bool, enableCronScript, IN, const char *, label, IN, int, enable,
	          Properties::NON_VIRTUAL,
	          __bool__enableCronScript__C5_char_P1__int,
	          "",
	          "");
	I_Method1(bool, removeCronScript, IN, const char *, label,
	          Properties::NON_VIRTUAL,
	          __bool__removeCronScript__C5_char_P1,
	          "",
	          "");
	I_Method5(bool, addEventScript, IN, bool, serverSide, IN, const std::string &, label, IN, const std::string &, eventName, IN, const std::string &, scriptPath, IN, const std::string &, params,
	          Properties::NON_VIRTUAL,
	          __bool__addEventScript__bool__C5_std_string_R1__C5_std_string_R1__C5_std_string_R1__C5_std_string_R1,
	          "",
	          "");
	I_Method2(bool, callEventScript, IN, const std::string &, eventName, IN, cppintrospection::ValueList &, args,
	          Properties::NON_VIRTUAL,
	          __bool__callEventScript__C5_std_string_R1__cppintrospection_ValueList_R1,
	          "",
	          "");
	I_Method2(bool, enableEventScript, IN, const char *, label, IN, int, enable,
	          Properties::NON_VIRTUAL,
	          __bool__enableEventScript__C5_char_P1__int,
	          "",
	          "");
	I_Method1(bool, removeEventScript, IN, const char *, label,
	          Properties::NON_VIRTUAL,
	          __bool__removeEventScript__C5_char_P1,
	          "",
	          "");
	I_ProtectedMethod1(bool, legalParent, IN, spin::t_symbol *, newParent,
	                   Properties::NON_VIRTUAL,
	                   Properties::NON_CONST,
	                   __bool__legalParent__t_symbol_P1,
	                   "",
	                   "TO BE DEPRECATED? The idea is that one type of node can only be attached to certain types of other nodes, but that has not been implemented. Currently, the only illegal parents include the node itself, or any children. ");
	I_ProtectedMethod1(void, setNodeType, IN, std::string, t,
	                   Properties::NON_VIRTUAL,
	                   Properties::NON_CONST,
	                   __void__setNodeType__std_string,
	                   "",
	                   "");
	I_SimpleProperty(float, Alpha, 
	                 __float__getAlpha, 
	                 __void__setAlpha__float);
	I_SimpleProperty(osg::Group *, AttachmentNode, 
	                 __osg_Group_P1__getAttachmentNode, 
	                 __int__setAttachmentNode__osg_Group_P1);
	I_SimpleProperty(std::vector< spin::ReferencedNode * >, Children, 
	                 __std_vectorT1_ReferencedNode_P1___getChildren, 
	                 0);
	I_SimpleProperty(const char *, Context, 
	                 __C5_char_P1__getContext, 
	                 __void__setContext__C5_char_P1);
	I_SimpleProperty(std::string, ContextString, 
	                 __std_string__getContextString, 
	                 0);
	I_SimpleProperty(std::string, ID, 
	                 __std_string__getID, 
	                 0);
	I_SimpleProperty(spin::t_symbol *, NodeSymbol, 
	                 __t_symbol_P1__getNodeSymbol, 
	                 0);
	I_SimpleProperty(std::string, NodeType, 
	                 __std_string__getNodeType, 
	                 0);
	I_SimpleProperty(const char *, Parent, 
	                 0, 
	                 __void__setParent__C5_char_P1);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_SimpleProperty(const char *, StateSet, 
	                 __C5_char_P1__getStateSet, 
	                 __void__setStateSet__C5_char_P1);
	I_SimpleProperty(const char *, StateSetFromFile, 
	                 0, 
	                 __void__setStateSetFromFile__C5_char_P1);
	I_PublicMemberProperty(osg::NodePath, currentNodePath_);
	I_PublicMemberProperty(bool, scheduleForDeletion_);
END_REFLECTOR

BEGIN_VALUE_REFLECTOR(spin::ReferencedNode_callback)
	I_DeclaringFile("ReferencedNode.h");
	I_Constructor0(____ReferencedNode_callback,
	               "",
	               "");
END_REFLECTOR

BEGIN_VALUE_REFLECTOR(spin::ReferencedNode_data)
	I_DeclaringFile("ReferencedNode.h");
	I_Constructor1(IN, spin::ReferencedNode *, n,
	               Properties::NON_EXPLICIT,
	               ____ReferencedNode_data__ReferencedNode_P1,
	               "",
	               "");
	I_Method1(void, update, IN, osg::NodeVisitor *, nv,
	          Properties::NON_VIRTUAL,
	          __void__update__osg_NodeVisitor_P1,
	          "",
	          "");
END_REFLECTOR

TYPE_NAME_ALIAS(std::vector< osg::ref_ptr< spin::ReferencedNode > >, spin::nodeListType)

TYPE_NAME_ALIAS(std::map< std::string COMMA  spin::nodeListType >, spin::nodeMapType)

TYPE_NAME_ALIAS(std::pair< std::string COMMA  spin::nodeListType >, spin::nodeMapPair)

TYPE_NAME_ALIAS(std::vector< osg::ref_ptr< spin::ReferencedStateSet > >, spin::ReferencedStateSetList)

TYPE_NAME_ALIAS(std::map< std::string COMMA  spin::ReferencedStateSetList >, spin::ReferencedStateSetMap)

TYPE_NAME_ALIAS(std::pair< std::string COMMA  spin::ReferencedStateSetList >, spin::ReferencedStateSetPair)

TYPE_NAME_ALIAS(std::map< const std::string COMMA  spin::CronScript * >, spin::CronScriptList)

TYPE_NAME_ALIAS(std::map< const std::string COMMA  spin::EventScript * >, spin::EventScriptList)

TYPE_NAME_ALIAS(std::map< std::string COMMA  std::string >, spin::stringParamType)

TYPE_NAME_ALIAS(std::map< std::string COMMA  float >, spin::floatParamType)

STD_MAP_REFLECTOR(std::map< const std::string COMMA  spin::CronScript * >)

STD_MAP_REFLECTOR(std::map< const std::string COMMA  spin::EventScript * >)

STD_MAP_REFLECTOR(std::map< std::string COMMA  float >)

STD_MAP_REFLECTOR(std::map< std::string COMMA  spin::ReferencedStateSetList >)

STD_MAP_REFLECTOR(std::map< std::string COMMA  spin::nodeListType >)

STD_MAP_REFLECTOR(std::map< std::string COMMA  std::string >)

STD_PAIR_REFLECTOR(std::pair< std::string COMMA  spin::ReferencedStateSetList >)

STD_PAIR_REFLECTOR(std::pair< std::string COMMA  spin::nodeListType >)

STD_VECTOR_REFLECTOR(std::vector< osg::ref_ptr< spin::ReferencedNode > >)

STD_VECTOR_REFLECTOR(std::vector< osg::ref_ptr< spin::ReferencedStateSet > >)

STD_VECTOR_REFLECTOR(std::vector< spin::ReferencedNode * >)

