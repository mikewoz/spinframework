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
#include <ReferencedStateSet.h>
#include <SceneManager.h>
#include <SoundConnection.h>
#include <spinLog.h>
#include <spinUtil.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_VALUE_REFLECTOR(spin::SceneManager)
	I_DeclaringFile("SceneManager.h");
	I_Constructor1(IN, std::string, id,
	               Properties::NON_EXPLICIT,
	               ____SceneManager__std_string,
	               "",
	               "");
	I_Method0(void, init,
	          Properties::NON_VIRTUAL,
	          __void__init,
	          "",
	          "");
	I_Method0(void, debugContext,
	          Properties::NON_VIRTUAL,
	          __void__debugContext,
	          "",
	          "");
	I_Method0(void, debugNodes,
	          Properties::NON_VIRTUAL,
	          __void__debugNodes,
	          "",
	          "");
	I_Method0(void, debugStateSets,
	          Properties::NON_VIRTUAL,
	          __void__debugStateSets,
	          "",
	          "");
	I_Method0(void, debugSceneGraph,
	          Properties::NON_VIRTUAL,
	          __void__debugSceneGraph,
	          "",
	          "");
	I_Method0(void, debug,
	          Properties::NON_VIRTUAL,
	          __void__debug,
	          "",
	          "");
	I_Method1(void, setGraphical, IN, bool, b,
	          Properties::NON_VIRTUAL,
	          __void__setGraphical__bool,
	          "",
	          "");
	I_Method0(bool, isGraphical,
	          Properties::NON_VIRTUAL,
	          __bool__isGraphical,
	          "",
	          "");
	I_Method1(void, setLog, IN, spin::spinLog &, log,
	          Properties::NON_VIRTUAL,
	          __void__setLog__spinLog_R1,
	          "",
	          "");
	I_Method1(void, registerStateSet, IN, spin::ReferencedStateSet *, s,
	          Properties::NON_VIRTUAL,
	          __void__registerStateSet__ReferencedStateSet_P1,
	          "",
	          "");
	I_Method1(void, unregisterStateSet, IN, spin::ReferencedStateSet *, s,
	          Properties::NON_VIRTUAL,
	          __void__unregisterStateSet__ReferencedStateSet_P1,
	          "",
	          "");
	I_MethodWithDefaults2(void, sendNodeList, IN, std::string, type, , IN, lo_address, txAddr, 0,
	                      Properties::NON_VIRTUAL,
	                      __void__sendNodeList__std_string__lo_address,
	                      "",
	                      "");
	I_MethodWithDefaults1(void, sendConnectionList, IN, lo_address, txAddr, 0,
	                      Properties::NON_VIRTUAL,
	                      __void__sendConnectionList__lo_address,
	                      "",
	                      "");
	I_Method0(std::vector< std::string >, getAllNodeTypes,
	          Properties::NON_VIRTUAL,
	          __std_vectorT1_std_string___getAllNodeTypes,
	          "",
	          "");
	I_Method2(spin::ReferencedNode *, createNode, IN, std::string, id, IN, std::string, type,
	          Properties::NON_VIRTUAL,
	          __ReferencedNode_P1__createNode__std_string__std_string,
	          "",
	          "");
	I_Method2(spin::ReferencedNode *, createNode, IN, const char *, id, IN, const char *, type,
	          Properties::NON_VIRTUAL,
	          __ReferencedNode_P1__createNode__C5_char_P1__C5_char_P1,
	          "",
	          "");
	I_Method1(spin::ReferencedNode *, getNode, IN, std::string, id,
	          Properties::NON_VIRTUAL,
	          __ReferencedNode_P1__getNode__std_string,
	          "",
	          "");
	I_Method1(spin::ReferencedNode *, getNode, IN, const char *, id,
	          Properties::NON_VIRTUAL,
	          __ReferencedNode_P1__getNode__C5_char_P1,
	          "",
	          "");
	I_Method2(spin::ReferencedNode *, getNode, IN, const char *, id, IN, const char *, type,
	          Properties::NON_VIRTUAL,
	          __ReferencedNode_P1__getNode__C5_char_P1__C5_char_P1,
	          "",
	          "");
	I_Method2(spin::ReferencedNode *, getOrCreateNode, IN, const char *, id, IN, const char *, type,
	          Properties::NON_VIRTUAL,
	          __ReferencedNode_P1__getOrCreateNode__C5_char_P1__C5_char_P1,
	          "",
	          "");
	I_Method1(spin::ReferencedStateSet *, getStateSet, IN, const char *, id,
	          Properties::NON_VIRTUAL,
	          __ReferencedStateSet_P1__getStateSet__C5_char_P1,
	          "",
	          "");
	I_Method2(spin::ReferencedStateSet *, createStateSet, IN, const char *, id, IN, const char *, type,
	          Properties::NON_VIRTUAL,
	          __ReferencedStateSet_P1__createStateSet__C5_char_P1__C5_char_P1,
	          "",
	          "");
	I_Method1(spin::ReferencedStateSet *, createStateSet, IN, const char *, fname,
	          Properties::NON_VIRTUAL,
	          __ReferencedStateSet_P1__createStateSet__C5_char_P1,
	          "",
	          "");
	I_Method1(void, setWorldStateSet, IN, const char *, s,
	          Properties::NON_VIRTUAL,
	          __void__setWorldStateSet__C5_char_P1,
	          "",
	          "");
	I_Method1(std::vector< spin::t_symbol * >, findNodes, IN, const char *, pattern,
	          Properties::NON_VIRTUAL,
	          __std_vectorT1_t_symbol_P1___findNodes__C5_char_P1,
	          "",
	          "");
	I_Method0(std::vector< spin::SoundConnection * >, getConnections,
	          Properties::NON_VIRTUAL,
	          __std_vectorT1_SoundConnection_P1___getConnections,
	          "",
	          "");
	I_Method1(void, deleteNode, IN, const char *, id,
	          Properties::NON_VIRTUAL,
	          __void__deleteNode__C5_char_P1,
	          "",
	          "This method removes a node from the scene, however the actual work is done by the doDelete() method.NOTE: All children of the node will remain, but will be re-attached to the 'world' node instead. If you want to destroy all children as well, then use deleteGraph(). ");
	I_Method1(void, deleteGraph, IN, const char *, id,
	          Properties::NON_VIRTUAL,
	          __void__deleteGraph__C5_char_P1,
	          "",
	          "deleteGraph() operates similarly to deleteNode(), except that all children are also deleted. ");
	I_Method1(void, doDelete, IN, spin::ReferencedNode *, n,
	          Properties::NON_VIRTUAL,
	          __void__doDelete__ReferencedNode_P1,
	          "",
	          "The doDelete method performs all of the necessary steps to remove a node from the scene: The node's detach() method is called, which will actually remove it from the scene graph, and eliminate the OSC callback. The callbackUpdate() function is unregistered. And finally, a message is broadcasted to all clients. ");
	I_Method1(void, doDelete, IN, spin::ReferencedStateSet *, n,
	          Properties::NON_VIRTUAL,
	          __void__doDelete__ReferencedStateSet_P1,
	          "",
	          "The doDelete method performs all of the necessary steps to delete a stateset. It is similar to the doDelete method for a note: it calls the node's removeFromScene() method, releases resources (eg, videos), removes it from the stateMap list, etc. ");
	I_Method0(void, clear,
	          Properties::NON_VIRTUAL,
	          __void__clear,
	          "",
	          "Clears scene elements that are not part of any user's subgraphs ");
	I_Method0(void, clearUsers,
	          Properties::NON_VIRTUAL,
	          __void__clearUsers,
	          "",
	          "Clears only the users from the scene (and any attached nodes in their subgraphs ");
	I_Method0(void, clearStates,
	          Properties::NON_VIRTUAL,
	          __void__clearStates,
	          "",
	          "Forces a removal of all states from the scene graph. This should only be used to clean up upon exit. ");
	I_Method0(void, update,
	          Properties::NON_VIRTUAL,
	          __void__update,
	          "",
	          "The update method is where any thread-safe changes to the scene graph should go. The method is guaranteed to be called only when there are no traversals being performed. ");
	I_Method1(osg::Matrix, getWorldCoords, IN, spin::t_symbol *, id,
	          Properties::NON_VIRTUAL,
	          __osg_Matrix__getWorldCoords__t_symbol_P1,
	          "",
	          "");
	I_Method2(void, exportScene, IN, const char *, nodeID, IN, const char *, filename,
	          Properties::NON_VIRTUAL,
	          __void__exportScene__C5_char_P1__C5_char_P1,
	          "",
	          "");
	I_Method1(std::string, getStateAsXML, IN, std::vector< lo_message >, nodeState,
	          Properties::NON_VIRTUAL,
	          __std_string__getStateAsXML__std_vectorT1_lo_message_,
	          "",
	          "");
	I_Method2(std::string, getNodeAsXML, IN, spin::ReferencedNode *, n, IN, bool, withUsers,
	          Properties::NON_VIRTUAL,
	          __std_string__getNodeAsXML__ReferencedNode_P1__bool,
	          "",
	          "");
	I_Method0(std::string, getConnectionsAsXML,
	          Properties::NON_VIRTUAL,
	          __std_string__getConnectionsAsXML,
	          "",
	          "");
	I_Method2(std::vector< spin::t_symbol * >, getSavableStateSets, IN, spin::ReferencedNode *, n, IN, bool, withUsers,
	          Properties::NON_VIRTUAL,
	          __std_vectorT1_t_symbol_P1___getSavableStateSets__ReferencedNode_P1__bool,
	          "",
	          "");
	I_MethodWithDefaults2(bool, saveXML, IN, const char *, filename, , IN, bool, withUsers, false,
	                      Properties::NON_VIRTUAL,
	                      __bool__saveXML__C5_char_P1__bool,
	                      "",
	                      "");
	I_Method1(bool, saveUsers, IN, const char *, s,
	          Properties::NON_VIRTUAL,
	          __bool__saveUsers__C5_char_P1,
	          "",
	          "");
	I_Method2(bool, createNodeFromXML, IN, TiXmlElement *, XMLnode, IN, const char *, parentNode,
	          Properties::NON_VIRTUAL,
	          __bool__createNodeFromXML__TiXmlElement_P1__C5_char_P1,
	          "",
	          "");
	I_Method1(bool, createStateSetFromXML, IN, TiXmlElement *, XMLnode,
	          Properties::NON_VIRTUAL,
	          __bool__createStateSetFromXML__TiXmlElement_P1,
	          "",
	          "");
	I_Method1(bool, createConnectionsFromXML, IN, TiXmlElement *, XMLnode,
	          Properties::NON_VIRTUAL,
	          __bool__createConnectionsFromXML__TiXmlElement_P1,
	          "",
	          "");
	I_Method1(bool, loadXML, IN, const char *, filename,
	          Properties::NON_VIRTUAL,
	          __bool__loadXML__C5_char_P1,
	          "",
	          "");
	I_Method0(void, refreshAll,
	          Properties::NON_VIRTUAL,
	          __void__refreshAll,
	          "",
	          "The refreshAll method results in a broadcast of all nodelists so that clients can create any missing nodes. Then, the full node state is broadcasted, for ALL nodes. ");
	I_Method1(void, refreshSubscribers, IN, const std::map< std::string COMMA  lo_address > &, clients,
	          Properties::NON_VIRTUAL,
	          __void__refreshSubscribers__C5_std_mapT1_std_stringComma_lo_address__R1,
	          "",
	          "The refreshSubscribers method results in a publication of all nodelists to the given TCP subscribers. Then, the full node state is published to the subscribers, for ALL nodes. ");
	I_SimpleProperty(std::vector< std::string >, AllNodeTypes, 
	                 __std_vectorT1_std_string___getAllNodeTypes, 
	                 0);
	I_SimpleProperty(std::vector< spin::SoundConnection * >, Connections, 
	                 __std_vectorT1_SoundConnection_P1___getConnections, 
	                 0);
	I_SimpleProperty(std::string, ConnectionsAsXML, 
	                 __std_string__getConnectionsAsXML, 
	                 0);
	I_SimpleProperty(bool, Graphical, 
	                 0, 
	                 __void__setGraphical__bool);
	I_SimpleProperty(spin::spinLog &, Log, 
	                 0, 
	                 __void__setLog__spinLog_R1);
	I_SimpleProperty(const char *, WorldStateSet, 
	                 0, 
	                 __void__setWorldStateSet__C5_char_P1);
	I_PublicMemberProperty(std::string, sceneID);
	I_PublicMemberProperty(osg::ref_ptr< osg::Group >, rootNode);
	I_PublicMemberProperty(osg::ref_ptr< osg::ClearNode >, worldNode);
	I_PublicMemberProperty(osg::ref_ptr< osg::Geode >, gridGeode);
	I_PublicMemberProperty(spin::t_symbol *, worldStateSet_);
	I_PublicMemberProperty(bool, graphicalMode);
	I_PublicMemberProperty(osg::ref_ptr< spin::GroupNode >, globalObserver);
	I_PublicMemberProperty(std::string, resourcesPath);
	I_PublicMemberProperty(spin::MediaManager *, mediaManager);
END_REFLECTOR

TYPE_NAME_ALIAS(std::map< std::string COMMA  spin::nodeListType >, spin::nodeMapType)

TYPE_NAME_ALIAS(std::pair< std::string COMMA  spin::nodeListType >, spin::nodeMapPair)

TYPE_NAME_ALIAS(std::vector< osg::ref_ptr< spin::ReferencedStateSet > >, spin::ReferencedStateSetList)

TYPE_NAME_ALIAS(std::map< std::string COMMA  spin::ReferencedStateSetList >, spin::ReferencedStateSetMap)

TYPE_NAME_ALIAS(std::pair< std::string COMMA  spin::ReferencedStateSetList >, spin::ReferencedStateSetPair)

STD_MAP_REFLECTOR(std::map< std::string COMMA  lo_address >)

STD_MAP_REFLECTOR(std::map< std::string COMMA  spin::ReferencedStateSetList >)

STD_MAP_REFLECTOR(std::map< std::string COMMA  spin::nodeListType >)

STD_PAIR_REFLECTOR(std::pair< std::string COMMA  spin::ReferencedStateSetList >)

STD_PAIR_REFLECTOR(std::pair< std::string COMMA  spin::nodeListType >)

STD_VECTOR_REFLECTOR(std::vector< spin::SoundConnection * >)

STD_VECTOR_REFLECTOR(std::vector< spin::t_symbol * >)

STD_VECTOR_REFLECTOR(std::vector< std::string >)

