AM_CPPFLAGS = $(all_includes)

# use noinst_LIBRARIES to create a library that won't be installed
noinst_LIBRARIES = libSPINwrappers.a

libSPINwrappers_a_SOURCES = ReferencedNode.cpp GroupNode.cpp ShapeNode.cpp ModelNode.cpp \
          VideoNode.cpp LightSource.cpp RayNode.cpp Contour.cpp ConstraintsNode.cpp \
          PointerNode.cpp MeasurementNode.cpp SoundNode.cpp SoundSpace.cpp \
          DSPNode.cpp SoundConnection.cpp UserNode.cpp SharedVideoNode.cpp

#$INCLUDES = -I$(includedir) -I../tinyxml
#INCLUDES = -I$(includedir)
INCLUDES = -I$(top_srcdir)/src/tinyxml -I$(top_srcdir)/src/mivilleEngine
