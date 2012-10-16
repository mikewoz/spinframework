AM_CPPFLAGS = \
    $(BOOST_CPPFLAGS) \
    $(OPENTHREADS_CFLAGS) \
    $(OSG_CFLAGS) 

bin_PROGRAMS = lodifier

lodifier_SOURCES = lodifier.cpp
lodifier_LDADD = $(OPENTHREADS_LIBS) $(OSG_LIBS) $(BOOST_SYSTEM_LIBS)

###############################################################################
