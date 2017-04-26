lib_LTLIBRARIES = libemanelayerdlep.la

libemanelayerdlep_la_CPPFLAGS= \
 -I@top_srcdir@/include \
 $(libemane_CFLAGS) \
 $(libdlep_CFLAGS) \
 -DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS \
 -DBOOST_MPL_LIMIT_VECTOR_SIZE=30 \
 -DBOOST_MPL_LIMIT_LIST_SIZE=30 \
 -DBOOST_TEST_DYN_LINK

libemanelayerdlep_la_SOURCES= \
 shimlayer.cc \
 dlepclientimpl.cc \
 dlepmodemservice.cc

EXTRA_DIST = \
 shimlayer.h \
 dlepclientimpl.h \
 dlepmodemservice.h


libemanelayerdlep_la_LIBADD= \
 $(libemane_LIBS) \
 $(libdlep_LIBS)  \
 -lboost_system   \
 -lboost_thread

libemanelayerdlep_la_LDFLAGS= \
 -avoid-version
