ACLOCAL_AMFLAGS = -I m4
AM_DEFAULT_SOURCE_EXT = .cpp

libi2ccmdsdir = ${libdir}/ipmid-providers
libi2ccmds_LTLIBRARIES = libi2ccmds.la
libi2ccmds_la_SOURCES = i2c.cpp

libi2ccmds_la_LDFLAGS = \
	$(LIBIPMID_LIBS)
	-version-info 0:0:0 -shared

libi2ccmds_la_CXXFLAGS = -flto \
	$(LIBIPMID_CFLAGS)
