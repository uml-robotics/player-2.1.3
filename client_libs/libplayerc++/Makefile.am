SUBDIRS = . test

lib_LTLIBRARIES =
if INCLUDE_PLAYERCC
 lib_LTLIBRARIES += libplayerc++.la
endif

#pthread goes last because it needs to define some stuff
AM_CPPFLAGS = -I$(top_builddir)/client_libs \
              -I$(top_srcdir) \
              -I$(top_srcdir)/client_libs \
              -Wall -g $(PLAYERCC_EXTRA_CFLAGS)

# create the pkg-config entry for the C++
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = playerc++.pc

EXTRA_DIST = playerc++.pc.in
CLEANFILES = playerc++.pc

# the +'s are turned into _'s by automake
libplayerc___la_LDFLAGS = $(PLAYER_VERSION_INFO) -rpath $(libdir) $(top_builddir)/libplayerxdr/libplayerxdr.la $(top_builddir)/client_libs/libplayerc/libplayerc.la $(top_builddir)/libplayercore/libplayerutils.la

libplayerc___la_SOURCES =     playerc++.h \
                              debug.h \
                              playerclient.h \
                              playererror.h \
                              playerc++.cc \
                              playerclient.cc \
                              playererror.cc \
                              clientproxy.cc \
                              actarrayproxy.cc \
                              aioproxy.cc \
                              audioproxy.cc \
                              blackboardproxy.cc \
                              blobfinderproxy.cc \
                              bumperproxy.cc \
                              cameraproxy.cc \
                              dioproxy.cc \
                              fiducialproxy.cc \
                              gpsproxy.cc \
                              graphics2dproxy.cc \
                              graphics3dproxy.cc \
                              gripperproxy.cc \
                              healthproxy.cc \
                              imuproxy.cc \
                              irproxy.cc \
                              laserproxy.cc \
                              limbproxy.cc \
                              localizeproxy.cc \
                              logproxy.cc \
                              mapproxy.cc \
                              opaqueproxy.cc \
                              plannerproxy.cc \
															pointcloud3d.cc \
                              position1dproxy.cc \
                              position2dproxy.cc \
                              position3dproxy.cc \
                              powerproxy.cc \
                              ptzproxy.cc \
                              rangerproxy.cc \
                              rfidproxy.cc \
                              simulationproxy.cc \
                              sonarproxy.cc \
                              speechproxy.cc \
                              speechrecognitionproxy.cc \
                              vectormapproxy.cc \
                              wifiproxy.cc \
			      wsnproxy.cc

# not in libplayerc yet \
                              blinkenlightproxy.cc \
                              energyproxy.cc \
                              mcomproxy.cc \
                              speechrecognitionproxy.cc


#PLAYERCC_LIB_DIRS=-L${top_builddir}/libplayerxdr \
#                  -L${top_builddir}/client_libs/libplayerc

#libplayerc___la_LIBADD = $(PLAYERCC_LIB_DIRS) $(PLAYERCC_LIB)
libplayerc___la_LIBADD = $(PLAYERCC_LIB)

libplayerc___la_DEPENDENCIES = $(top_builddir)/client_libs/libplayerc/libplayerc.la

libplayerc___includedir = $(includedir)/player-$(PLAYER_MAJOR_VERSION).$(PLAYER_MINOR_VERSION)/libplayerc++
libplayerc___include_HEADERS = playerc++.h playerclient.h playererror.h playerc++config.h clientproxy.h utility.h
