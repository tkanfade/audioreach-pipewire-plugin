pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = pw-pal.pc
EXTRA_DIST = $(pkgconfig_DATA)

AM_CFLAGS = -Wno-unused-parameter -Wno-unused-result

lib_LTLIBRARIES      = libpipewire-module-pal.la
libpipewire_module_pal_la_SOURCES   = src/pw-pal-plugin.c
libpipewire_module_pal_la_CFLAGS = $(AM_CFLAGS) $(PALHEADERS_CFLAGS) $(VUIINTFHEADER_CFLAGS) @PIPEWIRE_CFLAGS@
libpipewire_module_pal_la_LDFLAGS   = -shared -avoid-version
libpipewire_module_pal_la_LIBADD   = -ltinyalsa -ldl -lexpat -lpal -lagm

install-exec-hook:
	mkdir -p $(DESTDIR)/usr/lib/pipewire-0.3
	$(INSTALL_PROGRAM) .libs/libpipewire-module-pal.so $(DESTDIR)/usr/lib/pipewire-0.3/
	rm -f $(DESTDIR)/usr/lib/libpipewire-module-pal.so

install-data-hook:
	mkdir -p $(DESTDIR)/usr/share/pipewire/pipewire.conf.d/
	mkdir -p $(DESTDIR)/usr/share/wireplumber/wireplumber.conf.d/
	mkdir -p $(DESTDIR)/usr/share/wireplumber/scripts/
	$(INSTALL_DATA) $(srcdir)/configs/pw-pal-plugin.conf $(DESTDIR)/usr/share/pipewire/pipewire.conf.d/
	$(INSTALL_DATA) $(srcdir)/configs/60-disable-alsa.conf $(DESTDIR)/usr/share/wireplumber/wireplumber.conf.d/
	$(INSTALL_DATA) $(srcdir)/configs/90-device-detection.conf  $(DESTDIR)/usr/share/wireplumber/wireplumber.conf.d/
	$(INSTALL_DATA) $(srcdir)/configs/90-device-detection.lua  $(DESTDIR)/usr/share/wireplumber/scripts/
