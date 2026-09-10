# Process this file with autoconf to produce a configure script

# Requires autoconf tool later than 2.61
AC_PREREQ(2.69)
# Initialize the smwrapper package version 1.0.0
AC_INIT([pw-pal],1.0.0)
# Does not strictly follow GNU Coding standards
AM_INIT_AUTOMAKE([foreign subdir-objects])

# Checks for programs.
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_CXX
AC_PROG_LIBTOOL
AC_PROG_AWK
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET
PKG_PROG_PKG_CONFIG

PKG_CHECK_MODULES([AGM], [agm])
AC_SUBST([AGM_CFLAGS])
AC_SUBST([AGM_LIBS])

PKG_CHECK_MODULES([PIPEWIRE], [libpipewire-0.3])
AC_SUBST([PIPEWIRE_CFLAGS])

PKG_CHECK_MODULES([PALHEADERS], [pal-headers])
AC_SUBST([PALHEADERS_CFLAGS])

PKG_CHECK_MODULES([VUIINTFHEADER], [vui-intf-header])
AC_SUBST([VUIINTFHEADER_CFLAGS])

AC_CONFIG_FILES([ Makefile pw-pal.pc ])
AC_OUTPUT
