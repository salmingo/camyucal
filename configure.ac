dnl Process this file with autoconf to produce a configure script.

AC_PREREQ(2.59)
AC_INIT(camyucal, 0.1)

AC_CANONICAL_SYSTEM
AM_INIT_AUTOMAKE()

AC_PROG_CXX
AC_PROG_CC

AC_CHECK_FUNCS([socket memset sqrt])
AC_CHECK_HEADERS([string.h arpa/inet.h netinet/in.h sys/socket.h])
AC_CHECK_HEADER_STDBOOL
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT8_T

AC_CONFIG_FILES(Makefile src/Makefile)
AC_OUTPUT

