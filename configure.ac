# Initialization
AC_PREREQ([2.69])
AC_INIT([i2c-ipmi], [1.0], [https://www.github.com/google-ipmi-i2c/issues])
AC_LANG([C++])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIRS([m4])
AC_CONFIG_AUX_DIR([build-aux])
AM_INIT_AUTOMAKE([subdir-objects -Wall -Werror foreign dist-xz])
AM_SILENT_RULES([yes])

# Checks for programs.
AC_PROG_CXX
AM_PROG_AR
AC_PROG_INSTALL
AC_PROG_MAKE_SET

# Checks for typedefs, structures, and compiler characteristics.
AX_CXX_COMPILE_STDCXX_17([noext])
AX_APPEND_COMPILE_FLAGS([-Wall -Werror], [CXXFLAGS])

# Checks for libraries.
AC_CHECK_HEADER([host-ipmid], [AC_MSG_ERROR(["phosphor-host-ipmid required and not found."])])

# We require linux i2c functionality
AC_CHECK_HEADERS([linux/i2c-dev.h], [], [
  AC_MSG_ERROR([Could not find linux/i2c-dev.h])
])
AC_CHECK_HEADERS([linux/i2c.h], [], [
  AC_MSG_ERROR([Could not find linux/i2c.h])
])


# Checks for library functions.
LT_INIT # Required for systemd linking

# Create configured output
AC_CONFIG_FILES([Makefile])
AC_OUTPUT
