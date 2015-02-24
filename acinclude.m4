m4_include([ulapi.m4])
m4_include([doxygen.m4])

dnl This is for the MTConnect adapter
m4_include([mtconnect.m4])

dnl The file 'gomotion.m4' is provided for applications that use gomotion. These
dnl should call the combined ACX_GOMOTION macro. Here, we include that file,
dnl and call only the ACX_PREGOMOTION macro in configure.ac

m4_include([gomotion.m4])
