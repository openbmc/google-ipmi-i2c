project(
  'google-ipmi-i2c',
  'cpp',
  version: '0.1',
  meson_version: '>=0.57.0',
  default_options: [
    'cpp_std=c++20',
    'warning_level=3',
    'werror=true',
  ]
)
meson.get_compiler('cpp').has_header_symbol(
  'ipmid/api.h',
  'ipmid_get_sd_bus_connection')

shared_module(
  'i2ccmds',
  'i2c.cpp',
  implicit_include_directories: false,
  include_directories: include_directories('.'),
  dependencies: [
    dependency('libipmid'),
  ],
  install: true,
  install_dir: get_option('libdir') / 'ipmid-providers'
)
