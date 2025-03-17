project(
    'google-ipmi-i2c',
    'cpp',
    version: '0.1',
    meson_version: '>=1.1.1',
    default_options: ['cpp_std=c++23', 'warning_level=3', 'werror=true'],
)

cpp = meson.get_compiler('cpp')

ipmid = dependency('libipmid')

assert(cpp.check_header('linux/i2c-dev.h'))
assert(cpp.check_header('linux/i2c.h'))

shared_module(
    'i2ccmds',
    'i2c.cpp',
    implicit_include_directories: false,
    include_directories: include_directories('.'),
    dependencies: [ipmid],
    install: true,
    install_dir: get_option('libdir') / 'ipmid-providers',
)
