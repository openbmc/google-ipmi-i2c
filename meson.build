project(
  'i2c-ipmi',
  'cpp',
  default_options: [
    'warning_level=3',
    'werror=true',
    'cpp_std=c++17'
  ],
  license: 'Apache-2.0',
  version: '0.1',
)

libipmid = dependency('libipmid')

# We require linux i2c functionality.
comp = meson.get_compiler('cpp')
assert(comp.check_header('linux/i2c-dev.h'), 'Could not find <linux/i2c-dev.h>')
assert(comp.check_header('linux/i2c.h'), 'Could not find <linux/i2c.h>')

sources = [
  'i2c.cpp',
]

# meson enforces '--no-undefined' if compiling as shared_library, which doesn't
# work since ipmid is going to dlopen the object.
libi2ccmds = shared_module(
  'i2ccmds',
  sources,
  dependencies: [
    libipmid,
  ],
  link_args: ['-Wl,-flto']
)

# Various bitbake recipes and QA checks don't like a non-symlinked .so, so
# manually copy a .so.0 for them.
copy_lib = custom_target(
  'copy_lib',
  output: 'libi2ccmds.so.0',
  input: libi2ccmds,
  command: ['cp', '@INPUT@', '@OUTPUT@'],
  install: true,
  install_dir: get_option('libdir') / 'ipmid-providers',
)
