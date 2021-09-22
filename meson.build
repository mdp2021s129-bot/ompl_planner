project('ompl', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3',
                     'cpp_std=c++20', 'buildtype=debug'])

ompl = dependency('ompl', method: 'cmake')
grpc = dependency('grpc')

exe = executable('ompl', ['ompl.cpp', 'collision.cpp'],
  install : true, dependencies: [ompl])

test('basic', exe)
