project('ompl', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3',
                     'cpp_std=c++20', 'buildtype=debug'])

fs = import('fs')

ompl = dependency('ompl', method: 'cmake')
grpc = dependency('grpc++')
protobuf = dependency('protobuf')

protoc = find_program('protoc')
grpc_cpp_plugin = find_program('grpc_cpp_plugin')
grpc_cpp_plugin_path = grpc_cpp_plugin.path()

protofile = files('pathserver.proto')

grpc_generated = custom_target(
  'grpc_generated',
  output: ['pathserver.pb.h', 'pathserver.pb.cc', 
           'pathserver.grpc.pb.h', 'pathserver.grpc.pb.cc'],
  input: protofile,
  command: [protoc, '-I@CURRENT_SOURCE_DIR@', f'--plugin=protoc-gen-grpc=@grpc_cpp_plugin_path@',
            '--grpc_out=@OUTDIR@', '--cpp_out=@OUTDIR@', 
           '@INPUT@']
)

exe = executable('ompl', ['ompl.cpp', 'collision.cpp', grpc_generated[1], grpc_generated[3]],
  install: true, dependencies: [ompl, grpc, protobuf])

test('basic', exe)
