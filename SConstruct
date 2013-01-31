import platform
if platform.machine() != 'armv6l':
	import os
	tool_chain_root_path = '../tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/'
	tool_chain_prefix = 'arm-linux-gnueabihf'
	env = Environment(CPPPATH='inc:' + os.path.join(tool_chain_root_path, 'arm-linux-gnueabihf/include/c++/4.7.2') , LIBPATH=os.path.join(tool_chain_root_path, 'libc'))

	#add toolchain bin path to path
	env.PrependENVPath('PATH', os.path.join(tool_chain_root_path, 'bin'))

	env['CXX'] = 'arm-linux-gnueabihf-g++'
else:
	env = Environment(CPPPATH='inc')
env.Program('armhex', Glob('src/*.cpp'), LIBS=['rt'])
