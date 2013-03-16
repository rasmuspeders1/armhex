import platform

cxx_flags = '-O2'

raspberry_pi_hostname = '192.168.1.143'

if platform.machine() != 'armv6l':
	import os
	tool_chain_root_path = os.path.abspath('../tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/')
	if not os.path.exists(tool_chain_root_path):
		print('toolchain path does not exist!')
	tool_chain_prefix = 'arm-linux-gnueabihf'
	env = Environment(CXXFLAGS=cxx_flags, CPPPATH='inc:' + os.path.join(tool_chain_root_path, 'arm-linux-gnueabihf/include/c++/4.7.2'))

	#add toolchain bin path to path
	env.PrependENVPath('PATH', os.path.join(tool_chain_root_path, 'bin'))

	env['CXX'] = 'arm-linux-gnueabihf-g++'
else:
	env = Environment(CPPPATH='inc', CXXFLAGS=cxx_flags)

armhex = env.Program(target='armhex', source=Glob('src/*.cpp'), LIBS=['rt'])


if platform.machine() == 'armv6l':
	Default(armhex)


test = Command( target = 'transfer',
                source = armhex,
                action = 'scp armhex pi@' + raspberry_pi_hostname + ':/home/pi/armhex/armhex' )
                
     
