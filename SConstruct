# -*- python -*-

##
##  Store all signatures in the '.sconsign.dblite'
##  file at the top-level SConstruct directory.
##
SConsignFile()

##
##  Create a new construction environment variable and
##  restrict it's build methods to be POSIX based only.
##
##  If you want to use the user's env variable PATH to find
##  the compiler and build tools, uncomment these two lines,
##  and remove the hard-coded paths to the arm-linux-*
##  tools below.
##
# import os
# env = Environment(platform = 'posix',ENV= os.environ)
##
##  Otherwise, do this:
##
env = Environment(platform = 'posix')

##
##  TODO - Check out missing files from source control.
##
##  scons will, by default, fetch files from SCCS or RCS subdirecto-
##  ries without explicit configuration.  This takes some extra pro-
##  cessing time to search for the necessary source code  management
##  files  on disk.  You can avoid these extra searches and speed up
##  your build a little by disabling these searches as follows:
##
env.SourceCode('.', None)

##
##  Define it's compiler flags for the all build tools.
##-O2 <-> -g
env['CCFLAGS'] = Split("""
    -Wall -O2 -g
""")

env['CXXFLAGS'] = Split("""
    -Wall -O2 -g
""")

# env['LINKFLAGS'] = Split("""
#     -rdynamic
# """)

##
##  Define it's C/C++ include paths for all builds.
##
env['CPPPATH'] = Split("""
  /jnet/linux/include
  #/dsm/modules
  #/dsm/class
  #/disc/class
  .
""")

##
##  Adjust the env for cross-building to the xScale ARM processor...
##
arm_env = env.Copy()

# arm_env.AppendUnique(CCFLAGS=Split("""
#   -mcpu=xscale
# """))

arm_env.AppendUnique(CPPPATH = Split("""
    /net/opt_lnx/local_arm/isffLib/include
    /net/opt_lnx/local_arm/include
"""))

## arm_env.AppendUnique(LIBS = Split("""
##       Dsm
##       isa
##       pthread
##       dl
##     """))

arm_env.AppendUnique(LIBPATH = Split("""
    /net/opt_lnx/local_arm/isffLib/lib
    /net/opt_lnx/local_arm/lib
    #dsm/class/arm
"""))

##
## Specify RPATH to avoid the need for LD_LIBRARY_PATH later 
##
arm_env.AppendUnique(RPATH = Split("""
    /usr/local/lib
"""))

arm_env.Replace(AR	= '/opt/arm_tools/bin/arm-linux-ar')
arm_env.Replace(AS	= '/opt/arm_tools/bin/arm-linux-as')
arm_env.Replace(CC	= '/opt/arm_tools/bin/arm-linux-gcc')
arm_env.Replace(CXX	= '/opt/arm_tools/bin/arm-linux-g++')
arm_env.Replace(LINK	= '/opt/arm_tools/bin/arm-linux-g++')
arm_env.Replace(RANLIB	= '/opt/arm_tools/bin/arm-linux-ranlib')
arm_env.Replace(LEX	= '/opt/arm_tools/arm-linux/bin/flex++')

##
##  Adjust the env for building to the x86 processor...
##
x86_env = env.Copy()

x86_env.AppendUnique(CPPPATH = Split("""
    /net/opt_lnx/local_rh90/isffLib/include
    /net/opt_lnx/local_rh90/include
"""))

## x86_env.AppendUnique(LIBS = Split("""
##      Dsm
##       isa
##       pthread
##       dl
##     """))

##     Disc

x86_env.AppendUnique(LIBPATH = Split("""
    /net/opt_lnx/local_rh90/isffLib/lib
    /net/opt_lnx/local_rh90/lib
    #dsm/class/x86
"""))

##
## Specify RPATH to avoid the need for LD_LIBRARY_PATH later
##
x86_env.AppendUnique(RPATH = Split("""
    /net/opt_lnx/local_rh90/lib
"""))

##
##  Export the environments to the SConscript files
##
## Export('arm_env')
## Export('x86_env')

##
##  Build dsm/modules/???.o
##
SConscript('dsm/modules/SConscript',
	build_dir='dsm/modules/arm',
	duplicate=0,exports={'env':arm_env})

##
##  Build libDsm.a
##
SConscript('dsm/class/SConscript',
	build_dir='dsm/class/arm',
	duplicate=0,exports={'env':arm_env})

SConscript('dsm/class/SConscript',
	build_dir='dsm/class/x86',
	duplicate=0,exports={'env':x86_env})

##
##  
##

SConscript('dsm/src/SConscript',
	build_dir='dsm/src/arm',
	duplicate=0,exports={'env':arm_env})

SConscript('dsm/src/SConscript',
	build_dir='dsm/src/x86',
	duplicate=0,exports={'env':x86_env})

##
##  Build lib/libArmDisc.a and lib/libX86Disc.a
##
SConscript('disc/class/SConscript',
	build_dir='disc/class/x86',
	duplicate=0,exports={'env':x86_env})

##
##  Build bin/discAsync, bin/discSync, bin/discComm
##
SConscript('disc/src/SConscript',
	build_dir='disc/src/x86',
	duplicate=0,exports={'env':x86_env})
