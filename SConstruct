

env = Environment(CPPPATH   = ['include'], 
                  CPPFLAGS  = ['-D_LINUX_VER'],
                  LIBS      = ['pcm'], 
                  LINKFLAGS = ['-pthread', '-lrt'],
                  LIBPATH   = ['lib'])

env.Program('bin/falsesharing', 'src/falsesharing.cpp')
