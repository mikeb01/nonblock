

env = Environment(CPPPATH   = ['include'], 
                  CPPFLAGS  = ['-D_LINUX_VER'],
                  LIBS      = ['pcm'], 
                  LINKFLAGS = ['-pthread'],
                  LIBPATH   = ['lib'])

env.Program('bin/falsesharing', 'src/falsesharing.cpp')
