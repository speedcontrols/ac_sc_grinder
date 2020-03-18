Import("env", "projenv")

for e in [ env, projenv ]:
    #
    # Fix options after "_bare.py"
    #
    e.Replace(LINKFLAGS = [i for i in e['LINKFLAGS'] if i not in [ '-nostartfiles', '-nostdlib' ]])
    e.Append(LINKFLAGS = [ "--specs=nano.specs", "--specs=nosys.specs" ])
    e.Replace(AS = '$CC', ASCOM = '$ASPPCOM')

    #
    # .ld script should be passed to linker directly as "-T<path>"
    # instead of "-Wl,-T<path>", to avoid 1K RAM loss for ".data.impure_data"
    #
    e.Replace(BUILD_FLAGS = [i for i in e['BUILD_FLAGS'] if not i.startswith('-Wl,-T')])
    e.Replace(LINKFLAGS = [(i[4:] if i.startswith('-Wl,-T') else i) for i in e['LINKFLAGS']])

#print('=====================================')
#print(env.Dump())
