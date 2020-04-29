Import("env", "projenv")

for e in [ env, projenv ]:
    #
    # Fix options after "_bare.py"
    #
    e.Replace(LINKFLAGS = [i for i in e['LINKFLAGS'] if i not in [ '-nostartfiles', '-nostdlib' ]])
    e.Append(LINKFLAGS = [ "--specs=nano.specs", "--specs=nosys.specs" ])
    e.Replace(AS = '$CC', ASCOM = '$ASPPCOM')

#print('=====================================')
#print(env.Dump())
