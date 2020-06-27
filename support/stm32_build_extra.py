Import("env", "projenv")

for e in [ env, projenv ]:
    # Update options after "_bare.py"
    e.Append(LINKFLAGS = [ "--specs=nano.specs", "--specs=nosys.specs" ])
    e.Replace(AS = '$CC', ASCOM = '$ASPPCOM')

#print('=====================================')
#print(env.Dump())
