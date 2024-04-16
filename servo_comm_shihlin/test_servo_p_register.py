from servo_p_register import Register, PA, PC, PD, PE



PA.init_registers()
PC.init_registers()
PD.init_registers()
PE.init_registers()

print(PA.STY)
print(PA.STY.address)
print(PA.STY.value)
