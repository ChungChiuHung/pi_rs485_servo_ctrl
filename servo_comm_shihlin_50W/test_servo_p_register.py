from servo_p_register import Register, PA, PC, PD, PE



PA.init_registers()
PC.init_registers()
PD.init_registers()
PE.init_registers()

print(f"STY Description: {PA.STY.description}")
print(f"STY Address: {hex(PA.STY.address)}")
print(f"STY Value: {hex(PA.STY.value)}") 

encode_value = PA.encode_HMOV(1,2,3)
print(f"Encoded HMOV: {hex(encode_value)}")
print(f"Explain HMOV: {PA.explain_HMOV(encode_value)}")
