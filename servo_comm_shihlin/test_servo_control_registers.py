from servo_control_registers import ServoControlRegistry
from modbus_rtu_client import ModbusRTUClient

# Access a register's details
print(ServoControlRegistry.MOTOR_FEEDBACK_PULSE)

# Calculate a dynamic address
try:
    pa_address = ServoControlRegistry.calculate_dynamic_address("PA", 1)
    print(f"Address for PA01: {hex(pa_address)}")
    pb_address = ServoControlRegistry.calculate_dynamic_address("PB", 40)
    print(f"Address for PB40: {hex(pb_address)}")
    pc_address = ServoControlRegistry.calculate_dynamic_address("PC", 38)
    print(f"Address for PC38: {hex(pc_address)}")
    pd_address = ServoControlRegistry.calculate_dynamic_address("PD", 31)
    print(f"Address for PD41: {hex(pd_address)}")
    pe_address = ServoControlRegistry.calculate_dynamic_address("PE", 30)
    print(f"Address for PE30: {hex(pe_address)}")
    pf_address = ServoControlRegistry.calculate_dynamic_address("PF", 82)
    print(f"Address for PF82: {hex(pf_address)}")

    

except ValueError as error:
    print(error)