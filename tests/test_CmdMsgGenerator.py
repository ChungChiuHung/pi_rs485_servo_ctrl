import unittest
from .servo_communication.MessageGenerator import MessageGenerator, CmdCode

class TestMessageGenerator(unittest.TestCase):
    def test_nop_command(self):
        generator = MessageGenerator(destination_address=0x01, control_code=0x00)
        nop_command_hex = generator.get_command(CmdCode.NOP, return_as_str=True)
        self.assertEqual(nop_command_hex, "Expected NOP command hex")
    
    def test_set_param_2_command(self):
        generator = MessageGenerator(destination_address=0x01, control_code=0x00)
        # Expected hex needs to be defined based on your implementation
        expected_hex = "Some expected hex value"
        set_param_2_command_hex = generator.get_command(
            CmdCode.SET_PARAM_2,
            return_as_str=True,
            param_group=[0x01, 0x02],
            write_value=[0x03, 0x04]
        )
        self.assertEqual(set_param_2_command_hex, expected_hex)
    
if __name__ == '__main__':
    unittest.main()