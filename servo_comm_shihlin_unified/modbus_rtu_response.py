from typing import Union

from modbus_command_code import CmdCode
from modbus_utils import ModbusUtils


class ModbusRTUResponse:
    """Parses a raw Modbus RTU binary response (device addr + function code +
    payload + CRC16, no ':'/hex-ASCII framing -- unlike ModbusResponse, which
    only understands the ASCII-mode framing used elsewhere in this project).

    Kept response-shape-compatible with ModbusResponse (same get_value()
    semantics, same "high word + low word swapped" quirk) so callers like
    absolute_mode_check.check_absolute_mode() can accept either as a
    response_parser without caring which serial protocol produced it.
    """

    def __init__(self, response: Union[bytes, bytearray]):
        if not isinstance(response, (bytes, bytearray)):
            raise ValueError("RTU response must be bytes/bytearray, not ASCII text")
        if len(response) < 5:
            raise ValueError(f"RTU response too short ({len(response)} bytes)")

        body, received_crc = bytes(response[:-2]), bytes(response[-2:])
        expected_crc = ModbusUtils().calculate_crc(body)
        if received_crc != expected_crc:
            raise ValueError(
                f"CRC mismatch: got {received_crc.hex()}, expected {expected_crc.hex()}"
            )

        self.adr = body[0]
        self.cmd_value = body[1]

        if self.cmd_value & 0x80:
            self.exception_code = body[2] if len(body) > 2 else None
            raise ValueError(
                f"Modbus exception response: function {hex(self.cmd_value & 0x7F)}, "
                f"exception code {self.exception_code}"
            )

        if self.cmd_value == CmdCode.READ_DATA.value:
            self._parse_read_data(body)
        elif self.cmd_value in (CmdCode.WRITE_DATA.value, CmdCode.WRITE_MULTI_DATA.value):
            self._parse_write_data(body)
        else:
            raise ValueError(f"Unsupported command code: {hex(self.cmd_value)}")

    def _parse_read_data(self, body: bytes) -> None:
        self.data_count = body[2]  # byte count, not word count
        self.data_bytes = body[3:3 + self.data_count]

    def _parse_write_data(self, body: bytes) -> None:
        self.start_address = body[2:4]
        self.data_content = body[4:6]

    def get_value(self) -> Union[int, None]:
        """Same word-swap convention as ModbusResponse.get_value(): the
        driver returns 32-bit values as [low word][high word], so the two
        words are swapped back before combining into one big-endian int.
        """
        if not hasattr(self, 'data_bytes'):
            return None

        high_byte = self.data_bytes[2:4]
        low_byte = self.data_bytes[0:2]
        return int.from_bytes(bytes(high_byte) + bytes(low_byte), byteorder='big')

    def __str__(self):
        if hasattr(self, 'data_bytes'):
            return (f"Modbus RTU Response: ADR={self.adr}, CMD={hex(self.cmd_value)}, "
                    f"Data={self.data_bytes.hex()}, Value={self.get_value()}")
        return f"Modbus RTU Response: ADR={self.adr}, CMD={hex(self.cmd_value)}"
