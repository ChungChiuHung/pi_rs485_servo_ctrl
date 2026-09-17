"""
Fail-safe check for PA28 (ABS, absolute-encoder mode select).

Background: docs/servo_comm_shihlin_merge_design.md §2.4 decided to fix the
encoder-overflow risk by reading PA32(APR)/PA33(APP) instead of the raw
0x0000/0x0024 registers. Per the driver manual (SDE_English_manual_UL_v107.pdf,
p.88-89), PA32/PA33 are only valid "when PA28 is set as 1" (absolute mode).
Nothing in the existing codebase (servo_comm_shihlin/, servo_comm_shihlin_50W/)
ever sets or checks PA28, which suggests it's a one-time hardware/DIP setting
rather than something software-controlled -- but that has not been confirmed
on the real driver yet.

This module reads PA28 and logs its status clearly, so that:
  * running it against real hardware answers the open question directly, and
  * once wired into ServoController, it acts as a startup fail-safe -- refuse
    to trust PA32/PA33-based position reads unless PA28 has been confirmed
    as 1 for this session.
"""
import logging

from servo_p_register import PA
from modbus_response import ModbusResponse

PA.init_registers()

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def check_absolute_mode(modbus_client, response_parser=None):
    """Read PA28 and log whether the driver is in absolute-encoder mode.

    Args:
        modbus_client: an object exposing build_read_message(address, word_length)
            and send_and_receive(message), e.g. ModbusASCIIClient or
            ModbusRTUClient -- this function doesn't care which serial
            protocol produced the response, only that response_parser can
            parse it.
        response_parser: callable taking the raw response and exposing
            get_value(), e.g. ModbusResponse (ASCII, the default -- looked
            up at call time so tests can still patch the module-level name)
            or ModbusRTUResponse (RTU).

    Returns:
        True  -- PA28 == 1. Absolute mode confirmed; PA32/PA33 are safe to use
                 for the encoder-overflow fix.
        False -- PA28 == 0, or any other value the manual doesn't document.
                 PA32/PA33 would be invalid. Do NOT implement/rely on the
                 PA32/PA33-based overflow fix until this is corrected on the
                 physical driver.
        None  -- Could not read or parse a response at all (communication
                 failure). Treat as UNKNOWN, never as "safe" -- callers must
                 refuse to proceed on None, same as on False.
    """
    logger.info(
        "Checking PA%s (%s) at address %s to confirm absolute-encoder mode...",
        PA.ABS.no, PA.ABS.name, hex(PA.ABS.address)
    )

    if response_parser is None:
        response_parser = ModbusResponse

    message = modbus_client.build_read_message(PA.ABS.address, 2)
    response = modbus_client.send_and_receive(message)

    if response is None:
        logger.error(
            "No response reading PA28 (communication failure). Absolute-mode "
            "status UNKNOWN -- do not assume PA32/PA33 are valid."
        )
        return None

    try:
        response_object = response_parser(response)
        value = response_object.get_value()
    except Exception as e:
        logger.error(
            "Failed to parse PA28 response (%s). Absolute-mode status UNKNOWN.",
            e
        )
        return None

    if value is None:
        logger.error(
            "PA28 response contained no data. Absolute-mode status UNKNOWN."
        )
        return None

    if value == 1:
        logger.info(
            "PA28 = 1 (absolute mode). PA32/PA33 are valid -- safe to use for "
            "the encoder-overflow fix."
        )
        return True

    if value == 0:
        logger.warning(
            "PA28 = 0 (incremental mode), NOT absolute mode. PA32/PA33 would "
            "be invalid. Do not implement or rely on the PA32/PA33-based "
            "overflow fix until PA28 is set to 1 on the physical driver -- "
            "see docs/servo_comm_shihlin_merge_design.md §2.4."
        )
        return False

    logger.warning(
        "PA28 = %s -- unexpected value (manual only documents 0 or 1). "
        "Treating as NOT confirmed absolute mode; investigate before "
        "proceeding.",
        value
    )
    return False
