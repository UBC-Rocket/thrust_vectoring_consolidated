import time
from typing import TYPE_CHECKING

from pyocd.coresight.ap import AccessPort, APv1Address, MEM_AP
from pyocd.coresight.cortex_m import CortexM
from pyocd.core.exceptions import TargetSupportError
from pyocd.core.memory_map import FlashRegion

if TYPE_CHECKING:
    from pyocd.core.session import Session
    from pyocd.core.soc_target import SoCTarget
    from pyocd.utility.sequencer import CallSequence
    from pyocd.coresight.dap import DebugPort

# RM0399 2.3.2 Table 6
FLASH_BANK_1_BASE = 0x08000000
FLASH_BANK_2_BASE = 0x08100000

# RM0399 2.3.2 Table 6
AXI_SRAM_BASE = 0x24000000
AXI_SRAM_SIZE = 0x0007FFFF

# Field names used in FlmFlashRegionBuilder._select_flash_ram
# to select the RAM region to place the flashing algorithm into
#
# Taken from commit d1974ffdd16369148ba678478fa85886282d09b1
PYOCD_FLASH_ALGO_RAM_REGION_START_FIELD_NAME = "_RAMstart"
PYOCD_FLASH_ALGO_RAM_REGION_SIZE_FIELD_NAME = "_RAMsize"

# pyOCD https://pyocd.io/docs/options.html
SESSION_OPTION_CONNECT_MODE = "connect_mode"
CONNECT_MODE_UNDER_RESET = "under-reset"

# RM0399 63.4.2
AP_CORTEX_M7 = 0
AP_CORTEX_M4 = 3

# pyOCD script globals
dp: DebugPort
session: Session


def create_mem_ap(ap_address: int) -> MEM_AP:
    address = APv1Address(ap_address)
    ap = AccessPort.create(dp, address)

    if not isinstance(ap, MEM_AP):
        raise TargetSupportError(f"AP {ap_address} is not MEM-AP type")

    return ap


def ap_set_reset_catch(ap: MEM_AP) -> None:
    demcr = ap.read32(CortexM.DEMCR)

    if (demcr & CortexM.DEMCR_VC_CORERESET) == 0:
        ap.write32(CortexM.DEMCR, demcr | CortexM.DEMCR_VC_CORERESET)


def ap_clear_reset_catch(ap: MEM_AP) -> None:
    demcr = ap.read32(CortexM.DEMCR)

    if (demcr & CortexM.DEMCR_VC_CORERESET) != 0:
        ap.write32(CortexM.DEMCR, demcr & ~CortexM.DEMCR_VC_CORERESET)


def ap_halt(ap: MEM_AP) -> None:
    ap.write32(CortexM.DHCSR, CortexM.DBGKEY | CortexM.C_DEBUGEN | CortexM.C_HALT)


def unassert_reset_for_discovery() -> None:
    assert dp.is_reset_asserted(), "Device must still be under reset"

    ap_cm7 = create_mem_ap(AP_CORTEX_M7)
    ap_cm4 = create_mem_ap(AP_CORTEX_M4)

    # Enable debug and try to halt
    ap_halt(ap_cm7)
    ap_halt(ap_cm4)
    dp.flush()

    # Ensure we don't run any code on the cores by halting them right
    # after reset
    ap_set_reset_catch(ap_cm7)
    ap_set_reset_catch(ap_cm4)
    dp.flush()

    # Release from reset so that discovery can find CM7
    dp.assert_reset(False)

    # TODO: detect cores are reset vector halted instead of hardcoding a wait?
    time.sleep(0.1)

    # Only halt on reset vector once (for this setup)
    ap_clear_reset_catch(ap_cm7)
    ap_clear_reset_catch(ap_cm4)
    dp.flush()


def set_flash_region_algo_ram_region(
    region: FlashRegion, ram_start: int, ram_size: int
) -> None:
    region.attributes[PYOCD_FLASH_ALGO_RAM_REGION_START_FIELD_NAME] = ram_start
    region.attributes[PYOCD_FLASH_ALGO_RAM_REGION_SIZE_FIELD_NAME] = ram_size


def patch_flash_algorithm_ram_region_for_cm7(target: SoCTarget) -> None:
    flash_bank_1_region = target.memory_map.get_region_for_address(FLASH_BANK_1_BASE)
    flash_bank_2_region = target.memory_map.get_region_for_address(FLASH_BANK_2_BASE)

    if not isinstance(flash_bank_1_region, FlashRegion):
        raise TargetSupportError("Missing flash region for bank 1")

    if not isinstance(flash_bank_2_region, FlashRegion):
        raise TargetSupportError("Missing flash region for bank 2")

    set_flash_region_algo_ram_region(flash_bank_1_region, AXI_SRAM_BASE, AXI_SRAM_SIZE)
    set_flash_region_algo_ram_region(flash_bank_2_region, AXI_SRAM_BASE, AXI_SRAM_SIZE)


# Called by pyOCD
def will_init_target(target: SoCTarget, init_sequence: CallSequence) -> None:
    if session.options.get(SESSION_OPTION_CONNECT_MODE) == CONNECT_MODE_UNDER_RESET:
        init_sequence.insert_after(
            "dp_init", ("unassert_reset_for_discovery", unassert_reset_for_discovery)
        )

    patch_flash_algorithm_ram_region_for_cm7(target)
