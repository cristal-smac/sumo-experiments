from abc import ABC, abstractmethod
from zeus.monitor import ZeusMonitor

# Fix zeus v0.15.0 bug: AppleSiliconMeasurement defines zero_all_fields but
# the ABC expects zeroAllFields (camelCase). Monkey-patch it so instantiation works.
try:
    from zeus.device.soc.apple import AppleSiliconMeasurement
    if not hasattr(AppleSiliconMeasurement, 'zeroAllFields') or \
       'zeroAllFields' in getattr(AppleSiliconMeasurement, '__abstractmethods__', set()):
        AppleSiliconMeasurement.zeroAllFields = AppleSiliconMeasurement.zero_all_fields
        AppleSiliconMeasurement.__abstractmethods__ = frozenset(
            AppleSiliconMeasurement.__abstractmethods__ - {'zeroAllFields'}
        )
except ImportError:
    pass


class Strategy(ABC):
    """
    Abstract class to create control strategies for all the traffic lights in a network.
    """

    def __init__(self):
        self.zeus_monitor = ZeusMonitor()
        self.energy_consumption = 0

    @abstractmethod
    def run_all_agents(self, traci):
        """
        Perform the choose action for all agents of the strategy.
        :param traci: The simulation instance of Traci
        :return: Nothing
        """
        pass

    def get_energy_consumption(self, measurements):
        """
        Get the total energy consumption of a measurement window.
        Handles both x86/Linux (dict-based metrics) and Apple Silicon (SoC object with mJ fields).
        """

        if measurements.soc_energy is not None and hasattr(measurements.soc_energy, 'cpu_total_mj'):
            # Apple Silicon: SoC object has cpu + dram in millijoules, convert to Joules.
            # cpu_energy / dram_energy dicts are None on Apple Silicon, so only use soc_energy.
            soc_energy = (measurements.soc_energy.cpu_total_mj + measurements.soc_energy.dram_mj + measurements.soc_energy.gpu_mj) / 1000
            return soc_energy

        # x86 / Linux: separate per-device dicts in Joules
        gpu_energy = sum([measurements.gpu_energy[key] for key in measurements.gpu_energy]) if measurements.gpu_energy is not None else 0
        cpu_energy = sum([measurements.cpu_energy[key] for key in measurements.cpu_energy]) if measurements.cpu_energy is not None else 0
        dram_energy = sum([measurements.dram_energy[key] for key in measurements.dram_energy]) if measurements.dram_energy is not None else 0
        soc_energy = sum([measurements.soc_energy[key] for key in measurements.soc_energy]) if measurements.soc_energy is not None else 0
        return sum([gpu_energy, cpu_energy, dram_energy, soc_energy])