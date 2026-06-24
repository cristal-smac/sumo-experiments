import os
import torch
from abc import ABC, abstractmethod
from itertools import count
from threading import Lock
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


class _ZeroZeusMeasurement:
    """Mock Zeus measurement that returns zeros to prevent simulation crashes on failure."""
    def __init__(self):
        self.gpu_energy = None
        self.cpu_energy = None
        self.dram_energy = None
        self.soc_energy = None


class MultiprocessSafeZeusMonitor:
    """
    Wraps ZeusMonitor to isolate keys across multiple processes (PIDs) 
    and multi-threaded strategy instances, guarding against hard failures with muted logging.
    """
    def __init__(self, monitor, monitor_id: int):
        self._monitor = monitor
        self._lock = Lock()
        self._prefix = f"pid{os.getpid()}:monitor-{monitor_id}"
        self._key_map = {}
        self._logged_begin_error = False
        self._logged_end_error = False

    def begin_window(self, key, *args, **kwargs):
        scoped_key = f"{self._prefix}:{key}"
        with self._lock:
            try:
                self._monitor.begin_window(scoped_key, *args, **kwargs)
                self._key_map[key] = scoped_key
            except Exception as exc:
                if not self._logged_begin_error:
                    print(f"[Zeus debug] begin_window failed for {scoped_key}: {exc.__class__.__name__}: {exc}")
                    self._logged_begin_error = True
                self._key_map[key] = None

    def end_window(self, key, *args, **kwargs):
        with self._lock:
            scoped_key = self._key_map.pop(key, None)
        
        if scoped_key is None:
            return _ZeroZeusMeasurement()
            
        try:
            return self._monitor.end_window(scoped_key, *args, **kwargs)
        except Exception as exc:
            if not self._logged_end_error:
                print(f"[Zeus debug] end_window failed for {scoped_key}: {exc.__class__.__name__}: {exc}")
                self._logged_end_error = True
            return _ZeroZeusMeasurement()


class Strategy(ABC):
    """
    Abstract class to create control strategies for all the traffic lights in a network.
    """

    _zeus_monitor_id_counter = count()
    _zeus_monitor_id_lock = Lock()

    def __init__(self):
        self.zeus_monitor_id = self._next_zeus_monitor_id()
        raw_monitor = self._create_zeus_monitor(self.zeus_monitor_id)
        self.zeus_monitor = MultiprocessSafeZeusMonitor(raw_monitor, self.zeus_monitor_id)
        self.energy_consumption = 0

    @classmethod
    def _next_zeus_monitor_id(cls):
        with cls._zeus_monitor_id_lock:
            return next(cls._zeus_monitor_id_counter)

    @staticmethod
    def _create_zeus_monitor(monitor_id):
        # Fallback to empty gpu_indices list if CUDA is not available or errors out
        gpu_indices = None
        try:
            if not torch.cuda.is_available():
                gpu_indices = []
        except Exception:
            gpu_indices = []

        if gpu_indices is not None:
            monitor = ZeusMonitor(gpu_indices=gpu_indices)
        else:
            monitor = ZeusMonitor()

        return monitor

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
            soc_energy = (measurements.soc_energy.cpu_total_mj + measurements.soc_energy.dram_mj + measurements.soc_energy.gpu_mj) / 1000
            return soc_energy

        gpu_energy = sum([measurements.gpu_energy[key] for key in measurements.gpu_energy]) if measurements.gpu_energy is not None else 0
        cpu_energy = sum([measurements.cpu_energy[key] for key in measurements.cpu_energy]) if measurements.cpu_energy is not None else 0
        dram_energy = sum([measurements.dram_energy[key] for key in measurements.dram_energy]) if measurements.dram_energy is not None else 0
        soc_energy = sum([measurements.soc_energy[key] for key in measurements.soc_energy]) if measurements.soc_energy is not None else 0
        return sum([gpu_energy, cpu_energy, dram_energy, soc_energy])