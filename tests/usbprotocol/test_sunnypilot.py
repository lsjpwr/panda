import unittest
from unittest.mock import Mock, call

from panda import Panda
from opendbc.car.structs import CarParams


class TestSunnyPilotHost(unittest.TestCase):
  def setUp(self):
    self.panda = Panda.__new__(Panda)
    self.panda._handle = Mock()
    self.panda.health_version = Panda.HEALTH_PACKET_VERSION

  def test_safety_parameters_keep_separate_16_bit_values(self):
    self.panda.set_alternative_experience(0x1234, 0x8009)
    self.panda.set_safety_mode(CarParams.SafetyModel.tesla, 6)
    self.assertEqual(self.panda._handle.controlWrite.call_args_list, [
      call(Panda.REQUEST_OUT, 0xdf, 0x1234, 0x8009, b''),
      call(Panda.REQUEST_OUT, 0xdc, CarParams.SafetyModel.tesla, 6, b''),
    ])

  def test_heartbeat_preserves_independent_mads_engagement(self):
    for engaged in (False, True):
      for mads in (False, True):
        with self.subTest(engaged=engaged, mads=mads):
          self.panda.send_heartbeat(engaged, mads)
          self.panda._handle.controlWrite.assert_called_with(Panda.REQUEST_OUT, 0xf3, engaged, mads, b'')

  def test_existing_call_defaults(self):
    self.panda.set_alternative_experience(0)
    self.panda.send_heartbeat()
    self.assertEqual(self.panda._handle.controlWrite.call_args_list, [
      call(Panda.REQUEST_OUT, 0xdf, 0, 0, b''),
      call(Panda.REQUEST_OUT, 0xf3, True, True, b''),
    ])

  def test_health_decodes_firmware_packet(self):
    # Packed health_t: counters 1..8, all flags, Tesla safety, independent MADS lateral control.
    payload = bytes.fromhex(
      '01000000 0200 0300 04000000 05000000 06000000 07000000 08000000 ' +
      'ff01 01 0a 0600 02 3412 ff 32 0900 e803 d007 b80b 41 01'
    )
    self.panda._handle.controlRead.return_value = payload
    health = self.panda.health()
    self.assertEqual(health['uptime'], 1)
    self.assertEqual(health['safety_param'], 6)
    self.assertEqual(health['alternative_experience'], 0x1234)
    self.assertEqual(health['spi_error_count'], 9)
    self.assertEqual(health['temperature'], 25)
    self.assertEqual(health['interrupt_load'], 1.0)
    self.assertEqual(health['controls_allowed_lateral'], 1)
    self.assertEqual(health['controls_allowed_longitudinal'], 0)
    for key in ('ignition_line', 'ignition_can', 'controls_allowed', 'heartbeat_lost',
                'power_save_enabled', 'safety_rx_checks_invalid', 'som_reset_triggered', 'nmi_reset', 'hardfault_reset'):
      self.assertTrue(health[key], key)

  def test_health_rejects_incompatible_firmware(self):
    self.panda.health_version ^= 1
    with self.assertRaisesRegex(RuntimeError, 'health packet version mismatch'):
      self.panda.health()
    self.panda._handle.controlRead.assert_not_called()
