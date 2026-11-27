"""Exposure protocol and restoration regressions without camera hardware."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import receive_photo


class ExposureTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parent.parent / 'k210_command_photo_230400.py'
        spec = importlib.util.spec_from_file_location('exposure_firmware', path)
        self.firmware = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.firmware)

    def test_command_validation(self):
        for scale in (.5, .75):
            command = dict(type='capture', version=1, request_id='test', exposure_scale=scale)
            self.assertEqual(self.firmware.parse_command_line(json.dumps(command).encode()), command)
        for scale in (True, 0, 2):
            command = dict(type='capture', version=1, request_id='test', exposure_scale=scale)
            self.assertEqual(self.firmware.parse_command_line(json.dumps(command).encode())['type'], 'command_error')

    def test_restore_auto_exposure_even_when_save_fails(self):
        events = []
        def fail_save(*args, **kwargs):
            raise OSError('SD write failed')
        photo = SimpleNamespace(width=lambda: 640, height=lambda: 480, save=fail_save)
        sensor = SimpleNamespace(VGA=1, QVGA=2, set_framesize=lambda size: events.append(size),
                                 set_vflip=lambda value: None, set_hmirror=lambda value: None,
                                 skip_frames=lambda **kwargs: None, snapshot=lambda: photo,
                                 get_exposure_us=lambda: 10000,
                                 set_auto_exposure=lambda *args, **kwargs: events.append((args, kwargs)))
        with patch.dict(sys.modules, sensor=sensor):
            with self.assertRaises(OSError):
                self.firmware.capture_vga_photo(.5)
        self.assertIn(((False,), {'exposure_us': 5000}), events)
        self.assertIn(((True,), {}), events)
        self.assertEqual(events[-1], sensor.QVGA)

    def test_invalid_exposure_readback_retries_once_then_uses_auto(self):
        for readbacks, expected_scale in [([0, 10000], .5), ([-1, None], 1.0)]:
            with self.subTest(readbacks=readbacks):
                events = []
                values = iter(readbacks)
                photo = SimpleNamespace(width=lambda: 640, height=lambda: 480,
                                        save=lambda *_args, **_kwargs: None)

                def read_exposure():
                    events.append("read")
                    return next(values)

                sensor = SimpleNamespace(
                    VGA=1, QVGA=2, set_framesize=lambda size: events.append(size),
                    set_vflip=lambda value: None, set_hmirror=lambda value: None,
                    skip_frames=lambda **kwargs: events.append(kwargs["time"]),
                    snapshot=lambda: photo, get_exposure_us=read_exposure,
                    set_auto_exposure=lambda enabled, **kwargs: events.append((enabled, kwargs)))
                with patch.dict(sys.modules, sensor=sensor):
                    self.assertEqual(self.firmware.capture_vga_photo(.5), (640, 480, expected_scale))
                self.assertEqual(events[:5], [sensor.VGA, 600, "read", 250, "read"])
                self.assertEqual((False, {"exposure_us": 5000}) in events, expected_scale == .5)
                self.assertEqual(events[-3:], [(True, {}), sensor.QVGA, 300])

    def test_host_negotiates_new_and_old_firmware(self):
        class Sent(Exception):
            pass
        for supported in (False, True):
            commands = []
            def write(payload):
                commands.append(json.loads(payload.decode().split(':', 1)[1]))
                raise Sent()
            pong = {'capabilities': ['capture'] + (['exposure_scale'] if supported else []), 'busy': False}
            with patch.object(receive_photo, 'exchange_ping', return_value=pong):
                with self.assertRaises(Sent):
                    receive_photo._receive_requested_photo(SimpleNamespace(write=write), '.',
                                                           boot_wait=0, exposure_scale=.5)
            self.assertEqual(commands[0].get('exposure_scale'), .5 if supported else None)


if __name__ == '__main__':
    unittest.main()
