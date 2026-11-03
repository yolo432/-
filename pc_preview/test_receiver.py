import unittest
from receive_video import Assembler, HEADER, drain_packets
from receive_video import color_lut, apply_color, render_preview
import numpy as np

def packet(fid, row, values):
    payload = b''.join(value.to_bytes(2, 'big') for value in values)
    return HEADER.pack(b'CAM1', fid, 2, 2, row, 1, 0, len(payload)) + payload

class ReceiverTests(unittest.TestCase):
    def test_640_frame_and_stale_cleanup(self):
        rx = Assembler()
        rx.feed(packet(65535, 0, [0, 0]))
        image = None
        for row in reversed(range(360)):
            payload = (0x07e0).to_bytes(2, 'big') * 640
            data = HEADER.pack(b'CAM1', 0, 640, 360, row, 1, 0, 1280) + payload
            image = rx.feed(data)
        self.assertEqual(image.shape, (360, 640, 3))
        self.assertEqual(image[359, 639].tolist(), [0, 255, 0])
        self.assertEqual(rx.dimensions, (640, 360))
        self.assertEqual(len(rx.frames), 0)
        self.assertEqual(rx.incomplete, 1)
        self.assertIsNone(rx.feed(packet(65535, 1, [0, 0])))

    def test_320_frame_reordered_rows(self):
        rx = Assembler()
        image = None
        for row in reversed(range(180)):
            payload = (0xf800 if row % 2 == 0 else 0x001f).to_bytes(2, 'big') * 320
            data = HEADER.pack(b'CAM1', 1, 320, 180, row, 1, 0, 640) + payload
            image = rx.feed(data)
        self.assertEqual(image.shape, (180, 320, 3))
        self.assertEqual(image[0, 319].tolist(), [0, 0, 255])
        self.assertEqual(image[179, 0].tolist(), [255, 0, 0])
        self.assertEqual(rx.completed, 1)

    def test_display_identity_and_channel_order(self):
        image = np.array([[[12, 60, 120], [255, 0, 0]]], dtype=np.uint8)
        np.testing.assert_array_equal(apply_color(image, color_lut(1)), image)
        corrected = apply_color(image, color_lut(1, (2, 1, 1)))
        self.assertEqual(corrected.tolist(), [[[12, 60, 240], [255, 0, 0]]])
        np.testing.assert_array_equal(image[0, 0], [12, 60, 120])

    def test_display_gamma_ramp_and_endpoints(self):
        lut = color_lut(2.2)
        self.assertEqual(lut[0].tolist(), [0, 0, 0])
        self.assertEqual(lut[255].tolist(), [255, 255, 255])
        self.assertGreater(int(lut[64, 0]), 64)
        self.assertTrue(np.all(np.diff(lut.astype(int), axis=0) >= 0))
        for gamma, gains in [(0, (1, 1, 1)), (float('nan'), (1, 1, 1)),
                             (1, (1, -1, 1)), (1, (1, 1))]:
            with self.assertRaises(ValueError):
                color_lut(gamma, gains)

    def test_batch_drain_and_limit(self):
        np.testing.assert_array_equal(color_lut(), color_lut(1))
        image = np.full((3, 4, 3), 64, dtype=np.uint8)
        for method in ['nearest', 'linear', 'cubic']:
            display = render_preview(image, 3, method, .3)
            self.assertEqual(display.shape, (9, 12, 3))
            np.testing.assert_array_equal(display, np.full_like(display, 64))
        np.testing.assert_array_equal(render_preview(image, 1), image)
        with self.assertRaises(ValueError):
            render_preview(image, sharpen=float('nan'))
        class Socket:
            def __init__(self, items):
                self.items = iter(items)
            def recvfrom(self, size):
                try:
                    return next(self.items)
                except StopIteration:
                    raise BlockingIOError from None
        peer = ('192.168.137.2', 6002)
        sock = Socket([(packet(1, 0, [0, 0]), peer),
                       (b'ignored', ('127.0.0.1', 6002)),
                       (packet(1, 1, [0, 0]), peer)])
        rx = Assembler()
        count, image = drain_packets(sock, rx, limit=2)
        self.assertEqual(count, 1)
        self.assertIsNone(image)
        count, image = drain_packets(sock, rx)
        self.assertEqual(count, 1)
        self.assertIsNotNone(image)
        self.assertEqual(rx.completed, 1)
        self.assertEqual(drain_packets(sock, rx), (0, None))

    def test_reorder_duplicate_and_colors(self):
        rx = Assembler()
        last = packet(1, 1, [0x001f, 0xffff])
        self.assertIsNone(rx.feed(last))
        self.assertIsNone(rx.feed(last))
        image = rx.feed(packet(1, 0, [0xf800, 0x07e0]))
        self.assertEqual(image.tolist(), [[[0, 0, 255], [0, 255, 0]],
                                          [[255, 0, 0], [255, 255, 255]]])
        self.assertIsNone(rx.feed(last))
        self.assertEqual(rx.completed, 1)

    def test_missing_rows_and_bad_packets(self):
        rx = Assembler()
        self.assertIsNone(rx.feed(b'bad'))
        self.assertIsNone(rx.feed(packet(1, 0, [0, 0])[:-1]))
        for fid in range(1, 6):
            self.assertIsNone(rx.feed(packet(fid, 0, [0, 0])))
        self.assertEqual(rx.completed, 0)
        self.assertEqual(rx.incomplete, 2)
        self.assertEqual(len(rx.frames), 3)

    def test_wraparound(self):
        rx = Assembler()
        for fid in [65535, 0, 1]:
            self.assertIsNone(rx.feed(packet(fid, 0, [0, 0])))
            self.assertIsNotNone(rx.feed(packet(fid, 1, [0, 0])))
        self.assertEqual(rx.completed, 3)

if __name__ == '__main__':
    unittest.main()
