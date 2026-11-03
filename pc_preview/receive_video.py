"""CAM1 UDP receiver. Only complete, consistent frames reach the display."""
import argparse
import select
import socket
import struct
import time
from datetime import datetime
from collections import OrderedDict
from pathlib import Path
import cv2
import numpy as np

HEADER = struct.Struct('!4sHHHHBBH')

def positive_float(value):
    number = float(value)
    if not np.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return number

def color_lut(gamma=1.0, rgb_gains=(1.0, 1.0, 1.0)):
    if (not np.isfinite(gamma) or gamma <= 0 or len(rgb_gains) != 3
            or any(not np.isfinite(g) or g <= 0 for g in rgb_gains)):
        raise ValueError('gamma and three RGB gains must be finite and positive')
    # OpenCV frames use BGR. Apply linear gains before display gamma.
    values = np.arange(256, dtype=np.float64)[:, None] / 255.0
    linear = np.clip(values * np.asarray(rgb_gains[::-1]), 0, 1)
    return np.rint(255 * linear ** (1.0 / gamma)).astype(np.uint8)

def apply_color(image, lut):
    return lut[image, np.arange(3)]

def render_preview(image, scale=3, interpolation='cubic', sharpen=0.0):
    if scale not in range(1, 7) or not np.isfinite(sharpen) or not 0 <= sharpen <= 1:
        raise ValueError('scale must be 1..6 and sharpen must be 0..1')
    methods = {'nearest': cv2.INTER_NEAREST, 'cubic': cv2.INTER_CUBIC,
               'linear': cv2.INTER_LINEAR}
    display = cv2.resize(image, (image.shape[1]*scale, image.shape[0]*scale),
                         interpolation=methods[interpolation])
    if sharpen:
        blurred = cv2.GaussianBlur(display, (0, 0), 1.0)
        display = cv2.addWeighted(display, 1 + sharpen, blurred, -sharpen, 0)
    return display

def draw_record_controls(image, recording, record_path=None):
    """Draw a clickable control without changing the frame sent to the writer."""
    button_color = (40, 40, 210) if recording else (40, 150, 40)
    cv2.rectangle(image, (10, 10), (142, 52), button_color, -1)
    cv2.rectangle(image, (10, 10), (142, 52), (240, 240, 240), 1)
    cv2.putText(image, 'STOP' if recording else 'REC',
                (32, 39), cv2.FONT_HERSHEY_SIMPLEX, 0.72,
                (255, 255, 255), 2, cv2.LINE_AA)
    if recording:
        name = record_path.name if record_path is not None else 'recording'
        cv2.putText(image, name[:42], (155, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                    (0, 220, 255), 1, cv2.LINE_AA)

def drain_packets(sock, assembler, limit=512):
    """Bounded nonblocking drain; GUI work must not run once per datagram."""
    packets = 0
    latest = None
    for _ in range(limit):
        try:
            packet, peer = sock.recvfrom(4096)
        except BlockingIOError:
            break
        if peer == ('192.168.137.2', 6002):
            packets += 1
            image = assembler.feed(packet)
            if image is not None:
                latest = image
    return packets, latest

class Assembler:
    def __init__(self):
        self.frames = OrderedDict()
        self.completed = 0
        self.incomplete = 0
        self.last_complete = None
        self.payload_bytes = 0
        self.dimensions = None

    def feed(self, packet):
        if len(packet) < HEADER.size:
            return None
        magic, fid, width, height, row, fmt, flags, length = HEADER.unpack_from(packet)
        if (magic != b'CAM1' or fmt != 1 or flags != 0 or not 1 <= width <= 1280
                or not 1 <= height <= 720 or row >= height or length != width * 2
                or len(packet) != HEADER.size + length):
            return None
        if self.last_complete is not None:
            distance = (fid - self.last_complete) & 0xffff
            if distance == 0 or distance >= 0x8000:
                return None
        key = (fid, width, height)
        self.payload_bytes += len(packet)
        if key not in self.frames:
            self.frames[key] = {}
        rows = self.frames[key]
        rows[row] = packet[HEADER.size:]
        while len(self.frames) > 3:
            self.frames.popitem(last=False)
            self.incomplete += 1
        if len(rows) != height:
            return None
        pixels = np.frombuffer(b''.join(rows[y] for y in range(height)), dtype='>u2')
        pixels = pixels.reshape(height, width)
        image = np.empty((height, width, 3), dtype=np.uint8)
        image[:, :, 2] = ((pixels >> 11) * 255 // 31).astype(np.uint8)
        image[:, :, 1] = (((pixels >> 5) & 63) * 255 // 63).astype(np.uint8)
        image[:, :, 0] = ((pixels & 31) * 255 // 31).astype(np.uint8)
        del self.frames[key]
        self.last_complete = fid
        self.dimensions = (width, height)
        for stale in list(self.frames):
            if ((fid - stale[0]) & 0xffff) < 0x8000:
                del self.frames[stale]
                self.incomplete += 1
        self.completed += 1
        return image

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bind', default='192.168.137.1')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--seconds', type=float, default=0)
    parser.add_argument('--gamma', type=positive_float, default=1.0)
    parser.add_argument('--scale', type=int, choices=range(1, 7), default=3)
    parser.add_argument('--interpolation', choices=['nearest', 'linear', 'cubic'], default='cubic')
    parser.add_argument('--sharpen', type=float, default=0.0)
    parser.add_argument('--rgb-gains', nargs=3, type=positive_float,
                        default=(1.0, 1.0, 1.0), metavar=('R', 'G', 'B'))
    parser.add_argument('--raw', action='store_true', help='disable display correction')
    parser.add_argument('--save-frame', type=Path, help='save the first complete native BGR frame as PNG')
    args = parser.parse_args()
    if not np.isfinite(args.sharpen) or not 0 <= args.sharpen <= 1:
        parser.error('--sharpen must be finite and between 0 and 1')
    lut = color_lut(args.gamma, args.rgb_gains)
    raw = args.raw
    redraw = False
    assembler = Assembler()
    start = last_report = time.monotonic()
    packets = 0
    last_packets = last_frames = 0
    next_gui = start
    latest = None
    saved = False
    writer = None
    recording = False
    record_path = None
    gui_state = {'toggle_record': False}

    def on_mouse(event, x, y, _flags, state):
        if (event == cv2.EVENT_LBUTTONUP and
                10 <= x <= 142 and 10 <= y <= 52):
            state['toggle_record'] = True

    def stop_recording():
        nonlocal writer, recording
        if writer is not None:
            writer.release()
            writer = None
        if recording:
            print('Recording saved: %s' % record_path.resolve())
        recording = False

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        sock.bind((args.bind, 6001))
        sock.setblocking(False)
        print('Listening on %s:6001; board 192.168.137.2:6002; press Q to exit.' % args.bind)
        print('Display gamma=%g RGB gains=%s; R toggles raw/corrected view.' %
              (args.gamma, args.rgb_gains))
        if not args.headless:
            # Keep the window fitted to the single received frame; a freely
            # resized window can leave a misleading blank strip at the side.
            cv2.namedWindow('FPGA Camera UDP', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback('FPGA Camera UDP', on_mouse, gui_state)
            waiting = np.zeros((180, 320, 3), dtype=np.uint8)
            cv2.putText(waiting, 'Waiting for CAM1 frame...',
                        (18, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 220, 255), 1, cv2.LINE_AA)
            draw_record_controls(waiting, False)
            cv2.imshow('FPGA Camera UDP', waiting)
            cv2.waitKey(1)
        while not args.seconds or time.monotonic() - start < args.seconds:
            readable, _, _ = select.select([sock], [], [], 0.01)
            if readable:
                count, image = drain_packets(sock, assembler)
                packets += count
                if image is not None:
                    latest = image
                    redraw = True
                    if args.save_frame and not saved:
                        ok, encoded = cv2.imencode('.png', image)
                        if not ok:
                            raise RuntimeError('PNG encoding failed')
                        encoded.tofile(str(args.save_frame))
                        saved = True
                        print('Saved native frame: %s' % args.save_frame.resolve())
            now = time.monotonic()
            if not args.headless and now >= next_gui:
                if gui_state['toggle_record']:
                    gui_state['toggle_record'] = False
                    if recording:
                        stop_recording()
                    elif latest is not None:
                        record_path = Path(__file__).resolve().parent / (
                            'fpga_record_%s.mp4' %
                            datetime.now().strftime('%Y%m%d_%H%M%S'))
                        display_size = (
                            latest.shape[1] * args.scale,
                            latest.shape[0] * args.scale)
                        writer = cv2.VideoWriter(
                            str(record_path),
                            cv2.VideoWriter_fourcc(*'mp4v'),
                            30.0, display_size)
                        if not writer.isOpened():
                            writer.release()
                            writer = None
                            record_path = None
                            print('Recording failed: MP4 writer unavailable')
                        else:
                            recording = True
                            print('Recording started: %s' % record_path.resolve())

                if latest is not None:
                    display = latest if raw else apply_color(latest, lut)
                    preview = render_preview(
                        display, args.scale, args.interpolation, args.sharpen)
                    if recording and writer is not None:
                        writer.write(preview)
                    draw_record_controls(preview, recording, record_path)
                    cv2.imshow('FPGA Camera UDP', preview)
                    redraw = False
                key = cv2.waitKey(1) & 255
                if key == ord('q'):
                    break
                if key == ord('r'):
                    raw = not raw
                    redraw = True
                    print('Display: %s' % ('raw' if raw else 'corrected'))
                next_gui = time.monotonic() + 1 / 30
            if now - last_report >= 2:
                elapsed = now - last_report
                print('resolution=%s received_payload_Mbps=%.2f' %
                      (assembler.dimensions, assembler.payload_bytes * 8 / (now-start) / 1e6))
                print('packets=%d complete_frames=%d discarded_incomplete=%d average_fps=%.2f recent_fps=%.2f packets_per_sec=%.1f' %
                      (packets, assembler.completed, assembler.incomplete, assembler.completed/(now-start),
                       (assembler.completed-last_frames)/elapsed, (packets-last_packets)/elapsed))
                last_packets, last_frames = packets, assembler.completed
                last_report = now
        stop_recording()
    if not args.headless:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
