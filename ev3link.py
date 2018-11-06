import math
import socket
import struct
import threading
import time
import timeit

import numpy as np

class EV3Link(object):
    def __init__(self, addr=None, port=8800):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', port))
        if addr is None:
            self.ev3_addr = None
        else:
            self.ev3_addr = (addr, port)

        self.message_index = 0
        self.condition = threading.Condition()
        self.msg_times = {}
        self.msg_results = {}
        self.last_msg_time = timeit.default_timer()

        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()

        self.no_message_thread = None
        self.no_message_handlers = []

    def on_no_message(self, function):
        if self.no_message_thread is None:
            self.no_message_thread = threading.Thread(target=self.no_message)
            self.no_message_thread.daemon = True
            self.no_message_thread.start()
        self.no_message_handlers.append(function)

    def no_message(self):
        while True:
            now = timeit.default_timer()
            if now > self.last_message_time + 0.5:
                for handler in self.no_message_handlers:
                    handler()
                self.last_message_time = now
            time.sleep(0.1)

    def wait_for_connection(self):
        while self.ev3_addr is None:
            time.sleep(0.01)

    def run(self):
        while True:
            data, addr = self.socket.recvfrom(1024)
            if self.ev3_addr is None:
                self.ev3_addr = addr
            self.parse_data(data)

    def parse_data(self, data):
        r = struct.unpack('HHHH', data[:8])
        self.last_fn = data[8:8 + r[2]].strip()
        self.last_data = data[8 + r[2]:].strip()

        if r[1] == self.message_index:
            with self.condition:
                self.condition.notify()


    def send(self, type, index, fn, data_len=None, data=None, msg_period=None):
        if self.ev3_addr is None:
            print 'Still looking for EV3'
            return

        now = timeit.default_timer()
        self.last_message_time = now

        if msg_period is not None:
            last_time = self.msg_times.get(fn, None)
            if last_time is not None and now < last_time + msg_period:
                return
            self.msg_times[fn] = now

        if data is not None:
            data_len = len(data)

        header = struct.pack('HHHH', type, index, len(fn)+1, data_len)
        message = header + fn + '\x00'
        if data is not None:
            message = message + data
        self.socket.sendto(message, self.ev3_addr)

    def request(self, type, fn, data=None, data_len=None, blocking=True,
                msg_period=None):

        if msg_period is not None:
            last_time = self.msg_times.get(fn, None)
            now = timeit.default_timer()
            if last_time is not None and now < last_time + msg_period:
                return self.msg_results.get(fn, None)
            self.msg_times[fn] = now

        self.message_index += 1
        if blocking:
            with self.condition:
                self.send(type, self.message_index, fn, 
                          data=data, data_len=data_len)
                self.condition.wait()
            self.msg_results[fn] = self.last_data
            return self.last_data
        else:
            self.send(type, self.message_index, fn, 
                      data=data, data_len=data_len)

    def dir(self, fn, data_len=100, msg_period=None):
        return self.request(5, fn, data_len=data_len, msg_period=msg_period)

    def read(self, fn, data_len=1000, msg_period=None):
        return self.request(3, fn, data_len=data_len, msg_period=msg_period)

    def write(self, fn, data, msg_period=None, blocking=False):
        self.request(1, fn, data=data, blocking=blocking,
                     msg_period=msg_period)


if __name__ == '__main__':
    link = EV3Link('192.168.0.103')
    link.wait_for_connection()
    print link.dir('/sys/class/tacho-motor')
    link.write('/sys/class/tacho-motor/motor0/command', 'run-direct')

    data = []

    now = time.time()
    for i in range(500):
        p = 0
        p = link.read('/sys/class/tacho-motor/motor0/position')
        t = time.time() - now
        data.append((t, float(p)))
        value = '%d' % (30 * math.sin(t*math.pi*2))
        link.write('/sys/class/tacho-motor/motor0/duty_cycle_sp', data=value)
        time.sleep(0.0001)
    print time.time() - now

    time.sleep(0.1)
    link.write('/sys/class/tacho-motor/motor0/duty_cycle_sp', data='0')

    import pylab
    data = np.array(data)
    pylab.plot(data[:,0], data[:,1])
    pylab.show()
