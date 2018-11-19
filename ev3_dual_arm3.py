import pid
import nengo
import numpy as np
import socket
import threading
import timeit

dt = 0.001
pos_scale = 1.0/90
power_scale = 1.0
period = 2
dscale = 0.002   # TODO: figure out why this number
send_synapse = None
n_neurons = 500
radius = 1.0
adapt_synapse=0.001
learning_rate = 1e-6

Kp = 1.0
Kd = 10.0
Ki = 0.0
do_adapt = True


class Communication(object):
    def __init__(self):
        self.recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv.bind(('192.168.0.2', 8500))
        self.send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_info = ('192.168.0.1', 8500)
        self.running = True
        self.data = [0]
        self.command = [0]
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        counter = 0
        t_start = timeit.default_timer()
        while self.running:
            data, addr = self.recv.recvfrom(1024)
            self.data = [float(x) for x in data.split()]

            self.send.sendto(('%d' % self.command[0]).encode(), self.send_info)
            now = timeit.default_timer()
            counter += 1
            if now > t_start + 1.0:
                rate = (now - t_start) / counter
                print('recv period: %1.3fms' % (rate*1000))
                t_start = now
                counter = 0
        self.send.sendto('0'.encode(), self.send_info)

ev3 = Communication()


model = nengo.Network()
with model:
    def read_func(t):
        return [x*pos_scale for x in ev3.data]
    read_pos = nengo.Node(read_func)

    def send_func(t, x):
        ev3.command[0] = int(np.clip(x*100, -100, 100))
    send_power = nengo.Node(send_func, size_in=1)



    target = nengo.Node(lambda t: np.cos(t*2*np.pi/period)*0.3)

    controller = pid.PID(Kp=Kp, Kd=Kd, Ki=Ki, tau_d=0.001, dt=dt)
    ctrl = nengo.Node(lambda t, x: power_scale * controller.step(
                                                   state=x[0:1],
                                                   desired_state=x[1:2],
                                                   desired_dstate=x[2:3]),
                      size_in=3)
    nengo.Connection(read_pos, ctrl[0], synapse=None)
    nengo.Connection(target, ctrl[1], synapse=None)

    # compute the derivative
    nengo.Connection(target, ctrl[2], transform=dscale/dt, synapse=None)
    nengo.Connection(target, ctrl[2], transform=-dscale/dt, synapse=0)

    nengo.Connection(ctrl, send_power, synapse=send_synapse)

    compare = nengo.Node(None, size_in=2)
    nengo.Connection(target, compare[0], synapse=None)
    nengo.Connection(read_pos, compare[1], synapse=None)

    if do_adapt:
        adapt = nengo.Ensemble(n_neurons, dimensions=1,
                               radius=radius)
        nengo.Connection(read_pos, adapt, synapse=None)
        conn = nengo.Connection(adapt, send_power, synapse=adapt_synapse,
                                function=lambda x: [0],
                                learning_rule_type=nengo.PES(learning_rate))

        def learning_rule_gate_fn(t, x):
            if np.abs(x) > 1.0:
                return 0
            else:
                return x
        learning_rule_gate = nengo.Node(learning_rule_gate_fn, size_in=1)
        nengo.Connection(ctrl, learning_rule_gate, synapse=None)
        nengo.Connection(learning_rule_gate, conn.learning_rule, synapse=None, transform=-1)


        import nengo_learning_display
        learn = nengo_learning_display.Plot1D(conn, np.linspace(-1, 1, 50),
                                              range=(-0.1, 0.1))



def on_close(sim):
    ev3.running = False

def on_step(sim):
    learn.update(sim)

import nengo_gui
import webbrowser
import sys
host = 'localhost'
port = 8080
server_settings = nengo_gui.guibackend.GuiServerSettings((host, port))
model_context = nengo_gui.guibackend.ModelContext(
                   model=model,
                   locals=locals(),
                   filename=sys.argv[0],
                   writeable=False)
page_settings = nengo_gui.page.PageSettings(
                   filename_cfg=sys.argv[0] + '.cfg',
                   backend='nengo',
                   editor_class=nengo_gui.components.editor.NoEditor)
server = nengo_gui.gui.BaseGUI(
                   model_context, server_settings, page_settings)
wb = webbrowser.get().open('%s://%s:%d/?token=%s' % (
                   'http', host, port, server.server.gen_one_time_token()))
server.start()

