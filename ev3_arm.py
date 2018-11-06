import ev3link
import pid
import nengo
import numpy as np

m1 = '/sys/class/tacho-motor/motor1/'

link = ev3link.EV3Link('192.168.0.1')
link.wait_for_connection()
link.write(m1+'position', '0')
link.write(m1+'command', 'run-direct')

target_pos = -1.0

dt = 0.001
position_scale = 1.0/300
power_scale = 20.0

n_neurons = 300
dimensions = 1
radius = 1
synapse = 0.01
learning_rate = 1e-4


model = nengo.Network()
with model:
    def send_command_fn(t, x):
        v = int(np.clip(x*100, -100, 100))
        link.write(m1+'duty_cycle_sp', str(v))
        return v
    send_command = nengo.Node(send_command_fn, size_in=1, size_out=1)

    def read_pos_fn(t):
        return float(link.read(m1+'position'))*position_scale
    read_pos = nengo.Node(read_pos_fn)

    period = 1
    target = nengo.Node(lambda t: (-np.cos(t*2*np.pi/period)+1)/2*target_pos)

    controller = pid.PID(Kp=1.0, Kd=10.0, Ki=0.0, tau_d=0.001, dt=dt)
    
    ctrl = nengo.Node(lambda t, x: power_scale * controller.step(
                                                   state=x[0:1],
                                                   desired_state=x[1:2],
                                                   desired_dstate=x[2:3]),
                      size_in=3)
    nengo.Connection(read_pos, ctrl[0], synapse=None)
    nengo.Connection(target, ctrl[1], synapse=None)

    # compute the derivative
    #nengo.Connection(target, ctrl[2], transform=1.0/dt, synapse=None)
    #nengo.Connection(target, ctrl[2], transform=-1.0/dt, synapse=0)

    nengo.Connection(ctrl, send_command, synapse=None)


    adapt = nengo.Ensemble(n_neurons, dimensions=1,
                           radius=radius)
    nengo.Connection(read_pos, adapt, synapse=None)
    conn = nengo.Connection(adapt, send_command, synapse=synapse,
                            function=lambda x: [0],
                            learning_rule_type=nengo.PES(learning_rate))
    nengo.Connection(ctrl, conn.learning_rule, synapse=None, transform=-1)


    import nengo_learning_display
    learn = nengo_learning_display.Plot1D(conn, np.linspace(-1, 1, 50),
                                          range=(-1, 1))


    compare = nengo.Node(None, size_in=2)
    nengo.Connection(read_pos, compare[0], synapse=None)
    nengo.Connection(target, compare[1], synapse=None)


def on_close(sim):
    link.write(m1+'duty_cycle_sp', '0')

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

