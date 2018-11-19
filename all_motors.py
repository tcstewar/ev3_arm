import socket
import timeit
import os

def find_motors():
    dirs = {}
    for motor in os.listdir(r'/sys/class/tacho-motor'):
        if not motor.startswith('motor'):
            continue
        with open(r'/sys/class/tacho-motor/%s/address' % motor) as f:
            address = f.read().strip()
        assert address.startswith('out')
        dirs[address[3:]] = '/sys/class/tacho-motor/%s' % motor
    return dirs

print('finding motors...')
motors = find_motors()
for m in 'ABCD':
    print('  motor %s: %s' % (m, motors[m]))
    with open(motors[m] + '/position', 'w') as f:
        f.write('0')
    with open(motors[m] + '/command', 'w') as f:
        f.write('run-direct')


send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
send_ip = '192.168.0.2'
send_port = 8500
send = (send_ip, send_port)

recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind(('192.168.0.1', 8500))
recv_sock.setblocking(False)


pos_f = {}
for m in 'ABCD':
    pos_f[m] = open(motors[m]+'/position')

duty_f = {}
for m in 'ABCD':
    duty_f[m] = open(motors[m]+'/duty_cycle_sp', 'w')


while True:
    pos = []
    for m in 'C':
        f = pos_f[m]
        f.seek(0)
        pos.append(int(f.read()))


    #sock.sendto('%d %d %d' % (pos[0], pos[1], (pos[2]+pos[3])/2), send)
    #sock.sendto('%d' % ((pos[0]+pos[1])//2), send)
    send_sock.sendto('%d' % pos[0], send)

    try:
        data, addr = recv_sock.recvfrom(128)
    except socket.error:
        data = None

    if data is not None:
        duty_f['C'].seek(0)
        duty_f['D'].seek(0)
        duty_f['C'].write(data)
        duty_f['D'].write(data)
        duty_f['C'].flush()
        duty_f['D'].flush()
~
~
~
~
