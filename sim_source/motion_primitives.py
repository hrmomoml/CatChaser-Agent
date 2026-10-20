"""Cartesian toy-tip primitives and a strict, non-executing command parser."""
import re
from dataclasses import dataclass

import numpy as np


ACTION_NAMES = {1: '蹲下', 2: '站起', 3: '上下摆动', 4: '左右摆动', 5: '画圆',
                6: '快速前伸—停顿—慢速收回', 7: '从左到右画正弦波', 8: '左右快扫—停顿'}


@dataclass(frozen=True)
class Command:
    posture: int | None = None
    arm: int | None = None

    def __post_init__(self):
        if self.posture not in (None, 1, 2) or self.arm not in (None, 3, 4, 5, 6, 7, 8):
            raise ValueError('姿态只能选 1 或 2，摆臂只能选 3～8。')
        if self.posture is None and self.arm is None:
            raise ValueError('指令不能为空。')

    def __str__(self):
        return '+'.join(str(value) for value in (self.posture, self.arm) if value is not None)

    @property
    def name(self):
        return ' + '.join(ACTION_NAMES[value] for value in (self.posture, self.arm) if value is not None)


def single_command(number):
    return Command(posture=number) if number in (1, 2) else Command(arm=number)


def parse_command(line):
    line = line.strip().lower().translate(str.maketrans('１２３４５６７８，＋', '12345678,+'))
    if line in ('q', 'quit', 'exit'):
        return 'quit'
    if line in ('0', 'stop'):
        return 'stop'
    if line in ('h', 'help', '?'):
        return 'help'
    if not line:
        return []
    if not re.fullmatch(r'[1-8\s,+]+', line):
        raise ValueError('请输入 1～8；空格表示依次执行，加号表示同时执行，例如 1+3 2+5。')
    line = re.sub(r'\s*\+\s*', '+', line)
    commands = []
    for token in re.split(r'[\s,]+', line):
        if not token:
            continue
        if '+' not in token:
            commands.extend(single_command(int(char)) for char in token)
            continue
        parts = token.split('+')
        if len(parts) != 2 or any(len(part) != 1 for part in parts):
            raise ValueError('同时组合只能是一种姿态和一种摆臂，例如 1+3 或 2+7。')
        values = [int(part) for part in parts]
        postures = [value for value in values if value in (1, 2)]
        arms = [value for value in values if value >= 3]
        if len(postures) != 1 or len(arms) != 1:
            raise ValueError('1 和 2 不能同时执行；同一时刻只能选择一种摆臂动作。')
        commands.append(Command(postures[0], arms[0]))
    return commands


def smoothstep(value):
    s = np.clip(value, 0., 1.)
    return s*s*s*(10.-15.*s+6.*s*s)


def displacement(action, elapsed, duration, amplitude, frequency):
    """Return a continuous offset that starts and ends at the common center.

    y is left, z is up, x is forward. All primitives keep the same front workspace.
    The dart uses a shorter reach than the lateral primitives, for arm workspace.
    """
    t = float(np.clip(elapsed, 0., duration))
    ramp = min(1., t/.7, (duration-t)/.7)
    envelope = float(np.sin(.5*np.pi*max(0., ramp))**2)
    phase = 2.*np.pi*frequency*t
    offset = np.zeros(3)
    if action == 3:
        offset[2] = amplitude*np.sin(phase)
    elif action == 4:
        offset[1] = amplitude*np.sin(phase)
    elif action == 5:
        phase = 2.*np.pi*min(frequency, 1.2)*t
        offset[1] = amplitude*np.cos(phase)
        offset[2] = amplitude*np.sin(phase)
    elif action == 6:
        local = t % 2.2
        reach = min(.10, amplitude)
        if local < .20:
            offset[0] = reach*smoothstep(local/.20)
        elif local < .65:
            offset[0] = reach
        elif local < 1.80:
            offset[0] = reach*(1.-smoothstep((local-.65)/1.15))
    elif action == 7:
        # Prepare at the left, sweep right through six waves, then recenter.
        sweep_amplitude = amplitude*(4./3.)
        wave_amplitude = min(.10, amplitude*(2./3.))
        wave_cycles = 6
        sweep_time = wave_cycles/frequency
        cycle_time = .5+sweep_time+.7
        local = t % cycle_time
        if local < .5:
            offset[1] = sweep_amplitude*smoothstep(local/.5)
        elif local < .5+sweep_time:
            s = smoothstep((local-.5)/sweep_time)
            offset[1] = sweep_amplitude*(1.-2.*s)
            offset[2] = wave_amplitude*np.sin(2.*np.pi*wave_cycles*s)
        else:
            offset[1] = -sweep_amplitude*(1.-smoothstep((local-.5-sweep_time)/.7))
    elif action == 8:
        # One left-pause-right-pause-left sequence, then hold and recenter.
        reach = amplitude*(4./3.)
        sweep = .4
        pause = 2.
        if t < sweep:
            offset[1] = reach*smoothstep(t/sweep)
        elif t < sweep+pause:
            offset[1] = reach
        elif t < 2*sweep+pause:
            offset[1] = reach*(1.-2.*smoothstep((t-sweep-pause)/sweep))
        elif t < 2*sweep+2*pause:
            offset[1] = -reach
        elif t < 3*sweep+2*pause:
            offset[1] = reach*(-1.+2.*smoothstep((t-2*sweep-2*pause)/sweep))
        else:
            offset[1] = reach
        # Preserve the full two-second pauses; only soften the final recenter.
        return offset*float(smoothstep((duration-t)/.7))
    else:
        raise ValueError(f'Unknown arm action: {action}')
    return offset*envelope
