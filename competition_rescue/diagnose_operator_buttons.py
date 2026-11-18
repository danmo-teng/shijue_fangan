#!/usr/bin/env python3
"""Live GPIO observation using the production recognizer; no UART/start writes."""
from __future__ import annotations

import argparse
import datetime
import fcntl
import json
from pathlib import Path
import queue
import threading
import time

from button_input import SysfsButtonInput

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    runtime = ROOT / 'rescue_map/runtime'
    # Same exclusive lock as the real button panel: a test press must never
    # simultaneously reach a running competition launcher.
    lock = (runtime / 'operator.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        raise SystemExit('正式按键入口正在运行，请先关闭它；测试尚未开始。')
    directory = runtime / 'button_tests' / datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True)
    path = directory / 'button_events.jsonl'
    stream = path.open('w', buffering=1)
    messages = queue.Queue()
    stop = threading.Event()
    reader = SysfsButtonInput()
    previous_levels = {}
    down_since = {}
    begin = time.monotonic()
    stats = {'polls': 0, 'max_poll_gap_s': 0.0}

    def record(kind, now, **fields):
        row = dict(event=kind, monotonic_s=now, elapsed_s=now-begin, **fields)
        stream.write(json.dumps(row, ensure_ascii=False)+'\n')
        messages.put(row)

    def poll_loop():
        previous_poll = None
        try:
            record('test_started', begin, note='原20ms轮询/30ms消抖；所有按键只记录；无串口/CONFIG/预热动作')
            while not stop.is_set():
                now = time.monotonic()
                gap = 0. if previous_poll is None else now-previous_poll
                previous_poll = now
                stats['polls'] += 1
                stats['max_poll_gap_s'] = max(stats['max_poll_gap_s'], gap)
                events = reader.poll(now)
                for key, button in reader._buttons.items():
                    raw = button.raw
                    if key not in previous_levels or previous_levels[key] != raw:
                        initial = key not in previous_levels
                        held = None
                        if raw:
                            down_since[key] = now
                        elif key in down_since:
                            held = now-down_since.pop(key)
                        record('initial_level' if initial else 'raw_edge', now, key=key,
                               pressed=raw, held_s=held, poll_gap_s=gap,
                               stable=button.stable, armed=button.armed,
                               long_sent=button.long_sent, chord_blocked=reader._chord_blocked)
                        previous_levels[key] = raw
                for event in events:
                    record('recognized', now, input_event=event,
                           chord_blocked=reader._chord_blocked)
                stop.wait(.02)
        except Exception as exc:
            record('test_error', time.monotonic(), error=repr(exc))
            stop.set()

    worker = threading.Thread(target=poll_loop, daemon=True)
    worker.start()
    names = {'start_pressed':'3号按下已确认', 'start':'短按3：识别成功（不发车）',
             'prewarm':'长按3：识别成功（不启动预热）', 'stop':'短按4：识别成功（仅记录）',
             'exit_session':'长按4：识别成功（原程序当前3秒；仅记录）'}

    def describe(row):
        prefix = f"{row['elapsed_s']:8.3f}s  "
        if row['event'] == 'recognized':
            event = row['input_event']
            return prefix+names.get(event['action'], str(event))
        if row['event'] in {'raw_edge', 'initial_level'}:
            state = '按下' if row['pressed'] else '松开' if row['pressed'] is False else '读取失败'
            held = '' if row['held_s'] is None else f"，保持{row['held_s']:.3f}秒"
            return prefix+f"GPIO {row['key']}：{state}{held}；组合拦截={row['chord_blocked']}"
        return prefix+str({k:v for k,v in row.items() if k not in {'elapsed_s','monotonic_s'}})

    try:
        if args.headless:
            print('测试已启动，只读GPIO，不发车。日志：'+str(path), flush=True)
            while not stop.is_set():
                try:
                    print(describe(messages.get(timeout=.2)), flush=True)
                except queue.Empty:
                    pass
        else:
            import tkinter as tk
            from tkinter.scrolledtext import ScrolledText
            window = tk.Tk()
            window.title('按键现场诊断——短按不会发车')
            window.geometry('950x620')
            tk.Label(window, text='独立测试：只读按键，不发任何下位机指令', font=('',16)).pack(pady=8)
            tk.Label(window, text='先松开所有键，再长按3约1.5秒并松开；随后短按3。每次之间停1秒。').pack()
            tk.Label(window, text='原程序识别逻辑；此轮只测按键输入，不测真实预热和蜂鸣器。').pack()
            box = ScrolledText(window, font=('',12), wrap='word')
            box.pack(fill='both', expand=True, padx=8, pady=8)
            tk.Label(window, text=str(path), wraplength=920).pack()
            tk.Button(window, text='结束测试', command=window.destroy).pack(pady=8)
            def refresh():
                while True:
                    try:
                        row = messages.get_nowait()
                    except queue.Empty:
                        break
                    box.insert('end', describe(row)+'\n');box.see('end')
                window.after(50, refresh)
            refresh()
            window.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set();worker.join(timeout=2)
        record('test_finished', time.monotonic(), **stats)
        stream.close();lock.close()


if __name__ == '__main__':
    main()
