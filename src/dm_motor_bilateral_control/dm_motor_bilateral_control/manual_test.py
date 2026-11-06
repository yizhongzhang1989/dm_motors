"""Bounded manual interaction with separate, offline operator qualification."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.signals import SignalHandlerOptions
from dm_motor_interfaces.msg import BilateralState
from dm_motor_interfaces.srv import BilateralControl

from .auto_test import (AutomaticTest, namespace, plot_svg, recorded_velocity_resolutions,
                        write_json)
from .manual_analysis import (LIMITATION, confirm, read_rows, summarize_manual,
                              write_manifest)


class ManualTest(AutomaticTest):
    def __init__(self, args):
        super().__init__(args)
        self.report['manual'] = dict(axis=args.axis, scenario=args.scenario, events=[],
            baseline_seconds=5., contact_seconds=5., observe_seconds=5.,
            release_physically_verified=False, limitation=LIMITATION)
        self.input_ids = None

    def announce(self, event, message, **values):
        stamp = time.monotonic()
        item = dict(kind='manual_stage', event=event, host_time=stamp, message=message, **values)
        self.report['manual']['events'].append(item)
        print(json.dumps(item, ensure_ascii=False, allow_nan=False), flush=True)
        print(message, file=sys.stderr, flush=True)
        return stamp

    def phase(self, name):
        # execute() uses its existing success sentinel while merging STOP/guard
        # evidence. Never expose that sentinel as a manual-acceptance verdict.
        if name == 'finished':
            self.report['execution_result'] = self.report['result']
            if self.report['result'] == 'passed':
                self.report['result'] = 'analysis_pending'
        super().phase(name)

    def send_guard(self, op, **values):
        if op == 'phase' and values.get('name') == 'observation':
            values['release_cue_monotonic'] = self.report['manual']['release_cue_monotonic']
        super().send_guard(op, **values)

    def preflight(self):
        duration = super().preflight()
        config = self.report['configuration']
        if config['rate'] not in (500., 1000.) or config['required_velocity_bandwidth'] != 100.:
            raise RuntimeError('Manual test requires existing 500 or 1000 Hz control and independently verified V_BW=100')
        if not 0 < config['operator_timeout'] <= 1.:
            raise RuntimeError('Manual test requires operator timeout no greater than one second')
        return duration

    def start_guard(self, duration):
        config = dict(motor_a=self.args.motor_a, motor_b=self.args.motor_b, controller=self.args.controller,
            required_velocity_bandwidth=self.report['required_velocity_bandwidth'],
            test_kind='manual', baseline_seconds=5., contact_seconds=5., observe_seconds=5.,
            output_directory=str(self.args.output), environment=self.report['environment'],
            initial_positions=self.report['homing']['origins'], total_timeout=duration + 23.,
            active_timeout=16.5, max_speed=.8, max_torque=.7, max_temperature=45,
            feedback_timeout=.05, max_test_travel=.08, home_margin=.08, parent_timeout=.25)
        write_json(self.args.output / 'guard.json', config)
        self.guard_log = (self.args.output / 'guard.log').open('w')
        self.guard = subprocess.Popen([sys.executable, '-m', 'dm_motor_bilateral_control.auto_test_guard',
            '--config', str(self.args.output / 'guard.json')], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=self.guard_log, start_new_session=True)
        os.set_blocking(self.guard.stdout.fileno(), False)
        os.set_blocking(self.guard.stdin.fileno(), False)
        self.wait(lambda: self.guard_status and self.guard_status.get('ready'), 5)
        self.send_guard('arm')
        self.armed = True  # Parent STOP remains available if the arm acknowledgement is lost.
        self.wait(lambda: self.guard_status and self.guard_status.get('armed'), 1)

    def require_manual_normal(self):
        self.require_normal()
        state = self.follow
        if (state.angle_offset_active or state.angle_offset_ramping or
                not math.isfinite(state.angle_offset_applied) or state.angle_offset_applied != 0. or
                state.disturbance_torque != 0. or
                (state.disturbance_id, state.angle_offset_id) != self.input_ids):
            raise RuntimeError('Unexpected automatic excitation during manual test')

    def observe_phase(self, name, message):
        self.phase(name)
        self.wait(lambda: self.guard_status and self.guard_status.get('phase') == name, .5)
        started = time.monotonic()
        self.report[name] = dict(started_monotonic=started, duration=0.)
        self.announce(name, message)
        try:
            while time.monotonic() - started < 5.:
                self.pump()
                self.require_manual_normal()
        finally:
            self.report[name]['duration'] = time.monotonic() - started

    def run_coupling(self):
        self.input_ids = (self.follow.disturbance_id, self.follow.angle_offset_id)
        self.phase('coupling')
        # Bind the independent source/injection witness before ROS START can
        # produce output; pipe and ROS messages have no common arrival order.
        self.wait(lambda: self.guard_status and self.guard_status.get('phase') == 'coupling', .5)
        request = BilateralControl.Request(operation=BilateralControl.Request.START, session=self.session, valid_for=.2)
        request.header.stamp = self.get_clock().now().to_msg()
        reply = self.future(self.operator.call_async(request))
        if not reply.success:
            raise RuntimeError('Bilateral START rejected: ' + reply.message)
        self.coupling = True
        self.wait(lambda: self.follow.phase == BilateralState.ACTIVE and self.follow.operator_guarded and
            self.follow.quality == BilateralState.NORMAL and self.follow.recovery_scale == 1, 3)
        self.require_manual_normal()
        # The independent subscriber may observe ACTIVE after this parent.
        # Keep pumping/heartbeating until its own fresh readiness is confirmed.
        self.wait(lambda: self.guard_status and
            self.guard_status.get('manual_ready_for_baseline') is True, .5)
        self.require_manual_normal()
        self.report['manual']['origins'] = {
            self.args.motor_a: self.follow.origin_a, self.args.motor_b: self.follow.origin_b}
        self.observe_phase('baseline', '双边已启动：保持双手离开，静置观察 5 秒。')
        gesture = '轻推产生小幅运动' if self.args.scenario == 'touch' else '缓慢往返拖动'
        self.observe_phase('manual_contact', f'现在可对 {self.args.axis.upper()} 轴{gesture}，人工窗口 5 秒；等待松手提示。')
        cue = self.announce('release_cue', '\a现在松手：双手离开，接下来 5 秒不要再次触碰。此提示不是实际松手的测量。',
                            release_physically_verified=False)
        self.report['manual']['release_cue_monotonic'] = cue
        self.observe_phase('observation', '计划松手后的 5 秒观察开始；程序只记录响应，实际松手情况待操作者事后声明。')
        self.report['manual']['guard_timing'] = self.guard_status.get('manual_timing', {})
        self.finish_observation()


def write_manual_report(directory, report):
    write_json(directory / 'report.json', report)
    write_json(directory / 'manual-analysis.json', report['manual_analysis'])
    analysis = report['manual_analysis']
    lines = ['# 双边人工交互记录', '',
        f'原始结果：**{report["result"]}**；自动响应检查：{analysis["result"]}。',
        f'双轴新鲜失能确认：{report["stop_confirmed"]}；配置恢复：{report.get("configuration_restored", False)}。', '',
        f'指定轴：{report["manual"]["axis"].upper()}；场景：{report["manual"]["scenario"]}。',
        '流程：物理回到当前坐标 0 → 双边 START → 静置 5 秒 → 人工窗口 5 秒 → 松手提示后观察 5 秒 → 双停。', '',
        '没有自动力矩脉冲或角差注入。回零不修改编码器零点或 Flash。',
        '松手提示仅为计划时刻，无外力传感器证明实际松手。人工段未要求即时输入。',
        '待确认不等于人工验收通过；离线声明记录操作已完成、按提示松手、观察期间未再次触碰、是否感到振动及阻力是否可接受。',
        '接受本次验收还要求未感到振动且阻力可接受；这不宣称零阻力或零惯量，备注不替代五项明确声明。',
        '原始报告保持不变，最终人工验收另见 qualification.json。', '']
    config = report.get('configuration', {})
    lines += [f'控制模式：{config.get("control_mode", "unknown")}；原生 MIT 增益：{report.get("native_gains", {})}。',
        f'独立带宽证据：{report.get("velocity_bandwidth", {})}。', '',
        '| 指标 | A | B |', '|---|---:|---:|']
    for label, section, field in (
            ('人工段位置范围 / rad', 'contact', 'position_span'),
            ('释放尾 1 秒速度 RMS / rad/s', 'release', 'tail_velocity_rms'),
            ('释放尾 1 秒位置范围 / rad', 'release', 'tail_position_span'),
            ('释放段完整逆向行程数', 'release', 'complete_return_excursions')):
        values = [analysis.get('axes', {}).get(a, {}).get(section, {}).get(field, '无有效数据') for a in ('a', 'b')]
        lines.append(f'| {label} | {values[0]} | {values[1]} |')
    lines += ['', '同步误差、接触及释放段全部频谱窗口保留于 manual-analysis.json；图中保留原始采样。', '']
    for reason in analysis.get('failures', []) + analysis.get('insufficient_motion', []):
        lines.append('- ' + reason)
    if report.get('error'):
        lines += ['', '执行记录：' + report['error']]
    (directory / 'report.md').write_text('\n'.join(lines) + '\n')


def parse_arguments(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == 'confirm':
        parser = argparse.ArgumentParser(description='Offline operator declaration; original evidence is never rewritten')
        parser.add_argument('directory', type=Path)
        for flag in ('performed', 'released-on-cue', 'no-recontact', 'felt-vibration', 'resistance-acceptable'):
            parser.add_argument('--' + flag, required=True, choices=('yes', 'no'))
        parser.add_argument('--operator', required=True)
        parser.add_argument('--notes', required=True)
        args = parser.parse_args(argv[1:])
        if not args.operator.strip() or not args.notes.strip():
            parser.error('Operator identity and subjective notes must be nonempty')
        args.command = 'confirm'
        return args
    parser = argparse.ArgumentParser(description=__doc__)
    env = parser.add_mutually_exclusive_group(required=True)
    env.add_argument('--hardware', action='store_true')
    env.add_argument('--simulation', action='store_true')
    parser.add_argument('--controller', default='/bilateral_control')
    parser.add_argument('--motor-a', default='/motor_1')
    parser.add_argument('--motor-b', default='/motor_2')
    parser.add_argument('--axis', choices=('a', 'b'), required=True)
    parser.add_argument('--scenario', choices=('touch', 'drag'), required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    for key in ('controller', 'motor_a', 'motor_b'):
        setattr(args, key, namespace(getattr(args, key)))
        if getattr(args, key) == '/':
            parser.error('Nonempty ROS namespaces required')
    if args.motor_a == args.motor_b:
        parser.error('Motor namespaces must differ')
    args.output = (args.output or Path.home() / '.ros/dm_motor_bilateral_control/manual-tests' /
        (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8])).resolve()
    if args.output.exists() and (not args.output.is_dir() or any(args.output.iterdir())):
        parser.error('Output directory must be empty; prior experiments are never overwritten')
    args.command, args.test_kind = 'run', 'manual'
    args.offset_radians = args.transition_time = args.torque = args.pulse_duration = 0.
    args.observe_seconds = 5.
    return args


def main(argv=None):
    args = parse_arguments(argv)
    if args.command == 'confirm':
        try:
            result = confirm(args.directory, performed=args.performed == 'yes',
                released_on_cue=args.released_on_cue == 'yes', no_recontact=args.no_recontact == 'yes',
                felt_vibration=args.felt_vibration == 'yes', resistance_acceptable=args.resistance_acceptable == 'yes',
                operator=args.operator, notes=args.notes)
        except (OSError, ValueError, KeyError) as error:
            print(str(error), file=sys.stderr)
            return 2
        print(json.dumps(result, ensure_ascii=False), flush=True)
        return 0 if result['result'] == 'accepted' else 2
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = ManualTest(args)
    def interrupt(_signum, _frame):
        node.interrupted = True
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        report = node.execute()
        try:
            resolutions = recorded_velocity_resolutions(args.output)
        except (OSError, ValueError, TypeError) as error:
            report['execution_result'] = 'failed'
            report.setdefault('error', 'Raw state metadata unavailable: ' + str(error))
            resolutions = {}
        summarize_manual(args.output, report, resolutions)
        try:
            plot_svg(args.output, read_rows(args.output), phases=('coupling', *('baseline', 'manual_contact', 'observation')))
            plot = args.output / 'motion.svg'
            if plot.exists():
                plot.write_text(plot.read_text().replace('Bilateral automatic test: raw feedback after return to zero',
                    'Manual interaction: raw feedback; release cue is not measured release'))
        except (OSError, ValueError, KeyError) as error:
            report['result'] = report['automatic_result'] = 'failed'
            report['manual_analysis']['result'] = report['manual_analysis']['automatic_result'] = 'failed'
            report['manual_analysis']['failures'].append('Raw plot unavailable: ' + str(error))
        write_manual_report(args.output, report)
        write_json(args.output / 'progress.json', report)
        write_manifest(args.output)
        print(json.dumps(dict(result=report['result'], report=str(args.output / 'report.json'),
            stop_confirmed=report['stop_confirmed'], accepted=False), ensure_ascii=False), flush=True)
        return 0 if report['result'] == 'pending_operator_confirmation' else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
