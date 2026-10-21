"""Compile real VitoConnect code and actual site YAML lambdas with fake I/O.

Run: python tests/host_safety_checks.py [repository]
Requires PyYAML and g++. No secrets, ESP toolchain or physical boiler is used.

For the tested snippets, component id(name) expressions are rendered as pointers,
and immediate id(name).member syntax is rendered as pointer member access, as
in ESPHome process_lambda. Bare IDs must not be replaced with object references.
Globals remain values. This limited renderer is not ESPHome schema/codegen validation.
"""
from pathlib import Path
import os
import re
import shutil
import subprocess
import sys
import tempfile

import yaml
from host_stubs import STUBS


class Loader(yaml.SafeLoader):
    pass


Loader.add_constructor(None, lambda loader, node: loader.construct_scalar(node))


def run_checks(root: Path) -> str:
    config = yaml.load((root / 'heating_optolink.yaml').read_text(), Loader=Loader)
    subs = config['substitutions']
    def subst(text):
        return re.sub(r'\$\{([^}]+)\}', lambda m: str(subs[m.group(1)]), str(text))
    def item(key, ident):
        return next(x for x in config[key] if x.get('id') == ident)
    globals_ids = {g['id'] for g in config['globals']}
    component_ids = {
        'heating_setpoint_normal_direct', 'heating_setpoint_reduced_direct',
        'heating_setpoint_target_optimizer', 'heating_setpoint_control_armed',
        'restore_heating_setpoints_to_baseline',
    }
    def render_lambda(text):
        # Mirrors the relevant process_lambda distinction, not a C++ id() macro:
        # id(component).method -> component_ptr->method
        # id(component)        -> component_ptr
        # id(global)           -> value
        def replace(match):
            ident, dot = match.group(1), match.group(2) or ''
            if ident in globals_ids:
                return ident + dot
            if ident not in component_ids:
                raise ValueError(f'Unmodelled lambda component ID: {ident}')
            return f'{ident}_ptr' + ('->' if dot else '')
        return re.sub(r'\bid\(\s*(\w+)\s*\)(\.)?', replace, subst(text))
    def function(name, text, args='', result='void'):
        return f'{result} {name}({args}) {{\n{render_lambda(text)}\n}}\n'
    if not shutil.which('g++'):
        raise RuntimeError('g++ is required for the host regression tests')
    cpp = r'''
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <algorithm>
#include <functional>
#include <string>
#include <vector>
#include <iostream>
#define private public
#define protected public
#include "vitoconnect.h"
#include "number/vitoconnect_number.h"
#undef private
#undef protected
using namespace esphome;
using namespace esphome::vitoconnect;
using std::isnan;
uint32_t fake_millis = 1000;
OPTOLINKNumber heating_setpoint_normal_direct, heating_setpoint_reduced_direct;
struct FakeFloat { float state=22; void publish_state(float x) {state=x;} } heating_setpoint_target_optimizer;
struct FakeBool { bool state=false; void publish_state(bool x) {state=x;} } heating_setpoint_control_armed;
void restore_script();
struct RestoreScript { void execute(){restore_script();} } restore_heating_setpoints_to_baseline;
'''
    for ident in sorted(component_ids):
        cpp += f'auto * const {ident}_ptr = &{ident};\n'
    for g in config['globals']:
        cpp += f'{g["type"]} {g["id"]} = {subst(g["initial_value"])};\n'
    cpp += function('restore_script', item('script', 'restore_heating_setpoints_to_baseline')['then'][0]['lambda'])
    cpp += function('revert_script', item('script', 'revert_to_baseline')['then'][0]['lambda'])
    cpp += function('apply_script', item('script', 'apply_heating_setpoint')['then'][0]['lambda'])
    cpp += function('capture_lambda', item('button', 'set_baseline_from_current')['on_press'][0]['lambda'])
    cpp += function('arm_lambda', item('switch', 'heating_setpoint_control_armed')['turn_on_action'][0]['lambda'])
    cpp += function('heartbeat_lambda', item('button', 'optimizer_heartbeat')['on_press'][0]['lambda'])
    cpp += function('boot_lambda', config['esphome']['on_boot']['then'][0]['lambda'])
    watchdog = next(a['lambda'] for interval in config['interval'] for a in interval['then']
                    if 'lambda' in a and 'deadline_base' in a['lambda'])
    cpp += function('watchdog_lambda', watchdog)
    cpp += function('status_lambda', item('text_sensor', 'optimizer_actuator_status')['lambda'], result='std::string')
    cpp += function('target_lambda', item('number', 'heating_setpoint_target_optimizer')['set_action'][0]['lambda'], 'float x')
    # Model the documented framework call order around the actual set_action:
    # execute the full action chain, then optionally publish optimistically.
    optimistic = str(item('number', 'heating_setpoint_target_optimizer')['optimistic']).lower()
    cpp += f'''void target_control(float value) {{
  target_lambda(value);
  apply_script();
  if ({optimistic}) heating_setpoint_target_optimizer.publish_state(value);
}}
'''
    cpp += r'''
void reset(float actual=22) {
  fake_millis=1000;
  heating_setpoint_normal_direct = OPTOLINKNumber{};
  heating_setpoint_reduced_direct = OPTOLINKNumber{};
  heating_setpoint_normal_direct.setLength(1); heating_setpoint_normal_direct.setAddress(0x2306);
  heating_setpoint_reduced_direct.setLength(1); heating_setpoint_reduced_direct.setAddress(0x2307);
  uint8_t raw=(uint8_t)actual;
  heating_setpoint_normal_direct.decode(&raw,1); heating_setpoint_reduced_direct.decode(&raw,1);
  setpoint_control_armed=false; setpoint_baseline_valid=true; watchdog_tripped=false; restore_required=false;
  commissioning_required=false;
  restore_normal_sequence=0; restore_reduced_sequence=0; restore_recovery_requested=false;
  baseline_heating_setpoint_normal=22; baseline_heating_setpoint_reduced=22;
  desired_heating_setpoint_target=22; desired_offset=0;
  last_restore_attempt_ms=0; restore_recovery_requested_at_ms=0; last_heartbeat_ms=0;
  armed_at_ms=0; last_setpoint_apply_ms=0;
  heating_setpoint_target_optimizer.state=22; heating_setpoint_control_armed.state=false;
}
class FakeOptolink : public Optolink {
 public:
  FakeOptolink(uart::UARTDevice* u):Optolink(u){}
  void begin() override{} void loop() override{}
  int writes=0, reads=0; uint8_t boiler_normal=23, boiler_reduced=23;
  void drain(bool fail) {
    while (auto* front=_queue.front()) {
      auto req=*front;
      if(req.write) ++writes; else ++reads;
      if(fail) _onError(TIMEOUT,req.arg);
      else {
        uint8_t& value=req.address==0x2306 ? boiler_normal:boiler_reduced;
        if(req.write) value=req.data[0];
        _onData(&value,1,req.arg);
      }
      _queue.pop();
    }
  }
};
FakeOptolink* connect(VitoConnect& hub) {
  auto* transport=new FakeOptolink(&hub); hub._optolink=transport;
  transport->onData(&VitoConnect::_onData); transport->onError(&VitoConnect::_onError);
  hub.set_verify_writes(true); hub.set_max_write_failures(3);
  hub.register_datapoint(&heating_setpoint_normal_direct);
  hub.register_datapoint(&heating_setpoint_reduced_direct);
  return transport;
}
void tick() {
  watchdog_lambda();
  if(restore_required) restore_script();
}
void poll(VitoConnect& hub, FakeOptolink& transport, bool fail) {
  // Run the same watchdog and reconciliation intervals, including between polls.
  for(int i=0;i<12;++i){fake_millis+=5000;tick();}
  hub.update(); transport.drain(fail); tick();
}
int main() {
  reset(); setpoint_control_armed=true; last_restore_attempt_ms=500;
  heating_setpoint_normal_direct.control(23); heating_setpoint_reduced_direct.control(23);
  revert_script();
  assert(!setpoint_control_armed && restore_required);
  assert(heating_setpoint_normal_direct.getPendingCommandValue()==22);
  assert(heating_setpoint_reduced_direct.getPendingCommandValue()==22);
  std::cout << "PASS: a recent restore supersedes queued elevated writes and disarms\n";

  reset(); restore_script();
  assert(restore_required); // Matching cached values alone cannot complete restore.
  uint8_t baseline=22;
  heating_setpoint_normal_direct.decode(&baseline,1); restore_script();
  assert(restore_required); // One fresh read is insufficient.
  heating_setpoint_reduced_direct.decode(&baseline,1); restore_script();
  assert(!restore_required);
  std::cout << "PASS: restore requires new readbacks of both setpoints\n";

  reset(23); setpoint_control_armed=true; capture_lambda();
  assert(baseline_heating_setpoint_normal==22 && baseline_heating_setpoint_reduced==22);
  setpoint_control_armed=false; heating_setpoint_normal_direct.control(24); capture_lambda();
  assert(baseline_heating_setpoint_normal==22);
  reset(23); fake_millis+=181000; capture_lambda();
  assert(baseline_heating_setpoint_normal==22);
  reset(23); fake_millis+=16000; uint8_t raw=23; heating_setpoint_normal_direct.decode(&raw,1);
  capture_lambda(); assert(baseline_heating_setpoint_normal==22);
  heating_setpoint_reduced_direct.decode(&raw,1); capture_lambda();
  assert(baseline_heating_setpoint_normal==23 && baseline_heating_setpoint_reduced==23);
  std::cout << "PASS: capture refuses armed, pending, stale and incoherent state; accepts idle fresh pair\n";

  reset(); target_control(24);
  assert(heating_setpoint_target_optimizer.state==22 && desired_heating_setpoint_target==22);
  assert(!heating_setpoint_normal_direct.hasPendingCommand());
  setpoint_control_armed=true; watchdog_tripped=true; target_control(24);
  assert(heating_setpoint_target_optimizer.state==22);
  reset(); arm_lambda(); assert(setpoint_control_armed); target_control(24);
  assert(desired_heating_setpoint_target==24 && heating_setpoint_target_optimizer.state==24);
  assert(heating_setpoint_normal_direct.getPendingCommandValue()==24);
  std::cout << "PASS: the full number action acknowledges only accepted targets\n";

  const uint32_t lease_start=armed_at_ms;
  fake_millis+=5000; arm_lambda();
  assert(setpoint_control_armed && armed_at_ms==lease_start);
  assert(heating_setpoint_normal_direct.getPendingCommandValue()==24);
  reset(); heating_setpoint_normal_direct.control(24); arm_lambda();
  assert(!setpoint_control_armed && restore_required);
  assert(heating_setpoint_normal_direct.getPendingCommandValue()==22);
  std::cout << "PASS: repeated arm preserves its lease; rejected arm cannot orphan a pending write\n";

  reset(); arm_lambda(); assert(setpoint_control_armed);
  fake_millis+=181000; heartbeat_lambda(); assert(last_heartbeat_ms==0);
  watchdog_lambda(); assert(!setpoint_control_armed && watchdog_tripped && restore_required);
  arm_lambda(); assert(!setpoint_control_armed);
  std::cout << "PASS: boiler-only outage rejects heartbeats, disarms and prevents rearm\n";

  reset(23); setpoint_control_armed=true; armed_at_ms=1000; fake_millis=2000000;
  watchdog_lambda(); assert(watchdog_tripped && restore_required && !setpoint_control_armed);
  {
    VitoConnect hub; auto* transport=connect(hub);
    // Several complete failed retry budgets, not just one failed transaction.
    for(int i=0;i<12;++i) poll(hub,*transport,true);
    assert(restore_required && !setpoint_control_armed && transport->writes>=6);
    int failed_writes=transport->writes;
    for(int i=0;i<10 && restore_required;++i) poll(hub,*transport,false);
    assert(transport->writes>failed_writes);
    assert(transport->boiler_normal==22 && transport->boiler_reduced==22);
    assert(!restore_required && !watchdog_tripped && !setpoint_control_armed);
    int writes=transport->writes;
    for(int i=0;i<3;++i) poll(hub,*transport,false);
    assert(transport->writes==writes); // Successful recovery stops retrying.
  }
  std::cout << "PASS: baseline recovery resumes after multiple exhausted budgets with HA absent\n";

  reset(); setpoint_control_armed=true;
  {
    VitoConnect hub; auto* transport=connect(hub);
    transport->boiler_normal=22; transport->boiler_reduced=22;
    heating_setpoint_normal_direct.control(23); heating_setpoint_reduced_direct.control(23);
    const uint32_t old_seq=heating_setpoint_normal_direct.getLastUpdate();
    hub.update(); // The elevated requests are already in flight.
    revert_script(); // Same millisecond as the original write request.
    assert(heating_setpoint_normal_direct.getLastUpdate()!=old_seq);
    transport->drain(false); // Old ACK cannot clear the newer baseline intent.
    assert(heating_setpoint_normal_direct.getLastUpdate()!=0);
    assert(heating_setpoint_normal_direct.getPendingCommandValue()==22);
    for(int i=0;i<6 && restore_required;++i) poll(hub,*transport,false);
    assert(transport->boiler_normal==22 && transport->boiler_reduced==22 && !restore_required);
  }
  std::cout << "PASS: same-tick in-flight elevated writes are compensated by baseline writes\n";

  reset(23); restore_required=true;
  heating_setpoint_normal_direct=OPTOLINKNumber{}; heating_setpoint_reduced_direct=OPTOLINKNumber{};
  heating_setpoint_normal_direct.setLength(1); heating_setpoint_normal_direct.setAddress(0x2306);
  heating_setpoint_reduced_direct.setLength(1); heating_setpoint_reduced_direct.setAddress(0x2307);
  boot_lambda();
  assert(restore_required && !setpoint_control_armed);
  assert(heating_setpoint_normal_direct.getPendingCommandValue()==22);
  {
    VitoConnect hub; auto* transport=connect(hub);
    for(int i=0;i<6 && restore_required;++i) poll(hub,*transport,false);
    assert(!restore_required && transport->boiler_normal==22 && transport->boiler_reduced==22);
  }
  std::cout << "PASS: persisted restore obligation resumes at boot before fresh cached state exists\n";
  reset(23); boot_lambda();
  assert(restore_required && !setpoint_control_armed);
  assert(heating_setpoint_normal_direct.getPendingCommandValue()==22);
  std::cout << "PASS: boot restores a known baseline even if the last armed flag was not persisted\n";



  reset(); setpoint_baseline_valid=false;
  revert_script();
  assert(commissioning_required && !restore_required && !setpoint_control_armed);
  capture_lambda(); assert(!setpoint_baseline_valid); // pre-stop cache is not proof
  {
    VitoConnect hub; auto* transport=connect(hub);
    transport->boiler_normal=22; transport->boiler_reduced=22;
    for(int i=0;i<9;++i) poll(hub,*transport,false);
    assert(!setpoint_baseline_valid && commissioning_required && transport->writes==0);
    assert(transport->reads>=18); // Reads resume; no automatic baseline adoption.
    capture_lambda();
    assert(setpoint_baseline_valid && !commissioning_required && !restore_required);
    assert(!setpoint_control_armed && baseline_heating_setpoint_normal==22);
  }
  std::cout << "PASS: restore-before-commissioning permits fresh manual capture without automatic trust or writes\n";

  for(float invalid : {31.0f, NAN}) {
    reset(); baseline_heating_setpoint_normal=invalid;
    restore_required=true; watchdog_tripped=true; setpoint_control_armed=true;
    boot_lambda();
    assert(!setpoint_baseline_valid && commissioning_required && !restore_required && !setpoint_control_armed);
    assert(watchdog_tripped); // Only the operator can establish a replacement baseline.
    assert(!heating_setpoint_normal_direct.hasPendingCommand());
    assert(!heating_setpoint_reduced_direct.hasPendingCommand());
    capture_lambda(); assert(!setpoint_baseline_valid);
    uint8_t b=22; heating_setpoint_normal_direct.decode(&b,1); heating_setpoint_reduced_direct.decode(&b,1);
    capture_lambda();
    assert(setpoint_baseline_valid && !commissioning_required && !watchdog_tripped);
    assert(!setpoint_control_armed && baseline_heating_setpoint_normal==22);
  }
  reset(); setpoint_baseline_valid=false; restore_required=true;
  boot_lambda(); assert(commissioning_required && !restore_required);
  std::cout << "PASS: invalid persisted baselines and legacy unknown restore flags recover without storage erasure\n";

  reset(); setpoint_baseline_valid=false;
  {
    VitoConnect hub; auto* transport=connect(hub);
    transport->boiler_normal=22; transport->boiler_reduced=22;
    const auto before=heating_setpoint_normal_direct.getReadSequence();
    hub.update(); // Reads queued before stop may complete after it.
    heating_setpoint_normal_direct.control(24); heating_setpoint_reduced_direct.control(24);
    revert_script();
    assert(!heating_setpoint_normal_direct.hasPendingCommand());
    transport->drain(false);
    assert(heating_setpoint_normal_direct.getReadSequence()==before); // stale epoch rejected
    capture_lambda(); assert(!setpoint_baseline_valid);
    poll(hub,*transport,false); capture_lambda();
    assert(setpoint_baseline_valid && transport->writes==0);
  }
  std::cout << "PASS: uncommissioned stop cancels unsent commands and fences pre-stop read callbacks\n";

  for(bool fail : {false, true}) {
    reset(); setpoint_baseline_valid=false;
    VitoConnect hub; auto* transport=connect(hub);
    heating_setpoint_normal_direct.control(24); heating_setpoint_reduced_direct.control(24);
    hub.update(); // A transport request already queued/sent cannot be recalled.
    revert_script(); capture_lambda();
    assert(!setpoint_baseline_valid && heating_setpoint_normal_direct.hasActiveWriteCommand());
    transport->drain(fail); capture_lambda();
    assert(!setpoint_baseline_valid && !heating_setpoint_normal_direct.hasPendingCommand());
    transport->boiler_normal=22; transport->boiler_reduced=22; // Operator establishes intended boiler settings.
    int writes=transport->writes;
    poll(hub,*transport,false); capture_lambda();
    assert(setpoint_baseline_valid && transport->writes==writes && !setpoint_control_armed);
  }
  std::cout << "PASS: in-flight cancellation remains non-idle until completion and requires new reads before capture\n";

  reset(23); restore_script(); commissioning_required=true;
  capture_lambda(); assert(restore_required && baseline_heating_setpoint_normal==22);
  assert(setpoint_baseline_valid); // Never weaken a real known-baseline restore obligation.
  std::cout << "PASS: commissioning cannot bypass a known trusted-baseline restore obligation\n";

  reset(); fake_millis=0xFFFFFF00U;
  heating_setpoint_normal_direct.decode(&baseline,1);
  fake_millis=500;
  assert(heating_setpoint_normal_direct.readIsFresh(180000));
  fake_millis+=181000; heating_setpoint_normal_direct.expireRead(180000);
  fake_millis=0xFFFFFF01U;
  assert(!heating_setpoint_normal_direct.readIsFresh(180000));
  heating_setpoint_normal_direct.decode(&baseline,1);
  assert(heating_setpoint_normal_direct.readIsFresh(180000));
  std::cout << "PASS: read age works across millis rollover and latches long-outage expiry\n";

  reset(); fake_millis=0; heating_setpoint_normal_direct.control(23);
  assert(heating_setpoint_normal_direct.getLastUpdate()!=0);
  auto seq=heating_setpoint_normal_direct.getLastUpdate();
  heating_setpoint_normal_direct.control(NAN);
  assert(heating_setpoint_normal_direct.getLastUpdate()==seq);
  reset(); std::string status=status_lambda(); assert(status.size()<255);
  std::cout << "STATUS_FIXTURE=" << status << "\n";
  std::cout << "PASS: finite commands, tick-zero writes and bounded wire-evidence payload\n";
}
'''
    with tempfile.TemporaryDirectory(prefix='vito-safety-') as temp:
        build = Path(temp)
        for name, source in STUBS.items():
            out = build / name
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(source)
        main = build / 'checks.cpp'
        main.write_text(cpp)
        component = root / 'components/vitoconnect'
        sources = [component / name for name in (
            'vitoconnect.cpp', 'vitoconnect_datapoint.cpp', 'vitoconnect_optolink.cpp',
            'vitoconnect_optolinkDP.cpp', 'vitoconnect_optolinkP300.cpp',
            'vitoconnect_optolinkKW.cpp', 'vitoconnect_optolinkGWG.cpp', 'number/vitoconnect_number.cpp')]
        cmd = ['g++', '-std=c++17', '-O0', '-g', '-UNDEBUG', '-I', str(build), '-I', str(component),
               str(main), *map(str, sources), '-o', str(build / 'checks')]
        subprocess.run(cmd, check=True, timeout=60, env={**os.environ, "LC_ALL": "C"})
        return subprocess.run([str(build / 'checks')], check=True, text=True, capture_output=True, timeout=20).stdout


if __name__ == '__main__':
    print(run_checks(Path(sys.argv[1]).resolve() if len(sys.argv)>1 else Path(__file__).resolve().parents[1]), end='')
