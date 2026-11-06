"""Pure trajectory math parity, without ROS, a sender process or hardware."""
import importlib.util
import subprocess
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('release_reference_trial', HERE / 'run_trial.py')
trial = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trial)


@pytest.fixture(scope='module')
def native_math(tmp_path_factory):
    directory = tmp_path_factory.mktemp('release_native_math')
    source = (HERE / 'native_stream/stream.cpp').read_text()
    body = source[source.index('struct Reference'):source.index('int main')]
    path = directory / 'math.cpp'
    path.write_text('#include <cmath>\n#include <algorithm>\n#include <string>\n#include <array>\n#include <vector>\n'
                    '#include <iostream>\n#include <iomanip>\n' + body +
                    '\nint main(int argc,char**argv){auto r=reference('
                    'std::stod(argv[1]),std::stod(argv[2]),std::stod(argv[3]),argv[4]);'
                    'std::cout<<std::setprecision(17)<<r.q<<" "<<r.v<<" "<<r.phase;}\n')
    binary = directory / 'math'
    subprocess.run(['c++', '-std=c++17', '-O2', str(path), '-o', str(binary)], check=True)
    return binary


@pytest.mark.parametrize('protocol', ['sweep', 'release'])
def test_python_native_reference_parity_and_release_boundary(native_math, protocol):
    for speed in [.6, 3., 9., 13.2]:
        geometry = trial.geometry(speed, protocol)
        ramp = max(.25, speed / 15.)
        assert geometry['ramp'] == ramp
        points = [0., .3, .3 + ramp / 2, .3 + ramp,
                  geometry['duration'] - .001, geometry['duration']]
        if protocol == 'release':
            release = .3 + ramp + .6
            assert geometry['release_time'] == release
            assert geometry['distance'] == pytest.approx(speed * (ramp / 2 + .6))
            points += [release - 1e-6, release, release + 5.]
        for time in points:
            q, v, phase = trial.reference(time, -10.5, speed, protocol)
            actual = subprocess.check_output(
                [str(native_math), str(time), '-10.5', str(speed), protocol], text=True).split()
            assert q == pytest.approx(float(actual[0]), abs=1e-12)
            assert v == pytest.approx(float(actual[1]), abs=1e-12)
            # Sweep's Python plan labels whole legs; the sender labels ramp/
            # steady/brake within each leg. Its numeric reference is identical.
            if protocol == 'release':
                assert phase == actual[2]
            if protocol == 'release' and time >= geometry['release_time']:
                assert (q, v, phase) == (0., 0., 'release_neutral')
