"""Known-current command to reported-current response, never a sensor-only ID.

Quantization mirrors the existing DHI integer conversion. The fit includes
transport, current-loop and measurement dynamics and must not be written into
the position-acquisition sensor-delay settings as if those were separated.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from .automatic_calibration import content_hash, file_hash, harmonic, write_new
from .rest_response import extract


def response(parameters, frequency):
    gain, tau, delay=parameters
    w=2*np.pi*np.asarray(frequency)
    return gain*np.exp(-1j*w*delay)/(1+1j*w*tau)


def fit_response(rows):
    if len({r['frequency_hz'] for r in rows})<3:
        raise ValueError('Three frequencies required to compare gain, lag and delay')
    f=np.array([r['frequency_hz'] for r in rows])
    y=np.array([complex(r['real'],r['imag']) for r in rows])
    if not np.isfinite(y).all() or not np.isfinite(f).all() or min(f)<=0:
        raise ValueError('Invalid frequency response')
    def residual(z):
        e=response(z,f)-y
        return np.r_[e.real,e.imag]
    trials=[least_squares(residual,[1.,.02,d],bounds=([.1,0,0],[2.,.2,.1]),
                         xtol=1e-12,ftol=1e-12,gtol=1e-12)
            for d in (0.,.01,.04,.08)]
    best=min(trials,key=lambda a:np.sum(a.fun*a.fun))
    if not best.success:raise ValueError('Response fit failed')
    return dict(gain=float(best.x[0]),first_order_tau_s=float(best.x[1]),
                delay_s=float(best.x[2]),complex_rmse=float(np.sqrt(np.mean(abs(response(best.x,f)-y)**2))))


def shared_model(profiles, amplitude_relative_tolerance=.10, phase_tolerance_rad=.10):
    """Compare measured J2/J6 before sharing their reporting-response model.

    The tolerances are an engineering similarity test, not a robot safety or
    stability criterion. No unmeasured joint is relabelled as measured.
    """
    if (len(profiles)!=2 or {p['joint'] for p in profiles}!={2,6} or
            any(p['source']!='hardware' for p in profiles) or
            profiles[0]['identity']!=profiles[1]['identity']):
        raise ValueError('Matching physical J2 and J6 response records required')
    if any(p['held_repeat_complex_rmse']>.05 for p in profiles):
        raise ValueError('Response repetition does not support a shared fit')
    frequencies=sorted(set.intersection(*[{r['frequency_hz'] for r in p['rows']} for p in profiles]))
    if len(frequencies)<3:raise ValueError('Three common measured frequencies required')
    checks=[]; pooled=[]
    for f in frequencies:
        values=[]
        for p in profiles:
            h=np.mean([complex(r['real'],r['imag']) for r in p['rows'] if r['frequency_hz']==f])
            values.append(h)
            pooled.append(dict(frequency_hz=f,real=float(h.real),imag=float(h.imag)))
        a,b=values
        gain_difference=abs(abs(a)-abs(b))/((abs(a)+abs(b))/2)
        phase_difference=abs(float(np.angle(a/b)))
        checks.append(dict(frequency_hz=f,relative_amplitude_difference=float(gain_difference),
            phase_difference_rad=phase_difference,
            passed=bool(gain_difference<=amplitude_relative_tolerance and phase_difference<=phase_tolerance_rad)))
    passed=all(c['passed'] for c in checks)
    result=dict(schema='shared_combined_current_response_v1',identity=profiles[0]['identity'],
        similarity_passed=passed,comparison=checks,
        similarity_policy=dict(relative_amplitude_tolerance=amplitude_relative_tolerance,
                               phase_tolerance_rad=phase_tolerance_rad),
        evidence_sha256={f"joint{p['joint']}":content_hash(p) for p in profiles},
        measured_frequency_range_hz=[frequencies[0],frequencies[-1]],
        scope='goal_current_to_reported_present_current',
        sensor_timing_verified=False,output_torque_response_measured=False,
        can_deconvolve_position_control_current=False,hardware_deployable=False)
    if passed:
        result.update(common_parameters=fit_response(pooled),
            joints={f'joint{j}':dict(response_model='common_parameters',
                evidence='direct_measurement' if j in (2,6) else 'shared_unmeasured_assumption')
                for j in range(1,7)})
    return result


def shared_timing_model(profiles):
    """Share lag/delay only; preserve observed gain differences explicitly."""
    full=shared_model(profiles)
    models=[p['fitted_on_repeat_zero'] for p in profiles]
    taus=[m['first_order_tau_s'] for m in models]
    delays=[m['delay_s'] for m in models]
    tau_difference=abs(taus[0]-taus[1])/max(np.mean(taus),1e-12)
    delay_difference=abs(delays[0]-delays[1])
    normalized=[];checks=[]
    for f in sorted({r['frequency_hz'] for r in profiles[0]['rows']} &
                    {r['frequency_hz'] for r in profiles[1]['rows']}):
        values=[]
        for p,m in zip(profiles,models):
            h=np.mean([complex(r['real'],r['imag']) for r in p['rows'] if r['frequency_hz']==f])/m['gain']
            values.append(h);normalized.append(dict(frequency_hz=f,real=float(h.real),imag=float(h.imag)))
        a,b=values
        difference=abs(abs(a)-abs(b))/((abs(a)+abs(b))/2)
        phase=abs(float(np.angle(a/b)))
        checks.append(dict(frequency_hz=f,normalized_amplitude_difference=float(difference),
            phase_difference_rad=phase,passed=bool(difference<=.10 and phase<=.10)))
    passed=bool(tau_difference<=.10 and delay_difference<=.0025 and all(c['passed'] for c in checks))
    result=dict(full,schema='shared_current_timing_v1',full_response_similarity_passed=full['similarity_passed'],
        timing_similarity_passed=passed,timing_checks=checks,
        timing_policy=dict(relative_tau_tolerance=.10,absolute_delay_tolerance_s=.0025,
                           normalized_amplitude_tolerance=.10,phase_tolerance_rad=.10),
        observed_tau_relative_difference=float(tau_difference),observed_delay_difference_s=float(delay_difference),
        observed_gain_by_joint={f"joint{p['joint']}":m['gain'] for p,m in zip(profiles,models)},
        gains_shared=False,torque_constants_changed=False)
    # Do not inherit full-response parameters, even when both comparisons pass.
    result.pop('common_parameters',None);result.pop('joints',None)
    if passed:
        fitted=fit_response(normalized)
        result['common_parameters']={k:fitted[k] for k in ('first_order_tau_s','delay_s')}
        result['joints']={f'joint{j}':dict(timing_model='common_parameters',
            evidence='direct_measurement' if j in (2,6) else 'shared_unmeasured_assumption') for j in range(1,7)}
    return result


def load_shared_timing(path, identity):
    import yaml
    p=yaml.safe_load(Path(path).read_text())
    return validate_shared_timing(p,identity)


def validate_shared_timing(p,identity):
    if (p.get('schema')!='shared_current_timing_v1' or p.get('identity')!=identity or
            p.get('timing_similarity_passed') is not True or p.get('gains_shared') is not False or
            p.get('scope')!='goal_current_to_reported_present_current' or
            p.get('sensor_timing_verified') is not False or
            p.get('can_deconvolve_position_control_current') is not False or
            set(p.get('joints',{}))!={f'joint{j}' for j in range(1,7)}):
        raise ValueError('Shared reporting timing identity/scope mismatch')
    parameters=p['common_parameters']
    if (set(parameters)!={'first_order_tau_s','delay_s'} or
            not np.isfinite(list(parameters.values())).all() or
            not 0<=parameters['first_order_tau_s']<=.2 or not 0<=parameters['delay_s']<=.1):
        raise ValueError('Invalid reporting timing parameters')
    return p


def reporting_prediction(profile, time_s, command, joint):
    """Unit-gain reporting-response sensitivity, not delivered torque prediction."""
    if joint not in range(1,7) or f'joint{joint}' not in profile['joints']:
        raise ValueError('Joint has no shared reporting timing')
    t=np.asarray(time_s,float);u=np.asarray(command,float)
    if (t.ndim!=1 or len(t)<2 or u.shape!=t.shape or not np.isfinite(t).all() or
            not np.isfinite(u).all() or np.any(np.diff(t)<=0)):
        raise ValueError('Finite increasing sampled command required')
    tau=profile['common_parameters']['first_order_tau_s'];delay=profile['common_parameters']['delay_s']
    filtered=np.empty_like(u);filtered[0]=u[0]
    for k in range(1,len(t)):
        a=np.exp(-(t[k]-t[k-1])/tau) if tau>0 else 0.
        filtered[k]=a*filtered[k-1]+(1-a)*u[k]
    return np.interp(t-delay,t,filtered)


def review(folder, output, quantum_nm):
    if not np.isfinite(quantum_nm) or quantum_nm<=0:
        raise ValueError('Positive existing driver current conversion required')
    traces, rest=extract(folder,allow_neutral_response=True)
    rows=[]
    for trace in traces:
        t=np.asarray(trace['t']);command=np.asarray(trace['command_nm'])
        # C++ static_cast of signed current truncates toward zero; no change
        # to the driver or torque constant is made by this offline calculation.
        quantized=np.trunc(command/quantum_nm)*quantum_nm
        mask=(t>=max(1.,trace['ramp_s'])) & (t<=t[-1]-trace['ramp_s'])
        x=harmonic(t[mask]-t[mask][0],quantized[mask],trace['frequency_hz'])
        y=harmonic(t[mask]-t[mask][0],np.asarray(trace['current_proxy_nm'])[mask],trace['frequency_hz'])
        if x['amplitude']<=0:raise ValueError('No quantized excitation')
        h=y['amplitude']/x['amplitude']*np.exp(1j*(y['phase_rad']-x['phase_rad']))
        rows.append(dict(frequency_hz=trace['frequency_hz'],amplitude_nm=trace['amplitude_nm'],
            repeat=trace['repeat'],real=float(h.real),imag=float(h.imag),
            amplitude_ratio=float(abs(h)),phase_rad=float(np.angle(h)),
            quantized_command_amplitude_nm=x['amplitude'],reported_current_residual_rms_nm=y['residual_rms']))
    training=[r for r in rows if r['repeat']==0]
    held=[r for r in rows if r['repeat']!=0]
    if not held:raise ValueError('Independent repeated frequency blocks required')
    model=fit_response(training)
    params=[model[k] for k in ('gain','first_order_tau_s','delay_s')]
    e=response(params,[r['frequency_hz'] for r in held])-np.array([complex(r['real'],r['imag']) for r in held])
    per_amplitude=[]
    for amplitude in sorted({r['amplitude_nm'] for r in rows}):
        selected=[r for r in rows if r['amplitude_nm']==amplitude]
        per_amplitude.append(dict(amplitude_nm=amplitude,**fit_response(selected)))
    result=dict(schema='combined_current_response_v1',joint=traces[0]['joint'],
        source=traces[0]['source'],identity=traces[0]['identity'],rows=rows,
        fitted_on_repeat_zero=model,held_repeat_complex_rmse=float(np.sqrt(np.mean(abs(e)**2))),
        per_amplitude=per_amplitude,command_quantum_nm=quantum_nm,
        source_files_sha256=rest['source_files_sha256'],analysis_sha256=file_hash(__file__),
        combined_command_transport_loop_and_reporting_response=True,
        sensor_timing_verified=False,output_torque_response_measured=False,
        can_deconvolve_position_control_current=False,hardware_deployable=False)
    write_new(Path(output),result)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument('--run');source.add_argument('--profiles',nargs=2)
    p.add_argument('--output',required=True);p.add_argument('--quantum-nm',type=float)
    a=p.parse_args()
    if a.profiles:
        r=shared_timing_model([json.loads(Path(path).read_text()) for path in a.profiles])
        write_new(Path(a.output),r)
        print(json.dumps({k:r.get(k) for k in ('timing_similarity_passed','full_response_similarity_passed','common_parameters','gains_shared')},indent=2))
        if not r['timing_similarity_passed']:raise SystemExit(2)
    else:
        if a.quantum_nm is None:p.error('--run requires the existing --quantum-nm conversion')
        r=review(a.run,a.output,a.quantum_nm)
        print(json.dumps({k:r[k] for k in ('joint','fitted_on_repeat_zero','held_repeat_complex_rmse','per_amplitude','sensor_timing_verified')},indent=2))


if __name__=='__main__':main()
