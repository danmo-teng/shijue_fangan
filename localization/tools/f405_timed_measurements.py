"""Versioned feedback -> actual measurements with timing diagnostics.
No I/O and no target/T265 velocity. An observed clock exchange is NOT a drift
or sensor-delay calibration. Unknown bounds stay unknown, not an adoption gate.
"""
from collections import deque
import math
from f405_timed_protocol import offset_interval_ns

UNKNOWN=0xffffffff


class TimedMeasurements:
    def __init__(self):
        self.identity=None
        self.syncs=deque(maxlen=16)
        self.last_wheel=None
        self.last_rate=None
        self.stats=dict(wheel_samples=0,gyro_samples=0,duplicate_wheel=0,duplicate_rate=0)

    def set_identity(self, sample):
        if self.identity and self.identity["boot_epoch"]!=sample["boot_epoch"]:
            self.syncs.clear();self.last_wheel=None;self.last_rate=None
        self.identity=dict(sample)

    def sync(self, sample):
        if not self.identity or sample["boot_epoch"]!=self.identity["boot_epoch"] or sample.get("epoch_valid")!=1:
            return False
        try:
            lo,hi=offset_interval_ns(sample["host_write_before_ns"],sample["receive_monotonic_ns"],sample)
        except (ValueError,KeyError):return False
        self.syncs.append(dict(lower=lo,upper=hi,host_ns=sample["receive_monotonic_ns"],
                               nonce=sample["nonce"],boot_epoch=sample["boot_epoch"]))
        return True

    def clock_status(self, now):
        missing=[]
        if not self.identity or not self.identity.get("epoch_valid"):missing.append("boot_epoch_invalid")
        if self.identity and self.identity.get("interface_compatible",True) is not True:missing.append("device_interface_not_reviewed_or_mismatched")
        if not self.syncs:missing.append("no_matched_clock_exchange")
        if self.identity and self.identity.get("drift_bound_ppm",UNKNOWN)==UNKNOWN:missing.append("mcu_clock_drift_bound_unknown")
        if self.syncs and now-self.syncs[-1]["host_ns"]>2_000_000_000:missing.append("clock_exchange_stale")
        return dict(qualified=not missing,missing=missing,exchanges=len(self.syncs),
                    last_exchange_host_ns=self.syncs[-1]["host_ns"] if self.syncs else None)

    def map_time(self, tick_us, now):
        status=self.clock_status(now)
        if not self.syncs:return None,None,status["missing"]
        s=self.syncs[-1]
        midpoint=(s["lower"]+s["upper"])//2
        estimate=int(tick_us)*1000-midpoint
        if not status["qualified"]:return estimate,None,status["missing"]
        drift=self.identity["drift_bound_ppm"]
        error=math.ceil((s["upper"]-s["lower"])/2+abs(estimate-s["host_ns"])*drift/1e6)
        return estimate,error,[]

    def _base(self, sample, now):
        same=bool(self.identity and sample["boot_epoch"]==self.identity["boot_epoch"] and self.identity.get("epoch_valid")
                  and self.identity.get("interface_compatible",True) is True)
        return dict(mcu_epoch=sample["boot_epoch"] if same else None,reference_epoch=sample["reference_epoch"],
            host_action_id_at_receive=sample.get("host_action_id_at_receive"),host_chassis_session_id=sample.get("host_chassis_session_id"),
            receive_monotonic_ns=sample["receive_monotonic_ns"],capture_time_qualified=False,
            time_uncertainty_ns=None,raw_mcu_sample=sample,source_time_semantics="MCU event; not automatically physical acquisition",
            clock_status=self.clock_status(now))

    def wheel(self, sample, now):
        result=self._base(sample,now);flags=sample["flags"]
        identity=(sample["boot_epoch"],sample["measurement_id"])
        if identity==self.last_wheel:
            self.stats["duplicate_wheel"]+=1;return None
        if self.last_wheel and self.last_wheel[0]==identity[0]:
            step=(identity[1]-self.last_wheel[1])&UNKNOWN
            if not 0<step<0x80000000:return dict(result,measurement_valid=False,qualification_reasons=["out_of_order_wheel"])
        self.last_wheel=identity;self.stats["wheel_samples"]+=1
        start,error0,reasons0=self.map_time(sample["t0_us"],now)
        end,error1,reasons1=self.map_time(sample["t1_us"],now)
        reasons=list(dict.fromkeys(reasons0+reasons1))
        source_error=sample["source_uncertainty_us"]
        if source_error==UNKNOWN or not flags&2:reasons.append("wheel_capture_time_bound_unknown_including_rear_EXTI")
        if not result["mcu_epoch"]:reasons.append("boot_epoch_invalid")
        dt=(sample["t1_us"]-sample["t0_us"])/1e6
        valid=bool(flags&1) and not flags&(4|8|32|256|512|1024) and dt>0
        k=math.pi*.070/1061.268
        calculated=None
        if dt>0:
            calculated=[k*(sample["delta1"]-sample["delta2"])/(math.sqrt(3)*dt),
                        k*(sample["delta1"]+sample["delta2"]-2*sample["delta3"])/(3*dt),0.]
        measured=[sample["forward_um_s"]/1e6,sample["left_um_s"]/1e6,0.]
        if calculated and max(abs(a-b) for a,b in zip(measured,calculated))>.00001:
            valid=False;reasons.append("lower_body_velocity_disagrees_with_F405_raw_deltas")
        uncertainty=None if reasons else max(error0,error1)+source_error*1000
        result.update(source="measured_wheel_feedback",sample_id=sample["measurement_id"],
            v_body_mps=measured,host_recomputed_v_body_mps=calculated,
            alignment_clock="mcu_event",alignment_start_ns=sample["t0_us"]*1000,
            alignment_end_ns=sample["t1_us"]*1000,
            interval_start_ns=start,interval_end_ns=end,measurement_valid=valid,
            capture_time_qualified=bool(valid and not reasons and uncertainty<=5_000_000),
            time_uncertainty_ns=uncertainty,qualification_reasons=reasons,
            slip_state="unknown" if flags&128 else "not_certified_by_this_interface")
        return result

    def gyro(self, sample, now):
        result=self._base(sample,now);flags=sample["rate_flags"]
        identity=(sample["boot_epoch"],sample["rate_source_id"])
        if identity==self.last_rate:
            self.stats["duplicate_rate"]+=1;return None
        if self.last_rate and self.last_rate[0]==identity[0] and not 0<((identity[1]-self.last_rate[1])&UNKNOWN)<0x80000000:
            return dict(result,wire_valid=False,qualification_reasons=["out_of_order_rate"])
        self.last_rate=identity;self.stats["gyro_samples"]+=1
        capture,error,reasons=self.map_time(sample["rate_receive_us"],now)
        reasons=list(reasons);source_error=sample["rate_uncertainty_us"]
        if source_error==UNKNOWN or flags&512:reasons.append("IMU_acquisition_filter_link_to_ISR_delay_unbounded")
        if not result["mcu_epoch"]:reasons.append("boot_epoch_invalid")
        valid=(flags&3)==3 and not flags&(4|8|32|128|1024)
        uncertainty=None if reasons else error+source_error*1000
        result.update(wire_valid=valid,yaw_rate_deg_s=sample["rate_mdps"]/1000.,
            alignment_clock="mcu_event",alignment_capture_ns=sample["rate_receive_us"]*1000,
            sample_id=sample["rate_source_id"],angular_rate_capture_ns=capture,
            capture_time_qualified=bool(valid and not reasons and uncertainty<=5_000_000),
            time_uncertainty_ns=uncertainty,qualification_reasons=reasons)
        return result
