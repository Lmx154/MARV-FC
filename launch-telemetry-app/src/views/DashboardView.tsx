import { Dial } from "../components/telemetry/Dial";
import { MetricCard } from "../components/telemetry/MetricCard";
import { stages } from "../constants/telemetry";
import type { TelemetryPacket } from "../types";
import { formatFeet, formatMeters, formatSpeed, formatTime, stageLabel, stageShortLabel } from "../utils/format";

export function DashboardView({ packet }: { packet: TelemetryPacket }) {
  const stageIndex = stages.indexOf(packet.stage);

  return (
    <section className="dashboard-grid">
      <div className="hero-panel">
        <div className="hero-copy">
          <span className="eyebrow">Live Flight State</span>
          <strong>{stageLabel(packet.stage)}</strong>
          <span>T+ {formatTime(packet.missionTimeMs)} / seq {packet.seq}</span>
        </div>
        <div className="rocket-visual" aria-hidden="true">
          <div className="altitude-ladder">
            <i style={{ height: `${Math.min(96, packet.altitudeM / 80)}%` }} />
          </div>
          <div className={`rocket-body ${packet.stage === "BOOST" ? "boosting" : ""}`}>
            <span />
          </div>
          <div className="sparkline">
            <b style={{ width: `${Math.min(100, packet.missionTimeMs / 950)}%` }} />
          </div>
        </div>
      </div>

      <MetricCard label="Altitude AGL" value={formatFeet(packet.altitudeM)} unit="ft" trend={`${formatMeters(packet.altitudeM)} m`} />
      <MetricCard label="Max Altitude" value={formatFeet(packet.maxAltitudeM)} unit="ft" trend={`${formatMeters(packet.maxAltitudeM)} m`} />
      <MetricCard label="Vertical Speed" value={formatSpeed(packet.verticalSpeedMps)} unit="ft/s" trend={packet.verticalSpeedMps >= 0 ? "climbing" : "descending"} />
      <MetricCard label="Acceleration" value={packet.accelG.toFixed(1)} unit="g" trend="IMU magnitude" />
      <MetricCard label="Battery" value={packet.batteryV.toFixed(2)} unit="V" trend={packet.batteryV > 7.2 ? "nominal" : "watch"} />
      <MetricCard label="GPS" value={`${packet.gpsSats}`} unit="sats" trend={packet.gpsValid ? "valid fix" : "invalid fix"} />

      <div className="dial-row">
        <Dial label="RSSI" value={Math.max(0, Math.min(100, 100 + packet.rssiDbm))} readout={`${packet.rssiDbm.toFixed(0)} dBm`} />
        <Dial label="SNR" value={Math.max(0, Math.min(100, packet.snrDb * 8))} readout={`${packet.snrDb.toFixed(1)} dB`} />
        <Dial label="Packets" value={Math.min(100, packet.packetRateHz * 18)} readout={`${packet.packetRateHz.toFixed(1)} Hz`} />
      </div>

      <div className="stage-rail">
        {stages.slice(0, 7).map((stage, index) => (
          <div key={stage} className={index <= stageIndex ? "passed" : ""}>
            <span>{stageShortLabel(stage)}</span>
          </div>
        ))}
      </div>
    </section>
  );
}
