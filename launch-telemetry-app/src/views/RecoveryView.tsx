import { RecoveryMap } from "../components/telemetry/RecoveryMap";
import { MetricInline } from "../components/telemetry/MetricInline";
import type { RecoveryModel, TelemetryPacket, ThemeMode } from "../types";
import { bearingName, metersToMiles } from "../utils/format";

type RecoveryViewProps = {
  packet: TelemetryPacket;
  recovery: RecoveryModel;
  theme: ThemeMode;
};

export function RecoveryView({ packet, recovery, theme }: RecoveryViewProps) {
  const coordinates = `${packet.gpsLat.toFixed(6)}, ${packet.gpsLon.toFixed(6)}`;
  const launchCoordinates = `${recovery.launchPoint.lat.toFixed(6)}, ${recovery.launchPoint.lon.toFixed(6)}`;

  return (
    <section className="recovery-grid">
      <div className="recovery-summary">
        <span className="eyebrow">Last Known Rocket Location</span>
        <strong>{coordinates}</strong>
        <div className="summary-grid">
          <MetricInline label="GPS age" value={`${(packet.packetAgeMs / 1000).toFixed(1)} s`} />
          <MetricInline label="Fix" value={packet.gpsValid ? `${packet.gpsSats} sats` : "invalid"} />
          <MetricInline label="Launch T-0" value={launchCoordinates} />
          <MetricInline label="Distance" value={`${metersToMiles(recovery.distanceM).toFixed(2)} mi`} />
          <MetricInline label="Bearing" value={`${recovery.bearingDeg.toFixed(0)} deg ${bearingName(recovery.bearingDeg)}`} />
        </div>
        <div className="button-row">
          <button onClick={() => void navigator.clipboard?.writeText(coordinates)}>Copy Coordinates</button>
          <button onClick={() => void navigator.clipboard?.writeText(launchCoordinates)}>Copy Launch Point</button>
        </div>
      </div>

      <RecoveryMap packet={packet} recovery={recovery} theme={theme} />
    </section>
  );
}
