import { useEffect, useMemo, useRef, useState } from "react";
import type { PointerEvent, WheelEvent } from "react";

import { CommandHistory } from "../components/ui/CommandHistory";
import { HeaderLine } from "../components/ui/HeaderLine";
import { Icon } from "../components/ui/Icon";
import { Metric } from "../components/ui/Metric";
import type { AppState, BackendCommand } from "../types";
import { findValue } from "../utils/telemetry";

const DEFAULT_FIELD_POSITION = { lat: 26.310942, lon: -98.174728, alt: 28.711437 };
const TILE_SIZE = 256;
const MIN_ZOOM = 2;
const MAX_ZOOM = 19;
const DEFAULT_ZOOM = 17;
const MAX_MERCATOR_LAT = 85.05112878;

export function MissionView({ state, command }: { state: AppState | null; command: BackendCommand }) {
  const lat = findValue(state, "Navigation", "GPS_LAT", String(DEFAULT_FIELD_POSITION.lat));
  const lon = findValue(state, "Navigation", "GPS_LON", String(DEFAULT_FIELD_POSITION.lon));
  const alt = findValue(state, "Navigation", "GPS_ALT_MSL", String(DEFAULT_FIELD_POSITION.alt));
  const vehiclePosition = useMemo(() => {
    const source = state?.hil_comparison.latest_source;
    const parsed = {
      lat: Number(source?.lat_deg ?? lat),
      lon: Number(source?.lon_deg ?? lon),
      alt: Number(source?.alt_msl_m ?? alt),
    };

    if (!isMapCoordinate(parsed.lat, parsed.lon)) {
      return DEFAULT_FIELD_POSITION;
    }

    return parsed;
  }, [alt, lat, lon, state?.hil_comparison.latest_source]);
  const latestStamp = useMemo(
    () => ({
      refSimTick: state?.gazebo_bridge.last_sensor_sequence ?? 0,
      refSimTimeUs: state?.gazebo_bridge.last_sensor_time_us ?? 0,
    }),
    [state?.gazebo_bridge.last_sensor_sequence, state?.gazebo_bridge.last_sensor_time_us],
  );
  const latestSource = state?.hil_comparison.latest_source ?? null;
  const gpsReferenceReady = Boolean(
    latestSource &&
      latestSource.fix_type >= 3 &&
      latestSource.sats >= 4 &&
      Number.isFinite(latestSource.lat_deg) &&
      Number.isFinite(latestSource.lon_deg) &&
      Number.isFinite(latestSource.alt_msl_m),
  );
  const waypointControlReady = Boolean(
    state?.uart.connected &&
      state?.hil_comparison.hil_ready &&
      state?.hil_comparison.source_active &&
      gpsReferenceReady,
  );
  const latestResponseFlags = state?.hil_comparison.latest_response?.flags ?? 0;

  const [globalWaypoint, setGlobalWaypoint] = useState({
    latDeg: vehiclePosition.lat,
    lonDeg: vehiclePosition.lon,
    altMslM: vehiclePosition.alt,
    yawDeg: 0,
  });
  const [waypointPinned, setWaypointPinned] = useState(false);
  const [cvWaypoint, setCvWaypoint] = useState({ x: 1, y: 0, z: 0, confidence: 0.8 });
  const [tofWaypoint, setTofWaypoint] = useState({ distanceM: 10, bearingDeg: 0, elevationDeg: 0 });

  useEffect(() => {
    if (waypointPinned) {
      return;
    }

    setGlobalWaypoint((waypoint) => ({
      ...waypoint,
      latDeg: vehiclePosition.lat,
      lonDeg: vehiclePosition.lon,
      altMslM: vehiclePosition.alt,
    }));
  }, [vehiclePosition, waypointPinned]);

  const selectedWaypoint = useMemo(
    () =>
      isMapCoordinate(globalWaypoint.latDeg, globalWaypoint.lonDeg)
        ? { lat: globalWaypoint.latDeg, lon: globalWaypoint.lonDeg }
        : null,
    [globalWaypoint.latDeg, globalWaypoint.lonDeg],
  );

  const selectMapWaypoint = (latDeg: number, lonDeg: number) => {
    setWaypointPinned(true);
    setGlobalWaypoint((waypoint) => ({
      ...waypoint,
      latDeg,
      lonDeg,
      altMslM: Number.isFinite(waypoint.altMslM) ? waypoint.altMslM : vehiclePosition.alt,
    }));
  };

  return (
    <section className="page map-page">
      <main className="mission-workspace">
        <MissionMap
          vehiclePosition={vehiclePosition}
          waypoint={selectedWaypoint}
          onSelectWaypoint={selectMapWaypoint}
        />
        <div className="mission-grid">
          <section className="mission-card">
            <HeaderLine icon="my_location" title="Vehicle Telemetry" />
            <Metric label="Lat" value={lat} mono />
            <Metric label="Lon" value={lon} mono />
            <Metric label="Alt MSL" value={`${alt} m`} mono />
            <Metric label="Reference Tick" value={String(latestStamp.refSimTick || "--")} mono />
            <Metric label="Reference Time" value={String(latestStamp.refSimTimeUs || "--")} mono />
            <Metric label="MARV Ready" value={state?.hil_comparison.hil_ready ? "READY" : "WAIT"} mono />
            <Metric label="HIL Source" value={state?.hil_comparison.source_active ? "LIVE" : "IDLE"} mono />
            <Metric label="GPS Ref" value={gpsReferenceReady ? `FIX ${latestSource?.fix_type} / ${latestSource?.sats} sats` : "WAIT"} mono />
            <Metric label="Response Flags" value={formatHilResponseFlags(latestResponseFlags)} mono />
            <Metric label="Protocol Fault" value={state?.hil_comparison.latest_protocol_fault ?? "none"} mono />
          </section>

          <section className="mission-card static-panel">
            <HeaderLine icon="route" title="Global Waypoint" />
            <div className="control-grid mission-form">
              <NumberInput label="Lat Deg" value={globalWaypoint.latDeg} step={0.000001} onChange={(latDeg) => {
                setWaypointPinned(true);
                setGlobalWaypoint({ ...globalWaypoint, latDeg });
              }} />
              <NumberInput label="Lon Deg" value={globalWaypoint.lonDeg} step={0.000001} onChange={(lonDeg) => {
                setWaypointPinned(true);
                setGlobalWaypoint({ ...globalWaypoint, lonDeg });
              }} />
              <NumberInput label="Alt MSL" value={globalWaypoint.altMslM} step={0.1} onChange={(altMslM) => {
                setWaypointPinned(true);
                setGlobalWaypoint({ ...globalWaypoint, altMslM });
              }} />
              <NumberInput label="Yaw Deg" value={globalWaypoint.yawDeg} onChange={(yawDeg) => setGlobalWaypoint({ ...globalWaypoint, yawDeg })} />
            </div>
            <div className="command-row">
              <button disabled={!waypointControlReady} onClick={() => command("send_hilink_control_waypoint", { ...latestStamp, ...globalWaypoint })}>
                <Icon name="near_me" />
                Control
              </button>
              <button className="primary" disabled={!waypointControlReady} onClick={() => command("send_hilink_mission_waypoint", { ...latestStamp, ...globalWaypoint })}>
                <Icon name="upload" />
                Mission
              </button>
            </div>
          </section>

          <section className="mission-card static-panel">
            <HeaderLine icon="center_focus_strong" title="CV Waypoint" />
            <div className="control-grid mission-form">
              <NumberInput label="Body X" value={cvWaypoint.x} onChange={(x) => setCvWaypoint({ ...cvWaypoint, x })} />
              <NumberInput label="Body Y" value={cvWaypoint.y} onChange={(y) => setCvWaypoint({ ...cvWaypoint, y })} />
              <NumberInput label="Body Z" value={cvWaypoint.z} onChange={(z) => setCvWaypoint({ ...cvWaypoint, z })} />
              <NumberInput label="Confidence" value={cvWaypoint.confidence} step={0.01} onChange={(confidence) => setCvWaypoint({ ...cvWaypoint, confidence })} />
            </div>
            <div className="command-row">
              <button
                className="primary"
                disabled={!state?.uart.connected}
                onClick={() =>
                  command("send_hilink_cv_waypoint", {
                    command: {
                      ref_sim_tick: latestStamp.refSimTick,
                      ref_sim_time_us: latestStamp.refSimTimeUs,
                      dir_body: [cvWaypoint.x, cvWaypoint.y, cvWaypoint.z],
                      confidence: cvWaypoint.confidence,
                    },
                  })
                }
              >
                <Icon name="upload" />
                Send CV
              </button>
            </div>
          </section>

          <section className="mission-card static-panel">
            <HeaderLine icon="settings_ethernet" title="ToF Waypoint" />
            <div className="control-grid mission-form">
              <NumberInput label="Distance M" value={tofWaypoint.distanceM} onChange={(distanceM) => setTofWaypoint({ ...tofWaypoint, distanceM })} />
              <NumberInput label="Bearing Deg" value={tofWaypoint.bearingDeg} onChange={(bearingDeg) => setTofWaypoint({ ...tofWaypoint, bearingDeg })} />
              <NumberInput label="Elevation Deg" value={tofWaypoint.elevationDeg} onChange={(elevationDeg) => setTofWaypoint({ ...tofWaypoint, elevationDeg })} />
            </div>
            <div className="command-row">
              <button
                className="primary"
                disabled={!state?.uart.connected}
                onClick={() =>
                  command("send_hilink_tof_waypoint", {
                    command: {
                      ref_sim_tick: latestStamp.refSimTick,
                      ref_sim_time_us: latestStamp.refSimTimeUs,
                      distance_m: tofWaypoint.distanceM,
                      bearing_deg: tofWaypoint.bearingDeg,
                      elevation_deg: tofWaypoint.elevationDeg,
                    },
                  })
                }
              >
                <Icon name="upload" />
                Send ToF
              </button>
            </div>
          </section>
        </div>
      </main>
      <aside className="right-rail">
        <CommandHistory state={state} />
      </aside>
    </section>
  );
}

function MissionMap({
  vehiclePosition,
  waypoint,
  onSelectWaypoint,
}: {
  vehiclePosition: { lat: number; lon: number; alt: number };
  waypoint: { lat: number; lon: number } | null;
  onSelectWaypoint: (latDeg: number, lonDeg: number) => void;
}) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const dragRef = useRef<{
    pointerId: number;
    startX: number;
    startY: number;
    centerPixel: Point;
    moved: boolean;
  } | null>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });
  const [zoom, setZoom] = useState(DEFAULT_ZOOM);
  const [center, setCenter] = useState({ lat: vehiclePosition.lat, lon: vehiclePosition.lon });
  const [userMovedMap, setUserMovedMap] = useState(false);

  useEffect(() => {
    const node = containerRef.current;
    if (!node) {
      return;
    }

    const observer = new ResizeObserver(([entry]) => {
      setSize({
        width: Math.max(0, Math.round(entry.contentRect.width)),
        height: Math.max(0, Math.round(entry.contentRect.height)),
      });
    });
    observer.observe(node);
    return () => observer.disconnect();
  }, []);

  useEffect(() => {
    if (!userMovedMap && isMapCoordinate(vehiclePosition.lat, vehiclePosition.lon)) {
      setCenter({ lat: vehiclePosition.lat, lon: vehiclePosition.lon });
    }
  }, [userMovedMap, vehiclePosition.lat, vehiclePosition.lon]);

  const centerPixel = project(center.lat, center.lon, zoom);
  const tiles = useMemo(() => visibleTiles(center, zoom, size), [center, size, zoom]);
  const vehicleScreen = toScreenPoint(vehiclePosition.lat, vehiclePosition.lon, centerPixel, zoom, size);
  const waypointScreen = waypoint ? toScreenPoint(waypoint.lat, waypoint.lon, centerPixel, zoom, size) : null;

  const zoomBy = (delta: number) => {
    setUserMovedMap(true);
    setZoom((value) => clamp(value + delta, MIN_ZOOM, MAX_ZOOM));
  };

  const recenter = () => {
    setUserMovedMap(false);
    setCenter({ lat: vehiclePosition.lat, lon: vehiclePosition.lon });
  };

  const handlePointerDown = (event: PointerEvent<HTMLDivElement>) => {
    event.currentTarget.setPointerCapture(event.pointerId);
    dragRef.current = {
      pointerId: event.pointerId,
      startX: event.clientX,
      startY: event.clientY,
      centerPixel: project(center.lat, center.lon, zoom),
      moved: false,
    };
  };

  const handlePointerMove = (event: PointerEvent<HTMLDivElement>) => {
    const drag = dragRef.current;
    if (!drag || drag.pointerId !== event.pointerId) {
      return;
    }

    const dx = event.clientX - drag.startX;
    const dy = event.clientY - drag.startY;
    if (Math.abs(dx) > 2 || Math.abs(dy) > 2) {
      drag.moved = true;
      setUserMovedMap(true);
    }

    const nextCenter = unproject(drag.centerPixel.x - dx, drag.centerPixel.y - dy, zoom);
    setCenter(nextCenter);
  };

  const handlePointerUp = (event: PointerEvent<HTMLDivElement>) => {
    const drag = dragRef.current;
    dragRef.current = null;
    if (!drag || drag.pointerId !== event.pointerId) {
      return;
    }

    if (drag.moved) {
      return;
    }

    const rect = event.currentTarget.getBoundingClientRect();
    const x = centerPixel.x + event.clientX - rect.left - size.width / 2;
    const y = centerPixel.y + event.clientY - rect.top - size.height / 2;
    const coordinate = unproject(x, y, zoom);
    onSelectWaypoint(roundCoordinate(coordinate.lat), roundCoordinate(coordinate.lon));
  };

  const handleWheel = (event: WheelEvent<HTMLDivElement>) => {
    event.preventDefault();
    zoomBy(event.deltaY > 0 ? -1 : 1);
  };

  return (
    <section className="mission-map-panel">
      <div
        ref={containerRef}
        className="interactive-map"
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={() => {
          dragRef.current = null;
        }}
        onWheel={handleWheel}
      >
        <div className="map-tile-layer">
          {tiles.map((tile) => (
            <img
              alt=""
              className="map-tile"
              draggable={false}
              key={`${tile.z}-${tile.x}-${tile.y}`}
              src={`https://tile.openstreetmap.org/${tile.z}/${tile.urlX}/${tile.y}.png`}
              style={{ transform: `translate(${tile.left}px, ${tile.top}px)` }}
            />
          ))}
        </div>
        {vehicleScreen && (
          <MapMarker
            className="vehicle-marker"
            icon="navigation"
            label="Device"
            point={vehicleScreen}
          />
        )}
        {waypointScreen && (
          <MapMarker
            className="waypoint-marker"
            icon="flag"
            label="Waypoint"
            point={waypointScreen}
          />
        )}
        <div className="map-controls">
          <button type="button" title="Zoom in" onClick={() => zoomBy(1)}>
            <Icon name="add" />
          </button>
          <button type="button" title="Zoom out" onClick={() => zoomBy(-1)}>
            <Icon name="remove" />
          </button>
          <button type="button" title="Center on device" onClick={recenter}>
            <Icon name="my_location" />
          </button>
        </div>
        <div className="map-status">
          <span>{formatCoordinate(center.lat)}</span>
          <span>{formatCoordinate(center.lon)}</span>
          <span>z{zoom}</span>
        </div>
        <a
          className="map-attribution"
          href="https://www.openstreetmap.org/copyright"
          rel="noreferrer"
          target="_blank"
        >
          &copy; OpenStreetMap contributors
        </a>
      </div>
    </section>
  );
}

function MapMarker({
  className,
  icon,
  label,
  point,
}: {
  className: string;
  icon: string;
  label: string;
  point: Point;
}) {
  return (
    <div className={`map-marker ${className}`} style={{ transform: `translate(${point.x}px, ${point.y}px)` }}>
      <span className="marker-pin">
        <Icon name={icon} />
      </span>
      <span className="marker-label">{label}</span>
    </div>
  );
}

function NumberInput({ label, value, step, onChange }: { label: string; value: number; step?: number; onChange: (value: number) => void }) {
  return (
    <label>
      <span>{label}</span>
      <input type="number" value={value} step={step} onChange={(event) => onChange(Number(event.currentTarget.value))} />
    </label>
  );
}

type Point = { x: number; y: number };

function visibleTiles(center: { lat: number; lon: number }, zoom: number, size: { width: number; height: number }) {
  if (!size.width || !size.height) {
    return [];
  }

  const centerPixel = project(center.lat, center.lon, zoom);
  const minX = Math.floor((centerPixel.x - size.width / 2) / TILE_SIZE);
  const maxX = Math.floor((centerPixel.x + size.width / 2) / TILE_SIZE);
  const minY = Math.floor((centerPixel.y - size.height / 2) / TILE_SIZE);
  const maxY = Math.floor((centerPixel.y + size.height / 2) / TILE_SIZE);
  const tileCount = 2 ** zoom;
  const tiles: { z: number; x: number; y: number; urlX: number; left: number; top: number }[] = [];

  for (let y = minY; y <= maxY; y += 1) {
    if (y < 0 || y >= tileCount) {
      continue;
    }

    for (let x = minX; x <= maxX; x += 1) {
      const urlX = ((x % tileCount) + tileCount) % tileCount;
      tiles.push({
        z: zoom,
        x,
        y,
        urlX,
        left: x * TILE_SIZE - centerPixel.x + size.width / 2,
        top: y * TILE_SIZE - centerPixel.y + size.height / 2,
      });
    }
  }

  return tiles;
}

function toScreenPoint(
  lat: number,
  lon: number,
  centerPixel: Point,
  zoom: number,
  size: { width: number; height: number },
) {
  if (!size.width || !size.height || !isMapCoordinate(lat, lon)) {
    return null;
  }

  const point = project(lat, lon, zoom);
  return {
    x: point.x - centerPixel.x + size.width / 2,
    y: point.y - centerPixel.y + size.height / 2,
  };
}

function project(lat: number, lon: number, zoom: number): Point {
  const scale = TILE_SIZE * 2 ** zoom;
  const clampedLat = clamp(lat, -MAX_MERCATOR_LAT, MAX_MERCATOR_LAT);
  const sinLat = Math.sin((clampedLat * Math.PI) / 180);
  return {
    x: ((wrapLongitude(lon) + 180) / 360) * scale,
    y: (0.5 - Math.log((1 + sinLat) / (1 - sinLat)) / (4 * Math.PI)) * scale,
  };
}

function unproject(x: number, y: number, zoom: number) {
  const scale = TILE_SIZE * 2 ** zoom;
  const lon = wrapLongitude((x / scale) * 360 - 180);
  const n = Math.PI - (2 * Math.PI * y) / scale;
  const lat = (Math.atan(Math.sinh(n)) * 180) / Math.PI;
  return { lat: clamp(lat, -MAX_MERCATOR_LAT, MAX_MERCATOR_LAT), lon };
}

function isMapCoordinate(lat: number, lon: number) {
  return Number.isFinite(lat) && Number.isFinite(lon) && Math.abs(lat) <= 90 && Math.abs(lon) <= 180;
}

function clamp(value: number, min: number, max: number) {
  return Math.min(max, Math.max(min, value));
}

function wrapLongitude(lon: number) {
  return ((((lon + 180) % 360) + 360) % 360) - 180;
}

function roundCoordinate(value: number) {
  return Number(value.toFixed(7));
}

function formatCoordinate(value: number) {
  return value.toFixed(6);
}

function formatHilResponseFlags(flags: number) {
  const labels = [
    [1 << 0, "ARMED"],
    [1 << 1, "FAILSAFE"],
    [1 << 2, "EST"],
    [1 << 3, "MOTORS"],
    [1 << 4, "CTRL"],
    [1 << 5, "CLAMP"],
    [1 << 6, "SENSOR"],
  ] as const;
  const active = labels.filter(([mask]) => (flags & mask) !== 0).map(([, label]) => label);
  return active.length ? active.join("|") : "none";
}
