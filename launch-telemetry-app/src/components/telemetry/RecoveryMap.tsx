import { useEffect, useRef } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";

import type { RecoveryModel, TelemetryPacket, ThemeMode } from "../../types";
import { formatFeet } from "../../utils/format";

type RecoveryMapProps = {
  packet: TelemetryPacket;
  recovery: RecoveryModel;
  theme: ThemeMode;
};

const tileThemes: Record<ThemeMode, { attribution: string; url: string }> = {
  dark: {
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> &copy; <a href="https://carto.com/attributions">CARTO</a>',
    url: "https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png",
  },
  light: {
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> &copy; <a href="https://carto.com/attributions">CARTO</a>',
    url: "https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png",
  },
};

const launchIcon = L.divIcon({
  className: "recovery-map-marker launch",
  html: "<span>Launch T-0</span>",
  iconAnchor: [14, 14],
});

const rocketIcon = L.divIcon({
  className: "recovery-map-marker rocket",
  html: "<span>Rocket</span>",
  iconAnchor: [14, 14],
});

export function RecoveryMap({ packet, recovery, theme }: RecoveryMapProps) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<L.Map | null>(null);
  const tileLayerRef = useRef<L.TileLayer | null>(null);
  const launchMarkerRef = useRef<L.Marker | null>(null);
  const rocketMarkerRef = useRef<L.Marker | null>(null);
  const pathRef = useRef<L.Polyline | null>(null);
  const initialRecoveryRef = useRef(recovery);
  const didFitInitialBoundsRef = useRef(false);

  useEffect(() => {
    if (!containerRef.current || mapRef.current) return;

    const initialRecovery = initialRecoveryRef.current;
    const launchLatLng = toLatLng(initialRecovery.launchPoint);
    const map = L.map(containerRef.current, {
      attributionControl: true,
      zoomControl: true,
    }).setView(launchLatLng, 15);
    map.attributionControl.setPrefix(false);

    launchMarkerRef.current = L.marker(launchLatLng, { icon: launchIcon }).addTo(map);
    rocketMarkerRef.current = L.marker(toLatLng(initialRecovery.rocketPoint), { icon: rocketIcon }).addTo(map);
    pathRef.current = L.polyline([], {
      color: pathColor(theme),
      opacity: 0.9,
      weight: 4,
    }).addTo(map);

    mapRef.current = map;

    const resizeObserver = new ResizeObserver(() => map.invalidateSize());
    resizeObserver.observe(containerRef.current);

    return () => {
      resizeObserver.disconnect();
      map.remove();
      mapRef.current = null;
      tileLayerRef.current = null;
      launchMarkerRef.current = null;
      rocketMarkerRef.current = null;
      pathRef.current = null;
    };
  }, []);

  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;

    tileLayerRef.current?.remove();

    const tileTheme = tileThemes[theme];
    tileLayerRef.current = L.tileLayer(tileTheme.url, {
      attribution: tileTheme.attribution,
      maxZoom: 20,
      subdomains: "abcd",
    }).addTo(map);

    pathRef.current?.setStyle({ color: pathColor(theme) });
  }, [theme]);

  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;

    const launchLatLng = toLatLng(recovery.launchPoint);
    const rocketLatLng = toLatLng(recovery.rocketPoint);
    const trail = recovery.trail.map(toLatLng);

    launchMarkerRef.current?.setLatLng(launchLatLng);
    rocketMarkerRef.current?.setLatLng(rocketLatLng);
    pathRef.current?.setLatLngs(trail);

    if (!didFitInitialBoundsRef.current) {
      fitMapToRecovery(map, recovery);
      didFitInitialBoundsRef.current = true;
    }
  }, [recovery]);

  return (
    <div className="map-panel">
      <div ref={containerRef} className="leaflet-recovery-map" />
      <button className="map-recenter" type="button" onClick={() => mapRef.current && fitMapToRecovery(mapRef.current, recovery)}>
        Recenter
      </button>
      <div className="map-readout">
        <span>Launch {recovery.launchPoint.lat.toFixed(6)}, {recovery.launchPoint.lon.toFixed(6)}</span>
        <span>Rocket {packet.gpsLat.toFixed(6)}, {packet.gpsLon.toFixed(6)}</span>
        <span>Alt {formatFeet(packet.gpsAltM)} ft</span>
      </div>
    </div>
  );
}

function toLatLng(point: { lat: number; lon: number }): L.LatLngExpression {
  return [point.lat, point.lon];
}

function fitMapToRecovery(map: L.Map, recovery: RecoveryModel) {
  const launchLatLng = toLatLng(recovery.launchPoint);
  const rocketLatLng = toLatLng(recovery.rocketPoint);

  if (recovery.distanceM < 10) {
    map.setView(launchLatLng, 16, { animate: false });
    return;
  }

  map.fitBounds(L.latLngBounds([launchLatLng, rocketLatLng]).pad(0.35), {
    animate: false,
    maxZoom: 15,
  });
}

function pathColor(theme: ThemeMode) {
  return theme === "dark" ? "#ffc247" : "#8f5800";
}
