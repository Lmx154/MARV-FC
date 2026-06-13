import { useEffect, useMemo, useState } from "react";

import { buildDebugLines, buildDemoPacket, buildLaunchPoint, buildRecovery } from "../data/demoTelemetry";

export function useDemoTelemetry() {
  const [startedAt] = useState(() => Date.now() - 12_400);
  const [launchPoint] = useState(() => buildLaunchPoint());
  const [now, setNow] = useState(() => Date.now());

  useEffect(() => {
    const timer = window.setInterval(() => setNow(Date.now()), 500);
    return () => window.clearInterval(timer);
  }, []);

  const packet = useMemo(() => buildDemoPacket((now - startedAt) / 1000), [now, startedAt]);
  const recovery = useMemo(() => buildRecovery(packet, launchPoint), [launchPoint, packet]);
  const debugLines = useMemo(() => buildDebugLines(packet), [packet]);

  return { debugLines, packet, recovery };
}
