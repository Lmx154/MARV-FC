import { useEffect, useMemo, useState } from "react";

import { AppHeader } from "./components/layout/AppHeader";
import { TabBar } from "./components/layout/TabBar";
import { useDemoTelemetry } from "./hooks/useDemoTelemetry";
import { useLiveTelemetry } from "./hooks/useLiveTelemetry";
import { isTauri } from "./lib/backend";
import type { DataSource, DebugFilter, TabId, ThemeMode } from "./types";
import { saveRawLog } from "./utils/logging";
import { DashboardView } from "./views/DashboardView";
import { DebugView } from "./views/DebugView";
import { RecoveryView } from "./views/RecoveryView";
import { SettingsView } from "./views/SettingsView";

function App() {
  const [activeTab, setActiveTab] = useState<TabId>("dashboard");
  const [theme, setTheme] = useState<ThemeMode>("dark");
  const [autoscroll, setAutoscroll] = useState(true);
  const [paused, setPaused] = useState(false);
  const [recording, setRecording] = useState(true);
  const [debugFilter, setDebugFilter] = useState<DebugFilter>("ALL");
  const [clearedAtSeq, setClearedAtSeq] = useState(0);
  const [source, setSource] = useState<DataSource>(() => (isTauri() ? "live" : "demo"));

  // Both hooks always run (rules of hooks); we render whichever source is selected. The live
  // hook only subscribes to Tauri events, the demo hook only runs a timer, so the idle one is cheap.
  const demo = useDemoTelemetry();
  const live = useLiveTelemetry();
  const { debugLines, packet, recovery } = source === "live" ? live : demo;

  const visibleDebugLines = useMemo(
    () => debugLines.filter((line) => packet.seq >= clearedAtSeq && (debugFilter === "ALL" || line.kind === debugFilter)),
    [clearedAtSeq, debugFilter, debugLines, packet.seq],
  );

  useEffect(() => {
    document.documentElement.dataset.theme = theme;
  }, [theme]);

  return (
    <div className="app-shell" data-theme={theme}>
      <AppHeader packet={packet} theme={theme} onToggleTheme={() => setTheme((value) => (value === "dark" ? "light" : "dark"))} />
      <TabBar activeTab={activeTab} onChange={setActiveTab} />

      <main className="content">
        {activeTab === "dashboard" && <DashboardView packet={packet} />}
        {activeTab === "recovery" && <RecoveryView packet={packet} recovery={recovery} theme={theme} />}
        {activeTab === "debug" && (
          <DebugView
            autoscroll={autoscroll}
            debugFilter={debugFilter}
            lines={visibleDebugLines}
            paused={paused}
            recording={recording}
            onClear={() => setClearedAtSeq(packet.seq + 1)}
            onFilterChange={setDebugFilter}
            onSave={() => saveRawLog(debugLines)}
            onToggleAutoscroll={() => setAutoscroll((value) => !value)}
            onTogglePaused={() => setPaused((value) => !value)}
            onToggleRecording={() => setRecording((value) => !value)}
          />
        )}
        {activeTab === "settings" && (
          <SettingsView
            theme={theme}
            onThemeChange={setTheme}
            source={source}
            onSourceChange={setSource}
          />
        )}
      </main>
    </div>
  );
}

export default App;
