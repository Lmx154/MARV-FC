import { useEffect, useMemo, useState } from "react";

import { AppHeader } from "./components/layout/AppHeader";
import { TabBar } from "./components/layout/TabBar";
import { buildDebugLines } from "./data/demoTelemetry";
import { useDemoTelemetry } from "./hooks/useDemoTelemetry";
import type { DebugFilter, TabId, ThemeMode } from "./types";
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
  const { debugLines, packet, recovery } = useDemoTelemetry();

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
            onSave={() => saveRawLog(buildDebugLines(packet))}
            onToggleAutoscroll={() => setAutoscroll((value) => !value)}
            onTogglePaused={() => setPaused((value) => !value)}
            onToggleRecording={() => setRecording((value) => !value)}
          />
        )}
        {activeTab === "settings" && <SettingsView theme={theme} onThemeChange={setTheme} />}
      </main>
    </div>
  );
}

export default App;
