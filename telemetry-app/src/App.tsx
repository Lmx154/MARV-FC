import { useState } from "react";

import "./App.css";
import { SideNav } from "./components/layout/SideNav";
import { TopBar } from "./components/layout/TopBar";
import { useBackendSnapshot } from "./hooks/useBackendSnapshot";
import type { ViewId } from "./types";
import { useTelemetryLookup } from "./utils/telemetry";
import { ActuatorView } from "./views/ActuatorView";
import { Dashboard } from "./views/Dashboard";
import { DiagnosticsView } from "./views/DiagnosticsView";
import { HilView } from "./views/HilView";
import { MissionView } from "./views/MissionView";

function App() {
  const [activeView, setActiveView] = useState<ViewId>("dashboard");
  const { state, backendError, commandStatus, command, refreshPorts } = useBackendSnapshot();
  const lookup = useTelemetryLookup(state);

  return (
    <div className="app-shell">
      <TopBar state={state} backendError={backendError} />
      <div className="app-body">
        <SideNav activeView={activeView} onChange={setActiveView} />
        <div className="view-root">
          {activeView === "dashboard" && (
            <Dashboard
              state={state}
              lookup={lookup}
              command={command}
              commandStatus={commandStatus}
              onRefreshPorts={refreshPorts}
            />
          )}
          {activeView === "mission" && <MissionView state={state} command={command} />}
          {activeView === "hil" && <HilView state={state} command={command} />}
          {activeView === "actuators" && <ActuatorView state={state} command={command} />}
          {activeView === "diagnostics" && <DiagnosticsView state={state} command={command} />}
        </div>
      </div>
    </div>
  );
}

export default App;
