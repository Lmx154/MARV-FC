import type { ViewId } from "../types";

export const views: { id: ViewId; label: string; icon: string }[] = [
  { id: "dashboard", label: "Dashboard", icon: "grid_view" },
  { id: "mission", label: "Mission Inputs", icon: "explore" },
  { id: "hil", label: "HIL Lab", icon: "memory" },
  { id: "actuators", label: "Actuator Tools", icon: "settings_input_component" },
  { id: "diagnostics", label: "Diagnostics", icon: "terminal" },
];
