import type { ReactNode } from "react";

export function SettingsPanel({ children, title }: { children: ReactNode; title: string }) {
  return (
    <div className="settings-panel">
      <h2>{title}</h2>
      {children}
    </div>
  );
}
