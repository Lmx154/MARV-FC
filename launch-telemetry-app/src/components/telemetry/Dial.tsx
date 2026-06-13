import type { CSSProperties } from "react";

export function Dial({ label, readout, value }: { label: string; readout: string; value: number }) {
  return (
    <div className="dial-card">
      <div className="dial" style={{ "--dial-value": `${value}%` } as CSSProperties}>
        <span>{readout}</span>
      </div>
      <strong>{label}</strong>
    </div>
  );
}
