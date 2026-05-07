export function MotorDiagram({ motors }: { motors: number[] }) {
  return (
    <div className="motor-diagram">
      <div className="fcu">FCU</div>
      <i className="arm a" />
      <i className="arm b" />
      {motors.slice(0, 4).map((motor, index) => (
        <div key={index} className={`motor m${index + 1}`}>
          <span>M{index + 1}</span>
          <strong>{motor}</strong>
        </div>
      ))}
    </div>
  );
}

export function MotorBar({ label, value }: { label: string; value: number }) {
  const percent = Math.max(0, Math.min(100, (value / 2000) * 100));
  return (
    <div className="motor-bar">
      <div>
        <b style={{ height: `${percent}%` }} />
      </div>
      <span>{label}</span>
    </div>
  );
}
