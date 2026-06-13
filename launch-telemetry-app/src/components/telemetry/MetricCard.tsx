export function MetricCard({ label, value, unit, trend }: { label: string; value: string; unit: string; trend: string }) {
  return (
    <div className="metric-card">
      <span>{label}</span>
      <strong>
        {value}
        <small>{unit}</small>
      </strong>
      <em>{trend}</em>
    </div>
  );
}
