export function StatusChip({ label, value, tone }: { label: string; value: string; tone: "accent" | "good" | "neutral" }) {
  return (
    <div className={`status-chip ${tone}`}>
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}
