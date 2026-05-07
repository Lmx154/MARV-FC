export function StatBox({ label, value, accent = false }: { label: string; value: string; accent?: boolean }) {
  return (
    <div className={`stat-box ${accent ? "accent" : ""}`}>
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}
