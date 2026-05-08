export function Metric({
  label,
  value,
  large = false,
  mono = false,
}: {
  label: string;
  value: string;
  large?: boolean;
  mono?: boolean;
}) {
  return (
    <div className={`metric ${large ? "large" : ""} ${mono ? "mono" : ""}`}>
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}
