import { Icon } from "./Icon";

export function HeaderLine({ icon, title, danger = false }: { icon: string; title: string; danger?: boolean }) {
  return (
    <div className={`header-line ${danger ? "danger" : ""}`}>
      <Icon name={icon} />
      <h3>{title}</h3>
    </div>
  );
}
