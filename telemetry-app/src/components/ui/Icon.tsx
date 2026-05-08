export function Icon({ name, fill = false }: { name: string; fill?: boolean }) {
  return <span className={`material-symbols-outlined${fill ? " icon-fill" : ""}`}>{name}</span>;
}
