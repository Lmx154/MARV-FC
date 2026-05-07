import type { ReactNode } from "react";

import { Icon } from "./Icon";

export function Panel({
  title,
  icon,
  accent,
  className = "",
  children,
}: {
  title: string;
  icon?: string;
  accent?: string;
  className?: string;
  children: ReactNode;
}) {
  return (
    <section className={`panel ${className}`}>
      <header className="panel-header">
        <h3>
          {icon && <Icon name={icon} />}
          {title}
        </h3>
        {accent && <span>{accent}</span>}
      </header>
      <div className="panel-body">{children}</div>
    </section>
  );
}
