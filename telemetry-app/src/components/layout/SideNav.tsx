import { views } from "../../constants/navigation";
import type { ViewId } from "../../types";
import { Icon } from "../ui/Icon";

export function SideNav({ activeView, onChange }: { activeView: ViewId; onChange: (view: ViewId) => void }) {
  return (
    <aside className="sidebar">
      <nav className="nav-list">
        {views.map((view) => (
          <button
            key={view.id}
            className={`nav-item ${activeView === view.id ? "active" : ""}`}
            onClick={() => onChange(view.id)}
          >
            <Icon name={view.icon} />
            <span>{view.label}</span>
          </button>
        ))}
      </nav>
      <nav className="nav-list footer-nav">
        <button className="nav-item">
          <Icon name="help" />
          <span>Support</span>
        </button>
        <button className="nav-item">
          <Icon name="history" />
          <span>Logs</span>
        </button>
      </nav>
    </aside>
  );
}
