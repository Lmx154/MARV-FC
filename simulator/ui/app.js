(() => {
  const connDot = document.getElementById("conn-dot");
  const connLabel = document.getElementById("conn-label");
  const btnConnect = document.getElementById("btn-connect");

  const simTick = document.getElementById("sim-tick");
  const simTime = document.getElementById("sim-time");
  const simRunning = document.getElementById("sim-running");
  const simRate = document.getElementById("sim-rate");
  const simFile = document.getElementById("sim-file");

  const inputRate = document.getElementById("input-rate");
  const inputStep = document.getElementById("input-step");
  const btnPause = document.getElementById("btn-pause");
  const btnResume = document.getElementById("btn-resume");
  const btnRate = document.getElementById("btn-rate");
  const btnStep = document.getElementById("btn-step");

  const rootSelect = document.getElementById("root-select");
  const btnLoadRoot = document.getElementById("btn-load-root");
  const btnUp = document.getElementById("btn-up");
  const btnRefresh = document.getElementById("btn-refresh");
  const includeHidden = document.getElementById("include-hidden");
  const fsPath = document.getElementById("fs-path");
  const fsTableBody = document.getElementById("fs-table-body");

  const previewPath = document.getElementById("preview-path");
  const previewContent = document.getElementById("preview-content");

  const eventLog = document.getElementById("event-log");

  let ws = null;
  let currentPath = null;

  function log(message) {
    const now = new Date().toISOString();
    eventLog.textContent = `[${now}] ${message}\n` + eventLog.textContent;
  }

  function wsUrl() {
    const protocol = window.location.protocol === "https:" ? "wss" : "ws";
    return `${protocol}://${window.location.host}/ws`;
  }

  function setWsState(connected) {
    connDot.classList.toggle("online", connected);
    connDot.classList.toggle("offline", !connected);
    connLabel.textContent = connected ? "Connected" : "Disconnected";
  }

  function sendWsCommand(payload) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      log("WebSocket is not connected");
      return;
    }
    ws.send(JSON.stringify(payload));
  }

  function updateSnapshot(snapshot) {
    simTick.textContent = String(snapshot.tick);
    simTime.textContent = Number(snapshot.sim_time_s).toFixed(3);
    simRunning.textContent = snapshot.running ? "true" : "false";
    simRate.textContent = Number(snapshot.rate_hz).toFixed(2);
    simFile.textContent = snapshot.selected_data_file || "-";
    inputRate.value = Number(snapshot.rate_hz).toFixed(2);
  }

  function connectWs() {
    if (ws && ws.readyState === WebSocket.OPEN) {
      return;
    }

    ws = new WebSocket(wsUrl());

    ws.addEventListener("open", () => {
      setWsState(true);
      log("WebSocket connected");
    });

    ws.addEventListener("close", () => {
      setWsState(false);
      log("WebSocket closed");
    });

    ws.addEventListener("error", () => {
      log("WebSocket error");
    });

    ws.addEventListener("message", (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg.type === "snapshot" && msg.snapshot) {
          updateSnapshot(msg.snapshot);
        }
        if (msg.type === "ack") {
          log(`ACK ${msg.command}: ${msg.message}`);
        }
        if (msg.type === "error") {
          log(`ERR: ${msg.message}`);
        }
      } catch (err) {
        log(`invalid ws payload: ${err}`);
      }
    });
  }

  async function fetchJson(url, options = {}) {
    const res = await fetch(url, options);
    if (!res.ok) {
      const text = await res.text();
      throw new Error(`${res.status} ${res.statusText} :: ${text}`);
    }
    return res.json();
  }

  function renderFsRows(entries) {
    fsTableBody.innerHTML = "";

    for (const entry of entries) {
      const tr = document.createElement("tr");

      const tdName = document.createElement("td");
      tdName.textContent = entry.name;

      const tdType = document.createElement("td");
      tdType.textContent = entry.is_dir ? "dir" : "file";

      const tdSize = document.createElement("td");
      tdSize.textContent = entry.size_bytes != null ? String(entry.size_bytes) : "-";

      const tdActions = document.createElement("td");
      if (entry.is_dir) {
        const openBtn = document.createElement("button");
        openBtn.type = "button";
        openBtn.textContent = "Open";
        openBtn.addEventListener("click", () => {
          browsePath(entry.path);
        });
        tdActions.appendChild(openBtn);
      } else {
        const previewBtn = document.createElement("button");
        previewBtn.type = "button";
        previewBtn.textContent = "Preview";
        previewBtn.addEventListener("click", () => {
          previewFile(entry.path);
        });

        const selectBtn = document.createElement("button");
        selectBtn.type = "button";
        selectBtn.textContent = "Select";
        selectBtn.addEventListener("click", () => {
          selectDataFile(entry.path);
        });

        tdActions.appendChild(previewBtn);
        tdActions.appendChild(selectBtn);
      }

      tr.appendChild(tdName);
      tr.appendChild(tdType);
      tr.appendChild(tdSize);
      tr.appendChild(tdActions);
      fsTableBody.appendChild(tr);
    }
  }

  async function loadRoots() {
    const data = await fetchJson("/api/fs/roots");
    rootSelect.innerHTML = "";
    for (const root of data.roots) {
      const option = document.createElement("option");
      option.value = root;
      option.textContent = root;
      rootSelect.appendChild(option);
    }

    if (data.roots.length > 0) {
      currentPath = data.roots[0];
      await browsePath(currentPath);
    }
  }

  async function browsePath(path) {
    const include = includeHidden.checked ? "true" : "false";
    const data = await fetchJson(
      `/api/fs/list?path=${encodeURIComponent(path)}&include_hidden=${include}`,
    );
    currentPath = data.path;
    fsPath.textContent = data.path;
    renderFsRows(data.entries);
  }

  async function previewFile(path) {
    const data = await fetchJson(`/api/fs/read?path=${encodeURIComponent(path)}&max_bytes=131072`);
    previewPath.textContent = data.path;
    previewContent.textContent = data.content;
  }

  async function selectDataFile(path) {
    const data = await fetchJson("/api/sim/select_data_file", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ path }),
    });
    updateSnapshot(data);
    log(`selected data file: ${path}`);
  }

  function parentPath(path) {
    if (!path || path === "/") {
      return path;
    }

    const clean = path.endsWith("/") ? path.slice(0, -1) : path;
    const idx = clean.lastIndexOf("/");
    if (idx <= 0) {
      return "/";
    }
    return clean.slice(0, idx);
  }

  async function refreshState() {
    const state = await fetchJson("/api/sim/state");
    updateSnapshot(state);
  }

  btnConnect.addEventListener("click", connectWs);
  btnPause.addEventListener("click", () => sendWsCommand({ type: "pause" }));
  btnResume.addEventListener("click", () => sendWsCommand({ type: "resume" }));
  btnRate.addEventListener("click", () => {
    const hz = Number(inputRate.value);
    sendWsCommand({ type: "set_rate_hz", hz });
  });
  btnStep.addEventListener("click", () => {
    const dtMs = Number(inputStep.value);
    sendWsCommand({ type: "step", dt_ms: dtMs });
  });

  btnLoadRoot.addEventListener("click", () => {
    if (rootSelect.value) {
      browsePath(rootSelect.value).catch((err) => log(err.message));
    }
  });

  btnUp.addEventListener("click", () => {
    if (currentPath) {
      browsePath(parentPath(currentPath)).catch((err) => log(err.message));
    }
  });

  btnRefresh.addEventListener("click", () => {
    if (currentPath) {
      browsePath(currentPath).catch((err) => log(err.message));
    }
  });

  includeHidden.addEventListener("change", () => {
    if (currentPath) {
      browsePath(currentPath).catch((err) => log(err.message));
    }
  });

  (async () => {
    try {
      await refreshState();
      await loadRoots();
      connectWs();
    } catch (err) {
      log(String(err));
    }
  })();
})();
