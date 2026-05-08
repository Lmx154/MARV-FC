# 10. Architecture and Abstractions Visualization (Mermaid)

This page turns the architecture docs plus current workspace structure into visual Mermaid maps.

It intentionally combines:

- conceptual boundaries from `01` to `09`
- concrete workspace crates and target files
- FC hardware domains from `pcbhardware.md`

---

## 10.1 Workspace architecture and abstraction boundaries

```mermaid
flowchart TB
    guard["Guard rail: common is HAL-agnostic and must not import concrete embassy_rp peripheral types"]

    subgraph common["common/ (portable logic)"]
        c_drivers["drivers<br/>sensor + radio + storage adapters"]
        c_protocol["protocol<br/>framing + packet formats"]
        c_coms["coms<br/>transport + link/session behavior"]
        c_tasks["tasks<br/>portable task bodies"]
        c_local["localization<br/>estimation logic"]
        c_telem["telemetry + policies + params + commands"]
        c_types["types + utils + sd"]

        c_protocol --> c_coms
        c_drivers --> c_tasks
        c_types --> c_tasks
        c_tasks --> c_local
        c_tasks --> c_telem
    end

    subgraph device["device/ (embedded RP235x runtime assembly)"]
        d_template["MARV-FC-RL-RP2354B<br/>resources pinmap buses channels watchdog core0 core1"]
        d_fc["rocket-fc | rp235x | drone-fc"]
        d_link["radio | gs"]
        d_test["test"]
    end

    subgraph simulator["sim/ (SITL host runtime)"]
        s_loop["sim_loop + state"]
        s_api["api + ws + fs_access"]
        s_ui["ui (index.html, app.js, styles.css)"]

        s_loop --> s_api --> s_ui
    end

    common --> device
    common --> simulator
    guard -.enforces.-> common

    device --> hw["RP2354A/B hardware targets"]
    simulator --> host["Host OS + browser"]
```

---

## 10.2 Runtime behavior: tiers, tasks, cores, and watchdog contract

```mermaid
flowchart LR
        subgraph model["FC runtime model"]
        subgraph core0["Core 0 deterministic island (feed-critical)"]
            isr["ISR class<br/>latch, timestamp, notify"]
            tier0["Tier 0 edge I/O<br/>SPI/I2C/UART/PWM + watchdog HW access"]
            fastpub["Core 0 local pub-sub<br/>fast samples fan out without backpressure"]
            tier1["Tier 1 pure transforms<br/>conversion, calibration, parsing, freshness math"]
            tier2["Tier 2 stateful services<br/>estimation -> control -> actuator intent"]
            tap0["Core 0 local subscribers<br/>estimator + local diagnostics + time-sensitive logging"]
            wds["Watchdog supervisor<br/>liveness contract evaluation"]
            wdhw["Hardware feed owner<br/>device/.../watchdog.rs"]

            isr --> tier0 --> fastpub --> tier1 --> tier2 --> wds --> wdhw
            fastpub --> tap0
        end

        subgraph core1["Core 1 variable-latency island"]
            bg["telemetry, GPS, bridge services, diagnostics, indicators<br/>from Core 1 local inputs or explicit mirrors only"]
        end

        subgraph channels["Typed channel topology (example from FC target)"]
            core0ch["core0_local<br/>FastSensorSample + EnvironmentalSample + AuxiliaryNavigationSample + WatchdogStatus"]
            core1ch["core1_local<br/>FcRadioTraffic + CompanionTraffic"]
            xcore["cross_core_bridges<br/>explicit only, empty by default"]
        end

        fastpub --> core0ch
        bg --> core1ch
        xcore -.optional and target-owned.-> bg
    end

    wds -.deny feed on invalid progress.-> reset["Watchdog reset + reset-cause reporting"]
```

---

## 10.3 FC hardware domains and ownership wiring

```mermaid
flowchart TB
    subgraph pins["FC RP2354B bus domains (pcbhardware + resources.rs)"]
        spi1["SPI1 sensor cluster<br/>GP10/11/12 + CS13/14/16<br/>BMI088 + LSM6DSV32X"]
        spi0["SPI0 storage domain<br/>GP18/19/20 + CS21<br/>microSD"]
        i2c0["I2C0 environmental domain<br/>GP8/9<br/>BMP581"]
        i2c1["I2C1 auxiliary-nav domain<br/>GP2/3<br/>BMM350 + NEO-M9N"]
        uart0["UART0 link domain<br/>GP0/1<br/>FC <-> Radio"]
        uart1["UART1 companion domain<br/>GP4/5<br/>FC <-> SBC"]
        pwm["PWM actuator domain<br/>GP39/38/35/36 + GP37"]
        led["Status LED<br/>GP6"]
    end

    subgraph files["device/MARV-FC-RL-RP2354B ownership files"]
        pinmap["pinmap.rs"]
        buses["buses.rs"]
        resources["resources.rs"]
        channels["channels.rs"]
        core0["core0.rs"]
        core1["core1.rs"]
        watchdog["watchdog.rs"]
    end

    pinmap --> resources
    buses --> resources
    resources --> core0
    resources --> core1
    core0 --> channels
    core1 --> channels
    core0 --> watchdog

    resources --> spi1
    resources --> spi0
    resources --> i2c0
    resources --> i2c1
    resources --> uart0
    resources --> uart1
    resources --> pwm
    resources --> led
```

---

## 10.4 Notes

- Tasks are scheduling boundaries, not architecture layers.
- `common/` owns reusable logic; `device/` owns concrete HAL assembly; `sim/` preserves parity on host.
- Watchdog feed authority should stay centralized under Core 0 supervisory logic.
- Pub-sub fan-out is core-local by default; cross-core visibility requires an explicit target-owned bridge.
- Time-sensitive logging capture belongs with the publishers on Core 0; slow sinks still require buffering and must not stall the fast path.
