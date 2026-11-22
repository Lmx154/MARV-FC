#!/usr/bin/env python3
"""The guard of invariant R1 (_plan/decision-ram-code-20261115.md in MARV-V2-drivers, Q5): after
`core1_main` starts, the flight core makes no XIP access.

Usage: tools/check-ram-code.py <elf>

A static may-reach closure over the linked Thumb ELF, from the flight core's roots:
  roots      `core1_main` (of the flight image or of the N0 bench `eskf_bench`), `DMA_IRQ_0`, `IO_IRQ_BANK0`, `SIO_IRQ_FIFO`, `PIO2_IRQ_0`, and the park function of the
             feature `xip-park` when present; each must match exactly once.
  edges      direct branches that leave the function (long-branch thunks included); constants formed
             by movw/movt pairs; pc-relative literal loads; adr; and the code and data pointers in every
             aligned word wholly inside each constant or data object reached that way.
  terminals  the panic machinery (not followed, may stay in flash).
It FAILS (exit 1) when a non-terminal function, a data object or a constant address formed by
reached code lies in the XIP window 0x10000000-0x1FFFFFFF, and prints each with its call chain.
A constant in the window but beyond the end of the flash image (a register value such as
0x19444412, not the address of anything linked) is printed as a note.
Tool errors exit 2. PASS exits 0.

Blind spot: a function pointer formed only by core 0 and called on core 1 (none today); the board
check of the feature `xip-park` covers it.

Needs python3 (stdlib only) and arm-none-eabi-binutils (readelf, objdump). Symbol types come from
`readelf -sW` (FUNC/OBJECT): `.ram_text` holds code and constants, so nm's section-based letters
cannot tell them apart.
"""

import bisect
import collections
import re
import struct
import subprocess
import sys

XIP_LO, XIP_HI = 0x1000_0000, 0x2000_0000

ROOTS = [
    (r"(16marv_v2_firmware|10eskf_bench)10core1_main$", True),  # the flight image, or the N0 bench
    (r"^DMA_IRQ_0$", True),
    (r"^IO_IRQ_BANK0$", True),
    (r"^SIO_IRQ_FIFO$", True),
    (r"^PIO2_IRQ_0$", True),
    (r"16marv_v2_firmware8xip_park10park_core0$", False),  # feature xip-park only
]

TERMINAL = re.compile(
    r"4core9panicking|13unwrap_failed|13expect_failed|^rust_begin_unwind$|5defmt6export|^_defmt_"
    r"|^__defmt_default_panic$|11panic_probe|^HardFault|^DefaultHandler"
    r"|4core5slice5index\w*_fail|4core3str\w*slice_error_fail|4core4cell\w*panic_already"
)


def fail(msg):
    print(f"check-ram-code: ERROR: {msg}", file=sys.stderr)
    sys.exit(2)


def tool(*args):
    try:
        out = subprocess.run(args, capture_output=True, text=True, check=True)
    except (OSError, subprocess.CalledProcessError) as e:
        fail(f"{' '.join(args[:2])}: {e}")
    return out.stdout


class Elf:
    def __init__(self, path):
        self.data = open(path, "rb").read()
        self.secs = {}  # name -> dict(size, vma, off, contents, code)
        lines = tool("arm-none-eabi-objdump", "-h", path).splitlines()
        for i, line in enumerate(lines):
            p = line.split()
            if len(p) >= 7 and p[0].isdigit() and i + 1 < len(lines):
                flags = lines[i + 1]
                if "ALLOC" not in flags:
                    continue
                self.secs[p[1]] = dict(
                    size=int(p[2], 16),
                    vma=int(p[3], 16),
                    lma=int(p[4], 16),
                    off=int(p[5], 16),
                    contents="CONTENTS" in flags and "LOAD" in flags,
                    code="CODE" in flags,
                )
        if not self.secs:
            fail("no allocated sections")
        # The flash image: every loaded section's load address range, .ram_text's and .data's too.
        self.image_end = max(
            (s["lma"] + s["size"] for s in self.secs.values() if s["contents"] and XIP_LO <= s["lma"] < XIP_HI),
            default=XIP_LO,
        )
        raw = tool("arm-none-eabi-readelf", "-sW", path).splitlines()
        dem = tool("arm-none-eabi-readelf", "-sW", "-C", path).splitlines()
        if len(raw) != len(dem):
            fail("readelf symbol tables differ in length")
        self.funcs = {}  # start -> [size, mangled, demangled]
        self.objs = {}  # start -> size
        self.names = collections.defaultdict(list)  # mangled -> [starts], FUNC only
        sym = re.compile(r"^\s*\d+:\s+([0-9a-f]+)\s+(\d+)\s+(\w+)\s+\w+\s+\w+\s+(\w+)\s+(\S.*)$")
        for r, d in zip(raw, dem):
            m, md = sym.match(r), sym.match(d)
            if not m or not md or m.group(4) in ("UND", "ABS"):
                continue
            val, size, typ, name = int(m.group(1), 16), int(m.group(2)), m.group(3), m.group(5)
            if typ == "FUNC":
                a = val & ~1
                if self.sec_of(a) is None:
                    continue
                self.names[name].append(a)
                if a not in self.funcs or (size and not self.funcs[a][0]):
                    self.funcs[a] = [size, name, md.group(5)]
            elif typ == "OBJECT" and self.sec_of(val) is not None:
                self.objs[val] = max(self.objs.get(val, 0), size)
        if not self.funcs:
            fail("no function symbols")
        self.fstarts = sorted(self.funcs)
        # Size 0 (LLD's long-branch thunks, assembly without .size): up to the next symbol.
        for i, a in enumerate(self.fstarts):
            if self.funcs[a][0] == 0:
                nxt = self.fstarts[i + 1] if i + 1 < len(self.fstarts) else a + 16
                self.funcs[a][0] = max(2, min(nxt - a, 64))
        self.ostarts = sorted(self.objs)

    def sec_of(self, a):
        for n, s in self.secs.items():
            if s["vma"] <= a < s["vma"] + s["size"]:
                return n
        return None

    def word(self, a):
        n = self.sec_of(a)
        if n is None or not self.secs[n]["contents"] or a + 4 > self.secs[n]["vma"] + self.secs[n]["size"]:
            return None
        return struct.unpack_from("<I", self.data, self.secs[n]["off"] + a - self.secs[n]["vma"])[0]

    def func_of(self, a):
        a &= ~1
        i = bisect.bisect_right(self.fstarts, a) - 1
        if i >= 0 and a < self.fstarts[i] + self.funcs[self.fstarts[i]][0]:
            return self.fstarts[i]
        return None


def scan_code(elf, path):
    """Per function: functions it reaches directly, data addresses it forms, XIP constants it forms."""
    calls = collections.defaultdict(set)
    drefs = collections.defaultdict(set)
    xconst = collections.defaultdict(set)
    ins = re.compile(r"^\s*([0-9a-f]+):\s+(\S+)\s*(.*)$")
    target = re.compile(r"(?:^|, )([0-9a-f]+) <[^>]*>$")
    imm = re.compile(r"^(r\d+|sb|sl|fp|ip|lr), #(\d+)")
    comment_addr = re.compile(r"@ \(([0-9a-f]+)")
    adr = re.compile(r"(?:^|@ \()adr(?:\.w|\.n)?\s+\S+, ([0-9a-f]+)|@ \(adr\S*\s+\S+, ([0-9a-f]+)")
    for name, s in elf.secs.items():
        if not s["contents"] or not any(s["vma"] <= a < s["vma"] + s["size"] for a in elf.fstarts):
            continue
        dis = tool("arm-none-eabi-objdump", "-d" if s["code"] else "-D", "--no-show-raw-insn", "-j", name, path)
        cur = None
        movw = {}
        for line in dis.splitlines():
            m = ins.match(line)
            if not m:
                continue
            a = int(m.group(1), 16)
            f = elf.func_of(a)
            if f is None:
                continue
            if f != cur:
                cur, movw = f, {}
            mn, ops = m.group(2), m.group(3)

            def ref(v, f=f):
                g = elf.func_of(v)
                if g is not None:
                    if g != f:
                        calls[f].add(g)
                elif elf.sec_of(v) is not None:
                    drefs[f].add(v)
                if XIP_LO <= v < XIP_HI and g is None:
                    xconst[f].add(v)

            if mn.startswith(("b", "cb")):
                t = target.search(ops)
                if t:
                    ref(int(t.group(1), 16))
                continue
            if mn.startswith("movw"):
                mm = imm.match(ops)
                if mm:
                    movw[mm.group(1)] = int(mm.group(2))
                continue
            if mn.startswith("movt"):
                mm = imm.match(ops)
                if mm and mm.group(1) in movw:
                    ref((int(mm.group(2)) << 16) | movw.pop(mm.group(1)))
                continue
            if mn.startswith("ldr") and "[pc" in ops:
                mm = comment_addr.search(ops)
                if mm:
                    lit = int(mm.group(1), 16)
                    for k in range(2 if mn.startswith("ldrd") else 1):
                        w = elf.word(lit + 4 * k)
                        if w is not None:
                            ref(w)
                continue
            mm = adr.search(mn + " " + ops)
            if mm:
                ref(int(mm.group(1) or mm.group(2), 16))
    return calls, drefs, xconst


def main():
    if len(sys.argv) != 2:
        fail("usage: tools/check-ram-code.py <elf>")
    path = sys.argv[1]
    elf = Elf(path)
    calls, drefs, xconst = scan_code(elf, path)

    bounds = sorted(set(elf.ostarts) | set(elf.fstarts) | {v for s in drefs.values() for v in s})

    def extent(v):
        i = bisect.bisect_right(elf.ostarts, v) - 1
        if i >= 0 and elf.objs[elf.ostarts[i]] and v < elf.ostarts[i] + elf.objs[elf.ostarts[i]]:
            return elf.ostarts[i], elf.ostarts[i] + elf.objs[elf.ostarts[i]]
        s = elf.secs[elf.sec_of(v)]
        j = bisect.bisect_right(bounds, v)
        end = bounds[j] if j < len(bounds) else s["vma"] + s["size"]
        return v, max(v + 4, min(end, s["vma"] + s["size"], v + 4096))

    roots = []
    for pat, required in ROOTS:
        hits = sorted(
            {a for n, starts in elf.names.items() if re.search(pat, n) and "Thunk_" not in n for a in starts}
        )
        if len(hits) > 1 or (required and not hits):
            fail(f"root /{pat}/ matched {len(hits)} functions (exactly one wanted)")
        roots += hits

    # Breadth first, so that each chain printed is a shortest one.
    parent = {r: None for r in roots}
    dparent = {}
    queue = collections.deque(roots)
    seen_d = {}
    terminals = set()
    while queue:
        f = queue.popleft()
        if TERMINAL.search(elf.funcs[f][1]):
            terminals.add(f)
            continue
        nxt = sorted(calls[f])
        stack = sorted(drefs[f])
        for v in stack:
            dparent.setdefault(extent(v)[0], f)
        while stack:
            lo, hi = extent(stack.pop())
            if lo in seen_d:
                continue
            seen_d[lo] = hi - lo
            for p in range((lo + 3) & ~3, hi - 3, 4):
                w = elf.word(p)
                if w is None or elf.sec_of(w) is None:
                    continue
                g = elf.func_of(w)
                if g is not None and w & 1:
                    nxt.append(g)
                elif g is None:
                    d = extent(w)[0]
                    if d not in seen_d:
                        dparent.setdefault(d, dparent.get(lo, f))
                        stack.append(w)
        for g in nxt:
            if g not in parent:
                parent[g] = f
                queue.append(g)

    def chain(f):
        out = []
        while f is not None:
            out.append(elf.funcs[f][2])
            f = parent[f]
        return " <- ".join(out)

    def short(name):
        return name if len(name) <= 140 else name[:137] + "..."

    reached = [f for f in parent if f not in terminals]
    bad_f = sorted(f for f in reached if XIP_LO <= f < XIP_HI)
    bad_d = sorted(d for d in seen_d if XIP_LO <= d < XIP_HI)
    bad_c = sorted((v, f) for f in reached for v in xconst[f] if v < elf.image_end)
    beyond = sorted((v, f) for f in reached for v in xconst[f] if v >= elf.image_end)
    code = sum(elf.funcs[f][0] for f in reached)
    const = sum(seen_d.values())
    print(
        f"check-ram-code: {path}\n"
        f"  roots {len(roots)}: {', '.join(elf.funcs[r][2] for r in roots)}\n"
        f"  reached {len(reached)} functions ({code} B), {len(seen_d)} data objects ({const} B), "
        f"{len(terminals)} panic terminals"
    )
    for v, f in beyond[:20]:
        print(f"  note: constant {v:#010x} (in the XIP window, beyond the flash image) formed in {short(elf.funcs[f][2])}")
    if not (bad_f or bad_d or bad_c):
        print("  PASS: nothing the flight core can reach after core1_main lies in XIP (0x10000000-0x1FFFFFFF)")
        return 0
    print(f"  FAIL: {len(bad_f)} functions, {len(bad_d)} data objects, {len(bad_c)} constant addresses in XIP")
    for f in bad_f[:40]:
        print(f"  function {f:#010x} {short(chain(f))}")
    for d in bad_d[:20]:
        print(f"  data     {d:#010x} ({seen_d[d]} B) used by {short(chain(dparent[d]))}")
    for v, f in bad_c[:20]:
        print(f"  constant {v:#010x} formed in {short(chain(f))}")
    shown = min(len(bad_f), 40) + min(len(bad_d), 20) + min(len(bad_c), 20)
    if shown < len(bad_f) + len(bad_d) + len(bad_c):
        print(f"  ... {len(bad_f) + len(bad_d) + len(bad_c) - shown} more")
    return 1


if __name__ == "__main__":
    sys.exit(main())
