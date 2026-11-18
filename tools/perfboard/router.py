"""Perfboard maze router: places nothing, only connects terminals of each net with solder bridges
(orthogonal hole-to-hole segments on the solder side) and, if unavoidable, short wire jumpers on the
component side. Every hole carries at most one net. Prints ASCII maps and writes JSON for the viewer."""
import heapq, json, sys

def hdr(h, n):
    r, odd = (n - 1) >> 1, n % 2 == 1
    return (13 if odd else 14, 12 + r) if h == 'CN7' else (38 if odd else 39, 12 + r)

class Board:
    def __init__(self, name, cols, rows, header):
        self.name, self.c0, self.c1, self.r0, self.r1 = name, cols[0], cols[1], rows[0], rows[1]
        self.occ = {}          # hole -> net
        self.fixed = {}        # hole -> description
        self.parts, self.pads, self.paths, self.jumpers = [], [], [], []
        self.order = []
        for n in range(1, 39):
            self.occ[hdr(header, n)] = f'_{header}-{n}'
            self.fixed[hdr(header, n)] = f'{header}-{n}'
        self.header = header

    def inside(self, h):
        return self.c0 <= h[0] <= self.c1 and self.r0 <= h[1] <= self.r1

    def pin(self, n, net):
        h = hdr(self.header, n)
        self.occ[h] = net
        return h

    def take(self, h, net, what):
        assert self.inside(h), (self.name, h, what)
        if h in self.occ and self.occ[h] != net:
            raise ValueError(f'{self.name}: hole {h} already {self.occ[h]}, wanted {net} ({what})')
        self.occ[h] = net
        self.fixed[h] = what

    def part(self, ref, t, v, leads, card, note='', **kw):
        """leads: list of (hole, net, label)"""
        for h, net, lab in leads:
            self.take(h, net if net else f'_{ref}.{lab}', f'{ref}.{lab}')
        self.parts.append(dict(ref=ref, t=t, v=v, leads=[[h[0], h[1], net, lab] for h, net, lab in leads], card=card, note=note, **kw))

    def pad(self, h, net, label, side, card):
        self.take(h, net, 'pad ' + label)
        self.pads.append(dict(at=list(h), net=net, label=label, side=side, card=card))

    def preroute(self, net, pts):
        cells = []
        for a, b in zip(pts, pts[1:]):
            dx, dy = (b[0] > a[0]) - (b[0] < a[0]), (b[1] > a[1]) - (b[1] < a[1])
            x, y = a
            cells.append((x, y))
            while (x, y) != tuple(b):
                x, y = x + dx, y + dy
                cells.append((x, y))
        for c in cells:
            assert self.inside(c), (self.name, c, net)
            if c in self.occ and self.occ[c] != net:
                raise ValueError(f'{self.name}: preroute {net} hits {c} = {self.occ[c]}')
            self.occ[c] = net
        self.paths.append(dict(net=net, pts=[list(p) for p in pts]))

    # ---- routing ----
    def terminals(self, net):
        return [h for h, n in self.occ.items() if n == net]

    def components(self, net):
        """connected groups of holes of this net, via paths and jumpers"""
        holes = set(self.terminals(net))
        adj = {h: set() for h in holes}
        def link(a, b):
            if a in adj and b in adj:
                adj[a].add(b); adj[b].add(a)
        for p in self.paths:
            if p['net'] != net:
                continue
            pts = p['pts']
            for a, b in zip(pts, pts[1:]):
                dx, dy = (b[0] > a[0]) - (b[0] < a[0]), (b[1] > a[1]) - (b[1] < a[1])
                x, y = a
                while (x, y) != tuple(b):
                    nx, ny = x + dx, y + dy
                    link((x, y), (nx, ny)); x, y = nx, ny
        for j in self.jumpers:
            if j['net'] == net:
                link(tuple(j['a']), tuple(j['b']))
        seen, comps = set(), []
        for h in holes:
            if h in seen:
                continue
            st, comp = [h], set()
            while st:
                c = st.pop()
                if c in seen:
                    continue
                seen.add(c); comp.add(c); st.extend(adj[c] - seen)
            comps.append(comp)
        return comps

    def free(self, h, net):
        return self.inside(h) and self.occ.get(h) in (None, net)

    def route(self, net, jumper_cost=40, max_jump=6, avoid=()):
        guard = 0
        while True:
            comps = self.components(net)
            if len(comps) <= 1:
                return True
            guard += 1
            if guard > 40:
                raise RuntimeError(net)
            comps.sort(key=len, reverse=True)
            src, targets = comps[0], set().union(*comps[1:])
            # Dijkstra with state (hole, dir)
            pq, best, prev = [], {}, {}
            tick = iter(range(10**9))
            for h in src:
                heapq.heappush(pq, (0, next(tick), h, None))
                best[(h, None)] = 0
            found = None
            while pq:
                d, _, h, dr = heapq.heappop(pq)
                if best.get((h, dr), 1e9) < d:
                    continue
                if h in targets and h not in src:
                    found = (h, dr); break
                for ddx, ddy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    n = (h[0] + ddx, h[1] + ddy)
                    if not self.free(n, net) or n in avoid:
                        continue
                    if self.occ.get(n) == net and n not in targets and n not in src:
                        pass
                    nd = d + 1 + (0.6 if dr not in (None, (ddx, ddy)) else 0) + (0.4 if self.occ.get(n) is None and n in self.fixed else 0)
                    key = (n, (ddx, ddy))
                    if nd < best.get(key, 1e9):
                        best[key] = nd; prev[key] = ((h, dr), 'b'); heapq.heappush(pq, (nd, next(tick), n, (ddx, ddy)))
                for ddx, ddy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    if h in self.fixed:
                        break
                    for L in range(2, max_jump + 1):
                        n = (h[0] + ddx * L, h[1] + ddy * L)
                        if not self.free(n, net) or n in self.fixed:
                            continue
                        nd = d + jumper_cost + L
                        key = (n, ('J', ddx, ddy))
                        if nd < best.get(key, 1e9):
                            best[key] = nd; prev[key] = ((h, dr), 'j'); heapq.heappush(pq, (nd, next(tick), n, ('J', ddx, ddy)))
            if not found:
                raise RuntimeError(f'{self.name}: no route for {net}')
            # backtrack
            seq, k = [], found
            while k in prev:
                (pk, kind) = prev[k]
                seq.append((pk[0], k[0], kind)); k = pk
            seq.reverse()
            run = []
            def flush():
                if len(run) >= 2:
                    pts = [run[0]]
                    for a, b, c in zip(run, run[1:], run[2:]):
                        if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
                            pts.append(b)
                    pts.append(run[-1])
                    self.paths.append(dict(net=net, pts=[list(p) for p in pts]))
            for a, b, kind in seq:
                if kind == 'b':
                    if not run:
                        run.append(a)
                    run.append(b)
                    for c in (a, b):
                        if self.occ.get(c) is None:
                            self.occ[c] = net
                else:
                    flush(); run = []
                    for c in (a, b):
                        if self.occ.get(c) is None:
                            self.occ[c] = net
                    self.jumpers.append(dict(net=net, a=list(a), b=list(b)))
            flush()

    def check(self, nets):
        bad = []
        for net in nets:
            if len(self.components(net)) != 1:
                bad.append(net)
        # every path cell belongs to its net
        for p in self.paths:
            pts = p['pts']
            for a, b in zip(pts, pts[1:]):
                dx, dy = (b[0] > a[0]) - (b[0] < a[0]), (b[1] > a[1]) - (b[1] < a[1])
                x, y = a
                while True:
                    assert self.occ.get((x, y)) == p['net'], (self.name, (x, y), self.occ.get((x, y)), p['net'])
                    if (x, y) == tuple(b):
                        break
                    x, y = x + dx, y + dy
        return bad

    def ascii(self, legend):
        out = []
        out.append('    ' + ''.join(f'{c % 10}' for c in range(self.c0, self.c1 + 1)))
        for r in range(self.r0, self.r1 + 1):
            row = ''
            for c in range(self.c0, self.c1 + 1):
                n = self.occ.get((c, r))
                if n is None:
                    row += '·'
                elif n.startswith('_'):
                    row += '#'
                else:
                    row += legend.get(n, '?')
            out.append(f'{r:3d} ' + row)
        return '\n'.join(out)
