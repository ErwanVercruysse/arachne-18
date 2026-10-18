"""Load KiCad library symbols (resolving 'extends') and list their pins."""
import os, copy
from sexp import parse, find, find1, Sym
SYMDIR = '/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols/'
_cache = {}

def lib(name, path=None):
    if name not in _cache:
        _cache[name] = parse(open(path or SYMDIR + name + '.kicad_sym').read())
    return _cache[name]

def get(libid, path=None):
    """Return a standalone symbol definition (with extends flattened)."""
    ln, sn = libid.split(':')
    L = lib(ln, path)
    syms = {s[1]: s for s in find(L, 'symbol')}
    s = copy.deepcopy(syms[sn])
    ext = find1(s, 'extends')
    if ext:
        base = copy.deepcopy(syms[ext[1]])
        # take properties from derived, graphics/pins from base
        props = {p[1]: p for p in find(s, 'property')}
        out = [Sym('symbol'), sn]
        for e in base[2:]:
            if isinstance(e, list) and e[0] == 'property' and e[1] in props:
                out.append(props.pop(e[1]))
            elif isinstance(e, list) and e[0] == 'symbol':
                sub = copy.deepcopy(e); sub[1] = sub[1].replace(ext[1], sn, 1); out.append(sub)
            else:
                out.append(e)
        out.extend(props.values())
        s = out
    return s

def pins(sym):
    """[(unit, number, name, type, x, y, angle, length)]"""
    res = []
    for sub in find(sym, 'symbol'):
        parts = sub[1].rsplit('_', 2)
        unit = int(parts[-2]) if len(parts) == 3 else 0
        for p in find(sub, 'pin'):
            at = find1(p, 'at'); ln = find1(p, 'length')
            name = find1(p, 'name')[1]; num = find1(p, 'number')[1]
            res.append((unit, num, name, str(p[1]), float(at[1]), float(at[2]), float(at[3]) if len(at) > 3 else 0., float(ln[1])))
    return res
