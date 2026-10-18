"""Draw placement plan + report courtyard overlaps (run with a python that has matplotlib)."""
import json, sys
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
d = json.load(open(sys.argv[1])); png = sys.argv[2]
fig, ax = plt.subplots(figsize=(24, 15)); ax.set_aspect('equal')
import json as _j; _o=_j.load(open('outline.json'))['edge']; ax.plot([p[0] for p in _o],[p[1] for p in _o],'k',lw=2)
for f in d:
    if f['ref']=='U3': x,y=f['pos']; f['box']=[x-9.78,y-12.9,x+9.78,y+13.47]
for f in d:
    x0, y0, x1, y1 = f['box']
    big = f['ref'] in ('M1',)
    ax.add_patch(Rectangle((x0, y0), x1-x0, y1-y0, fill=False, ec='b' if not big else 'g', lw=.8))
    for n, net, a, c in f['pads']:
        ax.add_patch(Rectangle(a, c[0]-a[0], c[1]-a[1], color='orange' if net else 'grey', alpha=.6, lw=0))
    if not big:
        ax.text((x0+x1)/2, (y0+y1)/2, f['ref'], ha='center', va='center', fontsize=7, color='k')
ov = []
for i, a in enumerate(d):
    for b in d[i+1:]:
        pass
        A, B = a['box'], b['box']
        if A[0] < B[2] and B[0] < A[2] and A[1] < B[3] and B[1] < A[3]:
            ov.append((a['ref'], b['ref']))
print('overlaps:', ov)
ax.set_xlim(-50, 50); ax.set_ylim(44, -38); ax.grid(alpha=.3); ax.set_xticks(range(-70, 71, 5)); ax.set_yticks(range(-40, 36, 5))
fig.savefig(png, dpi=70, bbox_inches='tight')
