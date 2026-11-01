"""ARACHNE body v4: an isolated, one-piece six-servo chassis (millimetres).

The v4 leg is a read-only dependency; the released robot is never imported.
"""
from pathlib import Path
import os, sys, math, json, hashlib
import cadquery as cq

from v4_leg import box, cylinder, collar, CASE_SEAT

RADIUS=86.0
ANGLES=(45,90,135,225,270,315)
ANCHORS=[(RADIUS*math.cos(math.radians(a)),RADIUS*math.sin(math.radians(a)),a) for a in ANGLES]
TOP=32.0
BOTTOM=27.0
INSERTS=[(x,y) for x in (-64,64) for y in (-25,25)]

def place(s,anchor):
    x,y,a=anchor
    return s.rotate((0,0,0),(0,0,1),a).translate((x,y,0))

def bed(s):
    s=s.rotate((0,0,0),(1,0,0),180)
    return s.translate((0,0,-s.BoundingBox().zmin))

def build():
    # The top of the deck is the single build-plate datum when inverted.
    deck=cq.Workplane('XY').ellipse(97,91).extrude(TOP-BOTTOM).val().translate((0,0,BOTTOM))
    for x,y,_ in ANCHORS:
        deck=deck.cut(cylinder((x,y,26),(0,0,1),26,8))
    deck=deck.clean()
    vertical=[e for e in deck.Edges() if e.BoundingBox().zlen>4.99
              and e.BoundingBox().xlen<.001 and e.BoundingBox().ylen<.001]
    deck=deck.fillet(3,vertical)
    spine=box(0,0,24,145,16,6)
    spine=cq.Workplane(obj=spine).edges('|Z').fillet(5).val()
    body=deck.fuse(spine)
    for anchor in ANCHORS:
        # Same proven case geometry as the v4 links, rotated to yaw orientation.
        cradle=collar(0).rotate((0,0,0),(1,0,0),90)
        # Broad rear bridge ties both cradle sides into the deck, outside the
        # moving fork. It does not obstruct insertion from the dummy side.
        bridge=box(-31.05,0,25.0,16.9,34,4.1)
        bridge=cq.Workplane(obj=bridge).edges('|Z').fillet(2.4).val()
        rib=box(-51,0,25,38,10,4)
        rib=cq.Workplane(obj=rib).edges('|Z').fillet(4).val()
        body=body.fuse(place(cradle.fuse(bridge,rib),anchor))
    # Dorsal mounting bosses accept RX-M3x5.7 inserts from the flat top.
    for x,y in INSERTS:
        body=body.fuse(cylinder((x,y,22),(0,0,1),5.5,10))
    cuts=[]
    for anchor in ANCHORS:
        for x in (-29,-8.3):
            for y in (-10.25,10.25):
                cuts += [place(cylinder((x,y,15.6),(0,0,1),1.1,3),anchor),
                         place(cylinder((x,y,CASE_SEAT),(0,0,1),2.6,25),anchor)]
    for x,y in INSERTS:
        cuts += [cylinder((x,y,TOP+.01),(0,0,-1),2,7.01),
                 cylinder((x,y,TOP+.01),(0,0,-1),2.25,.46)]
    # Four slots accept two 10 mm battery straps; rounded ends protect straps.
    for x in (-30,30):
        for y in (-25,25):
            slot=cq.Workplane('XY').slot2D(14,3.5,0).extrude(14).val().translate((x,y,20))
            cuts.append(slot)
    # Central loom slots stay above the servo tails, away from moving forks.
    for x in (-22,22):
        cuts.append(cq.Workplane('XY').slot2D(18,8,90).extrude(15).val().translate((x,46,20)))
        cuts.append(cq.Workplane('XY').slot2D(18,8,90).extrude(15).val().translate((x,-46,20)))
    return body.cut(*cuts).clean()
