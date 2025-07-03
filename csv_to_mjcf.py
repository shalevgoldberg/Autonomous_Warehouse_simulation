#!/usr/bin/env python3
"""csv_to_mjcf.py – v 1.5  (directional lanes **and** directional gateway boxes)
==========================================================================

Grid‑CSV → MuJoCo scene *plus* lanes.csv / boxes.csv / bays.csv
----------------------------------------------------------------
* **Lane tokens  (l… )** – letters n e s w are **travel directions**.
* **Box  tokens  (j… )** – letters after j are **allowed exits**.
  Contiguous j‑cells merge; letters union.
* **Bay tokens   (d i c)** – drop‑off / idle / charge bays.
  A 1‑cell **gateway box** is generated for every neighbouring lane cell; its
  allowed‑exit string is exactly the lane's direction letters.

No runtime block/unblock needed – all conflict areas are static.
"""
import argparse, csv
from collections import deque
from pathlib import Path
from typing import List, Tuple, Set, Dict

import numpy as np

# --------------------------- helpers ---------------------------

def load_grid(path: Path) -> List[List[str]]:
    """Load CSV grid with UTF-8 BOM support"""
    with open(path, newline="", encoding="utf-8-sig") as f:
        return [[tok.strip() for tok in row if tok.strip()] for row in csv.reader(f)]

def xy(r:int,c:int, cell:float)->Tuple[float,float]:
    """grid index -> metres (MuJoCo coords)"""
    return c*cell, -r*cell

BASE_DIR: Dict[str,str] = {"n":"N","e":"E","s":"S","w":"W"}

# --------------------------- main ------------------------------

def generate(grid:List[List[str]], cell:float):
    H,W = len(grid), len(grid[0])

    lanes: List[Tuple[str,float,float]]=[]               # dir,x,y
    boxes: List[Tuple[float,float,float,str]]=[]         # xc,yc,size,dirs
    bays : List[Tuple[str,float,float,float,float]]=[]   # type,xc,yc,gx,gy

    # ------- merge j blobs into boxes ------------
    seen: Set[Tuple[int,int]] = set()
    for r in range(H):
        for c in range(W):
            if (r,c) in seen or not grid[r][c].startswith('j'):
                continue
            # BFS blob
            q=deque([(r,c)])
            cells=[]; dirs=set()
            while q:
                rr,cc=q.popleft()
                if (rr,cc) in seen or rr<0 or rr>=H or cc<0 or cc>=W: continue
                tok=grid[rr][cc]
                if not tok.startswith('j'): continue
                seen.add((rr,cc)); cells.append((rr,cc))
                dirs.update(BASE_DIR.get(ch,'') for ch in tok[1:])
                for dr,dc in [(-1,0),(1,0),(0,-1),(0,1)]: q.append((rr+dr,cc+dc))
            if not dirs: dirs=set('NESW')
            # box centre and size
            rows=[p[0] for p in cells]; cols=[p[1] for p in cells]
            xc,yc = xy((min(rows)+max(rows)+1)/2, (min(cols)+max(cols)+1)/2, cell)
            size  = max(max(rows)-min(rows)+1, max(cols)-min(cols)+1)*cell
            boxes.append((xc+cell/2, yc-cell/2, size, ''.join(sorted(dirs))))

    # ------- pass 2: lanes, bays, gateway boxes --------
    gateway_added: Set[Tuple[int,int]] = set()
    for r in range(H):
        for c in range(W):
            tok=grid[r][c]
            xc,yc = xy(r,c,cell)
            cen_x,cen_y = xc+cell/2, yc-cell/2
            if tok.startswith('l'):
                for ch in tok[1:]:
                    if ch in BASE_DIR:
                        lanes.append((BASE_DIR[ch], cen_x, cen_y))
            elif tok in ('d','i','c'):
                bays.append((tok.upper(), cen_x, cen_y, cen_x, cen_y))
                # look 4-neigh lanes for gateway boxes
                for dr,dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                    rr,cc = r+dr, c+dc
                    if 0<=rr<H and 0<=cc<W and grid[rr][cc].startswith('l') and (rr,cc) not in gateway_added:
                        lane_tok = grid[rr][cc]
                        dirs = ''.join(sorted({BASE_DIR[ch] for ch in lane_tok[1:] if ch in BASE_DIR})) or 'NESW'
                        gx,gy = xy(rr,cc,cell)
                        boxes.append((gx+cell/2, gy-cell/2, cell, dirs))
                        gateway_added.add((rr,cc))

    return lanes, boxes, bays

# --------------------------- write files -----------------------

def write_outputs(out:Path, lanes, boxes, bays, grid, cell):
    out.mkdir(parents=True, exist_ok=True)
    # lanes.csv
    with open(out/'lanes.csv','w',newline='') as f:
        wr=csv.writer(f)
        for i,(d,x,y) in enumerate(lanes): wr.writerow([i,d,f"{x:.3f}",f"{y:.3f}"])
    # boxes.csv
    with open(out/'boxes.csv','w',newline='') as f:
        wr=csv.writer(f)
        for i,(x,y,s,d) in enumerate(boxes): wr.writerow([i,f"{x:.3f}",f"{y:.3f}",f"{s:.3f}",d])
    # bays.csv
    with open(out/'bays.csv','w',newline='') as f:
        wr=csv.writer(f)
        for i,(t,xc,yc,gx,gy) in enumerate(bays): wr.writerow([i,t,f"{xc:.3f}",f"{yc:.3f}",f"{gx:.3f}",f"{gy:.3f}"])
    # minimalist MJCF (floor + racks/walls)
    mj=["<mujoco model='warehouse'>","  <worldbody>","    <geom type='plane' size='50 50 0.1' rgba='0.8 0.8 0.8 1'/>"]
    H=len(grid); W=len(grid[0])
    for r in range(H):
        for c in range(W):
            if grid[r][c] in ('s','w'):
                x,y=xy(r,c,cell); col='0.3 0.3 0.3 1' if grid[r][c]=='s' else '0.1 0.1 0.7 1'
                mj.append(f"    <geom type='box' pos='{x+cell/2} {y-cell/2} 0.5' size='{cell/2} {cell/2} 1' rgba='{col}' contype='1' conaffinity='1'/>")
    mj.extend(["  </worldbody>","</mujoco>"])
    (out/'warehouse.mjcf').write_text('\n'.join(mj))

# --------------------------- cli -------------------------------

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('grid',type=Path)
    ap.add_argument('--cell',type=float,default=0.5)
    ap.add_argument('--out',type=Path,default=Path('./maps'))
    args=ap.parse_args()

    grid=load_grid(args.grid)
    lanes,boxes,bays=generate(grid,args.cell)
    write_outputs(args.out,lanes,boxes,bays,grid,args.cell)
    print(f"lanes={len(lanes)}, boxes={len(boxes)}, bays={len(bays)} written to {args.out}")

if __name__=='__main__':
    main() 