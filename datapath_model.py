"""Cycle-accurate model of the multi-row datapath (LOAD + SWEEP) mirroring the RTL.
Two-phase update: all 'next' values computed from current regs, then committed."""
import numpy as np

def sobel_ref(img):
    W = img.shape[0]; p = np.pad(img.astype(int), 1)
    out = np.zeros((W, W), int)
    for r in range(W):
        for c in range(W):
            a = p[r:r+3, c:c+3]
            gx = (a[0,2]+2*a[1,2]+a[2,2]) - (a[0,0]+2*a[1,0]+a[2,0])
            gy = (a[2,0]+2*a[2,1]+a[2,2]) - (a[0,0]+2*a[0,1]+a[0,2])
            out[r, c] = min(255, abs(gx)+abs(gy))
    return out


WR_LAT = 7

def lane_s1(p00,p01,p02,p10,p12,p20,p21,p22):
    return ((p02+2*p12+p22) & 0x3FF, (p00+2*p10+p20) & 0x3FF,
            (p20+2*p21+p22) & 0x3FF, (p00+2*p01+p02) & 0x3FF)
def to_s12(v):
    v &= 0xFFF
    return v-4096 if v & 0x800 else v
def lane_s2(s1):
    gxp,gxn,gyp,gyn = s1
    return (to_s12(gxp-gxn), to_s12(gyp-gyn))
def lane_s3(s2):
    out = []
    for v in s2:
        u = v & 0xFFF
        out.append(((~u & 0x7FF) + 1) & 0x7FF if u & 0x800 else u & 0x7FF)
    return tuple(out)
def lane_s4(s3):
    mag = s3[0] + s3[1]
    return 255 if mag > 255 else mag & 0xFF

def sobel_mag(p00,p01,p02,p10,p12,p20,p21,p22):
    sx = (p02 + 2*p12 + p22) - (p00 + 2*p10 + p20)
    sy = (p20 + 2*p21 + p22) - (p00 + 2*p01 + p02)
    m = abs(sx) + abs(sy)
    return 255 if m > 255 else m

def run_pass(flat, W, R, base):
    rb = [[0]*W for _ in range(R+2)]; rbq = [0]*(R+2)
    w = [[0]*(R+2) for _ in range(3)]       # w[0]=oldest col, w[2]=newest
    pipe1=[None]*R; pipe2=[None]*R; pipe3=[None]*R
    res = [0]*R
    ob = [[None]*W for _ in range(R)]
    img_q = 0; s1_v = 0; s1_row = 0; s1_col = 0; s1_in = 0
    state = 'LOAD'; ld_row = 0; ld_col = 0; sw_cnt = 0
    cycles = 0
    while True:
        cycles += 1
        src_row = base - 1 + ld_row
        in_range = 1 if 0 <= src_row < W else 0
        ld_addr = src_row*W + ld_col if in_range else 0
        rb_raddr = 0 if sw_cnt >= W else sw_cnt
        # ---- next values
        n_img_q = flat[ld_addr]
        n_s1 = (1 if state == 'LOAD' else 0, ld_row, ld_col, in_range)
        n_rbq = [rb[i][rb_raddr] for i in range(R+2)]       # sync read (old data)
        wr = None
        if s1_v: wr = (s1_row, s1_col, img_q if s1_in else 0)
        n_w = [row[:] for row in w]
        if state == 'SWEEP':
            if sw_cnt == 0:
                n_w = [[0]*(R+2) for _ in range(3)]
            elif sw_cnt <= W:
                n_w[0] = w[1][:]; n_w[1] = w[2][:]; n_w[2] = rbq[:]
            elif sw_cnt == W+1:
                n_w[0] = w[1][:]; n_w[1] = w[2][:]; n_w[2] = [0]*(R+2)
        n_p1 = [lane_s1(w[0][r],w[1][r],w[2][r], w[0][r+1],w[2][r+1],
                        w[0][r+2],w[1][r+2],w[2][r+2]) for r in range(R)]
        n_p2 = [lane_s2(p) if p is not None else None for p in pipe1]
        n_p3 = [lane_s3(p) if p is not None else None for p in pipe2]
        n_res = [lane_s4(p) if p is not None else 0 for p in pipe3]
        obw = None
        if state == 'SWEEP' and sw_cnt >= WR_LAT:
            obw = (sw_cnt-WR_LAT, res[:])
        # ---- fsm
        n_state, n_ld_row, n_ld_col, n_sw = state, ld_row, ld_col, sw_cnt
        done = False
        if state == 'LOAD':
            if ld_col == W-1:
                n_ld_col = 0
                if ld_row == R+1: n_state = 'SWEEP'; n_sw = 0
                else: n_ld_row = ld_row+1
            else: n_ld_col = ld_col+1
        elif state == 'SWEEP':
            if sw_cnt == W+WR_LAT-1: done = True
            else: n_sw = sw_cnt+1
        # ---- commit
        if wr: rb[wr[0]][wr[1]] = wr[2]
        if obw:
            for r in range(R): ob[r][obw[0]] = obw[1][r]
        img_q = n_img_q; s1_v,s1_row,s1_col,s1_in = n_s1
        rbq = n_rbq; w = n_w; res = n_res; pipe1=n_p1; pipe2=n_p2; pipe3=n_p3
        state, ld_row, ld_col, sw_cnt = n_state, n_ld_row, n_ld_col, n_sw
        if done: break
    return ob, cycles

def run_frame(img, R):
    W = img.shape[0]; flat = img.flatten().tolist()
    out = np.zeros((W, W), int); total = 0
    base = 0
    while base < W:
        ob, cyc = run_pass(flat, W, R, base); total += cyc
        rows_this = min(R, W-base)
        for r in range(rows_this): out[base+r] = ob[r]
        base += R
    return out, total

if __name__ == '__main__':
    rng = np.random.default_rng(1)
    ok = True
    for W in (4, 5, 8, 13, 16, 128):
        for R in (1, 2, 3, 4, 8, 16):
            img = rng.integers(0, 256, (W, W))
            if W == 128 and R in (3,): img = rng.integers(0, 256, (W, W))
            exp = sobel_ref(img); got, cyc = run_frame(img, R)
            good = (exp == got).all(); ok &= good
            if not good or W in (8, 128):
                print(f"W={W:3d} R={R:2d} {'PASS' if good else 'FAIL'}  compute cycles={cyc}")
    print("ALL PASS" if ok else "SOME FAIL")
