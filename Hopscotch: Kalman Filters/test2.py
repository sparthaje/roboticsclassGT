"""
Pygame-based estimation & jump debugger.

Usage:
  python test2.py                  # estimation mode  (cases 1-15)
  python test2.py --jump           # jump mode        (cases 16-32)
  python test2.py --case 5         # start at case 5 (estimation)
  python test2.py --jump --case 20 # start at case 20 (jump)

Controls:
  SPACE / RIGHT  - advance one timestep
  LEFT           - step back (replays from t=0)
  A              - toggle auto-play
  +/-            - speed up / slow down auto-play
  N              - next case
  P              - previous case
  Q / ESC        - quit

Estimation colours:
  White circle   - true asteroid position
  Green dot      - estimated position (match)
  Red dot        - estimated position (no match)
  Grey ring      - match-range threshold

Jump colours:
  White circle   - true asteroid position (next timestep)
  Cyan circle    - currently ridden asteroid
  Yellow circle  - jump target asteroid
  Green dot      - estimate (match)
  Red dot        - estimate (miss)
  Orange circle  - agent with jump-range ring
"""

import sys
import os
import json
import math
import random
import pygame
from copy import deepcopy

# Make sure project imports work
sys.path.insert(0, os.path.dirname(__file__))

from game_objects import Arena, Asteroid, AsteroidShower, Agent
from spaceship import Spaceship
from runner import add_observation_noise
from utilities import distance_formula

# ── case loading ──────────────────────────────────────────────────────────────

def load_cases():
    cases = {}
    base_dir = os.path.join(os.path.dirname(__file__), 'cases')
    for f in sorted(os.listdir(base_dir)):
        if f.endswith('.json'):
            cid = os.path.splitext(f)[0]
            with open(os.path.join(base_dir, f)) as fh:
                cases[cid] = json.load(fh)
    return cases

CASES = load_cases()
ESTIMATE_CASE_IDS = [f'case{i}' for i in range(1,  16) if f'case{i}' in CASES]
JUMP_CASE_IDS     = [f'case{i}' for i in range(16, 33) if f'case{i}' in CASES]

# ── estimation simulation ─────────────────────────────────────────────────────

def build_simulation(case_id):
    params = CASES[case_id]
    asteroids = [Asteroid(a) for a in params['asteroids']]
    random.seed(params['_args']['observation_noise_seed'])
    arena = Arena(params['arena_x_bounds'], params['arena_y_bounds'],
                  agent_xstart_min_max=params['agent_x_min_max'])
    agent = Agent(x_pos=params['agent_x_min_max'], y_pos=0,
                  jump_distance=params['agent_jump_distance'],
                  x_bounds=params['arena_x_bounds'],
                  y_bounds=params['arena_y_bounds'])
    x_start, y_start = agent.get_agent_position(t=0)
    spaceship = Spaceship(deepcopy(arena.bounds), (x_start, y_start))
    shower = AsteroidShower(arena, params['_args']['observation_noise_seed'],
                            asteroids, spaceship)
    return params, shower, spaceship, arena

def precompute_true_locations(shower, time_limit):
    all_data = {}
    seen = set()
    for t in range(time_limit + 2):
        locs, outside, _ = shower.asteroid_locations(t, seen)
        seen.update(outside)
        all_data[t] = locs
    return all_data

def run_estimates(params, all_data, time_limit):
    asteroids = [Asteroid(a) for a in params['asteroids']]
    random.seed(params['_args']['observation_noise_seed'])
    arena = Arena(params['arena_x_bounds'], params['arena_y_bounds'],
                  agent_xstart_min_max=params['agent_x_min_max'])
    agent = Agent(x_pos=params['agent_x_min_max'], y_pos=0,
                  jump_distance=params['agent_jump_distance'],
                  x_bounds=params['arena_x_bounds'],
                  y_bounds=params['arena_y_bounds'])
    x_start, y_start = agent.get_agent_position(t=0)
    spaceship = Spaceship(deepcopy(arena.bounds), (x_start, y_start))
    shower = AsteroidShower(arena, params['_args']['observation_noise_seed'],
                            asteroids, spaceship)

    noise_x = params['noise_sigma_x']
    noise_y = params['noise_sigma_y']
    match_range = params['asteroid_match_range']

    steps = []
    seen = set()
    for t in range(time_limit):
        true_now, outside, _ = shower.asteroid_locations(t, seen)
        seen.update(outside)
        noisy = add_observation_noise(true_now, noise_x, noise_y)
        est = spaceship.predict_from_observations(deepcopy(noisy))
        true_next = all_data.get(t + 1, {})

        kf_vel = {}
        for idx, kf in spaceship.asteroids.items():
            if idx in est:
                kf_vel[idx] = (kf.x.value[2][0], kf.x.value[3][0])

        matches = {}
        for idx, (xe, ye) in est.items():
            if idx in true_next:
                xt, yt = true_next[idx]
                matches[idx] = distance_formula((xe, ye), (xt, yt)) <= match_range

        steps.append({'true': true_now, 'noisy': noisy, 'est': est,
                      'true_next': true_next, 'matches': matches, 'kf_vel': kf_vel})
    return steps

# ── jump simulation ───────────────────────────────────────────────────────────

def run_jump_steps(params):
    """Pre-simulate the full jump sequence; return (start_pos, list-of-step-dicts).

    Each step dict keys:
      true        - {id: (x,y)} true positions at time t
      true_next   - {id: (x,y)} true positions at time t+1
      noisy       - {id: (x,y)} noisy observations at time t
      estimates   - {id: (x,y)} or None  returned by spaceship.jump
      agent_pos   - (x,y) agent position AFTER applying jump decision
      ridden      - id of currently ridden asteroid (after this step), or None
      jump_to     - id student chose to jump to, or None
      jump_dest   - (x,y) true future position of jump target, or None
      valid_jump  - True/False if a jump was attempted, else None
      message     - 'SUCCESS', 'AGENT_OUT_OF_BOUNDS', 'JUMP_OUT_OF_RANGE',
                    'TARGET_OUT_OF_FIELD', or None (still running)
    """
    asteroids = [Asteroid(a) for a in params['asteroids']]
    random.seed(params['_args']['observation_noise_seed'])
    arena = Arena(params['arena_x_bounds'], params['arena_y_bounds'],
                  agent_xstart_min_max=params['agent_x_min_max'])
    agent = Agent(x_pos=params['agent_x_min_max'], y_pos=0,
                  jump_distance=params['agent_jump_distance'],
                  x_bounds=params['arena_x_bounds'],
                  y_bounds=params['arena_y_bounds'])
    x_start, y_start = agent.get_agent_position(t=0)
    spaceship = Spaceship(deepcopy(arena.bounds), (x_start, y_start))
    shower = AsteroidShower(arena, params['_args']['observation_noise_seed'],
                            asteroids, spaceship)

    time_limit = params['_args']['time_limit']
    noise_x    = params['noise_sigma_x']
    noise_y    = params['noise_sigma_y']

    # Pre-compute all true locations (with correct seen tracking)
    seen = set()
    all_data   = {}
    outside_lrb = {}
    for t in range(time_limit + 2):
        locs, outside_all, outside_sides = shower.asteroid_locations(t, seen)
        seen.update(outside_all)
        all_data[t]    = locs
        outside_lrb[t] = outside_sides

    agent_data = {'ridden_asteroid': None, 'jump_distance': agent.jump_distance}
    steps = []

    for t in range(time_limit):
        true_now  = all_data[t]
        true_next = all_data.get(t + 1, {})
        noisy     = add_observation_noise(true_now, noise_x, noise_y)

        agent_pos_before = agent.get_agent_position()

        selected_idx, estimates = spaceship.jump(deepcopy(noisy), deepcopy(agent_data))

        # Build per-asteroid debug info explaining why each was accepted/rejected
        candidates_debug = {}
        if estimates:
            double_jump = agent_data['ridden_asteroid'] is None
            jd = agent.jump_distance * (2 if double_jump else 1)
            apx, apy = agent_pos_before
            EPS_BOUNDS = 0.4
            x_bounds_loc = params['arena_x_bounds']
            y_bounds_loc = params['arena_y_bounds']
            for key, (ex, ey) in estimates.items():
                dist = ((ex - apx)**2 + (ey - apy)**2)**0.5
                reasons = []
                kf = spaceship.asteroids.get(key)
                vx = kf.x.value[2][0] if kf else 0.0
                vy = kf.x.value[3][0] if kf else 0.0
                speed = (vx**2 + vy**2)**0.5
                vy_unit = vy / (speed + 1e-9)
                if key == agent_data['ridden_asteroid']:
                    reasons.append('already riding')
                elif dist > jd - 0.1:
                    reasons.append(f'too far ({dist:.3f} > {jd:.3f})')
                else:
                    if kf:
                        if abs(ex - x_bounds_loc[0]) < EPS_BOUNDS and vx < 0:
                            reasons.append('hitting left wall')
                        if abs(ex - x_bounds_loc[1]) < EPS_BOUNDS and vx > 0:
                            reasons.append('hitting right wall')
                        if abs(ey - y_bounds_loc[0]) < EPS_BOUNDS and vy < 0:
                            reasons.append('hitting bottom wall')
                candidates_debug[key] = {
                    'dist': dist, 'est_pos': (ex, ey),
                    'reasons': reasons,
                    'selected': key == selected_idx,
                    'vy_unit': vy_unit,
                    'vx': vx, 'vy': vy,
                }

        ridden     = agent_data['ridden_asteroid']
        message    = None
        valid_jump = None
        jump_dest  = None

        if selected_idx is not None:
            if selected_idx in true_next:
                asteroid_fut = true_next[selected_idx]
                double_jump  = (ridden is None)
                if ridden is not None and ridden in true_next:
                    agent_fut = true_next[ridden]
                elif ridden is None:
                    agent_fut = (x_start, y_start)
                else:
                    message = 'AGENT_OUT_OF_BOUNDS'
                    agent_fut = None

                if message is None:
                    jd   = agent.jump_distance * (2 if double_jump else 1)
                    dist = ((agent_fut[0] - asteroid_fut[0])**2 +
                            (agent_fut[1] - asteroid_fut[1])**2) ** 0.5
                    valid_jump = dist <= jd
                    jump_dest  = asteroid_fut
                    if valid_jump:
                        agent_data['ridden_asteroid'] = selected_idx
                        agent.set_agent_position(*asteroid_fut)
                    else:
                        message = 'JUMP_OUT_OF_RANGE'
            else:
                valid_jump = False
                message    = 'TARGET_OUT_OF_FIELD'
        else:
            if ridden is not None:
                if ridden in true_next:
                    agent.set_agent_position(*true_next[ridden])
                else:
                    if ridden not in outside_lrb.get(t + 1, set()):
                        message = 'SUCCESS'
                    else:
                        message = 'AGENT_OUT_OF_BOUNDS'

        # also check y-position for success
        ay = agent.get_agent_position()[1]
        if ay >= params['arena_y_bounds'][1] - 0.01:
            message = 'SUCCESS'

        steps.append({
            'true':             true_now,
            'true_next':        true_next,
            'noisy':            noisy,
            'estimates':        estimates,
            'agent_pos':        agent.get_agent_position(),
            'agent_pos_before': agent_pos_before,
            'ridden':           agent_data['ridden_asteroid'],
            'jump_to':          selected_idx,
            'jump_dest':        jump_dest,
            'valid_jump':       valid_jump,
            'message':          message,
            'start_pos':        (x_start, y_start),
            'candidates_debug': candidates_debug,
        })

        if message in ('SUCCESS', 'AGENT_OUT_OF_BOUNDS',
                       'JUMP_OUT_OF_RANGE', 'TARGET_OUT_OF_FIELD'):
            break

    return (x_start, y_start), steps

# ── coordinate mapping ────────────────────────────────────────────────────────

def world_to_screen(x, y, x_bounds, y_bounds, W, H, margin=40):
    wx = x_bounds[1] - x_bounds[0]
    wy = y_bounds[1] - y_bounds[0]
    sx = margin + (x - x_bounds[0]) / wx * (W - 2 * margin)
    sy = H - margin - (y - y_bounds[0]) / wy * (H - 2 * margin)
    return int(sx), int(sy)

def world_radius(r, x_bounds, W, margin=40):
    wx = x_bounds[1] - x_bounds[0]
    return max(1, int(r / wx * (W - 2 * margin)))

# ── colours ───────────────────────────────────────────────────────────────────

WHITE    = (255, 255, 255)
GREEN    = (80,  220, 80)
RED      = (220, 80,  80)
GREY     = (120, 120, 120)
YELLOW   = (240, 200, 60)
CYAN     = (60,  200, 240)
ORANGE   = (255, 160, 40)
MAGENTA  = (220, 80,  220)
BG       = (30,  30,  40)
TEXT     = (220, 220, 220)
DIMWHITE = (160, 160, 160)

# ── estimation frame ──────────────────────────────────────────────────────────

def draw_frame(screen, font, small_font, step_data, case_id, t, total_t,
               x_bounds, y_bounds, match_range, paused):
    W, H = screen.get_size()
    screen.fill(BG)

    def s(x, y):
        return world_to_screen(x, y, x_bounds, y_bounds, W, H)

    def sr(r):
        return world_radius(r, x_bounds, W)

    tl = s(x_bounds[0], y_bounds[1])
    br = s(x_bounds[1], y_bounds[0])
    pygame.draw.rect(screen, GREY, pygame.Rect(tl, (br[0]-tl[0], br[1]-tl[1])), 2)

    true_now   = step_data['true']
    est        = step_data['est']
    true_next  = step_data['true_next']
    matches    = step_data['matches']
    noisy      = step_data['noisy']
    match_r_px = sr(match_range)

    for idx, (x, y) in true_now.items():
        sx, sy = s(x, y)
        pygame.draw.circle(screen, WHITE, (sx, sy), 5)
        if idx in true_next:
            nx, ny = true_next[idx]
            nsx, nsy = s(nx, ny)
            pygame.draw.circle(screen, GREY, (nsx, nsy), match_r_px, 1)

    for idx, (x, y) in noisy.items():
        sx, sy = s(x, y)
        pygame.draw.circle(screen, YELLOW, (sx, sy), 3)

    ARROW_LEN = 20
    kf_vel = step_data.get('kf_vel', {})
    for idx, (x, y) in est.items():
        color = GREEN if matches.get(idx, False) else RED
        sx, sy = s(x, y)
        pygame.draw.circle(screen, color, (sx, sy), 4)
        if idx in true_now:
            tx, ty = true_now[idx]
            pygame.draw.line(screen, color, s(tx, ty), (sx, sy), 1)
        if idx in kf_vel:
            vx, vy = kf_vel[idx]
            mag = (vx*vx + vy*vy) ** 0.5
            if mag > 0:
                ux = vx / mag * ARROW_LEN
                uy = -vy / mag * ARROW_LEN
                tip = (sx + int(ux), sy + int(uy))
                pygame.draw.line(screen, CYAN, (sx, sy), tip, 2)
                angle = math.atan2(uy, ux)
                for da in (0.5, -0.5):
                    hx = tip[0] - int(math.cos(angle + da) * 6)
                    hy = tip[1] - int(math.sin(angle + da) * 6)
                    pygame.draw.line(screen, CYAN, tip, (hx, hy), 2)

    n_total = len(est)
    n_match = sum(1 for v in matches.values() if v)
    hud_lines = [
        f"[ESTIMATE] Case: {case_id}   t={t}/{total_t}   {'PAUSED' if paused else 'PLAYING'}",
        f"Asteroids: {len(true_now)}   Estimates: {n_total}   Matched: {n_match}/{n_total}",
        f"Match range: {match_range:.4f}",
        "",
        "SPACE/RIGHT=step  LEFT=back  A=auto  N=next  P=prev  Q=quit",
    ]
    for i, line in enumerate(hud_lines):
        surf = small_font.render(line, True, TEXT)
        screen.blit(surf, (8, 6 + i * 16))

    legend = [("● true pos", WHITE), ("→ velocity", CYAN),
              ("● noisy obs", YELLOW), ("● est (match)", GREEN),
              ("● est (miss)", RED), ("○ match range", GREY)]
    for i, (label, color) in enumerate(legend):
        surf = small_font.render(label, True, color)
        screen.blit(surf, (W - 160, 6 + i * 16))

    pygame.display.flip()

# ── jump frame ────────────────────────────────────────────────────────────────

def draw_jump_frame(screen, font, small_font, step_data, case_id, t, total_t,
                    x_bounds, y_bounds, match_range, jump_distance,
                    start_pos, paused, selected_ast_id=None):
    W, H = screen.get_size()
    screen.fill(BG)

    def s(x, y):
        return world_to_screen(x, y, x_bounds, y_bounds, W, H)

    def sr(r):
        return world_radius(r, x_bounds, W)

    # arena border
    tl = s(x_bounds[0], y_bounds[1])
    br = s(x_bounds[1], y_bounds[0])
    pygame.draw.rect(screen, GREY, pygame.Rect(tl, (br[0]-tl[0], br[1]-tl[1])), 2)

    # goal line (top of arena)
    goal_y_px = s(x_bounds[0], y_bounds[1])[1]
    pygame.draw.line(screen, GREEN, (tl[0], goal_y_px), (br[0], goal_y_px), 2)

    true_next  = step_data['true_next']
    noisy      = step_data['noisy']
    estimates  = step_data['estimates'] or {}
    agent_pos  = step_data['agent_pos']
    ridden     = step_data['ridden']
    jump_to    = step_data['jump_to']
    jump_dest  = step_data['jump_dest']
    valid_jump = step_data['valid_jump']
    message    = step_data['message']
    match_r_px = sr(match_range)

    # draw true next-timestep positions
    for idx, (x, y) in true_next.items():
        if idx == ridden:
            color = CYAN
            radius = 7
        elif idx == jump_to:
            color = YELLOW
            radius = 7
        else:
            color = WHITE
            radius = 5
        sx, sy = s(x, y)
        pygame.draw.circle(screen, color, (sx, sy), radius)
        pygame.draw.circle(screen, GREY, (sx, sy), match_r_px, 1)

    # noisy observations (dim yellow)
    for idx, (x, y) in noisy.items():
        sx, sy = s(x, y)
        pygame.draw.circle(screen, (180, 150, 30), (sx, sy), 3)

    # estimates
    for idx, (xe, ye) in estimates.items():
        if idx in true_next:
            xt, yt = true_next[idx]
            is_match = distance_formula((xe, ye), (xt, yt)) <= match_range
            color = GREEN if is_match else RED
        else:
            color = RED
        sx, sy = s(xe, ye)
        pygame.draw.circle(screen, color, (sx, sy), 4)

    # jump line from current agent position to intended dest
    if jump_dest is not None:
        dep = step_data['agent_pos']
        line_color = GREEN if valid_jump else RED
        pygame.draw.line(screen, line_color, s(*dep), s(*jump_dest), 2)

    # start position marker (if agent not yet riding)
    if ridden is None:
        sx, sy = s(*start_pos)
        pygame.draw.circle(screen, ORANGE, (sx, sy), 6)

    # agent position with jump-distance ring
    ax, ay_ = agent_pos
    asx, asy = s(ax, ay_)
    pygame.draw.circle(screen, ORANGE, (asx, asy), 6)
    pygame.draw.circle(screen, ORANGE, (asx, asy), sr(jump_distance), 1)

    # status overlay for terminal messages
    if message:
        status_colors = {
            'SUCCESS':             GREEN,
            'AGENT_OUT_OF_BOUNDS': RED,
            'JUMP_OUT_OF_RANGE':   RED,
            'TARGET_OUT_OF_FIELD': RED,
        }
        color = status_colors.get(message, TEXT)
        surf = font.render(message, True, color)
        screen.blit(surf, (W // 2 - surf.get_width() // 2, H // 2 - 10))

    # HUD
    riding_str = f"riding #{ridden}" if ridden else "not riding"
    jump_str   = f"-> #{jump_to}" if jump_to is not None else "no jump"
    hud_lines = [
        f"[JUMP] Case: {case_id}   t={t}/{total_t}   {'PAUSED' if paused else 'PLAYING'}",
        f"{riding_str}   {jump_str}   valid={valid_jump}",
        f"Jump dist: {jump_distance:.3f}   Match range: {match_range:.4f}",
        "",
        "SPACE/RIGHT=step  LEFT=back  A=auto  N=next  P=prev  Q=quit   CLICK=asteroid info",
    ]
    for i, line in enumerate(hud_lines):
        surf = small_font.render(line, True, TEXT)
        screen.blit(surf, (8, 6 + i * 16))

    legend = [
        ("● asteroid (next t)", WHITE),
        ("● ridden asteroid",   CYAN),
        ("● jump target",       YELLOW),
        ("● agent + jump ring", ORANGE),
        ("● est (match)",       GREEN),
        ("● est (miss)",        RED),
        ("― goal line",         GREEN),
    ]
    for i, (label, color) in enumerate(legend):
        surf = small_font.render(label, True, color)
        screen.blit(surf, (W - 190, 6 + i * 16))

    # ── click-to-inspect tooltip ──────────────────────────────────────────────
    candidates_debug = step_data.get('candidates_debug', {})
    agent_before = step_data.get('agent_pos_before', step_data['agent_pos'])
    if selected_ast_id is not None and selected_ast_id in candidates_debug:
        info = candidates_debug[selected_ast_id]
        ex, ey = info['est_pos']
        sx, sy = s(ex, ey)
        dy = ey - agent_before[1]
        reasons = ', '.join(info['reasons']) if info['reasons'] else 'valid candidate'
        if info['selected']:
            tip_color = GREEN
            status = 'SELECTED'
        elif not info['reasons']:
            tip_color = YELLOW
            status = 'valid (not chosen)'
        else:
            tip_color = RED
            status = 'REJECTED'
        vy_unit = info.get('vy_unit', 0.0)
        vx      = info.get('vx', 0.0)
        vy      = info.get('vy', 0.0)
        lines = [
            f"Asteroid #{selected_ast_id}  {status}",
            f"dist={info['dist']:.3f}   Δy={dy:+.3f}",
            f"vel=({vx:+.3f}, {vy:+.3f})  vy_unit={vy_unit:+.3f}",
            f"reason: {reasons}",
        ]
        line_surfs = [small_font.render(ln, True, tip_color) for ln in lines]
        tw = max(ls.get_width() for ls in line_surfs) + 12
        th = len(line_surfs) * 17 + 8
        tx = min(sx + 12, W - tw - 4)
        ty = max(sy - th - 4, 4)
        pygame.draw.rect(screen, (40, 40, 55), (tx - 2, ty - 2, tw + 4, th + 4))
        pygame.draw.rect(screen, tip_color, (tx - 2, ty - 2, tw + 4, th + 4), 1)
        for i, surf in enumerate(line_surfs):
            screen.blit(surf, (tx + 4, ty + 4 + i * 17))
        # highlight the asteroid dot with a ring
        pygame.draw.circle(screen, tip_color, (sx, sy), 9, 2)

    pygame.display.flip()

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    jump_mode = '--jump' in sys.argv

    start_case = None
    if '--case' in sys.argv:
        idx = sys.argv.index('--case')
        if idx + 1 < len(sys.argv):
            try:
                start_case = int(sys.argv[idx + 1])
            except ValueError:
                print(f"Invalid --case value: {sys.argv[idx + 1]}")
                return

    pygame.init()
    W, H = 900, 700
    screen = pygame.display.set_mode((W, H), pygame.RESIZABLE)
    title  = "Hopscotch – Jump Debugger" if jump_mode else "Hopscotch – Estimation Debugger"
    pygame.display.set_caption(title)
    font       = pygame.font.SysFont('monospace', 16)
    small_font = pygame.font.SysFont('monospace', 14)
    clock = pygame.time.Clock()

    case_ids = JUMP_CASE_IDS if jump_mode else ESTIMATE_CASE_IDS
    if not case_ids:
        print("No cases found for this mode.")
        return

    case_idx = 0
    if start_case is not None:
        target_id = f'case{start_case}'
        if target_id in case_ids:
            case_idx = case_ids.index(target_id)
        else:
            print(f"Case {start_case} not found in {'jump' if jump_mode else 'estimate'} cases.")
            return
    case_steps    = None
    t             = 0
    auto_play     = False
    fps           = 30
    start_pos     = None
    selected_ast_id = None

    def load_case(idx):
        cid    = case_ids[idx]
        params = CASES[cid]
        print(f"Loading {cid}...", end=' ', flush=True)
        if jump_mode:
            sp, steps = run_jump_steps(params)
            print(f"done ({len(steps)} steps)")
            return cid, params, steps, sp
        else:
            time_limit = params['_args']['time_limit']
            _, shower, _, arena = build_simulation(cid)
            all_data = precompute_true_locations(shower, time_limit)
            steps    = run_estimates(params, all_data, time_limit)
            print(f"done ({len(steps)} steps)")
            return cid, params, steps, None

    cid, params, case_steps, start_pos = load_case(case_idx)
    t = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False

                elif event.key in (pygame.K_SPACE, pygame.K_RIGHT):
                    auto_play = False
                    t = min(t + 1, len(case_steps) - 1)

                elif event.key == pygame.K_LEFT:
                    auto_play = False
                    t = max(t - 1, 0)

                elif event.key == pygame.K_a:
                    auto_play = not auto_play

                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS):
                    fps = min(fps * 2, 240)

                elif event.key == pygame.K_MINUS:
                    fps = max(fps // 2, 1)

                elif event.key == pygame.K_n:
                    case_idx = (case_idx + 1) % len(case_ids)
                    cid, params, case_steps, start_pos = load_case(case_idx)
                    t = 0
                    auto_play = False
                    selected_ast_id = None

                elif event.key == pygame.K_p:
                    case_idx = (case_idx - 1) % len(case_ids)
                    cid, params, case_steps, start_pos = load_case(case_idx)
                    t = 0
                    auto_play = False
                    selected_ast_id = None

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and jump_mode:
                mx, my = event.pos
                W_cur, H_cur = screen.get_size()
                step = case_steps[t]
                candidates = step.get('candidates_debug', {})
                best_id, best_d = None, float('inf')
                for kid, info in candidates.items():
                    ex, ey = info['est_pos']
                    sx, sy = world_to_screen(
                        ex, ey,
                        params['arena_x_bounds'], params['arena_y_bounds'],
                        W_cur, H_cur)
                    d = ((sx - mx)**2 + (sy - my)**2) ** 0.5
                    if d < 15 and d < best_d:
                        best_d, best_id = d, kid
                selected_ast_id = None if best_id is None or best_id == selected_ast_id else best_id

        if auto_play:
            t += 1
            if t >= len(case_steps):
                t = len(case_steps) - 1
                auto_play = False

        if jump_mode:
            draw_jump_frame(
                screen, font, small_font,
                case_steps[t], cid, t, len(case_steps),
                params['arena_x_bounds'], params['arena_y_bounds'],
                params['asteroid_match_range'],
                params['agent_jump_distance'],
                start_pos,
                paused=not auto_play,
                selected_ast_id=selected_ast_id,
            )
        else:
            draw_frame(
                screen, font, small_font,
                case_steps[t], cid, t, len(case_steps),
                params['arena_x_bounds'], params['arena_y_bounds'],
                params['asteroid_match_range'],
                paused=not auto_play,
            )

        clock.tick(fps)

    pygame.quit()

if __name__ == '__main__':
    main()
