import pygame
import math
import sys

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("[WARN] pyserial not installed — running in simulation-only mode")
    print("       Install with: pip install pyserial")

# ─── Configuration ───────────────────────────────────────────────────
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200

# Real arm link lengths (cm)
L1 = 11.0
L2 = 8.5
L3 = 7.5

# Colors
BG_COLOR       = (30, 30, 40)
GRID_COLOR     = (50, 50, 60)
ARM_COLOR      = (220, 220, 230)
JOINT_COLOR    = (80, 180, 255)
EE_COLOR       = (255, 100, 80)
TARGET_COLOR   = (100, 255, 120)
BASE_COLOR     = (200, 160, 60)
TEXT_COLOR     = (180, 180, 190)
REACH_COLOR    = (60, 60, 75)

# ─── Pygame init ─────────────────────────────────────────────────────
pygame.init()
pygame.font.init()

infoObject = pygame.display.Info()
WINDOW_W = infoObject.current_w
WINDOW_H = infoObject.current_h

window = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
pygame.display.set_caption("3-DOF IK — Servo Controller")

font = pygame.font.SysFont("monospace", 16)
font_large = pygame.font.SysFont("monospace", 20, bold=True)

SCALE = int(min(WINDOW_W, WINDOW_H) / ((L1 + L2 + L3) * 2.2))

BASE_X = WINDOW_W // 2
BASE_Y = WINDOW_H // 2 + int(WINDOW_H * 0.2)

# ─── Serial setup ───────────────────────────────────────────────────
ser = None
if SERIAL_AVAILABLE:
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)
        print(f"[OK] Serial connected: {SERIAL_PORT} @ {SERIAL_BAUD}")
    except serial.SerialException as e:
        print(f"[WARN] Could not open {SERIAL_PORT}: {e}")
        print("       Running in simulation-only mode")
        ser = None


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


class Slider:
    def __init__(self, x, y, w, h, min_val, max_val, initial_val, label):
        self.rect = pygame.Rect(x, y, w, h)
        self.min_val = min_val
        self.max_val = max_val
        self.val = initial_val
        self.label = label
        self.dragging = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1 and self.rect.collidepoint(event.pos):
                self.dragging = True
                self.update_val(event.pos[0])
                return True

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self.dragging = False

        elif event.type == pygame.MOUSEMOTION:
            if self.dragging:
                self.update_val(event.pos[0])
                return True

        return False

    def update_val(self, mouse_x):
        ratio = (mouse_x - self.rect.x) / self.rect.w
        ratio = clamp(ratio, 0.0, 1.0)
        self.val = self.min_val + ratio * (self.max_val - self.min_val)

    def draw(self, surface):
        pygame.draw.rect(surface, (100, 100, 110), self.rect, border_radius=4)

        handle_x = self.rect.x + (
            (self.val - self.min_val) /
            (self.max_val - self.min_val)
        ) * self.rect.w

        handle_rect = pygame.Rect(
            handle_x - 5,
            self.rect.y - 4,
            10,
            self.rect.h + 8
        )

        pygame.draw.rect(surface, (200, 200, 200), handle_rect, border_radius=4)

        lbl_surf = font.render(
            f"{self.label}: {self.val:.0f}°",
            True,
            TEXT_COLOR
        )

        surface.blit(lbl_surf, (self.rect.x, self.rect.y - 22))


class Checkbox:
    def __init__(self, x, y, label, checked=False):
        self.rect = pygame.Rect(x, y, 20, 20)
        self.label = label
        self.checked = checked

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.checked = not self.checked
                return True
        return False

    def draw(self, surface):
        pygame.draw.rect(surface, (200, 200, 200), self.rect, 2)

        if self.checked:
            pygame.draw.line(
                surface,
                TARGET_COLOR,
                (self.rect.x + 4, self.rect.y + 10),
                (self.rect.x + 8, self.rect.y + 16),
                3
            )

            pygame.draw.line(
                surface,
                TARGET_COLOR,
                (self.rect.x + 8, self.rect.y + 16),
                (self.rect.x + 17, self.rect.y + 4),
                3
            )

        lbl = font.render(self.label, True, TEXT_COLOR)
        surface.blit(lbl, (self.rect.right + 10, self.rect.y))


def _solve_config(wx, wy, D2, phi, elbow_sign):
    cos_t2 = clamp(
        (D2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2),
        -1.0,
        1.0
    )

    sin_t2 = elbow_sign * math.sqrt(max(0.0, 1.0 - cos_t2 * cos_t2))
    t2 = math.atan2(sin_t2, cos_t2)

    t1 = (
        math.atan2(wy, wx)
        - math.atan2(L2 * sin_t2, L1 + L2 * cos_t2)
    )

    t3 = phi - t1 - t2

    sh = math.degrees(t1)
    el = math.degrees(t2) + 90.0
    wr = math.degrees(t3) + 90.0

    if 0 <= sh <= 180 and 0 <= el <= 180 and 0 <= wr <= 180:
        elbow_y = L1 * math.sin(t1)
        return (sh, el, wr, elbow_y)

    return None


def solve_ik(tx, ty, phi):
    wx = tx - L3 * math.cos(phi)
    wy = ty - L3 * math.sin(phi)

    D2 = wx * wx + wy * wy
    D = math.sqrt(D2)

    if D > L1 + L2:
        scale = (L1 + L2 - 0.01) / D
        wx *= scale
        wy *= scale
        D2 = wx * wx + wy * wy

    elif D < abs(L1 - L2):
        return None

    solutions = []

    for sign in [+1, -1]:
        result = _solve_config(wx, wy, D2, phi, sign)

        if result is not None:
            solutions.append(result)

    if not solutions:
        cos_t2 = clamp(
            (D2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2),
            -1.0,
            1.0
        )

        sin_t2 = -math.sqrt(max(0.0, 1.0 - cos_t2 * cos_t2))
        t2 = math.atan2(sin_t2, cos_t2)

        t1 = (
            math.atan2(wy, wx)
            - math.atan2(L2 * sin_t2, L1 + L2 * cos_t2)
        )

        t3 = phi - t1 - t2

        sh_fallback = clamp(math.degrees(t1), 0.0, 180.0)
        el_fallback = clamp(math.degrees(t2) + 90.0, 0.0, 180.0)
        wr_fallback = clamp(math.degrees(t3) + 90.0, 0.0, 180.0)

        return (
            sh_fallback,
            el_fallback,
            wr_fallback,
            False
        )

    solutions.sort(key=lambda s: s[3])
    best = solutions[0]

    return (best[0], best[1], best[2], True)


def solve_ik_auto(tx, ty):
    valid_solutions = []

    for phi_deg in range(0, 360, 5):
        phi = math.radians(phi_deg)

        result = solve_ik(tx, ty, phi)

        if result is None:
            continue

        sh, el, wr, valid = result

        if not valid:
            continue

        t1 = math.radians(sh)
        t2 = math.radians(el - 90.0)
        t3 = math.radians(wr - 90.0)

        x = (
            L1 * math.cos(t1)
            + L2 * math.cos(t1 + t2)
            + L3 * math.cos(t1 + t2 + t3)
        )

        y = (
            L1 * math.sin(t1)
            + L2 * math.sin(t1 + t2)
            + L3 * math.sin(t1 + t2 + t3)
        )

        err = math.sqrt((x - tx) ** 2 + (y - ty) ** 2)
        elbow_y = L1 * math.sin(t1)

        valid_solutions.append({
            'sh': sh,
            'el': el,
            'wr': wr,
            'valid': True,
            'err': err,
            'ey': elbow_y
        })

    if valid_solutions:
        valid_solutions.sort(
            key=lambda s: (round(s['err'], 1), s['ey'])
        )

        best = valid_solutions[0]

        return (
            best['sh'],
            best['el'],
            best['wr'],
            True
        )

    phi = (
        math.atan2(ty, tx)
        if (tx != 0 or ty != 0)
        else math.radians(90)
    )

    result = solve_ik(tx, ty, phi)

    return result if result else (90.0, 90.0, 90.0, False)


def world_to_screen(x, y):
    return (
        int(BASE_X + x * SCALE),
        int(BASE_Y - y * SCALE)
    )


def screen_to_world(sx, sy):
    return (
        (sx - BASE_X) / SCALE,
        (BASE_Y - sy) / SCALE
    )


def draw_grid(surface):
    max_reach = L1 + L2 + L3

    cx, cy = BASE_X, BASE_Y

    pygame.draw.circle(
        surface,
        REACH_COLOR,
        (cx, cy),
        int(max_reach * SCALE),
        1
    )

    for i in range(-30, 31, 5):
        sx = BASE_X + i * SCALE
        sy = BASE_Y - i * SCALE

        if 0 <= sx <= WINDOW_W:
            pygame.draw.line(
                surface,
                GRID_COLOR,
                (sx, 0),
                (sx, WINDOW_H),
                1
            )

        if 0 <= sy <= WINDOW_H:
            pygame.draw.line(
                surface,
                GRID_COLOR,
                (0, sy),
                (WINDOW_W, sy),
                1
            )

    pygame.draw.line(
        surface,
        (70, 70, 80),
        (0, BASE_Y),
        (WINDOW_W, BASE_Y),
        1
    )

    pygame.draw.line(
        surface,
        (70, 70, 80),
        (BASE_X, 0),
        (BASE_X, WINDOW_H),
        1
    )


def draw_arm(surface, theta1_rad, theta2_rad, theta3_rad):
    x0, y0 = 0.0, 0.0

    x1 = x0 + L1 * math.cos(theta1_rad)
    y1 = y0 + L1 * math.sin(theta1_rad)

    x2 = x1 + L2 * math.cos(theta1_rad + theta2_rad)
    y2 = y1 + L2 * math.sin(theta1_rad + theta2_rad)

    x3 = x2 + L3 * math.cos(theta1_rad + theta2_rad + theta3_rad)
    y3 = y2 + L3 * math.sin(theta1_rad + theta2_rad + theta3_rad)

    s0 = world_to_screen(x0, y0)
    s1 = world_to_screen(x1, y1)
    s2 = world_to_screen(x2, y2)
    s3 = world_to_screen(x3, y3)

    pygame.draw.line(surface, ARM_COLOR, s0, s1, 6)
    pygame.draw.line(surface, ARM_COLOR, s1, s2, 5)
    pygame.draw.line(surface, ARM_COLOR, s2, s3, 4)

    pygame.draw.circle(surface, BASE_COLOR, s0, 10)
    pygame.draw.circle(surface, JOINT_COLOR, s0, 7)
    pygame.draw.circle(surface, JOINT_COLOR, s1, 6)
    pygame.draw.circle(surface, JOINT_COLOR, s2, 5)
    pygame.draw.circle(surface, EE_COLOR, s3, 5)

    return s3


def draw_hud(surface, sh_deg, el_deg, wr_deg, valid, target_world):
    y = 15

    lines = [
        f"Shoulder: {sh_deg:6.1f}°",
        f"Elbow:    {el_deg:6.1f}°",
        f"Wrist:    {wr_deg:6.1f}°",
        f"Target:   ({target_world[0]:5.1f}, {target_world[1]:5.1f}) cm",
        f"Serial:   {'Connected' if ser else 'Disconnected'}",
    ]

    panel_w = 250
    panel_h = len(lines) * 22 + 16

    panel = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
    panel.fill((0, 0, 0, 140))

    surface.blit(panel, (10, 8))

    for line in lines:
        color = TEXT_COLOR if valid else (255, 80, 80)

        text_surf = font.render(line, True, color)
        surface.blit(text_surf, (18, y))

        y += 22

    if not valid:
        warn = font_large.render(
            "OUT OF RANGE",
            True,
            (255, 80, 80)
        )

        surface.blit(
            warn,
            (
                WINDOW_W // 2 - warn.get_width() // 2,
                WINDOW_H - 40
            )
        )

    instr = font.render(
        "Left-click: move end effector",
        True,
        (120, 120, 130)
    )

    surface.blit(
        instr,
        (
            WINDOW_W - instr.get_width() - 15,
            WINDOW_H - 25
        )
    )


def send_angles(sh, el, wr, w_roll, grip):
    if ser is None:
        return

    try:
        msg = (
            f"{sh:.1f},"
            f"{el:.1f},"
            f"{wr:.1f},"
            f"{w_roll:.1f},"
            f"{grip:.1f}\n"
        )

        ser.write(msg.encode())

    except serial.SerialException:
        pass


# ─── Main loop ───────────────────────────────────────────────────────
clock = pygame.time.Clock()
FPS = 60

target_x = 0.0
target_y = L1 + L2 + L3 - 2.0

# ─── IK Sliders ───
slider_roll = Slider(
    WINDOW_W - 350,
    40,
    250,
    10,
    0.0,
    180.0,
    90.0,
    "Wrist Roll (P04)"
)

slider_grip = Slider(
    WINDOW_W - 350,
    100,
    250,
    10,
    0.0,
    180.0,
    90.0,
    "Gripper (P05)"
)

# ─── Manual Controls ───
cb_manual = Checkbox(
    WINDOW_W - 350,
    150,
    "Manual Mode Override"
)

manual_sh = Slider(
    WINDOW_W - 350,
    220,
    250,
    10,
    0.0,
    180.0,
    90.0,
    "Shoulder"
)

manual_el = Slider(
    WINDOW_W - 350,
    290,
    250,
    10,
    0.0,
    180.0,
    90.0,
    "Elbow"
)

manual_wr = Slider(
    WINDOW_W - 350,
    360,
    250,
    10,
    0.0,
    180.0,
    90.0,
    "Wrist"
)

manual_ro = Slider(
    WINDOW_W - 350,
    430,
    250,
    10,
    0.0,
    180.0,
    90.0,
    "Wrist Roll"
)

manual_gr = Slider(
    WINDOW_W - 350,
    500,
    250,
    10,
    0.0,
    180.0,
    90.0,
    "Gripper"
)

cur_sh, cur_el, cur_wr = 90.0, 90.0, 90.0
cur_valid = True

draw_t1 = math.radians(90)
draw_t2 = 0.0
draw_t3 = 0.0

running = True

while running:
    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False

        cb_manual.handle_event(event)

        if cb_manual.checked:
            manual_sh.handle_event(event)
            manual_el.handle_event(event)
            manual_wr.handle_event(event)
            manual_ro.handle_event(event)
            manual_gr.handle_event(event)

        else:
            slider_roll.handle_event(event)
            slider_grip.handle_event(event)

    # ── Mouse Input ──
    if not cb_manual.checked:

        if pygame.mouse.get_pressed()[0] and not (
            slider_roll.dragging or slider_grip.dragging
        ):
            mx, my = pygame.mouse.get_pos()
            target_x, target_y = screen_to_world(mx, my)

        result = solve_ik_auto(target_x, target_y)

        if result is not None:
            sh_deg, el_deg, wr_deg, valid = result

            cur_sh, cur_el, cur_wr = sh_deg, el_deg, wr_deg
            cur_valid = valid

            draw_t1 = math.radians(sh_deg)
            draw_t2 = math.radians(el_deg - 90.0)
            draw_t3 = math.radians(wr_deg - 90.0)

            if valid:
                send_angles(
                    sh_deg,
                    el_deg,
                    wr_deg,
                    slider_roll.val,
                    slider_grip.val
                )

    else:
        # ── Manual Slider Mode ──
        cur_sh = manual_sh.val
        cur_el = manual_el.val
        cur_wr = manual_wr.val

        w_roll = manual_ro.val
        grip = manual_gr.val

        cur_valid = True

        draw_t1 = math.radians(cur_sh)
        draw_t2 = math.radians(cur_el - 90.0)
        draw_t3 = math.radians(cur_wr - 90.0)

        send_angles(
            cur_sh,
            cur_el,
            cur_wr,
            w_roll,
            grip
        )

    # ── Draw ──
    window.fill(BG_COLOR)

    draw_grid(window)

    ts = world_to_screen(target_x, target_y)

    pygame.draw.circle(window, TARGET_COLOR, ts, 8, 2)

    pygame.draw.line(
        window,
        TARGET_COLOR,
        (ts[0] - 12, ts[1]),
        (ts[0] + 12, ts[1]),
        1
    )

    pygame.draw.line(
        window,
        TARGET_COLOR,
        (ts[0], ts[1] - 12),
        (ts[0], ts[1] + 12),
        1
    )

    draw_arm(window, draw_t1, draw_t2, draw_t3)

    draw_hud(
        window,
        cur_sh,
        cur_el,
        cur_wr,
        cur_valid,
        (target_x, target_y)
    )

    cb_manual.draw(window)

    if cb_manual.checked:

        manual_sh.draw(window)
        manual_el.draw(window)
        manual_wr.draw(window)
        manual_ro.draw(window)
        manual_gr.draw(window)

        s_surf = pygame.Surface((300, 140), pygame.SRCALPHA)
        s_surf.fill((0, 0, 0, 160))

        slider_roll.draw(window)
        slider_grip.draw(window)

        window.blit(s_surf, (WINDOW_W - 360, 20))

    else:

        r_surf = pygame.Surface((320, 380), pygame.SRCALPHA)
        r_surf.fill((0, 0, 0, 160))

        manual_sh.draw(window)
        manual_el.draw(window)
        manual_wr.draw(window)
        manual_ro.draw(window)
        manual_gr.draw(window)

        window.blit(r_surf, (WINDOW_W - 360, 190))

        slider_roll.draw(window)
        slider_grip.draw(window)

    pygame.display.update()
    clock.tick(FPS)

# ── Cleanup ──
if ser:
    ser.close()

pygame.quit()
sys.exit()
