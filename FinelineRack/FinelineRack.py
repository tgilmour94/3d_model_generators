"""
Fineline Applicator Bottle Rack - parametric Fusion 360 script
---------------------------------------------------------------
Builds two components in the active design, positioned as assembled:

  Base : grid of interlocking rings with a floor (cups), plus four
         quarter-arc posts rising from the outer side of the corner cells.
  Top  : matching ring grid with a flared lead-in on every hole, plus
         arc-shaped sleeves at the corners that drop over the posts
         to locate the top piece (glue or friction fit).

All dimensions below are in INCHES. Edit, then run in a new design.
"""

import adsk.core, adsk.fusion, traceback, math

# ============================ PARAMETERS (inches) ============================
ROWS = 2
COLS = 3
INNER_D = 0.875          # hole diameter (bottle is 0.845 -> 0.010 radial clearance)
WALL = 0.12              # ring wall thickness; spacing is INNER_D + WALL (shared walls)

FLOOR = 0.08             # base floor thickness
CUP_DEPTH = 0.75         # depth of the cups above the floor

TOTAL_HEIGHT = 3.5      # assembled height: bottom of base to top face of top piece
                         # (post height = TOTAL_HEIGHT - FLOOR - CUP_DEPTH - TOP_HEIGHT)
POST_ARC = 90            # degrees of arc each corner post spans (90 = quarter circle)

TOP_HEIGHT = 0.5         # height of the top ring piece
FLARE_WIDTH = 0.04       # extra radius at the top of each hole (auto-capped below WALL/2)
FLARE_DEPTH = 0.20       # how far down the flare runs (must be < TOP_HEIGHT)

SLEEVE_WALL = 0.08       # thickness of the locating sleeves on the top piece
SLEEVE_DEPTH = 0.25      # how far the sleeves hang down over the posts
SLEEVE_EXTRA_ARC = 20    # sleeves span POST_ARC + this many degrees
JOINT_CLEARANCE = 0.008  # radial gap between post and sleeve (~0.2 mm)

# Press-fit tenon: the top of each post narrows into a tenon that presses into
# a matching arc slot in the underside of the top piece.
TENON_INSET = 0.05       # post wall removed on the bore side to form the tenon
                         # (tenon thickness = WALL - TENON_INSET; this is also the
                         #  wall left between the slot and the bottle hole)
TENON_DEPTH = 0.20       # tenon length = slot depth (keep below TOP_HEIGHT - FLARE_DEPTH)
PRESS_FIT = 0.004        # total interference across tenon thickness (~0.1 mm)
                         # 0 = line-to-line, negative = slip fit for gluing
SLOT_END_GAP = 0.01      # clearance at each arc end of the slot, so it only grips on the faces
# ============================================================================

IN = 2.54  # Fusion's internal unit is cm


def v(inches):
    return adsk.core.ValueInput.createByReal(inches * IN)


def P(x, y):
    return adsk.core.Point3D.create(x * IN, y * IN, 0)


def post_height():
    return TOTAL_HEIGHT - FLOOR - CUP_DEPTH - TOP_HEIGHT


def cell_centers():
    pitch = INNER_D + WALL
    cells = {}
    for r in range(ROWS):
        for c in range(COLS):
            cells[(r, c)] = ((c - (COLS - 1) / 2.0) * pitch,
                             (r - (ROWS - 1) / 2.0) * pitch)
    return cells


def corners():
    # (cell, outward-facing angle in degrees)
    return [((0, 0), 225), ((0, COLS - 1), 315),
            ((ROWS - 1, 0), 135), ((ROWS - 1, COLS - 1), 45)]


def plane_at(comp, z):
    if abs(z) < 1e-9:
        return comp.xYConstructionPlane
    inp = comp.constructionPlanes.createInput()
    inp.setByOffset(comp.xYConstructionPlane, v(z))
    pl = comp.constructionPlanes.add(inp)
    pl.isLightBulbOn = False
    return pl


def sketch_at(comp, z, name):
    sk = comp.sketches.add(plane_at(comp, z))
    sk.name = name
    return sk


def centroid(prof):
    c = prof.areaProperties(
        adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).centroid
    return c.x / IN, c.y / IN


def nearest_profile(sk, x, y):
    best, bd = None, 1e9
    for pr in sk.profiles:
        cx, cy = centroid(pr)
        d = math.hypot(cx - x, cy - y)
        if d < bd:
            best, bd = pr, d
    return best


def extrude(comp, profs, dist, op, body=None, downward=False):
    ext = comp.features.extrudeFeatures
    coll = adsk.core.ObjectCollection.create()
    for p in profs:
        coll.add(p)
    inp = ext.createInput(coll, op)
    direction = (adsk.fusion.ExtentDirections.NegativeExtentDirection if downward
                 else adsk.fusion.ExtentDirections.PositiveExtentDirection)
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(v(dist)), direction)
    if body is not None:
        inp.participantBodies = [body]
    return ext.add(inp)


def ring_block(comp, z0, height, name):
    """Solid union of all outer ring circles (six-pack outline), no holes yet."""
    sk = sketch_at(comp, z0, name)
    ro = INNER_D / 2 + WALL
    cells = list(cell_centers().values())
    for x, y in cells:
        sk.sketchCurves.sketchCircles.addByCenterRadius(P(x, y), ro * IN)
    keep = []
    for pr in sk.profiles:
        cx, cy = centroid(pr)
        # skip the open gaps between groups of four rings
        if any(math.hypot(cx - x, cy - y) < ro for x, y in cells):
            keep.append(pr)
    feat = extrude(comp, keep, height, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    return feat.bodies.item(0)


def cut_holes(comp, body, z0, depth, name):
    sk = sketch_at(comp, z0, name)
    for x, y in cell_centers().values():
        sk.sketchCurves.sketchCircles.addByCenterRadius(P(x, y), INNER_D / 2 * IN)
    extrude(comp, list(sk.profiles), depth,
            adsk.fusion.FeatureOperations.CutFeatureOperation, body)


def draw_sector(sk, cx, cy, r_in, r_out, mid_deg, span_deg):
    """Closed annular sector centred on (cx, cy)."""
    a0 = math.radians(mid_deg - span_deg / 2.0)
    sweep = math.radians(span_deg)
    arcs = sk.sketchCurves.sketchArcs
    lines = sk.sketchCurves.sketchLines
    a_in = arcs.addByCenterStartSweep(
        P(cx, cy), P(cx + r_in * math.cos(a0), cy + r_in * math.sin(a0)), sweep)
    a_out = arcs.addByCenterStartSweep(
        P(cx, cy), P(cx + r_out * math.cos(a0), cy + r_out * math.sin(a0)), sweep)
    lines.addByTwoPoints(a_in.startSketchPoint, a_out.startSketchPoint)
    lines.addByTwoPoints(a_in.endSketchPoint, a_out.endSketchPoint)


def new_component(root, name):
    """Use a sub-component when the document allows it; Fusion's single-component
    'Part Design' documents don't, so fall back to separate bodies in the root."""
    try:
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = name
        return occ.component
    except RuntimeError:
        return root


def build_base(comp):
    cells = cell_centers()
    base_h = FLOOR + CUP_DEPTH
    body = ring_block(comp, 0, base_h, 'Base rings')
    body.name = 'Base'
    cut_holes(comp, body, FLOOR, CUP_DEPTH + 0.01, 'Base cups')

    ri = INNER_D / 2
    sk = sketch_at(comp, base_h, 'Corner posts')
    for cell, ang in corners():
        x, y = cells[cell]
        draw_sector(sk, x, y, ri, ri + WALL, ang, POST_ARC)
    extrude(comp, list(sk.profiles), post_height(),
            adsk.fusion.FeatureOperations.JoinFeatureOperation, body)

    # Tenon on top of each post (outer part of the wall; the inner part is the shoulder)
    if TENON_DEPTH > 0:
        skt = sketch_at(comp, base_h + post_height(), 'Post tenons')
        for cell, ang in corners():
            x, y = cells[cell]
            draw_sector(skt, x, y, ri + TENON_INSET, ri + WALL, ang, POST_ARC)
        extrude(comp, list(skt.profiles), TENON_DEPTH,
                adsk.fusion.FeatureOperations.JoinFeatureOperation, body)
    return body


def build_top(comp):
    cells = cell_centers()
    z0 = TOTAL_HEIGHT - TOP_HEIGHT
    ri = INNER_D / 2
    ro = ri + WALL
    join = adsk.fusion.FeatureOperations.JoinFeatureOperation

    body = ring_block(comp, z0, TOP_HEIGHT, 'Top rings')
    body.name = 'Top'
    cut_holes(comp, body, z0 - 0.01, TOP_HEIGHT + 0.02, 'Top holes')

    span = POST_ARC + SLEEVE_EXTRA_ARC
    r1 = ro + JOINT_CLEARANCE
    r2 = r1 + SLEEVE_WALL

    # Connector: bridges the ring wall out to the sleeve, full top-piece height
    skc = sketch_at(comp, z0, 'Sleeve connectors')
    for cell, ang in corners():
        x, y = cells[cell]
        draw_sector(skc, x, y, ri + WALL / 2, r2, ang, span)
    extrude(comp, list(skc.profiles), TOP_HEIGHT, join, body)

    # Sleeve: hangs down outside the post, leaving JOINT_CLEARANCE
    if SLEEVE_DEPTH > 0:
        sks = sketch_at(comp, z0, 'Sleeves')
        for cell, ang in corners():
            x, y = cells[cell]
            draw_sector(sks, x, y, r1, r2, ang, span)
        extrude(comp, list(sks.profiles), SLEEVE_DEPTH, join, body, downward=True)

    # Press-fit slot: tenon size minus PRESS_FIT across its thickness,
    # plus SLOT_END_GAP at each arc end
    if TENON_DEPTH > 0:
        s_in = ri + TENON_INSET + PRESS_FIT / 2
        s_out = ro - PRESS_FIT / 2
        r_mid = (s_in + s_out) / 2
        slot_span = POST_ARC + 2 * math.degrees(SLOT_END_GAP / r_mid)
        skl = sketch_at(comp, z0 - 0.01, 'Tenon slots')
        for cell, ang in corners():
            x, y = cells[cell]
            draw_sector(skl, x, y, s_in, s_out, ang, slot_span)
        extrude(comp, list(skl.profiles), TENON_DEPTH + 0.01 + 0.015,
                adsk.fusion.FeatureOperations.CutFeatureOperation, body)

    # Flared lead-in on each hole (lofted cone cut)
    w = min(FLARE_WIDTH, WALL / 2 - 0.015)
    depth = min(FLARE_DEPTH, TOP_HEIGHT - 0.02)
    if w > 0 and depth > 0:
        z_face = z0 + TOP_HEIGHT
        skb = sketch_at(comp, z_face - depth, 'Flare bottom')
        skt = sketch_at(comp, z_face, 'Flare top')
        for x, y in cells.values():
            skb.sketchCurves.sketchCircles.addByCenterRadius(P(x, y), ri * IN)
            skt.sketchCurves.sketchCircles.addByCenterRadius(P(x, y), (ri + w) * IN)
        lofts = comp.features.loftFeatures
        for x, y in cells.values():
            li = lofts.createInput(adsk.fusion.FeatureOperations.CutFeatureOperation)
            li.loftSections.add(nearest_profile(skb, x, y))
            li.loftSections.add(nearest_profile(skt, x, y))
            try:
                li.participantBodies = [body]
            except Exception:
                pass
            lofts.add(li)
    return body, w


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Open or create a Design first.')
            return
        if ROWS < 1 or COLS < 1 or WALL <= 0 or INNER_D <= 0:
            ui.messageBox('ROWS/COLS must be >= 1 and WALL/INNER_D > 0.')
            return
        if TENON_DEPTH > 0 and not (0 < TENON_INSET < WALL - 0.03):
            ui.messageBox('TENON_INSET must be between 0 and WALL - 0.03 in.')
            return
        if TENON_DEPTH > 0 and TENON_DEPTH + 0.015 >= TOP_HEIGHT - min(FLARE_DEPTH, TOP_HEIGHT - 0.02):
            ui.messageBox('TENON_DEPTH is too deep: the slot would break into the flare. '
                          'Reduce TENON_DEPTH or FLARE_DEPTH, or raise TOP_HEIGHT.')
            return
        if post_height() <= 0:
            ui.messageBox('TOTAL_HEIGHT must be greater than FLOOR + CUP_DEPTH + TOP_HEIGHT '
                          '({:.3f} in).'.format(FLOOR + CUP_DEPTH + TOP_HEIGHT))
            return
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        root = design.rootComponent

        build_base(new_component(root, 'Rack Base'))
        _, flare_used = build_top(new_component(root, 'Rack Top'))

        pitch = INNER_D + WALL
        ro = INNER_D / 2 + WALL
        width = (COLS - 1) * pitch + 2 * ro
        depth = (ROWS - 1) * pitch + 2 * ro
        total_h = TOTAL_HEIGHT
        msg = ('Rack built.\n\n'
               'Ring footprint: {:.3f} x {:.3f} in (sleeves add ~{:.2f} in at corners)\n'
               'Assembled height: {:.3f} in (posts {:.3f} in)\n'
               'Top of top ring above floor: {:.3f} in\n'
               'Flare radius used: {:.3f} in').format(
                   width, depth, JOINT_CLEARANCE + SLEEVE_WALL, total_h, post_height(),
                   total_h - FLOOR, flare_used)
        if flare_used < FLARE_WIDTH:
            msg += '\n(FLARE_WIDTH was capped by WALL - raise WALL for a bigger flare)'
        ui.messageBox(msg)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
