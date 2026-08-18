import pcbnew
import os

# Create a new empty footprint file
fp = pcbnew.FOOTPRINT(None)
fp.SetReference("J1")

FINGER_W_MM   = 1.0
FINGER_GAP_MM = 0.5
FINGER_LEN_MM = 10.0
RAIL_W_MM     = 1.0
NUM_FINGERS   = 5
LAYER         = pcbnew.F_Cu

def mm(val):
    return pcbnew.FromMM(val)

pitch = FINGER_W_MM + FINGER_GAP_MM

def add_rect(fp, x1, y1, x2, y2, net_name=None):
    shape_cls = getattr(pcbnew, "PCB_SHAPE", None)
    rect = shape_cls(fp)
    rect.SetShape(pcbnew.SHAPE_T_RECT)
    rect.SetStart(pcbnew.VECTOR2I(mm(min(y1, y2)), mm(min(x1, x2))))
    rect.SetEnd(pcbnew.VECTOR2I(mm(max(y1, y2)), mm(max(x1, x2))))
    rect.SetLayer(LAYER)
    if hasattr(rect, "SetFilled"):
        rect.SetFilled(True)
    if hasattr(rect, "SetWidth"):
        rect.SetWidth(0)
    fp.Add(rect)

rail_a_y = 0.0
rail_b_y = FINGER_LEN_MM + RAIL_W_MM
total_width = (NUM_FINGERS * 2 - 1) * pitch + FINGER_W_MM

# Rails
add_rect(fp, 0, rail_a_y - RAIL_W_MM / 2, total_width, rail_a_y + RAIL_W_MM / 2)
add_rect(fp, 0, rail_b_y - RAIL_W_MM / 2, total_width, rail_b_y + RAIL_W_MM / 2)

# Fingers
for i in range(NUM_FINGERS * 2):
    x = i * pitch + FINGER_W_MM / 2
    if i % 2 == 0:
        add_rect(
            fp,
            x - FINGER_W_MM / 2,
            rail_a_y,
            x + FINGER_W_MM / 2,
            rail_a_y + FINGER_LEN_MM,
        )
    else:
        add_rect(
            fp,
            x - FINGER_W_MM / 2,
            rail_b_y - FINGER_LEN_MM,
            x + FINGER_W_MM / 2,
            rail_b_y,
        )

# Add two pads for net connection
for idx, (y, x, name) in enumerate([
    (0, rail_a_y, "SENSE"),
    (0, rail_b_y, "GND")
]):
    pad = pcbnew.PAD(fp)
    pad.SetShape(pcbnew.PAD_SHAPE_RECT)
    pad.SetSize(pcbnew.VECTOR2I(mm(RAIL_W_MM), mm(RAIL_W_MM)))
    pad.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    pad.SetNumber(str(idx + 1))
    pad.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
    if hasattr(pad, "SetNetName"):
        pad.SetNetName(name)
    if hasattr(pad, "SetLayerSet"):
        layer_set = pcbnew.LSET()
        layer_set.AddLayer(LAYER)
        pad.SetLayerSet(layer_set)
    else:
        pad.SetLayer(LAYER)
    fp.Add(pad)

def save_footprint(fp, name="test_CapSoilPad", out_dir="."):
    """Save a footprint in KiCad 10+ to <out_dir>/generated.pretty/<name>.kicad_mod."""
    # Ensure a stable footprint name for the output file.
    fp.SetValue(name)
    fp.SetFPID(pcbnew.LIB_ID("", name))

    pretty_dir = os.path.abspath(os.path.join(out_dir, "soil_pad.pretty"))
    os.makedirs(pretty_dir, exist_ok=True)

    io = pcbnew.PCB_IO_KICAD_SEXPR()
    io.FootprintSave(pretty_dir, fp)

    print(f"Footprint saved to {pretty_dir}/{name}.kicad_mod")


# Save
save_footprint(fp)
