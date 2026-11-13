"""Shared empirical open-jaw start policy; not a contact/force measurement.

The open command 385 settles around 381..385 in the observed hardware. A
three-count gate rejected a stable empty jaw after a one-count settling change.
Use six counts for this approximate open geometry; keep wrist tolerance three.
This does not loosen position/vision/loaded-gripper/holding acceptance.
"""
OPEN_JAW_POSITION=385
OPEN_JAW_TOLERANCE=6
WRIST_POSITION=496
WRIST_TOLERANCE=3

def start_state_error(positions,jaw=OPEN_JAW_POSITION,wrist=WRIST_POSITION,*,motion=None):
    actual_jaw=positions['1'];actual_wrist=positions['2']
    errors=[]
    minimum=jaw-OPEN_JAW_TOLERANCE
    if motion is not None:
        minimum=motion.get('open_jaw_minimum',minimum)
    maximum=jaw+OPEN_JAW_TOLERANCE
    if not minimum<=actual_jaw<=maximum:
        errors.append(f'夹爪实读{actual_jaw}，空爪起点允许{minimum}～{maximum}')
    if abs(actual_wrist-wrist)>WRIST_TOLERANCE:
        errors.append(f'腕轴实读{actual_wrist}，允许{wrist-WRIST_TOLERANCE}～{wrist+WRIST_TOLERANCE}')
    return '；'.join(errors)

def require_start_state(positions,jaw=OPEN_JAW_POSITION,wrist=WRIST_POSITION,*,motion=None):
    error=start_state_error(positions,jaw,wrist,motion=motion)
    if error:raise RuntimeError(error+'；不自动松爪、收紧或转腕')
