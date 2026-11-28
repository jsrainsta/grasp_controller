"""Known detector classes and measured packages used by the grasp controller."""
from medicine_match import normalise_text

MEDICINES = (
    ('med_003', '阿昔洛韦乳膏', (10.5, 3.4, 2.5)),
    ('erythromycin', '红霉素软膏', (10.5, 3.4, 2.5)),
    ('hydrocortisone_butyrate', '丁酸氢化可的松乳膏', (12., 3.3, 2.4)),
)


def medicine(name):
    if not isinstance(name,str):
        raise ValueError('药名必须是字符串')
    for index, (target_id, label, dimensions) in enumerate(MEDICINES):
        if normalise_text(name) == normalise_text(label):
            return {'id':target_id, 'name':label, 'class_id':index, 'dimensions_cm':dimensions}
    raise ValueError(f'未知药名 {name!r}，缺少已验证的类别和尺寸')


def confirmed_target(observation):
    if 'detection' in observation:
        return observation['detection'].get('selected') is not None
    return observation.get('ocr',{}).get('target_selection',{}).get('status') == 'target_found'


def motion_for_box(motion, observation):
    height = observation['geometry'].get('assumed_box_dimensions_cm',(10.5,3.4,2.5))[2]
    delta=height-2.5
    if not delta:
        return motion
    region=motion['grasp_region']
    floor=motion['minimum_nominal_pad_height_cm']
    return {**motion, 'box_height_cm':height, 'grasp_region':{
        **region,
        'tcp_height_target_cm':round(region['tcp_height_target_cm']+delta,4),
        'tcp_height_range_cm':[max(floor,round(value+delta,4)) for value in region['tcp_height_range_cm']],
    }}
