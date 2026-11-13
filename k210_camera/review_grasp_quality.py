"""Offline grasp-quality and timing audit. Never connects to camera or arm."""
import argparse,json,math,re
from datetime import datetime
from pathlib import Path
import numpy as np
from check_grasp_board import ROOT
from sample_grasp_handeye import nominal_model
from plan_fixed_box_grasp import ik

def review(trial_path):
    trial_path=Path(trial_path).resolve();trial=json.loads(trial_path.read_text(encoding='utf-8'))
    approach=json.loads(Path(trial['source']).read_text(encoding='utf-8'))
    scene=json.loads(Path(approach['source']).read_text(encoding='utf-8'))
    cal=json.loads(Path(scene['calibration']).read_text(encoding='utf-8'))['candidates'][0]
    Y=np.array(cal['T_base_board_cm']);up=-Y[:3,2]
    top=np.array(scene['geometry']['selected']['top_center_base_cm']);ground=top-up*2.5
    offset=np.array(json.loads(Path(approach['tcp_profile']).read_text(encoding='utf-8'))['offset_tool_cm'])
    model=nominal_model();limits={r['id']:(r['min'],r['max']) for r in scene['after']['status']['data']['servo_limits']}
    before=trial['observations'][0];actual=before['after']['joint_median'];visual=before['visual']
    pitch=model.forward({int(j):v for j,v in actual.items()})['pitch_deg']
    radial=ground.copy();radial[2]=0;radial/=np.linalg.norm(radial)
    candidates=[]
    for closer in (0,1,2,3,4):
        for angle in (-90,-85,-80,-75):
            point=ground+up*1.8-radial*closer;theta=math.radians(angle);yaw=math.atan2(point[1],point[0])
            for _ in range(6):
                c,s,ct,st=math.cos(yaw),math.sin(yaw),math.cos(theta),math.sin(theta)
                R=np.array([[c*ct,-s,-c*st],[s*ct,c,-s*st],[st,0,ct]])
                tool=point-R@offset;yaw=math.atan2(tool[1],tool[0])
            solutions=[]
            for sol in ik(tool,angle,model.to_config()):
                p=sol['illustrative_joint_positions_not_for_execution']
                if all(limits[int(j)][0]<=v<=limits[int(j)][1] for j,v in p.items()):
                    solutions.append({'joint_targets_not_for_execution':p,
                        'joint_limit_margin':min(min(v-limits[int(j)][0],limits[int(j)][1]-v) for j,v in p.items())})
            candidates.append({'hypothetical_closer_to_base_cm':closer,'open_pad_center_height_cm':1.8,
                               'nominal_pitch_deg':angle,'solutions':solutions})
    movement_s=sum((datetime.fromisoformat(c['task']['updated_at'])-datetime.fromisoformat(c['task']['created_at'])).total_seconds() for c in trial['commands'])
    timing={}
    for path in (ROOT/'controller_results').glob('*_pipeline.json'):
        r=json.loads(path.read_text(encoding='utf-8'))
        if r.get('verification') and Path(r['verification']).resolve()==trial_path:
            log=path.with_name(path.name.replace('_pipeline.json','.log')).read_text(encoding='utf-8')
            captures=[float(v) for v in re.findall(r'发出拍照指令到保存完成：([\d.]+) 秒',log)]
            timing={'stage_elapsed_s':sum(s['completed_at']-s['started_at'] for s in r['stages']),
                    'stages':r['stages'],'capture_count':len(captures),'capture_elapsed_s':sum(captures),
                    'move_task_confirmation_s':movement_s,'move_count':len(trial['commands'])}
    report={'type':'offline_grasp_quality_review','hardware_commands_sent':False,'for_execution':False,
            'trial':str(trial_path),'held_verification_passed':trial.get('grasp_confirmed'),
            'quality_accepted':False,'quality_reason':'User observed high edge contact and tilted fingers; holding success alone is not quality acceptance.',
            'observed_open_pad_center_height_cm':-visual['tcp_board_cm'][2],
            'known_box_height_cm':2.5,'nominal_pitch_at_close_deg':pitch,
            'ground_center_model_cm':ground.tolist(),'timing':timing,'reach_candidates':candidates,
            'limitation':'Nominal model and approximate open-pad TCP only. Neither closed contact height nor physical finger verticality is independently calibrated.'}
    out=trial_path.with_name(trial_path.stem+'_quality_review.json');out.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print('抓取高度cm：',report['observed_open_pad_center_height_cm'],'模型俯仰deg：',pitch)
    print('耗时：',timing)
    print('当前位置候选：',[(c['nominal_pitch_deg'],[s['joint_limit_margin'] for s in c['solutions']]) for c in candidates if c['hypothetical_closer_to_base_cm']==0])
    print('离线质量报告：',out)
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=Path)
    review(p.parse_args().trial)
