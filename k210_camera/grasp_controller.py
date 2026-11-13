"""Arrival-triggered local coordinator. No GPT/API model dependency.

Runs on the host to which K210 is attached. On Pi, use the Pi-local arm URL.
Only the current empirical box profile is implemented. A process lock and
persistent event journal prevent concurrent execution or duplicate arrivals.
"""
import argparse,hashlib,importlib.util,json,os,subprocess,sys,time,uuid
from collections import deque
from pathlib import Path
from datetime import datetime,timezone
from grasp_start_state import start_state_error,OPEN_JAW_POSITION,WRIST_POSITION,WRIST_TOLERANCE
from grasp_motion_profile import load_motion_profile

ROOT=Path(__file__).resolve().parent

def now():return datetime.now(timezone.utc).isoformat()

class ControllerLock:
    def __init__(self,path):self.path=Path(path);self.file=None
    def __enter__(self):
        self.path.parent.mkdir(parents=True,exist_ok=True)
        self.file=self.path.open('a+b');self.file.seek(0)
        if self.path.stat().st_size==0:self.file.write(b'0');self.file.flush();self.file.seek(0)
        try:
            if os.name=='nt':
                import msvcrt
                msvcrt.locking(self.file.fileno(),msvcrt.LK_NBLCK,1)
            else:
                import fcntl
                fcntl.flock(self.file.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError:
            self.file.close();raise RuntimeError('已有总控实例在运行，拒绝并行抓取')
        return self
    def __exit__(self,*args):self.file.close()

def load_config(path):
    path=Path(path).resolve();c=json.loads(path.read_text(encoding='utf-8'))
    profile=c.get('object_profile',{})
    if (profile.get('id')!='med_003' or profile.get('dimensions_cm')!=[10.5,3.4,2.5]
            or profile.get('close_target')!=520):
        raise ValueError('本版只实现med_003、10.5×3.4×2.5cm和520；不能仅改配置就声称支持新药瓶')
    for name in ('calibration','results_dir'):
        c[name]=str((path.parent/c[name]).resolve())
    for name in ('motion_profile','fallback_motion_profile'):
        if c.get(name):
            c[name]=str((path.parent/c[name]).resolve())
            load_motion_profile(c[name])
    if not Path(c['calibration']).is_file():raise ValueError('缺少相机安装标定文件')
    if not isinstance(c.get('camera_port'),str) or not c['camera_port']:raise ValueError('未设置K210串口')
    baudrate=c.get('camera_baudrate',115200)
    if type(baudrate) is not int or not 0<baudrate<=5000000:
        raise ValueError('K210 串口波特率无效')
    c['camera_baudrate']=baudrate
    return c

def resolve_port(value):
    if value!='auto':return value
    from serial.tools import list_ports
    found=[p.device for p in list_ports.comports() if 'CH340' in p.description.upper() or p.vid==0x1a86]
    if len(found)!=1:raise ValueError('无法唯一确定K210串口；在配置中填写稳定设备路径，不自动选择多个CH340之一')
    return found[0]

def check_environment(config):
    modules=('numpy','cv2','serial','PIL','rapidocr','onnxruntime')
    missing=[m for m in modules if importlib.util.find_spec(m) is None]
    files=[ROOT/'run_current_box_trial.py',ROOT/'camera_calibration_results/diagnostic_20261026_58dc9618/intrinsics_candidate.json',
           ROOT/'calibration_boards/charuco_7x5_print96_v1.json',ROOT/'calibration_boards/grasp_tcp_hover_candidate_20261105.json',
           ROOT.parent/'arm_service_review/received_20261105/arm_service/kinematics.py',
           ROOT.parent/'arm_service_review/received_20261105/config.serial.integration.json']
    absent=[str(p) for p in files if not p.is_file()]
    missing_features=[]
    if 'cv2' not in missing:
        import cv2
        if not hasattr(cv2,'SIFT_create'):missing_features.append('cv2.SIFT_create')
        if not hasattr(cv2,'solvePnPRefineLM'):missing_features.append('cv2.solvePnPRefineLM')
        for attr in ('CharucoBoard','CharucoDetector','ArucoDetector'):
            if not hasattr(getattr(cv2,'aruco',None),attr):missing_features.append('cv2.aruco.'+attr)
    return {'type':'controller_environment_check','missing_modules':missing,'missing_files':absent,
            'missing_opencv_features':missing_features,
            'ready_for_connection_check':not missing and not absent and not missing_features,'hardware_commands_sent':False,
            'independent_full_run_validated':False,'pi_hardware_validated':False}

class Coordinator:
    def __init__(self,config,runner=None):
        self.config=config;self.directory=Path(config['results_dir']);self.directory.mkdir(parents=True,exist_ok=True)
        self.runner=runner or self.run_pipeline
    def run_pipeline(self,event,record,*,preview=False):
        port=resolve_port(self.config['camera_port'])
        # Fresh status before opening serial or dispatching motion. A held box
        # must never be silently opened for the next arrival.
        from collect_grasp_reference import read_arm
        state=read_arm(self.config['arm_url']);j=state['joint_median']
        issue=start_state_error(j,motion=load_motion_profile(self.config.get('motion_profile')))
        if issue:
            raise RuntimeError(issue+'；若正在持物，先完成放置任务。总控不自动松爪或复位')
        log=self.directory/(record['run_id']+'.log')
        command=[sys.executable,'-X','utf8',str(ROOT/'run_current_box_trial.py'),'--port',port,
                 '--camera-baudrate',str(self.config.get('camera_baudrate',115200)),
                 '--arm-url',self.config['arm_url'],'--calibration',self.config['calibration'],
                 '--result',str(self.directory/(record['run_id']+'_pipeline.json')),
                 '--preview' if preview else '--execute']
        if self.config.get('resume_hover'):
            command.extend(['--resume-hover',self.config['resume_hover']])
        if self.config.get('resume_approach'):
            command.extend(['--resume-approach',self.config['resume_approach']])
        if self.config.get('motion_profile'):
            command.extend(['--motion-profile',self.config['motion_profile']])
        if self.config.get('fallback_motion_profile'):
            command.extend(['--fallback-motion-profile',self.config['fallback_motion_profile']])
        tail=deque(maxlen=80)
        with log.open('w',encoding='utf-8') as stream:
            process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                     text=True,encoding='utf-8',errors='replace')
            try:
                for line in process.stdout:
                    stream.write(line);stream.flush()
                    tail.append(line.strip())
                    print(line,end='',file=sys.stderr,flush=True)
                code=process.wait()
            finally:
                process.stdout.close()
                if process.poll() is None:
                    # Do not release the single-controller lock while its
                    # writer child is still executing. Child motions already
                    # have bounded task/vision waits; no reset/stop is sent.
                    process.wait()
        if code:
            reason=next((line for line in reversed(tail) if line.startswith(('ValueError:','RuntimeError:','OSError:','PermissionError:','TimeoutError:'))),'未取得具体错误')
            raise RuntimeError('抓取流程失败：'+reason+'；日志：'+str(log))
        result=json.loads((self.directory/(record['run_id']+'_pipeline.json')).read_text(encoding='utf-8'))
        if result.get('selected_motion_profile'):
            record['motion_profile']=result['selected_motion_profile']
        if preview:return result
        if result.get('grasp_confirmed') is not True:raise RuntimeError('流程未提供持物验收通过证据')
        return result
    def handle(self,event):
        if not isinstance(event,dict) or event.get('type')!='arrival' or event.get('status')!='OK':
            raise ValueError('事件须为arrival且status=OK')
        key=event.get('request_id')
        if not isinstance(key,str) or not 1<=len(key)<=128:raise ValueError('必须提供1..128字符的唯一到站request_id')
        if event.get('target_id','med_003')!='med_003':raise ValueError('本版目标只支持med_003')
        with ControllerLock(self.directory/'controller.lock'):
            path=self.directory/(hashlib.sha256(key.encode()).hexdigest()+'.json')
            if path.exists():
                old=json.loads(path.read_text(encoding='utf-8'));return {**old,'deduplicated':True}
            r={'type':'autonomous_grasp_result','version':1,'request_id':key,'run_id':uuid.uuid4().hex,
               'target_id':'med_003','started_at':now(),'status':'running','grasp_confirmed':False,
               'grasp_quality_validated':False,
               'motion_profile':self.config.get('motion_profile'),
               'deduplicated':False,'gpt_runtime_required':False}
            def save():
                tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8');os.replace(tmp,path)
            save()
            try:
                result=self.runner(event,r)
                if not isinstance(result,dict) or result.get('grasp_confirmed') is not True:
                    raise RuntimeError('缺少持物验收通过结果')
                r.update(status='held_verified',grasp_confirmed=True,pipeline=result)
            except BaseException as error:
                r.update(status='failed',error=str(error),recovery='No automatic reset, open, reconnect, release, or retry.')
                if isinstance(error,(KeyboardInterrupt,SystemExit)):r['status']='interrupted'
            r['completed_at']=now();save();return r

def prepare_empty_gripper(config,confirmed_empty,reader=None,sender=None):
    """An explicit empty-jaw setup task, separate from an arrival/grasp.

    No automatic inference of an empty gripper from joint position. Only ID1
    is commanded; all task IDs and the subsequent body feedback are recorded.
    """
    if confirmed_empty is not True:raise ValueError('准备张爪需要明确确认已取出物体')
    motion=load_motion_profile(config.get('motion_profile'))
    if reader is None:
        from collect_grasp_reference import read_arm
        reader=read_arm
    if sender is None:
        from sample_grasp_handeye import call
        sender=call
    directory=Path(config['results_dir']);directory.mkdir(parents=True,exist_ok=True)
    with ControllerLock(directory/'controller.lock'):
        path=directory/('prepare_empty_'+uuid.uuid4().hex+'.json')
        before=reader(config['arm_url']);j=before['joint_median']
        result={'type':'empty_gripper_preparation','confirmed_empty':True,'before':before,
                'commanded_joint_ids':[],'status':'starting'}
        def save():path.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        save()
        try:
            if abs(j['2']-WRIST_POSITION)>WRIST_TOLERANCE:raise RuntimeError('腕轴不符；此准备操作只动夹爪，不自动转腕')
            if start_state_error(j,motion=motion):
                if j['1']<OPEN_JAW_POSITION:raise RuntimeError('当前夹爪已更张开，此操作不自动收紧到起点')
                body={'request_id':'prepare_empty_'+uuid.uuid4().hex,'targets':[{'id':1,'position':OPEN_JAW_POSITION}],'duration_ms':20}
                result.update(request=body,commanded_joint_ids=[1]);save()
                r=sender(config['arm_url'],'move',body);result['response']=r;save()
                task=r['task'];deadline=time.monotonic()+22
                while task['status']=='sent' and time.monotonic()<deadline:
                    time.sleep(.25);task=sender(config['arm_url'],'tasks/'+task['task_id'])
                result['task']=task;save()
                if task['status']!='reached':raise RuntimeError('张爪结果未确认；不重发、释放、停止或重连')
            after=reader(config['arm_url']);result['after']=after
            if any(abs(after['joint_median'][k]-j[k])>6 for k in ('2','3','4','5','6')):
                raise RuntimeError('其他关节反馈发生变化；未向它们发送动作')
            issue=start_state_error(after['joint_median'],motion=motion)
            if issue:raise RuntimeError(issue)
            result['status']='ready'
        except BaseException as error:
            result.update(status='failed',error=str(error));raise
        finally:save()
        print('空爪准备完成，夹爪实读：',after['joint_median']['1'],'记录：',path,flush=True)
        return result

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--check',action='store_true',help='仅检查本机文件/依赖，不连接硬件')
    group.add_argument('--preview',action='store_true',help='拍照并检查当前场景路径，不发送机械臂运动')
    group.add_argument('--once',metavar='ARRIVAL_ID',help='一次到站OK，ID重复不会重发动作')
    group.add_argument('--listen',action='store_true',help='从标准输入逐行接收到站JSON；也可直接输入OK')
    group.add_argument('--prepare-empty',action='store_true',help='明确确认已取出物体后，只张开1号夹爪到起点')
    p.add_argument('--confirm-empty',action='store_true')
    resume=p.add_mutually_exclusive_group()
    resume.add_argument('--resume-hover',type=Path,help='接续已完成的悬停，不重复先前预接近动作')
    resume.add_argument('--resume-approach',type=Path,help='接续已确认运动的视觉阶段，不重复先前动作')
    p.add_argument('--execute',action='store_true')
    a=p.parse_args();c=load_config(a.config)
    if a.check:print(json.dumps(check_environment(c),ensure_ascii=False,indent=2));return
    if a.preview:
        if a.execute or a.resume_hover or a.resume_approach:
            p.error('--preview 不接受 --execute 或接续参数')
        coordinator=Coordinator(c)
        with ControllerLock(coordinator.directory/'controller.lock'):
            result=coordinator.run_pipeline({}, {'run_id':uuid.uuid4().hex},preview=True)
        print(json.dumps(result,ensure_ascii=False,indent=2))
        return
    if a.resume_hover or a.resume_approach:
        if not a.once:raise ValueError('现场接续需要--once提供新request_id')
        path=a.resume_hover or a.resume_approach
        if not path.is_file():raise ValueError('接续记录不存在')
        c['resume_hover' if a.resume_hover else 'resume_approach']=str(path.resolve())
    if not a.execute:raise ValueError('运行到站事件需要--execute显式启用已配置动作')
    if a.prepare_empty:
        prepare_empty_gripper(c,a.confirm_empty);return
    coordinator=Coordinator(c)
    if a.once:
        r=coordinator.handle({'type':'arrival','status':'OK','request_id':a.once})
        print(json.dumps(r,ensure_ascii=False,indent=2));sys.exit(0 if r['status']=='held_verified' else 1)
    print('等待到站OK；每行一个JSON事件或OK。相同request_id不会重复抓取。',flush=True)
    for line in sys.stdin:
        if not line.strip():continue
        try:
            event={'type':'arrival','status':'OK','request_id':uuid.uuid4().hex} if line.strip()=='OK' else json.loads(line)
            print(json.dumps(coordinator.handle(event),ensure_ascii=False),flush=True)
        except Exception as error:print(json.dumps({'status':'rejected','error':str(error)},ensure_ascii=False),flush=True)

if __name__=='__main__':main()
