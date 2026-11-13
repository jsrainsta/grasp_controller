"""Coordinator behavior tests: no camera, HTTP, serial or hardware writes."""
import json,tempfile,unittest
from pathlib import Path
from grasp_controller import Coordinator,ControllerLock,prepare_empty_gripper
from grasp_start_state import start_state_error

class CoordinatorTests(unittest.TestCase):
    def setUp(self):
        directory=Path(__file__).resolve().parent/'controller_test_tmp'
        directory.mkdir(exist_ok=True)
        self.tmp=tempfile.TemporaryDirectory(dir=directory);self.addCleanup(self.tmp.cleanup)
        self.config={'results_dir':self.tmp.name}
        self.event={'type':'arrival','status':'OK','request_id':'station-01','target_id':'med_003'}
    def test_duplicate_arrival_never_replays(self):
        calls=[]
        c=Coordinator(self.config,lambda e,r:calls.append(e) or {'grasp_confirmed':True})
        self.assertEqual(c.handle(self.event)['status'],'held_verified')
        self.assertTrue(c.handle(self.event)['deduplicated']);self.assertEqual(len(calls),1)
    def test_restart_failure_never_retries_same_arrival(self):
        def broken(e,r):raise RuntimeError('serial result unknown')
        self.assertEqual(Coordinator(self.config,broken).handle(self.event)['status'],'failed')
        def forbidden(e,r):raise AssertionError('must not replay')
        self.assertTrue(Coordinator(self.config,forbidden).handle(self.event)['deduplicated'])
    def test_missing_lift_evidence_is_failure(self):
        result=Coordinator(self.config,lambda e,r:{'grasp_confirmed':False}).handle(self.event)
        self.assertFalse(result['grasp_confirmed']);self.assertEqual(result['status'],'failed')
    def test_other_target_rejected_before_runner(self):
        def forbidden(e,r):raise AssertionError('wrong target must never dispatch')
        with self.assertRaises(ValueError):Coordinator(self.config,forbidden).handle({**self.event,'target_id':'bottle-01'})
    def test_parallel_controller_rejected(self):
        p=Path(self.tmp.name)/'controller.lock'
        with ControllerLock(p):
            with self.assertRaises(RuntimeError):
                with ControllerLock(p):pass
        with ControllerLock(p):pass
    def test_running_journal_survives_restart_without_replay(self):
        def interrupted(e,r):raise KeyboardInterrupt()
        result=Coordinator(self.config,interrupted).handle(self.event)
        self.assertEqual(result['status'],'interrupted')
        self.assertTrue(Coordinator(self.config,lambda e,r:self.fail('replayed')).handle(self.event)['deduplicated'])
    def test_empty_confirmation_required_before_any_io(self):
        def forbidden(*a):self.fail('unconfirmed operation dispatched')
        with self.assertRaises(ValueError):prepare_empty_gripper(self.config,False,forbidden,forbidden)
    def test_observed_settled_open_jaw_is_accepted(self):
        for value in (381,382,385):self.assertEqual(start_state_error({'1':value,'2':496}),'')
    def test_closed_jaw_and_wrong_wrist_are_still_rejected(self):
        for value in (414,475,490,520):self.assertTrue(start_state_error({'1':value,'2':496}))
        self.assertTrue(start_state_error({'1':381,'2':504}))
    def test_empty_setup_only_commands_gripper(self):
        before={'joint_median':{'1':491,'2':496,'3':181,'4':130,'5':412,'6':495}}
        after={'joint_median':{**before['joint_median'],'1':382}}
        readings=iter([before,after]);calls=[]
        def send(url,path,body=None):
            calls.append((path,body));return {'task':{'task_id':'empty-test','status':'reached'}}
        result=prepare_empty_gripper({**self.config,'arm_url':'test'},True,lambda u:next(readings),send)
        self.assertEqual(result['status'],'ready')
        self.assertEqual(calls[0][1]['targets'],[{'id':1,'position':385}]);self.assertEqual(len(calls),1)
    def test_empty_setup_unknown_result_never_replays(self):
        calls=[]
        def send(*args):calls.append(args);return {'task':{'task_id':'unknown','status':'outcome_unknown'}}
        with self.assertRaises(RuntimeError):
            prepare_empty_gripper({**self.config,'arm_url':'test'},True,
                lambda u:{'joint_median':{'1':491,'2':496,'3':181,'4':130,'5':412,'6':495}},send)
        self.assertEqual(len(calls),1)
    def test_trial_open_feedback_374_needs_no_second_open_command(self):
        profile=Path(__file__).with_name('grasp_motion_fixed_placement_trial.json')
        state={'joint_median':{'1':374,'2':497,'3':181,'4':130,'5':412,'6':495}}
        config={**self.config,'arm_url':'test','motion_profile':str(profile)}
        result=prepare_empty_gripper(config,True,lambda _:state,lambda *a:self.fail('already open'))
        self.assertEqual(result['status'],'ready')
        self.assertEqual(result['commanded_joint_ids'],[])

if __name__=='__main__':unittest.main()
