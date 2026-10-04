import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import openclaw_chat as chat
from assistant_language import EN, ZH


class AssistantProgressTest(unittest.TestCase):
    def setUp(self):
        self.job_id = 'e' * 32
        with chat._JOBS_LOCK:
            chat._JOBS[self.job_id] = {'status': 'queued', 'results': [], 'status_text': EN}
        self.addCleanup(lambda: chat._JOBS.pop(self.job_id, None))

    def detail(self, status, index=0, task_id='first'):
        return {'status': {'floor': 'test_101'}, 'task': {'task_id': task_id, 'task_status': status,
                'current_index': index, 'total_count': 2,
                'work_queue': ['navigation:elevator_waiting:test_101', 'navigation:goal:test_101']}}

    def test_same_status_next_subtask_and_final_completion_are_reported(self):
        snapshots = [self.detail('Navigating'), self.detail('Navigating'), self.detail('Navigating', 1),
                     self.detail('Finished', 2), self.detail('Navigating', task_id='second'),
                     self.detail('Finished', 2, 'second')]
        results = [{'name': 'navigate_to_point', 'ok': True, 'summary': EN['navigation_sent'], 'task_id': tid}
                   for tid in ('first', 'second')]
        actions = [{'name': 'navigate_to_point', 'arguments': {'robot_id': 'robot2'}}] * 2
        with mock.patch.object(chat, '_execute_action', side_effect=results), \
                mock.patch.object(chat, '_read_json', side_effect=snapshots), mock.patch.object(chat.time, 'sleep'):
            chat._run_action_job(self.job_id, actions, 8001)
        job = chat.get_action_job(self.job_id)
        messages = [event['message'] for event in job['events']]
        self.assertEqual(job['status'], 'completed')
        self.assertTrue(any('Subtask 1/2: Going to the elevator waiting point' in m for m in messages))
        self.assertTrue(any('Subtask 2/2: Going to the destination' in m for m in messages))
        self.assertEqual(sum('Subtask 2/2: Going to the destination' in m for m in messages), 1)
        self.assertEqual(job['events'][-1]['kind'], 'terminal')
        self.assertEqual(messages[-1], EN['plan_completed'].format(count=2))
        self.assertEqual([event['seq'] for event in job['events']], list(range(1, len(messages) + 1)))

    def test_failure_is_explicit_and_stops_remaining_actions(self):
        chat._JOBS[self.job_id]['status_text'] = ZH
        chat._JOBS[self.job_id]['conversation'] = ('linbo-test', 'isolated-session')
        actions = [{'name': 'navigate_to_point', 'arguments': {'robot_id': 'robot2'}}] * 2
        with mock.patch.object(chat, '_execute_action', return_value={'ok': True, 'task_id': 'first'}) as execute, \
                mock.patch.object(chat, '_read_json', return_value=self.detail('Failed')), \
                mock.patch('assistant_sessions.STORE.append') as append:
            chat._run_action_job(self.job_id, actions, 8001)
        job = chat.get_action_job(self.job_id)
        self.assertEqual(job['status'], 'failed')
        self.assertEqual(execute.call_count, 1)
        self.assertTrue(job['terminal_text'].startswith('执行失败：'))
        self.assertEqual(append.call_args.args[-1], job['terminal_text'])
        self.assertEqual(append.call_count, len(job['events']))

    def test_poll_snapshots_are_independent_and_repeated_phase_is_deduplicated(self):
        chat._publish_job_event(self.job_id, 'Going to pickup')
        chat._publish_job_event(self.job_id, 'Going to pickup')
        first = chat.get_action_job(self.job_id)
        first['events'].clear()
        self.assertEqual(len(chat.get_action_job(self.job_id)['events']), 1)

    def test_translated_point_alias_resolves_to_same_stable_id(self):
        point = {'id': 'pickup', 'name': '前台取货点', 'name_en': 'Reception pickup', 'x': 1, 'y': 2}
        with mock.patch.object(chat, '_read_json', return_value={'points': [point]}):
            self.assertEqual(chat._resolve_map_point('test_101', 'Reception pickup', 8001)['id'], 'pickup')
            self.assertEqual(chat._resolve_map_point('test_101', '前台取货点', 8001)['id'], 'pickup')
