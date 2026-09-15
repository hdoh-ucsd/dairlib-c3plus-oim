"""Full-suite manifest/preflight/resume checks without native execution."""
import argparse
from contextlib import ExitStack, redirect_stderr, redirect_stdout
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from c3plus.configs.catalog import TASKS, OBJECTS
from c3plus.configs.poses import load_pose_catalogue
from c3plus.runtime import environment as E, processes as P
from c3plus.utils import campaign as C


def args(output_root):
    return argparse.Namespace(suite='full', campaign_name=None, manifest=None, scenes=None,
                              objects=None, pairs=None, obstacle_cost='exponential', seed=42,
                              cap=None, port_base=19000, output_root=Path(output_root),
                              resume=False, dry_run=True)


def git_output(command, **kwargs):
    return 'a' * 40 + '\n' if kwargs.get('text') else b'fixed tracked source diff\n'


class FullCampaignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.root = Path(cls.temporary.name) / 'campaign'
        cls.raw_plans = []
        original = C.plan_run
        def capture(*values, **options):
            plan = original(*values, **options)
            cls.raw_plans.append(deepcopy(plan))
            return plan
        with patch.object(C, 'plan_run', side_effect=capture), \
                patch.object(C.subprocess, 'check_output', side_effect=git_output):
            cls.manifest = C.build_manifest(args(cls.root))
        cls.poses = load_pose_catalogue()

    def relocated(self, root, count=None):
        value = deepcopy(self.manifest)
        value['output_root'] = str(root)
        if count is not None:
            value['runs'] = value['runs'][:count]
            value['run_count'] = count
        for plan in value['runs']:
            plan['out'] = str(root / plan['scene'] / plan['object_name'] / plan['run_id'])
        return value

    def complete_fixture(self, plan):
        directory = Path(plan['out']); directory.mkdir(parents=True)
        video = directory / f"{plan['run_id']}.mp4"
        video.write_bytes(b'video bytes checked by package digest')
        runtime = {**plan, 'wrapper_rc': 0, 'postprocess_rc': 0, 'render_rc': 0,
                   'seed_verified': True, 'failures': []}
        result = {'run_id': plan['run_id'], 'runtime_status': runtime, 'recording': {},
                  'provenance': {'configuration': {'files': {'evaluation_scene_config.yaml': {'data': {}}}}},
                  'package': {'status': 'complete', 'cleanup_complete': True, 'video': {
                      'file': video.name, 'size_bytes': video.stat().st_size,
                      'sha256': hashlib.sha256(video.read_bytes()).hexdigest()}}}
        path = directory / f"{plan['run_id']}_result.json"
        path.write_text(json.dumps(result))
        return path, video

    def test_complete_750_manifest_has_canonical_order_poses_names_and_seed(self):
        manifest = self.manifest
        expected = [(task, obj, int(start), int(goal)) for task in TASKS for obj in OBJECTS
                    for start in self.poses[task]['starts'] for goal in self.poses[task]['goals']]
        actual = [(p['scene'], p['object_name'], p['start'], p['goal_index']) for p in manifest['runs']]
        self.assertEqual(actual, expected)
        self.assertEqual(manifest['run_count'], 750)
        self.assertEqual(manifest['pairs_per_task_object'], 25)
        self.assertEqual(len({p['run_id'] for p in manifest['runs']}), 750)
        self.assertEqual(manifest['tasks'], list(TASKS))
        self.assertEqual(manifest['objects'], list(OBJECTS))
        self.assertEqual(set(manifest['object_profiles']), set(OBJECTS))
        self.assertEqual(set(manifest['object_assets']), set(OBJECTS))
        self.assertEqual(manifest['pose_catalogue']['source_commit'], 'a31203d8e9a347ba7bf373954a4b7e4059d1e350')
        for plan in manifest['runs']:
            task, obj = plan['scene'], plan['object_name']
            self.assertEqual(plan['seed'], 42)
            self.assertEqual(plan['wall_cap_seconds'], 300)
            self.assertEqual(plan['canonical_start_pose'], self.poses[task]['starts'][str(plan['start'])])
            self.assertEqual(plan['canonical_goal_pose'], self.poses[task]['goals'][str(plan['goal_index'])])
            self.assertEqual(plan['start_pose'][4:6], plan['canonical_start_pose'][:2])
            self.assertEqual(plan['evaluation_goal'], plan['canonical_goal_pose'])
            self.assertEqual(Path(plan['out']).relative_to(self.root), Path(task) / obj / plan['run_id'])
            self.assertEqual(plan['native_scene'], 'open_task' if task == 'open_table' else task)
            self.assertEqual(plan['native_object_name'], 'T_block' if obj == 'T_shape' else obj)
            self.assertNotIn('object_profile', plan)
            self.assertNotIn('asset_sha256', plan)
            self.assertNotIn('pose_catalogue', plan)
        self.assertFalse(self.root.exists(), 'Manifest generation must not create output')

    def test_full_campaign_explicit_cap_overrides_default_for_every_run(self):
        options = args(self.root)
        options.cap = 600
        with patch.object(C.subprocess, 'check_output', side_effect=git_output):
            manifest = C.build_manifest(options)
        self.assertEqual(manifest['run_count'], 750)
        self.assertEqual({plan['wall_cap_seconds'] for plan in manifest['runs']}, {600})
        self.assertFalse(self.root.exists())

    def test_manifest_is_deterministic_for_unchanged_inputs(self):
        with patch.object(C, 'plan_run', side_effect=deepcopy(self.raw_plans)), \
                patch.object(C.subprocess, 'check_output', side_effect=git_output):
            repeated = C.build_manifest(args(self.root))
        self.assertEqual(repeated, self.manifest)

    def test_partial_and_corrupt_results_are_never_counted_pending(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'out'; manifest = self.relocated(root)
            initial = C.inventory_runs(manifest)
            self.assertEqual((initial['completed'], initial['pending'], initial['invalid']), (0, 750, 0))
            first, second = manifest['runs'][:2]
            partial = Path(first['out']); partial.mkdir(parents=True); (partial / 'recorder.log').write_text('preserve')
            corrupt = Path(second['out']); corrupt.mkdir(parents=True)
            (corrupt / f"{second['run_id']}_result.json").write_text('{broken')
            (corrupt / f"{second['run_id']}.mp4").write_bytes(b'video')
            inventory = C.inventory_runs(manifest)
            self.assertEqual((inventory['completed'], inventory['pending'], inventory['invalid']), (0, 748, 2))
            self.assertEqual((partial / 'recorder.log').read_text(), 'preserve')

    def test_resume_validates_saved_identity_and_video_before_skipping(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'out'; manifest = self.relocated(root, 1); plan = manifest['runs'][0]
            path, video = self.complete_fixture(plan)
            with patch.object(C, '_validate') as validate:
                self.assertEqual(C.inventory_runs(manifest)['completed'], 1)
                validate.assert_called_once()
            before = json.loads(path.read_text())
            changed = deepcopy(before); changed['runtime_status']['canonical_start_pose'][0] += .001
            path.write_text(json.dumps(changed))
            self.assertEqual(C.inventory_runs(manifest)['invalid'], 1)
            path.write_text(json.dumps(before)); video.write_bytes(video.read_bytes() + b'corrupt')
            self.assertEqual(C.inventory_runs(manifest)['invalid'], 1)

    def test_resume_runs_only_pending_and_never_overwrites_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'out'; manifest = self.relocated(root, 2)
            complete_path, video = self.complete_fixture(manifest['runs'][0])
            saved = (complete_path.read_bytes(), video.read_bytes())
            (root / 'manifest.json').write_text(json.dumps(manifest))
            with patch.object(C, 'build_manifest', return_value=manifest), \
                    patch.object(C, 'full_preflight', return_value={'valid': True}), \
                    patch.object(C, '_validate'), patch.object(C, 'write_summary'), \
                    patch.object(C, 'run_one', return_value={'failures': []}) as run, redirect_stdout(io.StringIO()):
                C.main(['--suite', 'full', '--resume', '--out', str(root)])
            run.assert_called_once()
            self.assertEqual(Path(run.call_args.args[4]), Path(manifest['runs'][1]['out']))
            self.assertEqual((complete_path.read_bytes(), video.read_bytes()), saved)

    def test_invalid_runtime_metadata_is_not_accepted_as_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            manifest = self.relocated(Path(tmp) / 'out', 1)
            path, _ = self.complete_fixture(manifest['runs'][0])
            result = json.loads(path.read_text()); result['runtime_status']['postprocess_rc'] = 17
            path.write_text(json.dumps(result))
            with patch.object(C, '_validate'):
                self.assertEqual(C.inventory_runs(manifest)['invalid'], 1)

    def test_debug_budget_or_yaw_override_cannot_resume_as_an_unlimited_canonical_run(self):
        for field, value in (('execution_step_budget', 3), ('goal_yaw_degrees', 90)):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as tmp:
                manifest = self.relocated(Path(tmp) / 'out', 1)
                plan = manifest['runs'][0]
                self.assertNotIn(field, plan)
                path, _ = self.complete_fixture(plan)
                result = json.loads(path.read_text())
                result['runtime_status'][field] = value
                # Canonical catalogue coordinates remain unchanged; the explicit
                # execution override must itself invalidate this resume candidate.
                self.assertEqual(result['runtime_status']['canonical_goal_pose'], plan['canonical_goal_pose'])
                path.write_text(json.dumps(result))
                with patch.object(C, '_validate'):
                    inventory = C.inventory_runs(manifest)
                self.assertEqual(inventory['completed'], 0)
                self.assertEqual(inventory['invalid'], 1)
                self.assertIn(field, inventory['invalid_runs'][0]['reason'])

    def test_stop_after_current_preserves_manifest_and_leaves_next_run_pending(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'out'; manifest = self.relocated(root, 2)
            def finish_current(*values, **options):
                (root / 'STOP_AFTER_CURRENT').write_text('stop requested')
                return {'failures': []}
            with patch.object(C, 'build_manifest', return_value=manifest), \
                    patch.object(C, 'full_preflight', return_value={'valid': True}), \
                    patch.object(C, 'write_summary'), patch.object(C, 'run_one', side_effect=finish_current) as run, \
                    redirect_stdout(io.StringIO()):
                C.main(['--suite', 'full', '--out', str(root)])
            run.assert_called_once()
            self.assertEqual(json.loads((root / 'manifest.json').read_text()), manifest)
            self.assertFalse(Path(manifest['runs'][1]['out']).exists())
            self.assertIn('[STOPPED_AFTER_CURRENT]', (root / 'campaign_driver.log').read_text())

    def test_partial_output_aborts_before_native_launch(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'out'; manifest = self.relocated(root, 1)
            partial = Path(manifest['runs'][0]['out']); partial.mkdir(parents=True)
            (partial / 'raw.jsonl').write_text('scientific evidence')
            (root / 'manifest.json').write_text(json.dumps(manifest))
            with patch.object(C, 'build_manifest', return_value=manifest), \
                    patch.object(C, 'full_preflight', return_value={'valid': True}), \
                    patch.object(C, 'run_one') as run, redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit):
                C.main(['--suite', 'full', '--resume', '--out', str(root)])
            run.assert_not_called()
            self.assertEqual((partial / 'raw.jsonl').read_text(), 'scientific evidence')

    def test_symlinked_campaign_metadata_cannot_modify_an_external_file(self):
        for name in ('campaign_driver.log', 'summary.csv.tmp', 'manifest.json.tmp'):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp) / 'out'; root.mkdir(); manifest = self.relocated(root, 1)
                (root / 'manifest.json').write_text(json.dumps(manifest))
                outside = Path(tmp) / 'outside.txt'; outside.write_text('untouched')
                (root / name).symlink_to(outside)
                with patch.object(C, 'build_manifest', return_value=manifest), \
                        patch.object(C, 'full_preflight', return_value={'valid': True}), \
                        patch.object(C, 'run_one') as run, redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()), \
                        self.assertRaises(SystemExit):
                    C.main(['--suite', 'full', '--out', str(root)])
                run.assert_not_called()
                self.assertEqual(outside.read_text(), 'untouched')


class FullPreflightTests(unittest.TestCase):
    def preflight_mocks(self, stack, repo):
        stack.enter_context(patch.object(E, 'REPO', repo))
        stack.enter_context(patch.object(E, 'collect_checks', return_value=[]))
        assets = stack.enter_context(patch.object(E, 'check_scene_assets', return_value='validated assets'))
        stack.enter_context(patch('c3plus.configs.geometry.validate_suite_geometry', return_value={'valid': True, 'errors': []}))
        stack.enter_context(patch.object(P, 'check_experiment_capabilities', return_value={}))
        return assets

    def test_last_object_task_asset_failure_aborts_preflight(self):
        with tempfile.TemporaryDirectory() as tmp, ExitStack() as stack:
            repo = Path(tmp); assets = self.preflight_mocks(stack, repo)
            def validate(task, obj, **kwargs):
                if (task, obj) == (TASKS[-1], OBJECTS[-1]):
                    raise ValueError('unsupported final combination')
                return 'ok'
            assets.side_effect = validate
            with self.assertRaisesRegex(RuntimeError, 'unsupported final combination'):
                E.full_preflight([], repo / 'results')
            self.assertEqual(assets.call_count, len(TASKS) * len(OBJECTS))
            self.assertFalse((repo / 'results').exists())

    def test_symlinked_run_destination_fails_before_output_creation(self):
        with tempfile.TemporaryDirectory() as tmp, ExitStack() as stack:
            repo = Path(tmp); self.preflight_mocks(stack, repo)
            root = repo / 'out'; root.mkdir(); outside = repo / 'elsewhere'; outside.mkdir()
            (root / 'icra_sign').symlink_to(outside, target_is_directory=True)
            plan = {'out': str(root / 'icra_sign/T_shape/run'), 'port': 19001}
            with self.assertRaisesRegex(RuntimeError, 'symlink|leaves'):
                E.full_preflight([plan], root, require_runtime=False)
            self.assertEqual(list(outside.iterdir()), [])

    def test_stale_planner_capabilities_request_a_rebuild_without_running_an_experiment(self):
        response = subprocess.CompletedProcess(['planner'], 0, json.dumps({'object_scene_support_version': 0}), '')
        with patch.object(P.subprocess, 'run', return_value=response) as process, \
                self.assertRaisesRegex(RuntimeError, 'Rebuild'):
            P.check_experiment_capabilities()
        self.assertEqual(process.call_args.args[0][-1], '--print_experiment_capabilities')

    def test_stale_planner_blocks_full_preflight_before_output_creation(self):
        with tempfile.TemporaryDirectory() as tmp, ExitStack() as stack:
            repo = Path(tmp); self.preflight_mocks(stack, repo)
            with patch.object(P, 'check_experiment_capabilities', side_effect=RuntimeError('stale planner; Rebuild')), \
                    self.assertRaisesRegex(RuntimeError, 'stale planner; Rebuild'):
                E.full_preflight([], repo / 'results')
            self.assertFalse((repo / 'results').exists())


if __name__ == '__main__':
    unittest.main()
