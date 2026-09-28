#!/usr/bin/env python3
"""Rebuild or validate the material-separated OMX assets locally."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--repo', type=Path, default=HERE.parents[2])
parser.add_argument('--source-models', type=Path, help='Directory containing leader/follower .glb and .json inputs')
parser.add_argument('--blender', help='Blender executable for reproducible visual decimation')
parser.add_argument('--output', type=Path, required=True, help='External build/QA scratch directory')
parser.add_argument('--base-revision', default='d31000d90c679af9c982e73de8b12d777c5ff7dd')
parser.add_argument('--validate-only', action='store_true')
args = parser.parse_args()
if not args.validate_only and (not args.source_models or not args.blender):
    parser.error('Rebuilding requires --source-models and --blender.')
args.output.mkdir(parents=True, exist_ok=True)
env = dict(os.environ, OMX_REPO_ROOT=str(args.repo.resolve()),
           OMX_BUILD_DIR=str(args.output.resolve()), OMX_BASE_REVISION=args.base_revision)
if args.source_models:
    env['OMX_MODEL_DIR'] = str(args.source_models.resolve())

def run(script):
    subprocess.run([sys.executable, str(HERE / script)], env=env, check=True)

if not args.validate_only:
    run('prepare_decimation.py')
    subprocess.run([args.blender, '-b', '--factory-startup', '--python', str(HERE / 'blender_decimate.py'),
                    '--', str(args.output.resolve() / 'decimation')], env=env, check=True)
    run('build_assets.py')
    run('sync_static_urdf.py')
    run('build_leader_mjcf.py')
    run('geometry_qa.py')
run('validate_assets.py')
