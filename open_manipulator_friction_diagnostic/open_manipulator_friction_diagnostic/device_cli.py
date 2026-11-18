"""Inspect/init/import local OMY data without opening any hardware interface."""
import argparse
import json
import yaml

from .artifact import import_calibration, resource_path
from .device_identity import data_directory, robot_unit


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('show', help='Show resolved device ID and storage paths')
    commands.add_parser('init', help='Create optional local sensor settings; never overwrite')
    command = commands.add_parser('import-calibration', help='Import a matching-unit legacy file; never overwrite')
    command.add_argument('path')
    args = parser.parse_args(argv)
    folder = data_directory()
    if args.command == 'init':
        data = yaml.safe_load(resource_path('config/device.example.yaml').read_text())
        data['robot_unit'] = robot_unit()
        folder.mkdir(parents=True, exist_ok=True)
        with (folder/'device.yaml').open('x') as stream:
            yaml.safe_dump(data, stream, sort_keys=False)
    elif args.command == 'import-calibration':
        print(import_calibration(args.path))
    print(json.dumps(dict(robot_unit=robot_unit(), device_config=str(folder/'device.yaml'),
        calibration_file=str(folder/'current.yaml'), collection=str(folder/'collection'),
        calibration_exists=(folder/'current.yaml').is_file()), indent=2))


if __name__ == '__main__':
    main()
