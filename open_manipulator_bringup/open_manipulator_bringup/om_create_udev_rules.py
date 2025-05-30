#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Wonho Yun

import os
import subprocess


def main():
    print('\nThis script copies the udev rule to /etc/udev/rules.d/')
    print('to configure the U2D2 device for the OpenMANIPULATOR.\n')

    try:
        udev_rule_path = os.path.join(
            subprocess.check_output(
                ['ros2', 'pkg', 'prefix', 'open_manipulator_bringup']
            ).decode().strip(),
            'share/open_manipulator_bringup/open-manipulator-cdc.rules'
        )
        subprocess.run(['sudo', 'cp', udev_rule_path, '/etc/udev/rules.d/'], check=True)

        print('\nReloading rules\n')
        subprocess.run(['sudo', 'udevadm', 'control', '--reload-rules'], check=True)
        subprocess.run(['sudo', 'udevadm', 'trigger'], check=True)

        print('\nUdev rules successfully updated.\n')
    except Exception as e:
        print(f'An error occurred: {e}')


if __name__ == '__main__':
    main()
