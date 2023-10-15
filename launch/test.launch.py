#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
	suggester = launch_ros.actions.Node(package='wili_suggester', executable='suggester', output='screen')
	baum_welch = launch_ros.actions.Node(package='wili_hmm', executable='baum_welch', output='screen')
	db_proxy = launch_ros.actions.Node(package='wili_io', executable='db_proxy', output='screen')
	socket_bridge = launch_ros.actions.Node(package='wili_io', executable='socket_bridge', output='screen')

	return launch.LaunchDescription([suggester, baum_welch, db_proxy, socket_bridge])