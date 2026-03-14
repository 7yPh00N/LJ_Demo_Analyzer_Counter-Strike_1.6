import os
import re
import sys
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
import math
from py_goldsrc_demo.CS16DemoParser import CS16DemoParser
from matplotlib.ticker import MultipleLocator
from matplotlib.patches import Rectangle
from matplotlib.collections import LineCollection
import tkinter as tk
from tkinter import filedialog
import io

def normalize_angle(angle):
    angle = angle % 360
    if angle < 0:
        angle += 360
    return angle

def calculate_angle_difference(angle1, angle2):
    diff = angle2 - angle1
    diff = (diff + 180) % 360 - 180
    return diff

def calculate_gain_for_theta(V, theta_deg, accelspeed=25):
    theta_rad = math.radians(theta_deg)
    currentspeed = V * math.cos(theta_rad)
    addspeed = 30 - currentspeed
    gain = 0
    V_new = V
    if addspeed <= 0:
        addspeed = 0
        V_new = V
        gain = 0
    else:
        if accelspeed > addspeed:
            accelspeed_actual = addspeed
        else:
            accelspeed_actual = accelspeed
        V_new = math.sqrt(V**2 + accelspeed_actual**2 + 2 * V * accelspeed_actual * math.cos(theta_rad))
        gain = V_new - V
    return gain

def find_jump_ground_frames(dem_file_path):
    print(f"\nFinding frames with +jump and on_ground=1...")
    frame_on_ground = {}
    frame_has_jump = {}
    try:
        with open(dem_file_path, 'rb') as f:
            old_stdout = sys.stdout
            sys.stdout = io.StringIO()
            cs16_parser = CS16DemoParser(f)
            sys.stdout = old_stdout
            current_frame = None
            pending_jump = False
            for directory in cs16_parser.directories:
                for event in directory.macros:
                    event_str = str(event)
                    try:
                        if "ConsoleCommandMacro" in event_str:
                            cmd_match = re.search(r"command: ([+\-]?\w+)", event_str)
                            if cmd_match:
                                command = cmd_match.group(1)
                                if command == "+jump":
                                    pending_jump = True
                        elif "ClientDataMacro" in event_str:
                            frame_match = re.search(r'frame: (\d+)', event_str)
                            if frame_match:
                                current_frame = int(frame_match.group(1))
                                if pending_jump:
                                    frame_has_jump[current_frame] = True
                                    pending_jump = False
                                else:
                                    if current_frame not in frame_has_jump:
                                        frame_has_jump[current_frame] = False
                        elif "NetMsgMacro" in event_str and current_frame is not None:
                            on_ground_match = re.search(r'on_ground: (\d+)', event_str)
                            if on_ground_match:
                                on_ground_state = int(on_ground_match.group(1))
                                frame_on_ground[current_frame] = on_ground_state
                    except Exception as e:
                        continue
            for frame in frame_has_jump.keys():
                if frame not in frame_on_ground:
                    found = False
                    for offset in range(1, 6):
                        if frame - offset in frame_on_ground:
                            frame_on_ground[frame] = frame_on_ground[frame - offset]
                            found = True
                            break
                    if not found:
                        frame_on_ground[frame] = 0
            jump_frames = []
            sorted_frames = sorted(set(list(frame_has_jump.keys()) + list(frame_on_ground.keys())))
            last_valid_jump_frame = None
            for frame in sorted_frames:
                has_jump = frame_has_jump.get(frame, False)
                on_ground = frame_on_ground.get(frame, 0)
                if has_jump and on_ground == 1:
                    if last_valid_jump_frame is not None and frame == last_valid_jump_frame + 1:
                        print(f"  Frame {frame}: +jump and on_ground=1")
                    else:
                        jump_frames.append(frame)
                        last_valid_jump_frame = frame
                        print(f"  Frame {frame}: +jump and on_ground=1")
        return sorted(set(jump_frames))
    except Exception as e:
        print(f"Error finding jump frames: {str(e)}")
        return []

def parse_dem_file(dem_file_path, start_frame=None, end_frame=None, force_scale_frames=None, min_frame=None):
    if end_frame is None:
        parse_all = True
    else:
        parse_all = False
    if min_frame is None:
        min_frame_used = start_frame if parse_all else 0
    else:
        min_frame_used = min_frame
    positions = []
    velocities = []
    all_rotations = []
    forward_moves = []
    side_moves = []
    air_accel_times = []
    times = []
    on_ground_dict = {}
    try:
        with open(dem_file_path, 'rb') as f:
            old_stdout = sys.stdout
            sys.stdout = io.StringIO()
            cs16_parser = CS16DemoParser(f)
            sys.stdout = old_stdout
            print(f"Parsing DEMO file: {os.path.basename(dem_file_path)}")
            print(f"Map: {cs16_parser.map_name}")
            if parse_all:
                print(f"Parsing all frames from frame {min_frame_used} onwards...")
            else:
                print(f"Frame Range: {start_frame} - {end_frame}")
            frame_data = {}
            current_frame = None
            current_data = {}
            duck_commands = []
            events = []
            pending_commands = []
            for directory in cs16_parser.directories:
                for event in directory.macros:
                    event_str = str(event)
                    try:
                        if "ConsoleCommandMacro" in event_str:
                            cmd_match = re.search(r"command: ([+\-]?\w+)", event_str)
                            if cmd_match:
                                command = cmd_match.group(1)
                                if command in ["+duck", "-duck"]:
                                    pending_commands.append(command)
                        elif "ClientDataMacro" in event_str:
                            frame_match = re.search(r'frame: (\d+)', event_str)
                            if frame_match:
                                frame_found = int(frame_match.group(1))
                                for cmd in pending_commands:
                                    events.append(("ConsoleCommandMacro", frame_found, cmd))
                                pending_commands = []
                                events.append(("ClientDataMacro", frame_found, None))
                    except Exception as e:
                        continue
            if pending_commands and events:
                last_frame = events[-1][1] if events[-1][1] is not None else 0
                for cmd in pending_commands:
                    events.append(("ConsoleCommandMacro", last_frame, cmd))
            for event_type, frame, command in events:
                if event_type == "ConsoleCommandMacro" and frame is not None:
                    duck_commands.append((frame, command))
            duck_commands.sort(key=lambda x: x[0])
            for frame, cmd in duck_commands:
                print(f"  Frame {frame}: {cmd}")
            same_frame_duck_events = {}
            for frame, cmd in duck_commands:
                if frame not in same_frame_duck_events:
                    same_frame_duck_events[frame] = []
                same_frame_duck_events[frame].append(cmd)
            same_frame_both = []
            for frame, cmds in same_frame_duck_events.items():
                if "+duck" in cmds and "-duck" in cmds:
                    same_frame_both.append(frame)
            duck_intervals = []
            plus_duck_frames = [f for f, cmd in duck_commands if cmd == "+duck"]
            minus_duck_frames = [f for f, cmd in duck_commands if cmd == "-duck"]
            plus_index = 0
            minus_index = 0
            processed_frames = set()
            while plus_index < len(plus_duck_frames) and minus_index < len(minus_duck_frames):
                plus_frame = plus_duck_frames[plus_index]
                minus_frame = minus_duck_frames[minus_index]
                if plus_frame in same_frame_both:
                    plus_index += 1
                    continue
                if minus_frame > plus_frame:
                    start_frame_interval = plus_frame + 1
                    end_frame_interval = minus_frame + 1
                    duck_intervals.append((start_frame_interval, end_frame_interval))
                    processed_frames.add(plus_frame)
                    processed_frames.add(minus_frame)
                    plus_index += 1
                    minus_index += 1
                else:
                    minus_index += 1
            for frame in same_frame_both:
                duck_intervals.append((frame + 1, frame + 1))
            for i in range(plus_index, len(plus_duck_frames)):
                plus_frame = plus_duck_frames[i]
                if plus_frame not in same_frame_both:
                    duck_intervals.append((plus_frame + 1, float('inf')))
            if duck_intervals:
                duck_intervals.sort(key=lambda x: x[0])
                merged_intervals = []
                current_start, current_end = duck_intervals[0]
                for i in range(1, len(duck_intervals)):
                    start, end = duck_intervals[i]
                    if start <= current_end + 1 or (current_end == float('inf') and start != float('inf')):
                        current_end = max(current_end, end)
                    else:
                        merged_intervals.append((current_start, current_end))
                        current_start, current_end = start, end
                merged_intervals.append((current_start, current_end))
                duck_intervals = merged_intervals
            current_frame = None
            current_data = {}
            for directory in cs16_parser.directories:
                for event in directory.macros:
                    event_str = str(event)
                    try:
                        if "ClientDataMacro" in event_str:
                            frame_match = re.search(r'frame: (\d+)', event_str)
                            if frame_match:
                                current_frame = int(frame_match.group(1))
                                current_data = {
                                    'frame': current_frame,
                                    'frame_time': 0.0,
                                    'air_accelerate': 0.0,
                                    'time': 0.0
                                }
                                position_match = re.search(r'position: Vector3\(x: ([^,]+), y: ([^,]+), z: ([^)]+)\)', event_str)
                                if position_match:
                                    current_data['position'] = (
                                        float(position_match.group(1)),
                                        float(position_match.group(2)),
                                        float(position_match.group(3))
                                    )
                                else:
                                    current_data['position'] = (0, 0, 0)
                                rotation_match = re.search(r'rotation:\s*Rotation\s*\(\s*pitch:\s*([^,]+),\s*yaw:\s*([^,]+),\s*roll:\s*([^)]+)\)', event_str)
                                if rotation_match:
                                    current_data['rotation'] = (
                                        float(rotation_match.group(1)),
                                        float(rotation_match.group(2)),
                                        float(rotation_match.group(3))
                                    )
                                else:
                                    rotation_match = re.search(r'rotation:\s*Rotation\s*\(pitch:\s*([^,]+),\s*yaw:\s*([^,]+),\s*roll:\s*([^)]+)\)', event_str)
                                    if rotation_match:
                                        current_data['rotation'] = (
                                            float(rotation_match.group(1)),
                                            float(rotation_match.group(2)),
                                            float(rotation_match.group(3))
                                        )
                                    else:
                                        print(f"Warning: Could not parse rotation for frame {current_frame}, using default")
                                        current_data['rotation'] = (0, 0, 0)
                                current_data['velocity'] = (0, 0, 0)
                                current_data['forward_move'] = 0.0
                                current_data['side_move'] = 0.0
                                current_data['duck_state'] = False
                        elif "NetMsgMacro" in event_str and current_frame is not None:
                            frame_time_match = re.search(r'frame_time: ([^,]+)', event_str)
                            if frame_time_match:
                                current_data['frame_time'] = float(frame_time_match.group(1))
                            air_accel_match = re.search(r'air_accelerate: ([^,]+)', event_str)
                            if air_accel_match:
                                current_data['air_accelerate'] = float(air_accel_match.group(1))
                            time_match = re.search(r'\btime:\s*([0-9.]+)', event_str)
                            if time_match:
                                current_data['time'] = float(time_match.group(1))
                            else:
                                print(f"Warning: Could not parse 'time' for frame {current_frame}")
                            on_ground_match = re.search(r'on_ground: (\d+)', event_str)
                            if on_ground_match:
                                on_ground_state = int(on_ground_match.group(1))
                                on_ground_dict[current_frame] = on_ground_state
                            velocity_match = re.search(r'sim_vel: Vector3\(x: ([^,]+), y: ([^,]+), z: ([^)]+)\)', event_str)
                            if velocity_match:
                                vx = float(velocity_match.group(1))
                                vy = float(velocity_match.group(2))
                                vz = float(velocity_match.group(3))
                                current_data['velocity'] = (vx, vy, vz)
                            else:
                                current_data['velocity'] = (0, 0, 0)
                            forward_match = re.search(r'forward_move: ([^,]+)', event_str)
                            if forward_match:
                                forward_move_val = float(forward_match.group(1))
                                current_data['forward_move'] = forward_move_val
                            else:
                                current_data['forward_move'] = 0.0
                            side_match = re.search(r'side_move: ([^,]+)', event_str)
                            if side_match:
                                side_move_val = float(side_match.group(1))
                                current_data['side_move'] = side_move_val
                            else:
                                current_data['side_move'] = 0.0
                            if current_frame is not None:
                                frame_data[current_frame] = current_data.copy()
                                air_accel_times.append((current_frame,
                                                        current_data['air_accelerate'],
                                                        current_data['frame_time']))
                                times.append((current_frame, current_data['time']))
                            current_frame = None
                            current_data = {}
                    except Exception as e:
                        continue
            sorted_frames = sorted(frame_data.keys())
            for frame in sorted_frames:
                if parse_all:
                    if frame >= min_frame_used:
                        include_frame = True
                    else:
                        include_frame = False
                else:
                    if start_frame <= frame <= end_frame:
                        include_frame = True
                    else:
                        include_frame = False
                if include_frame:
                    data = frame_data[frame]
                    duck_state = False
                    for start_f, end_f in duck_intervals:
                        if start_f <= frame <= end_f:
                            duck_state = True
                            break
                    if duck_state:
                        original_forward = data['forward_move']
                        original_side = data['side_move']
                        data['forward_move'] = original_forward * 0.333
                        data['side_move'] = original_side * 0.333
                    if force_scale_frames and frame in force_scale_frames:
                        if not duck_state:
                            data['forward_move'] = data['forward_move'] * 0.333
                            data['side_move'] = data['side_move'] * 0.333
                    positions.append((frame, data['position'][0], data['position'][1], data['position'][2]))
                    velocities.append((frame, data['velocity'][0], data['velocity'][1], data['velocity'][2]))
                    forward_moves.append((frame, data['forward_move']))
                    side_moves.append((frame, data['side_move']))
                if frame in frame_data:
                    data = frame_data[frame]
                    all_rotations.append((frame, data['rotation'][0], data['rotation'][1], data['rotation'][2]))
            print(f"Successfully parsed {len(positions)} frames")
    except FileNotFoundError:
        print(f"Error: File '{dem_file_path}' not found")
        return None, None, None, None, None, None, None, None
    except Exception as e:
        print(f"Error parsing DEMO file: {str(e)}")
        return None, None, None, None, None, None, None, None
    return positions, velocities, all_rotations, forward_moves, side_moves, air_accel_times, times, on_ground_dict

def calculate_yawspeed(all_rotations, start_frame, end_frame):
    yawspeeds = []
    yaw_dict = {}
    for frame, pitch, yaw, roll in all_rotations:
        yaw_normalized = normalize_angle(yaw)
        yaw_dict[frame] = yaw_normalized
    for frame in range(start_frame, end_frame + 1):
        if frame - 1 in yaw_dict and frame in yaw_dict:
            prev_yaw = yaw_dict[frame - 1]
            curr_yaw = yaw_dict[frame]
            yawspeed = calculate_angle_difference(prev_yaw, curr_yaw)
        else:
            yawspeed = 0
        yawspeeds.append((frame, yawspeed))
    return yawspeeds

def calculate_yaw_acceleration(all_rotations, start_frame, end_frame):
    yaw_accelerations = []
    extended_end = end_frame + 1
    extended_yawspeeds = calculate_yawspeed(all_rotations, start_frame, extended_end)
    speed_dict = {frame: speed for frame, speed in extended_yawspeeds}
    for frame in range(start_frame, end_frame + 1):
        if frame in speed_dict and (frame + 1) in speed_dict:
            curr_speed = speed_dict[frame]
            next_speed = speed_dict[frame + 1]
            acceleration = next_speed - curr_speed
        else:
            acceleration = 0
        yaw_accelerations.append((frame, acceleration))
    return yaw_accelerations

def calculate_distance_predictions(positions, velocities, start_frame, end_frame, time_dict):
    distance_predictions = []
    if not positions or not velocities:
        return distance_predictions, 0
    start_pos = None
    for pos in positions:
        if pos[0] == start_frame:
            start_pos = pos
            break
    if not start_pos:
        return distance_predictions, 0
    X1, Y1 = start_pos[1], start_pos[2]
    pos_dict = {frame: (x, y) for frame, x, y, z in positions}
    vel_dict = {frame: (vx, vy) for frame, vx, vy, vz in velocities}
    end_pos = None
    for pos in positions:
        if pos[0] == end_frame:
            end_pos = pos
            break
    if end_pos:
        X2, Y2 = end_pos[1], end_pos[2]
        actual_displacement = math.sqrt((X2 - X1)**2 + (Y2 - Y1)**2)
        actual_distance = actual_displacement + 32
        print(X1, Y1, X2, Y2)
    else:
        actual_distance = 0

    if start_frame in time_dict and end_frame in time_dict:
        total_time = time_dict[end_frame] - time_dict[start_frame]
        use_real_time = True
    else:
        total_time = (end_frame - start_frame) * 0.01
        use_real_time = False
        print(f"Warning: Missing time data for distance prediction")

    for frame in range(start_frame, end_frame + 1):
        if frame in pos_dict and frame in vel_dict:
            x_curr, y_curr = pos_dict[frame]
            vx_curr, vy_curr = vel_dict[frame]
            if use_real_time and frame in time_dict:
                elapsed_time = time_dict[frame] - time_dict[start_frame]
                remaining_time = total_time - elapsed_time
            else:
                remaining_frames = end_frame - frame
                remaining_time = remaining_frames * 0.01
            if remaining_time > 0:
                pred_x = x_curr + vx_curr * remaining_time
                pred_y = y_curr + vy_curr * remaining_time
                pred_displacement = math.sqrt((pred_x - X1)**2 + (pred_y - Y1)**2)
                predicted_distance = pred_displacement + 32
            else:
                predicted_distance = actual_distance
            distance_predictions.append((frame, predicted_distance))
    return distance_predictions, actual_distance

def calculate_interp_distance_predictions(positions, velocities, start_frame, interp_end_pos, total_time, time_dict):
    distance_predictions = []
    if not positions or not velocities:
        return distance_predictions, 0
    start_pos = None
    for pos in positions:
        if pos[0] == start_frame:
            start_pos = pos
            break
    if not start_pos:
        return distance_predictions, 0
    X1, Y1 = start_pos[1], start_pos[2]
    Xe, Ye = interp_end_pos
    actual_displacement = math.sqrt((Xe - X1)**2 + (Ye - Y1)**2)
    actual_distance = actual_displacement + 32
    pos_dict = {frame: (x, y) for frame, x, y, z in positions}
    vel_dict = {frame: (vx, vy) for frame, vx, vy, vz in velocities}

    if start_frame in time_dict:
        use_real_time = True
    else:
        use_real_time = False
        print("Warning: Missing start time for interpolated distance prediction")

    for frame in range(start_frame, max(pos_dict.keys()) + 1):
        if frame in pos_dict and frame in vel_dict:
            x_curr, y_curr = pos_dict[frame]
            vx_curr, vy_curr = vel_dict[frame]
            if use_real_time and frame in time_dict:
                elapsed_time = time_dict[frame] - time_dict[start_frame]
            else:
                elapsed_time = (frame - start_frame) * 0.01
            remaining_time = total_time - elapsed_time
            if remaining_time > 0:
                pred_x = x_curr + vx_curr * remaining_time
                pred_y = y_curr + vy_curr * remaining_time
                pred_displacement = math.sqrt((pred_x - X1)**2 + (pred_y - Y1)**2)
                pred_distance = pred_displacement + 32
            else:
                pred_distance = actual_distance
            distance_predictions.append((frame, pred_distance))
    return distance_predictions, actual_distance

def calculate_move_angle(forward_move, side_move, yaw_deg):
    if forward_move == 0 and side_move == 0:
        return 0
    yaw_rad = math.radians(yaw_deg)
    forward_vec = (math.cos(yaw_rad), math.sin(yaw_rad))
    right_vec = (math.cos(yaw_rad - math.pi/2), math.sin(yaw_rad - math.pi/2))
    move_vec_x = forward_move * forward_vec[0] + side_move * right_vec[0]
    move_vec_y = forward_move * forward_vec[1] + side_move * right_vec[1]
    move_angle_rad = math.atan2(move_vec_y, move_vec_x)
    move_angle_deg = math.degrees(move_angle_rad)
    move_angle_deg = move_angle_deg % 360
    if move_angle_deg < 0:
        move_angle_deg += 360
    return move_angle_deg

def calculate_theta_for_frame(vx, vy, forward_move, side_move, yaw_deg):
    if vx == 0 and vy == 0:
        return 0
    if forward_move == 0 and side_move == 0:
        return 0
    move_angle_deg = calculate_move_angle(forward_move, side_move, yaw_deg)
    move_angle_rad = math.radians(move_angle_deg)
    vel_angle_rad = math.atan2(vy, vx)
    vel_angle_deg = math.degrees(vel_angle_rad)
    theta = calculate_angle_difference(vel_angle_deg, move_angle_deg)
    return theta

def calculate_angle_with_sign(vx, vy, displacement_x, displacement_y):
    if (vx == 0 and vy == 0) or (displacement_x == 0 and displacement_y == 0):
        return 0
    vel_angle_rad = math.atan2(vy, vx)
    vel_angle_deg = math.degrees(vel_angle_rad)
    disp_angle_rad = math.atan2(displacement_y, displacement_x)
    disp_angle_deg = math.degrees(disp_angle_rad)
    angle_diff = vel_angle_deg - disp_angle_deg
    angle_diff = angle_diff % 360
    if angle_diff > 180:
        angle_diff -= 360
    return angle_diff

def calculate_theta_and_accelspeed_with_current_move_and_prev_vel(velocities, forward_moves, side_moves, all_rotations, air_accel_times):
    theta_angles = []
    accelspeeds = []
    forward_dict = {frame: value for frame, value in forward_moves}
    side_dict = {frame: value for frame, value in side_moves}
    yaw_dict = {}
    for frame, pitch, yaw, roll in all_rotations:
        yaw_dict[frame] = normalize_angle(yaw)
    air_accel_dict = {frame: (air_accel, ft) for frame, air_accel, ft in air_accel_times}
    vel_list = sorted(velocities, key=lambda x: x[0])
    for i in range(1, len(vel_list)):
        current_frame, vx_curr, vy_curr, vz_curr = vel_list[i]
        prev_frame, vx_prev, vy_prev, vz_prev = vel_list[i-1]
        forward_move = forward_dict.get(current_frame, 0)
        side_move = side_dict.get(current_frame, 0)
        yaw = yaw_dict.get(current_frame, 0)
        theta = calculate_theta_for_frame(vx_prev, vy_prev, forward_move, side_move, yaw)
        theta_angles.append((current_frame, theta))
        move_magnitude = math.sqrt(forward_move**2 + side_move**2)
        air_accel, frame_time = air_accel_dict.get(current_frame, (0.0, 0.01))
        # accelspeed = move_magnitude * air_accel * frame_time
        accelspeed = move_magnitude * air_accel * 0.01
        accelspeeds.append((current_frame, accelspeed))
    return theta_angles, accelspeeds

def plot_data(positions, velocities, all_rotations, forward_moves, side_moves, air_accel_times, times,
              dem_file_path, start_frame=None, end_frame=None,
              interp_end_pos=None, interp_total_time=None,
              interp_performed=True, fraction=None):
    if end_frame is None:
        end_frame = start_frame + 73
    if not positions or not velocities or not all_rotations or not forward_moves or not side_moves:
        print("Error: No valid data to plot")
        return
    file_name = os.path.basename(dem_file_path)
    base_name = os.path.splitext(file_name)[0]
    plot_title = f"{base_name}.dem (@7yPh00N)"
    start_pos = None
    end_pos = None
    for pos in positions:
        if pos[0] == start_frame:
            start_pos = pos
        if pos[0] == end_frame:
            end_pos = pos
    if start_pos is None:
        return
    X1, Y1 = start_pos[1], start_pos[2]

    time_dict = {frame: t for frame, t in times}
    frame_time_dict = {frame: ft for frame, _, ft in air_accel_times}

    print(f"start_frame: {start_frame}, end_frame: {end_frame}")
    if start_frame in time_dict:
        print(f"time_dict[{start_frame}] = {time_dict[start_frame]}")
    else:
        print(f"time_dict[{start_frame}] = MISSING")
    if end_frame in time_dict:
        print(f"time_dict[{end_frame}] = {time_dict[end_frame]}")
    else:
        print(f"time_dict[{end_frame}] = MISSING")

    if interp_end_pos is not None and interp_total_time is not None:
        X2, Y2 = interp_end_pos
        if start_frame in time_dict and end_frame in time_dict:
            original_air_time = time_dict[end_frame] - time_dict[start_frame]
            if original_air_time < 0:
                print(f"Warning: original time difference negative ({original_air_time:.6f}s), using absolute value")
                original_air_time = abs(original_air_time)
            interp_air_time = interp_total_time
        else:
            original_air_time = (end_frame - start_frame) * 0.01
            interp_air_time = interp_total_time
            print(f"Warning: Missing time data for original frames, using fallbacks: orig={original_air_time:.6f}s, interp={interp_air_time:.6f}s")
        
        if interp_performed and fraction is not None:
            interp_frame_num = end_frame + fraction
            end_label = f"Frame {interp_frame_num:.6f}"
        elif interp_performed:
            interp_frame_num = start_frame + interp_total_time * 100
            end_label = f"Frame {interp_frame_num:.6f}"
        else:
            end_label = f"Frame {end_frame:.6f}"
        print(f"original_air_time = {original_air_time:.6f}s, interp_air_time = {interp_air_time:.6f}s")
    else:
        if end_pos is None:
            return
        X2, Y2 = end_pos[1], end_pos[2]
        if start_frame in time_dict and end_frame in time_dict:
            original_air_time = time_dict[end_frame] - time_dict[start_frame]
            if original_air_time < 0:
                print(f"Warning: time difference negative ({original_air_time:.6f}s), using absolute value")
                original_air_time = abs(original_air_time)
        else:
            original_air_time = (end_frame - start_frame) * 0.01
            print(f"Warning: Missing time data, using frame-based fallback: {original_air_time:.6f}s")
        interp_air_time = original_air_time
        end_label = f"Frame {end_frame:.6f}"

    print(f"Final Airtime (Orig) = {original_air_time:.6f}s")
    print(f"Displacement Endpoint: ({X2}, {Y2})")
    print(f"Start Point: ({X1}, {Y1})")

    Displacement_X = X2 - X1
    Displacement_Y = Y2 - Y1
    Displacement = math.sqrt(Displacement_X**2 + Displacement_Y**2)
    Distance = Displacement + 32
    swap_axes = abs(Displacement_Y) > abs(Displacement_X)
    if swap_axes:
        x_coords_for_plot = [pos[2] for pos in positions]
        y_coords_for_plot = [pos[1] for pos in positions]
        start_x, start_y = Y1, X1
        end_x, end_y = Y2, X2
        x_label = 'Y-Coordinate'
        y_label = 'X-Coordinate'
    else:
        x_coords_for_plot = [pos[1] for pos in positions]
        y_coords_for_plot = [pos[2] for pos in positions]
        start_x, start_y = X1, Y1
        end_x, end_y = X2, Y2
        x_label = 'X-Coordinate'
        y_label = 'Y-Coordinate'

    frames = []
    vxs = []
    vys = []
    signed_angles = []
    yaw_dict = {}
    for frame, pitch, yaw, roll in all_rotations:
        yaw_dict[frame] = yaw
    forward_dict = {frame: value for frame, value in forward_moves}
    side_dict = {frame: value for frame, value in side_moves}
    theta_angles, accelspeeds_list = calculate_theta_and_accelspeed_with_current_move_and_prev_vel(
        velocities, forward_moves, side_moves, all_rotations, air_accel_times)
    for vel in velocities:
        frame = vel[0]
        vx_val = vel[1]
        vy_val = vel[2]
        V = math.sqrt(vx_val**2 + vy_val**2)
        if V != 0 and Displacement != 0:
            signed_angle = calculate_angle_with_sign(vx_val, vy_val, Displacement_X, Displacement_Y)
            V_eff = V * math.cos(math.radians(signed_angle))
        else:
            signed_angle = 0
            V_eff = 0
        frames.append(frame)
        vxs.append(V_eff)
        vys.append(V)
        signed_angles.append(signed_angle)

    actual_gain_values = []
    actual_gain_frames = []
    for i in range(1, len(vys)):
        actual_gain = vys[i] - vys[i-1]
        actual_gain_values.append(actual_gain)
        actual_gain_frames.append(frames[i])

    theoretical_gain_values = []
    theoretical_gain_frames = []
    accelspeeds = [accel for _, accel in accelspeeds_list]
    for i in range(len(theta_angles)):
        frame, theta = theta_angles[i]
        prev_V = vys[i]
        accelspeed = accelspeeds[i]
        theoretical_gain = calculate_gain_for_theta(prev_V, theta, accelspeed)
        theoretical_gain_values.append(theoretical_gain)
        theoretical_gain_frames.append(frame)

    yawspeeds = calculate_yawspeed(all_rotations, start_frame, end_frame)
    yawspeed_frames = [frame for frame, _ in yawspeeds]
    yawspeed_values = [value for _, value in yawspeeds]
    yaw_accelerations = calculate_yaw_acceleration(all_rotations, start_frame, end_frame)
    yaw_accel_frames = [frame for frame, _ in yaw_accelerations]
    yaw_accel_values = [value for _, value in yaw_accelerations]

    distance_predictions, actual_distance = calculate_distance_predictions(
        positions, velocities, start_frame, end_frame, time_dict)
    pred_frames = [frame for frame, _ in distance_predictions]
    pred_distances = [distance for _, distance in distance_predictions]

    if interp_end_pos is not None and interp_total_time is not None:
        interp_predictions, interp_actual_distance = calculate_interp_distance_predictions(
            positions, velocities, start_frame, interp_end_pos, interp_total_time, time_dict)
        interp_pred_frames = [frame for frame, _ in interp_predictions]
        interp_pred_distances = [dist for _, dist in interp_predictions]
    else:
        interp_predictions = None
        interp_pred_frames = []
        interp_pred_distances = []

    region1_end = -95
    region2_end = -80
    region3_end = 80
    region4_end = 95
    region5_end = 180
    region1_display_height = 1/3*85/330
    region2_display_height = 1/3
    region3_display_height = 1/3*160/330
    region4_display_height = 1/3
    region5_display_height = 1/3*85/330
    display_bottom = 0
    display_region1_top = region1_display_height
    display_region2_top = display_region1_top + region2_display_height
    display_region3_top = display_region2_top + region3_display_height
    display_region4_top = display_region3_top + region4_display_height
    display_region5_top = display_region4_top + region5_display_height
    total_points = 10086
    region2_points = total_points // 3
    region4_points = total_points // 3
    remaining_points = total_points - region2_points - region4_points
    region1_points = remaining_points // 3
    region3_points = remaining_points // 3
    region5_points = remaining_points // 3
    theta_region1 = np.linspace(-180, region1_end, region1_points, endpoint=False)
    theta_region2 = np.linspace(region1_end, region2_end, region2_points, endpoint=False)
    theta_region3 = np.linspace(region2_end, region3_end, region3_points, endpoint=False)
    theta_region4 = np.linspace(region3_end, region4_end, region4_points, endpoint=False)
    theta_region5 = np.linspace(region4_end, region5_end, region5_points, endpoint=True)
    theta_range = np.concatenate([theta_region1, theta_region2, theta_region3, theta_region4, theta_region5])
    theta_rad = np.radians(theta_range)
    gain_data = np.zeros((len(theta_range), len(vys)-1))
    for i, (prev_V, accelspeed) in enumerate(zip(vys[:-1], accelspeeds)):
        currentspeed = prev_V * np.cos(theta_rad)
        addspeed = 30 - currentspeed
        positive_mask = addspeed > 0
        gains = np.zeros_like(theta_rad)
        if np.any(positive_mask):
            accelspeed_actual = np.where(accelspeed > addspeed[positive_mask], 
                                         addspeed[positive_mask], 
                                         accelspeed)
            V_new = np.sqrt(prev_V**2 + accelspeed_actual**2 + 
                          2 * prev_V * accelspeed_actual * np.cos(theta_rad[positive_mask]))
            gains[positive_mask] = V_new - prev_V
        gain_data[:, i] = gains
    actual_theta_frames = [frame for frame, _ in theta_angles]
    actual_theta_values = [theta for _, theta in theta_angles]
    accelspeed_frames = [frame for frame, _ in accelspeeds_list]
    accelspeed_values = [accel for _, accel in accelspeeds_list]

    fig = plt.figure(figsize=(25.7, 14.5))
    gs = GridSpec(3, 3, figure=fig, width_ratios=[1, 1, 1.4], height_ratios=[1, 1, 1])
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(frames, vxs, label='Effective Speed', linewidth=3, color=plt.cm.tab10(0))
    ax1.plot(frames, vys, label='Speed', linewidth=3, color=plt.cm.tab10(1))
    ax1.set_xlabel('Frame', fontsize=12)
    ax1.set_ylabel('Speed [units/s]', fontsize=12)
    ax1.set_title(f'{plot_title}', fontsize=15)
    ax1.legend()
    ax1.grid(True, linestyle='--')
    ax1.xaxis.set_major_locator(MultipleLocator(10))
    ax1.yaxis.set_major_locator(MultipleLocator(10))

    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(frames, signed_angles, color=plt.cm.tab10(6), linewidth=3)
    ax2.set_xlabel('Frame', fontsize=12)
    ax2.set_ylabel('Angle [degrees]', fontsize=12)
    ax2.set_title('Velocity-Displacement Angle', fontsize=15)
    ax2.grid(True, linestyle='--')
    ax2.xaxis.set_major_locator(MultipleLocator(10))
    ax2.yaxis.set_major_locator(MultipleLocator(5))

    ax3 = fig.add_subplot(gs[1, 0])
    rect_size = 32
    half_size = rect_size / 2
    start_rect = Rectangle((start_x - half_size, start_y - half_size), rect_size, rect_size,
                           edgecolor=plt.cm.tab10(0), facecolor=plt.cm.tab10(0), alpha=0.3, linewidth=2, zorder=1)
    end_rect = Rectangle((end_x - half_size, end_y - half_size), rect_size, rect_size,
                         edgecolor=plt.cm.tab10(1), facecolor=plt.cm.tab10(1), alpha=0.3, linewidth=2, zorder=1)
    ax3.add_patch(start_rect)
    ax3.add_patch(end_rect)
    ax3.plot([start_x, end_x], [start_y, end_y], '--', linewidth=2.5, alpha=0.7, color=plt.cm.tab10(7), zorder=2)
    speed_for_color = vys
    points = np.array([x_coords_for_plot, y_coords_for_plot]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    speeds_for_segments = speed_for_color[:-1]
    if len(vys) > 0:
        vmin, vmax = min(vys), max(vys)
    else:
        vmin, vmax = 265, 360
    lc = LineCollection(segments, cmap='viridis', norm=plt.Normalize(vmin, vmax), zorder=3)
    lc.set_array(np.array(speeds_for_segments))
    lc.set_linewidth(3)
    line = ax3.add_collection(lc)
    start_marker = ax3.plot(start_x, start_y, 'o', markersize=10, alpha=0.8, 
                            color=plt.cm.tab10(0), zorder=4)[0]
    end_marker = ax3.plot(end_x, end_y, 'o', markersize=10, alpha=0.8, 
                          color=plt.cm.tab10(1), zorder=4)[0]
    ax3.set_xlabel(x_label, fontsize=12)
    ax3.set_ylabel(y_label, fontsize=12)
    ax3.set_title('Trajectory', fontsize=15)
    ax3.legend(loc='upper left', handles=[start_marker, end_marker], 
               labels=[f'Frame {start_frame:.6f}', end_label])
    ax3.grid(True, linestyle='--')
    ax3.set_aspect('equal', adjustable='datalim')
    ax3.xaxis.set_major_locator(MultipleLocator(20))
    ax3.yaxis.set_major_locator(MultipleLocator(20))

    # 进行线性插值后，添加从最后一帧至插值点部分的线段（颜色与最后一帧相同）
    if interp_end_pos is not None and len(vys) > 1:
        last_actual_frame = end_frame
        last_actual_pos = None
        for pos in positions:
            if pos[0] == last_actual_frame:
                last_actual_pos = pos
                break
        if last_actual_pos is not None:
            if swap_axes:
                last_actual_x = last_actual_pos[2]
                last_actual_y = last_actual_pos[1]
            else:
                last_actual_x = last_actual_pos[1]
                last_actual_y = last_actual_pos[2]
            # 获取最后一帧的速度颜色
            last_speed = vys[-1]
            last_color = plt.cm.viridis((last_speed - vmin) / (vmax - vmin))

            ax3.plot([last_actual_x, end_x], [last_actual_y, end_y], 
                     color=last_color, linewidth=3, linestyle='-', zorder=3)

    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(yawspeed_frames, yawspeed_values, color=plt.cm.tab10(4), linewidth=3)
    ax4.set_xlabel('Frame', fontsize=12)
    ax4.set_ylabel('Yaw Speed [degrees/frame]', fontsize=12)
    ax4.set_title('Yaw Speed', fontsize=15)
    ax4.grid(True, linestyle='--')
    ax4.xaxis.set_major_locator(MultipleLocator(10))

    ax5 = fig.add_subplot(gs[2, 0])
    # ax5.plot(pred_frames, pred_distances, color=plt.cm.tab10(2), linewidth=3,
    #          label='Predicted (Original End Frame)')
    # ax5.axhline(y=actual_distance, color='red', linestyle='--', linewidth=2, alpha=0.7,
    #             label=f'Actual (Orig): {math.floor(actual_distance * 1000) / 1000:.3f} ({original_air_time:.6f}s)')
    if len(pred_distances) > 0:
        best_pred_frame = pred_frames[np.argmax(pred_distances)]
        best_pred_distance = max(pred_distances)
        # ax5.plot(best_pred_frame, best_pred_distance, 'o', markersize=10, alpha=0.8,
        #          color=plt.cm.tab10(6),
        #          label=f'Best (Orig): {math.floor(best_pred_distance * 1000) / 1000:.3f} (Frame {best_pred_frame})')
    
    if interp_predictions is not None:
        ax5.plot(interp_pred_frames, interp_pred_distances, color=plt.cm.tab10(7), linewidth=3,
                label='Predicted Distance (Constant Velocity)')

        ax5.axhline(y=interp_actual_distance, color='black', linestyle='--', linewidth=2, alpha=0.8,
                label=f'Actual Distance: {math.floor(interp_actual_distance * 1000) / 1000:.3f} ({interp_air_time:.6f}s)')
        
        if len(interp_pred_distances) > 0:
            best_interp_frame = interp_pred_frames[np.argmax(interp_pred_distances)]
            best_interp_distance = max(interp_pred_distances)
            ax5.plot(best_interp_frame, best_interp_distance, 'o', markersize=10, alpha=0.8,
                     color='red',
                     label=f'Best Prediction: {math.floor(best_interp_distance * 1000) / 1000:.3f} (Frame {best_interp_frame})')

    strafe_info = []
    if interp_predictions is not None and interp_pred_frames:
        pred_dict = {frame: val for frame, val in zip(interp_pred_frames, interp_pred_distances)}
        start_val = pred_dict.get(start_frame, 0)
        start_val_trunc = int(start_val * 1000) / 1000
        
        max_overall_val = max(interp_pred_distances) if interp_pred_distances else 0
        max_overall_trunc = int(max_overall_val * 1000) / 1000
        
        # 计算加速次数
        eligible_frames = []
        for frame in range(start_frame, end_frame + 1):
            fwd = forward_dict.get(frame, 0)
            sd = side_dict.get(frame, 0)
            if (fwd == 0 and sd != 0) or (sd == 0 and fwd != 0):
                if fwd != 0:
                    axis = 'fwd'
                    val = fwd
                else:
                    axis = 'sd'
                    val = sd
                sign = 1 if val > 0 else -1
                eligible_frames.append((frame, axis, sign))
        
        if eligible_frames:
            first_frame, first_axis, first_sign = eligible_frames[0]
            intervals = []
            strafes = 1
            current_axis = first_axis
            current_sign = first_sign
            interval_frames = [first_frame]

            for i in range(1, len(eligible_frames)):
                frame, axis, sign = eligible_frames[i]
                if axis == current_axis:
                    if sign != current_sign:
                        max_val = None
                        max_frame = None
                        for f in interval_frames:
                            if f in pred_dict:
                                v = pred_dict[f]
                                if max_val is None or v > max_val:
                                    max_val = v
                                    max_frame = f
                        if max_val is not None:
                            intervals.append((strafes, max_val, max_frame))
                        strafes += 1
                        current_sign = sign
                        interval_frames = [frame]
                    else:
                        interval_frames.append(frame)
                else:
                    interval_frames.append(frame)
            if interval_frames:
                max_val = None
                max_frame = None
                for f in interval_frames:
                    if f in pred_dict:
                        v = pred_dict[f]
                        if max_val is None or v > max_val:
                            max_val = v
                            max_frame = f
                if max_val is not None:
                    intervals.append((strafes, max_val, max_frame))
            
            for idx, (strafe_num, max_val, max_frame) in enumerate(intervals):
                if strafe_num == 1:
                    diff = max_val - start_val
                else:
                    prev_max = intervals[idx-1][1]
                    diff = max_val - prev_max
                
                diff_trunc = int(diff * 1000) / 1000
                if diff_trunc >= 0:
                    diff_str = f"+{diff_trunc:.3f}"
                else:
                    diff_str = f"{diff_trunc:.3f}"
                strafe_info.append((strafe_num, max_frame, max_val, diff_str))
            
            for strafe_num, max_frame, max_val, _ in strafe_info:
                ax5.plot(max_frame, max_val, 'o', markersize=8, color=plt.cm.tab10(3), alpha=0.7)
            
            if strafe_info:
                text_lines = []
                for strafe_num, _, max_val, diff_str in strafe_info:
                    num_str = f"{strafe_num:02d}"
                    max_trunc = int(max_val * 1000) / 1000
                    line = f"Strafe {num_str}: {max_trunc:.3f} ({diff_str})"
                    text_lines.append(line)
                
                text_str = '\n'.join(text_lines)
                
                ax5.text(0.98, 0.03, text_str, transform=ax5.transAxes,
                         verticalalignment='bottom', horizontalalignment='right',
                         bbox=dict(boxstyle='round,pad=0.3',
                                   facecolor='whitesmoke', 
                                   alpha=0.7,
                                   edgecolor='lightgray',
                                   linewidth=0.5),
                         fontdict={'family': 'monospace', 'size': 10})

    ax5.set_xlabel('Frame', fontsize=12)
    ax5.set_ylabel('Distance [units]', fontsize=12)
    ax5.set_title('Distance Prediction', fontsize=15)
    ax5.grid(True, linestyle='--')
    ax5.legend(loc='upper left', fontsize=9)
    ax5.xaxis.set_major_locator(MultipleLocator(10))

    ax6 = fig.add_subplot(gs[2, 1])
    ax6.plot(yaw_accel_frames, yaw_accel_values, color=plt.cm.tab10(9), linewidth=3)
    ax6.set_xlabel('Frame', fontsize=12)
    ax6.set_ylabel('Yaw Acceleration [degrees/frame²]', fontsize=12)
    ax6.set_title('Yaw Acceleration', fontsize=15)
    ax6.grid(True, linestyle='--')
    ax6.xaxis.set_major_locator(MultipleLocator(10))

    ax7 = fig.add_subplot(gs[:, 2])
    x_min = frames[1] - 0.5
    x_max = frames[-1] + 0.5
    display_positions = np.zeros_like(theta_range)
    for i, theta in enumerate(theta_range):
        if theta < region1_end:
            frac = (theta - (-180)) / (region1_end - (-180))
            display_positions[i] = display_bottom + frac * region1_display_height
        elif theta < region2_end:
            frac = (theta - region1_end) / (region2_end - region1_end)
            display_positions[i] = display_region1_top + frac * region2_display_height
        elif theta < region3_end:
            frac = (theta - region2_end) / (region3_end - region2_end)
            display_positions[i] = display_region2_top + frac * region3_display_height
        elif theta < region4_end:
            frac = (theta - region3_end) / (region4_end - region3_end)
            display_positions[i] = display_region3_top + frac * region4_display_height
        else:
            frac = (theta - region4_end) / (region5_end - region4_end)
            display_positions[i] = display_region4_top + frac * region5_display_height
    gain_data_nonnegative = gain_data.copy()
    gain_data_negative = gain_data.copy()
    gain_data_nonnegative[gain_data < 0] = np.nan
    gain_data_negative[gain_data >= 0] = np.nan
    x_coords = np.linspace(x_min, x_max, gain_data.shape[1] + 1)
    y_display_coords = np.zeros(len(display_positions) + 1)
    y_display_coords[0] = display_positions[0] - (display_positions[1] - display_positions[0])/2
    y_display_coords[-1] = display_positions[-1] + (display_positions[-1] - display_positions[-2])/2
    for i in range(1, len(display_positions)):
        y_display_coords[i] = (display_positions[i-1] + display_positions[i]) / 2
    X, Y = np.meshgrid(x_coords, y_display_coords)
    viridis_cmap = plt.cm.viridis.copy()
    viridis_cmap.set_under('white')
    im1 = ax7.pcolormesh(X, Y, gain_data_nonnegative, 
                        cmap=viridis_cmap,
                        shading='flat',
                        vmin=1e-6,
                        vmax=np.nanmax(gain_data))
    im2 = ax7.pcolormesh(X, Y, gain_data_negative, 
                        cmap='Greys_r',
                        shading='flat',
                        vmin=np.nanmin(gain_data),
                        vmax=0)
    actual_theta_display = []
    for theta in actual_theta_values:
        if theta < region1_end:
            frac = (theta - (-180)) / (region1_end - (-180))
            display_pos = display_bottom + frac * region1_display_height
        elif theta < region2_end:
            frac = (theta - region1_end) / (region2_end - region1_end)
            display_pos = display_region1_top + frac * region2_display_height
        elif theta < region3_end:
            frac = (theta - region2_end) / (region3_end - region2_end)
            display_pos = display_region2_top + frac * region3_display_height
        elif theta < region4_end:
            frac = (theta - region3_end) / (region4_end - region3_end)
            display_pos = display_region3_top + frac * region4_display_height
        else:
            frac = (theta - region4_end) / (region5_end - region4_end)
            display_pos = display_region4_top + frac * region5_display_height
        actual_theta_display.append(display_pos)
    filtered_theta_frames = []
    filtered_theta_display = []
    for frame, disp in zip(actual_theta_frames, actual_theta_display):
        fwd = forward_dict.get(frame, 0)
        sd = side_dict.get(frame, 0)
        if fwd != 0 or sd != 0:
            filtered_theta_frames.append(frame)
            filtered_theta_display.append(disp)
    ax7.scatter(filtered_theta_frames, filtered_theta_display, 
                color='red', s=15, alpha=0.8,
                edgecolors='white', linewidth=0.5,
                label='Actual θ (using current frame move, angle with prev frame velocity)', zorder=5)
    ax7.set_xlabel('Frame', fontsize=12)
    ax7.set_ylabel('Wishdir-Velocity Angle [degrees]', fontsize=12)
    ax7.set_title('Gain & Loss Heatmap', fontsize=15)
    key_angles = [-180, -135, -95, -90, -85, -80, -45, 0, 45, 80, 85, 90, 95, 135, 180]
    key_display_positions = []
    for angle in key_angles:
        if angle < region1_end:
            frac = (angle - (-180)) / (region1_end - (-180))
            display_pos = display_bottom + frac * region1_display_height
        elif angle < region2_end:
            frac = (angle - region1_end) / (region2_end - region1_end)
            display_pos = display_region1_top + frac * region2_display_height
        elif angle < region3_end:
            frac = (angle - region2_end) / (region3_end - region2_end)
            display_pos = display_region2_top + frac * region3_display_height
        elif angle < region4_end:
            frac = (angle - region3_end) / (region4_end - region3_end)
            display_pos = display_region3_top + frac * region4_display_height
        else:
            frac = (angle - region4_end) / (region5_end - region4_end)
            display_pos = display_region4_top + frac * region5_display_height
        key_display_positions.append(display_pos)
    ax7.set_yticks(key_display_positions)
    ax7.set_yticklabels([f'{angle}' for angle in key_angles])
    ax7.set_ylim(0, 1)
    ax7.grid(True, linestyle='--', alpha=0.3, which='both')
    ax7.axhline(y=display_region1_top, color='black', linestyle='-', linewidth=0.5, alpha=0.5)
    ax7.axhline(y=display_region2_top, color='black', linestyle='-', linewidth=0.5, alpha=0.5)
    ax7.axhline(y=display_region3_top, color='black', linestyle='-', linewidth=0.5, alpha=0.5)
    ax7.axhline(y=display_region4_top, color='black', linestyle='-', linewidth=0.5, alpha=0.5)

    plt.tight_layout()
    dem_dir = os.path.dirname(dem_file_path)
    output_dir = dem_dir
    os.makedirs(output_dir, exist_ok=True)
    output_file = f'{output_dir}/{base_name}.png'
    plt.savefig(output_file, dpi=100, bbox_inches='tight')
    print(f"Graph saved as: {output_file}")

    if len(vys) > 0:
        prestrafe = vys[0] if len(vys) > 0 else 0
        topspeed = max(vys) if len(vys) > 0 else 0
        gain = topspeed - prestrafe
        avg_speed = np.mean(vys) if len(vys) > 0 else 0
        avg_effective_speed = np.mean(vxs) if len(vxs) > 0 else 0
        avg_signed_angle = np.mean(signed_angles) if len(signed_angles) > 0 else 0
        cos_values = [math.cos(math.radians(angle)) for angle in signed_angles]
        avg_cos_angle = np.mean(cos_values) if len(cos_values) > 0 else 0
        yawspeed_abs_values = [abs(speed) for speed in yawspeed_values]
        avg_abs_yawspeed = np.mean(yawspeed_abs_values) if len(yawspeed_abs_values) > 0 else 0
        yaw_accel_abs_values = [abs(accel) for accel in yaw_accel_values]
        avg_abs_yaw_accel = np.mean(yaw_accel_abs_values) if len(yaw_accel_abs_values) > 0 else 0
        max_yaw_accel = max(yaw_accel_values) if len(yaw_accel_values) > 0 else 0
        min_yaw_accel = min(yaw_accel_values) if len(yaw_accel_values) > 0 else 0
        if len(pred_distances) > 0:
            max_pred_distance = max(pred_distances)
            min_pred_distance = min(pred_distances)
            avg_pred_distance = np.mean(pred_distances)
            std_pred_distance = np.std(pred_distances) if len(pred_distances) > 1 else 0
            best_pred_error = abs(max_pred_distance - actual_distance)
            relative_error = (best_pred_error / actual_distance * 100) if actual_distance > 0 else 0
        positive_gain_data = gain_data[gain_data >= 0]
        negative_gain_data = gain_data[gain_data < 0]
        max_positive_gain = np.max(positive_gain_data) if len(positive_gain_data) > 0 else 0
        min_positive_gain = np.min(positive_gain_data) if len(positive_gain_data) > 0 else 0
        avg_positive_gain = np.mean(positive_gain_data) if len(positive_gain_data) > 0 else 0
        max_negative_gain = np.max(negative_gain_data) if len(negative_gain_data) > 0 else 0
        min_negative_gain = np.min(negative_gain_data) if len(negative_gain_data) > 0 else 0
        avg_negative_gain = np.mean(negative_gain_data) if len(negative_gain_data) > 0 else 0
        max_gain_thetas = []
        for i in range(gain_data.shape[1]):
            max_gain_idx = np.argmax(gain_data[:, i])
            max_gain_thetas.append(theta_range[max_gain_idx])
        avg_optimal_theta = np.mean(max_gain_thetas) if len(max_gain_thetas) > 0 else 0
        actual_theta_values_array = np.array(actual_theta_values)
        avg_actual_theta = np.mean(actual_theta_values_array) if len(actual_theta_values_array) > 0 else 0
        std_actual_theta = np.std(actual_theta_values_array) if len(actual_theta_values_array) > 1 else 0
        max_actual_theta = np.max(actual_theta_values_array) if len(actual_theta_values_array) > 0 else 0
        min_actual_theta = np.min(actual_theta_values_array) if len(actual_theta_values_array) > 0 else 0
        avg_accelspeed = np.mean(accelspeed_values) if len(accelspeed_values) > 0 else 0
        max_accelspeed = max(accelspeed_values) if len(accelspeed_values) > 0 else 0
        min_accelspeed = min(accelspeed_values) if len(accelspeed_values) > 0 else 0
        gain_str = f"+{gain:.6f}" if gain > 0 else f"{gain:.6f}"
        print("\n" + "="*50)
        print(f"Prestrafe: {prestrafe:.6f}")
        print(f"Topspeed: {topspeed:.6f} ({gain_str})")
        print(f"AVG Speed: {avg_speed:.6f}")
        print(f"AVG Effective Speed: {avg_effective_speed:.6f}")
        print(f"AVG Angle: {avg_signed_angle:.6f}")
        print(f"AVG cos(Angle): {avg_cos_angle:.6f}")
        print(f"AVG |Yaw Speed|: {avg_abs_yawspeed:.6f}")
        print(f"AVG |Yaw Acceleration|: {avg_abs_yaw_accel:.6f}")
        print(f"Min Yaw Acceleration: {min_yaw_accel:.6f}")
        print(f"Max Yaw Acceleration: {max_yaw_accel:.6f}")
        # 输出实际滞空时间，区分原始和插值
        # print(f"Actual Distance (orig): {actual_distance:.6f} ({original_air_time:.6f}s)")
        # print(f"Best Prediction (orig): {max_pred_distance:.6f} (Frame {best_pred_frame})")
        # if interp_predictions is not None:
            # print(f"Actual Distance (interp): {interp_actual_distance:.6f} ({interp_air_time:.6f}s)")
            # print(f"Best Prediction (interp): {max(interp_pred_distances):.6f} (Frame {interp_pred_frames[np.argmax(interp_pred_distances)]})")
        print("="*50)
        if len(actual_gain_values) > 0 and len(theoretical_gain_values) > 0:
            max_actual_gain = max(actual_gain_values)
            min_actual_gain = min(actual_gain_values)
            std_actual_gain = np.std(actual_gain_values) if len(actual_gain_values) > 1 else 0
            max_theoretical_gain = max(theoretical_gain_values)
            min_theoretical_gain = min(theoretical_gain_values)
            std_theoretical_gain = np.std(theoretical_gain_values) if len(theoretical_gain_values) > 1 else 0
            if len(actual_gain_values) == len(theoretical_gain_values):
                min_len = min(len(actual_gain_values), len(theoretical_gain_values))
                actual_gain_array = np.array(actual_gain_values[:min_len])
                theoretical_gain_array = np.array(theoretical_gain_values[:min_len])
                if np.std(actual_gain_array) > 0 and np.std(theoretical_gain_array) > 0:
                    correlation = np.corrcoef(actual_gain_array, theoretical_gain_array)[0, 1]
                else:
                    correlation = 0
                mse = np.mean((actual_gain_array - theoretical_gain_array)**2)
                rmse = np.sqrt(mse)
                mae = np.mean(np.abs(actual_gain_array - theoretical_gain_array))
            else:
                correlation = 0
                rmse = 0
                mae = 0
            total_actual_gain = np.sum(actual_gain_values)
            total_theoretical_gain = np.sum(theoretical_gain_values)
            cumulative_actual_gain = np.cumsum(actual_gain_values)
            cumulative_theoretical_gain = np.cumsum(theoretical_gain_values)
            max_cumulative_actual_gain_idx = np.argmax(cumulative_actual_gain)
            max_cumulative_theoretical_gain_idx = np.argmax(cumulative_theoretical_gain)
            max_cumulative_actual_gain = cumulative_actual_gain[max_cumulative_actual_gain_idx]
            max_cumulative_theoretical_gain = cumulative_theoretical_gain[max_cumulative_theoretical_gain_idx]

    fig.canvas.manager.set_window_title('CS1.6_LJ_Demo_Analyzer_V1.2.0')
    plt.show()

def select_air_time():
    print("\nSelect Jump Type:")
    print("  1. LJ/HJ/CJ/DCJ/WJ")
    print("  2. Stand-Up BJ")
    print("  3. Stand-Up CJ")
    print("  4. Bhop Jump")
    print("  5. Enter Airtime Manually")
    while True:
        try:
            choice = input("\nSelect (1-5): ")
            if choice == '1':
                air_time = 73
                jump_type = "LJ/HJ/CJ/DCJ/WJ"
                break
            if choice == '2':
                air_time = 66
                jump_type = "Stand-Up BJ"
                break
            elif choice == '3':
                air_time = 73
                jump_type = "Stand-Up CJ"
                break
            elif choice == '4':
                air_time = 65
                jump_type = "Bhop Jump"
                break
            elif choice == '5':
                while True:
                    try:
                        air_time = int(input("Enter Custom Airtime Value (Integer): "))
                        if air_time > 0:
                            jump_type = f"Custom Jump (Airtime={air_time})"
                            break
                        else:
                            print("Airtime Must be a Positive Integer")
                    except ValueError:
                        print("Please Enter a Valid Integer")
                break
            else:
                print("Invalid Selection, Please Try Again")
        except Exception as e:
            print(f"Input Error: {str(e)}")
    print(f"Selected: {jump_type}, Airtime = {air_time} frames")
    return air_time, jump_type


def select_file():
    root = tk.Tk()
    root.withdraw()
    initial_dir = os.getcwd()
    file_path = filedialog.askopenfilename(
        title="Select DEMO File",
        initialdir=initial_dir,
        filetypes=[("DEMO files", "*.dem"), ("All files", "*.*")]
    )
    return file_path


def main():
    print("=" * 50)
    print("CS1.6 Long Jump Demo Analyzer V1.1.0 (@7yPh00N)")
    print("=" * 50)
    dem_file_path = select_file()
    if not dem_file_path:
        print("No file selected. Program will exit.")
        return
    if not os.path.exists(dem_file_path):
        print(f"Error: File '{dem_file_path}' does not exist")
        return
    if not dem_file_path.lower().endswith('.dem'):
        print("Warning: File Extension is not .dem")
    jump_frames = find_jump_ground_frames(dem_file_path)
    if not jump_frames:
        print("\nNo Frames Found with +jump and on_ground=1")
        start_frame = int(input("Enter Start Frame Manually: "))
    else:
        print(f"\nFound {len(jump_frames)} frames with +jump and on_ground=1:")
        for i, frame in enumerate(jump_frames, 1):
            print(f"  {i}. Frame {frame} [START]")
        print("\nSelect start frame:")
        print("  0. Enter Manually")
        for i, frame in enumerate(jump_frames, 1):
            print(f"  {i}. Frame {frame} [START]")
        while True:
            try:
                choice = input("\nSelect (0-{}): ".format(len(jump_frames)))
                choice = int(choice)
                if choice == 0:
                    start_frame = int(input("Enter Start Frame: "))
                    break
                elif 1 <= choice <= len(jump_frames):
                    start_frame = jump_frames[choice - 1]
                    print(f"Selected Frame {start_frame} as Start Frame")
                    break
                else:
                    print("Invalid Selection, Please Try Again")
            except ValueError:
                print("Please Enter a Valid Number")
            except Exception as e:
                print(f"Input Error: {str(e)}")
    air_time, jump_type = select_air_time()

    positions, velocities, all_rotations, forward_moves, side_moves, air_accel_times, times, on_ground_dict = parse_dem_file(
        dem_file_path, start_frame, end_frame=None, force_scale_frames=None, min_frame=0
    )
    if not positions or not velocities or not all_rotations or not forward_moves or not side_moves:
        print("Failed to parse DEMO file.")
        return

    time_dict = {frame: t for frame, t in times}
    frame_time_dict = {frame: ft for frame, _, ft in air_accel_times}

    pos_dict = {frame: (x, y, z) for frame, x, y, z in positions}

    land_z = None
    if jump_type in ["LJ/HJ/CJ/DCJ/WJ"]:
        start_z = None
        for pos in positions:
            if pos[0] == start_frame:
                start_z = pos[3]
                break
        if start_z is not None:
            land_z = start_z - 18
            print(f"Start Z = {start_z:.6f}, Land Z = {land_z:.6f} (start_z - 18)")
        else:
            print(f"Warning: Start frame {start_frame} not found")
            land_z = None
    elif jump_type in ["Stand-Up CJ"]:
        # Stand-Up CJ: land_z = start_z
        start_z = None
        for pos in positions:
            if pos[0] == start_frame:
                start_z = pos[3]
                break
        if start_z is not None:
            land_z = start_z
            print(f"Start Z = {start_z:.6f}, Land Z = {land_z:.6f} (Stand-Up CJ)")
        else:
            print(f"Warning: Start frame {start_frame} not found")
            land_z = None
    elif jump_type in ["Stand-Up BJ", "Bhop Jump"]:
        # 在未开启mpbhop时板子会下沉从而影响地面高度判断，这里使用前一次跳跃的Z坐标进行计算
        prev_jump_frame = None
        for frame in sorted(jump_frames):
            if frame < start_frame:
                prev_jump_frame = frame
            else:
                break
        if prev_jump_frame is not None:
            prev_z = None
            for pos in positions:
                if pos[0] == prev_jump_frame:
                    prev_z = pos[3]
                    break
            if prev_z is not None:
                land_z = prev_z - 18
                print(f"Previous jump frame: {prev_jump_frame}, Z = {prev_z:.2f}, Land Z = {land_z:.2f}")
            else:
                print(f"Warning: Previous jump frame {prev_jump_frame} not found")
                land_z = None
        else:
            print("Warning: No previous jump frame found")
            land_z = None
    else:
        land_z = None

    end_frame = start_frame + air_time
    print(f"Initial end frame: {end_frame}")

    # 判断默认结束帧本身是否已着陆
    if end_frame in on_ground_dict and on_ground_dict[end_frame] == 1:
        # 直接使用默认结束帧作为插值结束点（不进行插值计算）
        if end_frame in pos_dict:
            X3, Y3, Z3 = pos_dict[end_frame]
            interp_end_pos = (X3, Y3)
            if start_frame in time_dict and end_frame in time_dict:
                interp_total_time = time_dict[end_frame] - time_dict[start_frame]
            else:
                interp_total_time = (end_frame - start_frame) * 0.01
            interp_performed = False
            fraction = 0.0
            print(f"End frame {end_frame} is on ground. Using it as interpolated landing point (no interpolation)")
            print(f"Landing point: ({X3:.3f}, {Y3:.3f}, {Z3:.3f}), total time={interp_total_time:.3f}s")
        else:
            print(f"Warning: End frame {end_frame} not found in positions, cannot use as landing.")
            interp_end_pos = None
            interp_total_time = None
            interp_performed = True
            fraction = None
    else:
        if land_z is not None:
            z_dict = {frame: z for frame, x, y, z in positions}
            adjusted = True
            while adjusted:
                adjusted = False
                for frame in range(start_frame, end_frame + 1):
                    if frame in z_dict and z_dict[frame] <= land_z:
                        new_end = frame - 1
                        if new_end >= start_frame:
                            print(f"Adjusting end_frame: found landing at frame {frame} (z={z_dict[frame]:.6f} <= {land_z:.6f}), setting end_frame to {new_end}")
                            end_frame = new_end
                            adjusted = True
                            break
                        else:
                            print(f"Warning: landing frame {frame} is start frame, cannot adjust further.")
                            adjusted = False
                            break
            print(f"Final end frame after adjustment: {end_frame}")

            frame_A = None
            for pos in positions:
                if pos[0] == end_frame:
                    frame_A = pos
                    break
            vel_A = None
            for vel in velocities:
                if vel[0] == end_frame:
                    vel_A = vel
                    break
            if frame_A is not None and vel_A is not None:
                x_A, y_A, z_A = frame_A[1], frame_A[2], frame_A[3]
                Vx_A, Vy_A, Vz_A = vel_A[1], vel_A[2], vel_A[3]
                
                print(f"x_A = {repr(x_A)}")
                print(f"y_A = {repr(y_A)}")
                print(f"z_A = {repr(z_A)}")
                print(f"Vx_A = {repr(Vx_A)}")
                print(f"Vy_A = {repr(Vy_A)}")
                print(f"Vz_A = {repr(Vz_A)}")
                
                dt = frame_time_dict.get(end_frame, 0.01)
                x_B = x_A + Vx_A * dt
                y_B = y_A + Vy_A * dt
                z_B = z_A + Vz_A * dt
                if abs(z_A - z_B) < 1e-9:
                    fraction = 0.0
                else:
                    fraction = (z_A - land_z) / (z_A - z_B)
                X3 = x_A + (x_B - x_A) * fraction
                Y3 = y_A + (y_B - y_A) * fraction
                Z3 = z_A + (z_B - z_A) * fraction
                interp_end_pos = (X3, Y3)

                # 计算原始滞空时间
                if start_frame in time_dict and end_frame in time_dict:
                    original_air_time = time_dict[end_frame] - time_dict[start_frame]
                    if original_air_time < 0:
                        print(f"Warning: original time difference negative ({original_air_time:.6f}s), using absolute value")
                        original_air_time = abs(original_air_time)
                else:
                    original_air_time = (end_frame - start_frame) * 0.01
                    print(f"Warning: Missing time data, using frame-based original_air_time: {original_air_time:.6f}s")

                interp_total_time = original_air_time + fraction * dt
                interp_performed = True

                print(f"\n--- Interpolation Debug ---")
                print(f"original_air_time = {original_air_time:.6f}s")
                print(f"dt (frame_time of end_frame) = {dt:.6f}s")
                print(f"fraction = {fraction:.6f}")
                print(f"fraction * dt = {fraction * dt:.6f}s")
                print(f"interp_total_time = {interp_total_time:.6f}s")
                print(f"Interpolated landing point based on frame {end_frame}:")
                print(f"  ({X3:.3f}, {Y3:.3f}, {Z3:.3f})")
            else:
                print(f"Warning: Missing data for frame {end_frame}. Cannot interpolate.")
                interp_end_pos = None
                interp_total_time = None
                interp_performed = True
                fraction = None
        else:
            interp_end_pos = None
            interp_total_time = None
            interp_performed = True
            fraction = None

    # 设置显示范围
    positions = [p for p in positions if start_frame <= p[0] <= end_frame]
    velocities = [v for v in velocities if start_frame <= v[0] <= end_frame]
    forward_moves = [f for f in forward_moves if start_frame <= f[0] <= end_frame]
    side_moves = [s for s in side_moves if start_frame <= s[0] <= end_frame]
    air_accel_times = [a for a in air_accel_times if start_frame <= a[0] <= end_frame]
    times = [t for t in times if start_frame <= t[0] <= end_frame]

    # SCJ的前2帧是induck状态，乘以0.333
    force_scale_frames = []
    if jump_type == "Stand-Up CJ":
        force_scale_frames = [start_frame, start_frame + 1]
    if force_scale_frames:
        for i, (frm, fwd) in enumerate(forward_moves):
            if frm in force_scale_frames:
                forward_moves[i] = (frm, fwd * 0.333)
        for i, (frm, sd) in enumerate(side_moves):
            if frm in force_scale_frames:
                side_moves[i] = (frm, sd * 0.333)

    plot_data(positions, velocities, all_rotations, forward_moves, side_moves, air_accel_times, times,
              dem_file_path, start_frame, end_frame, interp_end_pos, interp_total_time,
              interp_performed=interp_performed, fraction=fraction)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram interrupted")
    except Exception as e:
        print(f"\nRuntime error: {str(e)}")
    input("\nPress Enter to Exit...")