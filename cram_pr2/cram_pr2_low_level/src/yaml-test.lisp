(cpl:top-level (pr2-ll::call-giskard-yaml-action   :yaml-string 
                                                   "#
# Copyright (C) 2015-2016 Georg Bartels <georg.bartels@cs.uni-bremen.de>
#
# This file is part of giskard.
#
# giskard is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#

scope:
  # definition of some nice short-cuts
  - unit_x: {vector3: [1, 0, 0]}
  - unit_y: {vector3: [0, 1, 0]}
  - unit_z: {vector3: [0, 0, 1]}
  - pi: 3.14159265359
  - two_pi: {double-mul: [pi, 2.0]}

  # definition of joint input variables
  - torso_lift_joint: {input-var: 0}
  - l_shoulder_pan_joint: {input-var: 1}
  - l_shoulder_lift_joint: {input-var: 2}
  - l_upper_arm_roll_joint: {input-var: 3}
  - l_elbow_flex_joint: {input-var: 4}
  - l_forearm_roll_joint: {input-var: 5}
  - l_wrist_flex_joint: {input-var: 6}
  - l_wrist_roll_joint: {input-var: 7}
  - r_shoulder_pan_joint: {input-var: 8}
  - r_shoulder_lift_joint: {input-var: 9}
  - r_upper_arm_roll_joint: {input-var: 10}
  - r_elbow_flex_joint: {input-var: 11}
  - r_forearm_roll_joint: {input-var: 12}
  - r_wrist_flex_joint: {input-var: 13}
  - r_wrist_roll_joint: {input-var: 14}

  # definition goal input variables
  - torso_lift_goal: 0.30
  - l_shoulder_pan_joint_goal: 0.962039
  - l_shoulder_lift_joint_goal: 0.150617
  - l_upper_arm_roll_joint_goal: 1.56769
  - l_elbow_flex_joint_goal: -1.41351
  - l_forearm_roll_joint_goal: -6.01118
  - l_wrist_flex_joint_goal: -1.41351
  - l_wrist_roll_joint_goal: 4.70187
  - r_shoulder_pan_joint_goal: -0.556869
  - r_shoulder_lift_joint_goal: 0.240943
  - r_upper_arm_roll_joint_goal: -1.57977
  - r_elbow_flex_joint_goal: -1.02886
  - r_forearm_roll_joint_goal: -94.3445
  - r_wrist_flex_joint_goal: -1.18356
  - r_wrist_roll_joint_goal: 1.377

  # control params

  # definition goals and control laws
  - torso_lift_error: {double-sub: [torso_lift_goal, torso_lift_joint]}
  - l_shoulder_pan_error: {double-sub: [l_shoulder_pan_joint_goal, l_shoulder_pan_joint]}
  - l_shoulder_lift_error: {double-sub: [l_shoulder_lift_joint_goal, l_shoulder_lift_joint]}
  - l_upper_arm_roll_error: {double-sub: [l_upper_arm_roll_joint_goal, l_upper_arm_roll_joint]}
  - l_elbow_flex_error: {double-sub: [l_elbow_flex_joint_goal, l_elbow_flex_joint]}
  - l_forearm_roll_error_unnormalized: {double-sub: [l_forearm_roll_joint_goal, l_forearm_roll_joint]}
  - l_forearm_roll_error_normalized: 
      fmod: 
      - {double-add: [{fmod: [l_forearm_roll_error_unnormalized, two_pi]}, two_pi]}
      - two_pi
  - l_forearm_roll_error: 
      double-if: 
        - {double-sub: [l_forearm_roll_error_normalized, pi]}
        - {double-sub: [l_forearm_roll_error_normalized, two_pi]}
        - l_forearm_roll_error_normalized
  - l_wrist_flex_error: {double-sub: [l_wrist_flex_joint_goal, l_wrist_flex_joint]}
  - l_wrist_roll_error_unnormalized: {double-sub: [l_wrist_roll_joint_goal, l_wrist_roll_joint]}
  - l_wrist_roll_error_normalized: 
      fmod: 
      - {double-add: [{fmod: [l_wrist_roll_error_unnormalized, two_pi]}, two_pi]}
      - two_pi
  - l_wrist_roll_error: 
      double-if: 
        - {double-sub: [l_wrist_roll_error_normalized, pi]}
        - {double-sub: [l_wrist_roll_error_normalized, two_pi]}
        - l_wrist_roll_error_normalized
  - r_shoulder_pan_error: {double-sub: [r_shoulder_pan_joint_goal, r_shoulder_pan_joint]}
  - r_shoulder_lift_error: {double-sub: [r_shoulder_lift_joint_goal, r_shoulder_lift_joint]}
  - r_upper_arm_roll_error: {double-sub: [r_upper_arm_roll_joint_goal, r_upper_arm_roll_joint]}
  - r_elbow_flex_error: {double-sub: [r_elbow_flex_joint_goal, r_elbow_flex_joint]}
  - r_forearm_roll_error_unnormalized: {double-sub: [r_forearm_roll_joint_goal, r_forearm_roll_joint]}
  - r_forearm_roll_error_normalized: 
      fmod: 
      - {double-add: [{fmod: [r_forearm_roll_error_unnormalized, two_pi]}, two_pi]}
      - two_pi
  - r_forearm_roll_error: 
      double-if: 
        - {double-sub: [r_forearm_roll_error_normalized, pi]}
        - {double-sub: [r_forearm_roll_error_normalized, two_pi]}
        - r_forearm_roll_error_normalized
  - r_wrist_flex_error: {double-sub: [r_wrist_flex_joint_goal, r_wrist_flex_joint]}
  - r_wrist_roll_error_unnormalized: {double-sub: [r_wrist_roll_joint_goal, r_wrist_roll_joint]}
  - r_wrist_roll_error_normalized: 
      fmod: 
      - {double-add: [{fmod: [r_wrist_roll_error_unnormalized, two_pi]}, two_pi]}
      - two_pi
  - r_wrist_roll_error: 
      double-if: 
        - {double-sub: [r_wrist_roll_error_normalized, pi]}
        - {double-sub: [r_wrist_roll_error_normalized, two_pi]}
        - r_wrist_roll_error_normalized

  # some constants
  - weight_arm_joints: 1.0
  - weight_pos_control: 20.0
  - weight_rot_control: 5.0
  - weight_elbow_control: 0.1
  - neg_vel_limit_arm_joints: -0.6
  - pos_vel_limit_arm_joints: 0.6


controllable-constraints:
  # torso joint
  - controllable-constraint: [-0.02, 0.02, 100.0, 0, torso_lift_joint]
  # left arm joints
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 1, l_shoulder_pan_joint]
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 2, l_shoulder_lift_joint]
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 3, l_upper_arm_roll_joint]
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 4, l_elbow_flex_joint]
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 5, l_forearm_roll_joint]
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 6, l_wrist_flex_joint]
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 7, l_wrist_roll_joint]
  # right arm joints
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 8, r_shoulder_pan_joint]
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 9, r_shoulder_lift_joint]
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 10, r_upper_arm_roll_joint]
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 11, r_elbow_flex_joint]
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 12, r_forearm_roll_joint]
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 13, r_wrist_flex_joint]
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 14, r_wrist_roll_joint]

soft-constraints:
  - soft-constraint: [torso_lift_error, torso_lift_error , weight_pos_control, torso_lift_joint, torso_lift_joint control slack]
  - soft-constraint: [l_shoulder_pan_error, l_shoulder_pan_error , weight_pos_control, l_shoulder_pan_joint, l_shoulder_pan_joint control slack]
  - soft-constraint: [l_shoulder_lift_error, l_shoulder_lift_error , weight_pos_control, l_shoulder_lift_joint, l_shoulder_lift_joint control slack]
  - soft-constraint: [l_upper_arm_roll_error, l_upper_arm_roll_error , weight_pos_control, l_upper_arm_roll_joint, l_upper_arm_roll_joint control slack]
  - soft-constraint: [l_elbow_flex_error, l_elbow_flex_error , weight_pos_control, l_elbow_flex_joint, l_elbow_flex_joint control slack]
  - soft-constraint: [l_forearm_roll_error, l_forearm_roll_error , weight_pos_control, l_forearm_roll_joint, l_forearm_roll_joint control slack]
  - soft-constraint: [l_wrist_flex_error, l_wrist_flex_error , weight_pos_control, l_wrist_flex_joint, l_wrist_flex_joint control slack]
  - soft-constraint: [l_wrist_roll_error, l_wrist_roll_error , weight_pos_control, l_wrist_roll_joint, l_wrist_roll_joint control slack]
  - soft-constraint: [r_shoulder_pan_error, r_shoulder_pan_error , weight_pos_control, r_shoulder_pan_joint, r_shoulder_pan_joint control slack]
  - soft-constraint: [r_shoulder_lift_error, r_shoulder_lift_error , weight_pos_control, r_shoulder_lift_joint, r_shoulder_lift_joint control slack]
  - soft-constraint: [r_upper_arm_roll_error, r_upper_arm_roll_error , weight_pos_control, r_upper_arm_roll_joint, r_upper_arm_roll_joint control slack]
  - soft-constraint: [r_elbow_flex_error, r_elbow_flex_error , weight_pos_control, r_elbow_flex_joint, r_elbow_flex_joint control slack]
  - soft-constraint: [r_forearm_roll_error, r_forearm_roll_error , weight_pos_control, r_forearm_roll_joint, r_forearm_roll_joint control slack]
  - soft-constraint: [r_wrist_flex_error, r_wrist_flex_error , weight_pos_control, r_wrist_flex_joint, r_wrist_flex_joint control slack]
  - soft-constraint: [r_wrist_roll_error, r_wrist_roll_error , weight_pos_control, r_wrist_roll_joint, r_wrist_roll_joint control slack]

hard-constraints:
  - hard-constraint:
      - {double-sub: [0.0115, torso_lift_joint]}
      - {double-sub: [0.325, torso_lift_joint]}
      - torso_lift_joint
  - hard-constraint:
      - {double-sub: [-0.5646, l_shoulder_pan_joint]}
      - {double-sub: [2.1353, l_shoulder_pan_joint]}
      - l_shoulder_pan_joint
  - hard-constraint:
      - {double-sub: [-0.3536, l_shoulder_lift_joint]}
      - {double-sub: [1.2963, l_shoulder_lift_joint]}
      -  l_shoulder_lift_joint
  - hard-constraint:
      - {double-sub: [-0.65, l_upper_arm_roll_joint]}
      - {double-sub: [3.75, l_upper_arm_roll_joint]}
      - l_upper_arm_roll_joint
  - hard-constraint:
      - {double-sub: [-2.1213, l_elbow_flex_joint]}
      - {double-sub: [-0.15, l_elbow_flex_joint]}
      - l_elbow_flex_joint
  - hard-constraint:
      - {double-sub: [-2.0, l_wrist_flex_joint]}
      - {double-sub: [-0.1, l_wrist_flex_joint]}
      - l_wrist_flex_joint
  - hard-constraint:
      - {double-sub: [-2.1353, r_shoulder_pan_joint]}
      - {double-sub: [0.5646, r_shoulder_pan_joint]}
      - r_shoulder_pan_joint
  - hard-constraint:
      - {double-sub: [-0.3536, r_shoulder_lift_joint]}
      - {double-sub: [1.2963, r_shoulder_lift_joint]}
      -  r_shoulder_lift_joint
  - hard-constraint:
      - {double-sub: [-3.75, r_upper_arm_roll_joint]}
      - {double-sub: [0.65, r_upper_arm_roll_joint]}
      - r_upper_arm_roll_joint
  - hard-constraint:
      - {double-sub: [-2.1213, r_elbow_flex_joint]}
      - {double-sub: [-0.15, r_elbow_flex_joint]}
      - r_elbow_flex_joint
  - hard-constraint:
      - {double-sub: [-2.0, r_wrist_flex_joint]}
      - {double-sub: [-0.1, r_wrist_flex_joint]}
      - r_wrist_flex_joint"   
                                                   :convergence-key-value-pairs
                                                   '("torso_lift_error" 0.005
                                                     "l_shoulder_pan_error" 0.01
                                                     "l_shoulder_lift_error" 0.01
                                                     "l_upper_arm_roll_error" 0.01
                                                     "l_elbow_flex_error" 0.01
                                                     "l_forearm_roll_error" 0.01
                                                     "l_wrist_flex_error" 0.01
                                                     "l_wrist_roll_error" 0.01
                                                     "r_shoulder_pan_error" 0.01
                                                     "r_shoulder_lift_error" 0.01
                                                     "r_upper_arm_roll_error" 0.01
                                                     "r_elbow_flex_error" 0.01
                                                     "r_forearm_roll_error" 0.01
                                                     "r_wrist_flex_error" 0.01
                                                     "r_wrist_roll_error" 0.01)))
