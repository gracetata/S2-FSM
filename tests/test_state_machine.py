import unittest

from locomotion_controller.protocol import ArmCommand
from locomotion_controller.state_machine import (
    HIGH_MODE_ARM_STAND,
    HIGH_MODE_ARM_WALK,
    HIGH_MODE_NAVIGATION,
    HIGH_MODE_STAND_RECOVERY,
    HIGH_MODE_FREE_WALK_STAND,
    HIGH_MODE_EXTERNAL_CONTROL,
    LOW_MODE_TARGET_POSE,
    LOW_MODE_VELOCITY,
    MODEL_ACCURATE_ARRIVAL,
    MODEL_ARM_STAND,
    MODEL_ARM_WALK,
    MODEL_FREE_WALK,
    MODEL_STAND_RECOVERY,
    ZERO_COMMAND,
    LocomotionStateMachine,
)


class StateMachineTest(unittest.TestCase):
    def setUp(self):
        self.machine = LocomotionStateMachine(
            stand_duration_s=1.0,
            navigation_timeout_s=0.25,
            arm_timeout_s=0.20,
        )

    def test_waits_safely_before_first_mode(self):
        selection = self.machine.select(now=1.0)
        self.assertEqual(selection.model_name, MODEL_FREE_WALK)
        self.assertEqual(selection.command, ZERO_COMMAND)
        self.assertIsNone(selection.high_mode)

    def test_mode_one_routes_navigation_by_explicit_submode(self):
        self.machine.set_low_mode(LOW_MODE_VELOCITY)
        self.machine.set_high_mode(HIGH_MODE_NAVIGATION, now=1.0)

        standing = self.machine.select(now=1.5)
        self.assertTrue(standing.is_standing_transition)
        self.assertEqual(standing.model_name, MODEL_FREE_WALK)
        self.assertEqual(standing.command, ZERO_COMMAND)

        self.machine.set_navigation_command((0.3, 0.1, -0.2), now=2.0)
        velocity = self.machine.select(now=2.0)
        self.assertEqual(velocity.model_name, MODEL_FREE_WALK)
        self.assertEqual(velocity.command, (0.3, 0.1, -0.2))

        self.machine.set_low_mode(LOW_MODE_TARGET_POSE, now=2.0)
        immediate_transition = self.machine.select(now=2.0)
        self.assertEqual(
            immediate_transition.model_name,
            MODEL_FREE_WALK,
        )
        self.assertTrue(immediate_transition.is_standing_transition)

        low_mode_transition = self.machine.select(now=2.5)
        self.assertTrue(low_mode_transition.is_standing_transition)
        self.assertEqual(
            low_mode_transition.model_name,
            MODEL_FREE_WALK,
        )
        self.assertEqual(
            low_mode_transition.low_mode,
            LOW_MODE_TARGET_POSE,
        )
        self.assertEqual(low_mode_transition.command, ZERO_COMMAND)

        target_without_parameters = self.machine.select(now=3.0)
        self.assertEqual(
            target_without_parameters.model_name,
            MODEL_ACCURATE_ARRIVAL,
        )
        self.assertEqual(target_without_parameters.command, ZERO_COMMAND)

        self.machine.set_navigation_command((0.5, 0.0, 0.1), now=3.0)
        target = self.machine.select(now=3.0)
        self.assertEqual(target.model_name, MODEL_ACCURATE_ARRIVAL)
        self.assertEqual(target.command, (0.5, 0.0, 0.1))

    def test_repeated_high_mode_does_not_restart_standing(self):
        self.assertTrue(
            self.machine.set_high_mode(HIGH_MODE_NAVIGATION, now=1.0)
        )
        self.assertFalse(
            self.machine.set_high_mode(HIGH_MODE_NAVIGATION, now=1.8)
        )
        selection = self.machine.select(now=2.0)
        self.assertFalse(selection.is_standing_transition)
        self.assertEqual(selection.model_name, MODEL_FREE_WALK)

    def test_navigation_sent_before_mode_uses_zero_until_new_parameters(self):
        self.machine.set_low_mode(LOW_MODE_VELOCITY)
        self.machine.set_navigation_command((0.4, 0.0, 0.0), now=1.0)
        self.machine.set_high_mode(HIGH_MODE_NAVIGATION, now=1.0)

        selection = self.machine.select(now=2.0)
        self.assertEqual(selection.model_name, MODEL_FREE_WALK)
        self.assertEqual(selection.command, ZERO_COMMAND)

    def test_mode_one_low_one_fresh_zero_velocity_uses_free_walk(self):
        self.machine.set_low_mode(LOW_MODE_VELOCITY, now=0.0)
        self.machine.set_high_mode(HIGH_MODE_NAVIGATION, now=0.0)
        self.machine.set_navigation_command(ZERO_COMMAND, now=1.0)

        selection = self.machine.select(now=1.0)

        self.assertEqual(selection.model_name, MODEL_FREE_WALK)
        self.assertEqual(selection.command, ZERO_COMMAND)
        self.assertFalse(selection.is_standing_transition)

    def test_mode_one_low_one_missing_or_stale_velocity_uses_zero_free_walk(self):
        self.machine.set_low_mode(LOW_MODE_VELOCITY, now=0.0)
        self.machine.set_high_mode(HIGH_MODE_NAVIGATION, now=0.0)

        missing = self.machine.select(now=1.0)
        self.assertEqual(missing.model_name, MODEL_FREE_WALK)

        self.machine.set_navigation_command(ZERO_COMMAND, now=1.1)
        fresh = self.machine.select(now=1.1)
        self.assertEqual(fresh.model_name, MODEL_FREE_WALK)

        stale = self.machine.select(now=1.351)
        self.assertEqual(stale.model_name, MODEL_FREE_WALK)

    def test_mode_two_stands_then_uses_arm_stand_model(self):
        arm = ArmCommand(1, (0.1,) * 14, (0.0,) * 14, 1.0)
        self.machine.set_high_mode(HIGH_MODE_ARM_STAND, now=1.0)
        self.machine.set_arm_command(arm, now=2.0)
        selection = self.machine.select(now=2.0)
        self.assertEqual(selection.model_name, MODEL_ARM_STAND)
        self.assertEqual(selection.arm_command, arm)
        self.assertEqual(selection.command, ZERO_COMMAND)

    def test_mode_three_switches_directly_to_arm_walk(self):
        self.machine.set_high_mode(HIGH_MODE_ARM_WALK, now=1.0)
        selection = self.machine.select(now=1.0)
        self.assertEqual(selection.model_name, MODEL_ARM_WALK)
        self.assertFalse(selection.is_standing_transition)
        self.assertEqual(selection.command, ZERO_COMMAND)

    def test_mode_four_switches_directly_to_zero_command_recovery(self):
        self.machine.set_low_mode(LOW_MODE_TARGET_POSE, now=0.5)
        self.machine.set_navigation_command((0.5, 0.1, 0.2), now=0.5)
        self.machine.set_high_mode(HIGH_MODE_STAND_RECOVERY, now=1.0)

        selection = self.machine.select(now=1.0)

        self.assertEqual(selection.model_name, MODEL_STAND_RECOVERY)
        self.assertEqual(selection.high_mode, HIGH_MODE_STAND_RECOVERY)
        self.assertIsNone(selection.low_mode)
        self.assertEqual(selection.command, ZERO_COMMAND)
        self.assertIsNone(selection.arm_command)
        self.assertFalse(selection.is_standing_transition)

    def test_mode_five_switches_directly_to_zero_command_free_walk(self):
        self.machine.set_low_mode(LOW_MODE_TARGET_POSE, now=0.5)
        self.machine.set_navigation_command((0.5, 0.1, 0.2), now=0.5)
        self.machine.set_high_mode(HIGH_MODE_FREE_WALK_STAND, now=1.0)

        selection = self.machine.select(now=1.0)

        self.assertEqual(selection.model_name, MODEL_FREE_WALK)
        self.assertEqual(selection.high_mode, HIGH_MODE_FREE_WALK_STAND)
        self.assertIsNone(selection.low_mode)
        self.assertEqual(selection.command, ZERO_COMMAND)
        self.assertIsNone(selection.arm_command)
        self.assertFalse(selection.is_standing_transition)

    def test_mode_six_disables_s2_fsm_control_output(self):
        self.machine.set_low_mode(LOW_MODE_VELOCITY, now=0.5)
        self.machine.set_navigation_command((0.5, 0.1, 0.2), now=0.5)
        self.machine.set_high_mode(HIGH_MODE_EXTERNAL_CONTROL, now=1.0)

        selection = self.machine.select(now=1.0)

        self.assertEqual(selection.high_mode, HIGH_MODE_EXTERNAL_CONTROL)
        self.assertIsNone(selection.low_mode)
        self.assertEqual(selection.command, ZERO_COMMAND)
        self.assertIsNone(selection.arm_command)
        self.assertFalse(selection.is_standing_transition)
        self.assertFalse(selection.control_enabled)

    def test_mode_three_routes_low_one_velocity_and_arm_override(self):
        arm = ArmCommand(1, (0.1,) * 14, (0.2,) * 14, 0.8)
        self.machine.set_low_mode(LOW_MODE_VELOCITY, now=1.0)
        self.machine.set_high_mode(HIGH_MODE_ARM_WALK, now=1.0)
        self.machine.set_navigation_command((0.3, -0.1, 0.2), now=1.1)
        self.machine.set_arm_command(arm, now=1.1)

        selection = self.machine.select(now=1.1)

        self.assertEqual(selection.model_name, MODEL_ARM_WALK)
        self.assertEqual(selection.command, (0.3, -0.1, 0.2))
        self.assertEqual(selection.arm_command, arm)

    def test_mode_three_ignores_low_two_navigation(self):
        self.machine.set_low_mode(LOW_MODE_TARGET_POSE, now=1.0)
        self.machine.set_high_mode(HIGH_MODE_ARM_WALK, now=1.0)
        self.machine.set_navigation_command((0.5, 0.1, 0.2), now=1.1)

        selection = self.machine.select(now=1.1)

        self.assertEqual(selection.model_name, MODEL_ARM_WALK)
        self.assertEqual(selection.command, ZERO_COMMAND)

    def test_mode_three_requires_velocity_received_after_mode_entry(self):
        self.machine.set_low_mode(LOW_MODE_VELOCITY, now=1.0)
        self.machine.set_navigation_command((0.3, 0.0, 0.0), now=1.0)
        self.machine.set_high_mode(HIGH_MODE_ARM_WALK, now=1.1)

        selection = self.machine.select(now=1.1)

        self.assertEqual(selection.model_name, MODEL_ARM_WALK)
        self.assertEqual(selection.command, ZERO_COMMAND)

    def test_arm_mode_waits_for_a_pose_received_after_mode_entry(self):
        old_arm = ArmCommand(1, (0.1,) * 14, (0.0,) * 14, 1.0)
        new_arm = ArmCommand(2, (0.2,) * 14, (0.0,) * 14, 1.0)
        self.machine.set_arm_command(old_arm, now=1.0)

        self.machine.set_high_mode(HIGH_MODE_ARM_WALK, now=1.1)
        self.assertIsNone(self.machine.select(now=1.1).arm_command)
        with self.assertRaises(ValueError):
            self.machine.set_arm_command(old_arm, now=1.15)

        self.machine.set_arm_command(new_arm, now=1.2)
        self.assertEqual(self.machine.select(now=1.2).arm_command, new_arm)

    def test_switching_between_arm_modes_requires_a_new_pose(self):
        arm = ArmCommand(1, (0.1,) * 14, (0.0,) * 14, 1.0)
        self.machine.set_high_mode(HIGH_MODE_ARM_WALK, now=1.0)
        self.machine.set_arm_command(arm, now=1.1)
        self.assertEqual(self.machine.select(now=1.1).arm_command, arm)

        self.machine.set_high_mode(HIGH_MODE_ARM_STAND, now=2.0)
        selection = self.machine.select(now=3.0)
        self.assertEqual(selection.model_name, MODEL_ARM_STAND)
        self.assertIsNone(selection.arm_command)

    def test_low_mode_two_to_one_switches_directly_with_zero_default(self):
        self.machine.set_low_mode(LOW_MODE_TARGET_POSE)
        self.machine.set_high_mode(HIGH_MODE_NAVIGATION, now=0.0)
        self.machine.set_navigation_command((0.5, 0.0, 0.1), now=1.0)
        self.assertEqual(
            self.machine.select(now=1.0).model_name,
            MODEL_ACCURATE_ARRIVAL,
        )

        self.machine.set_low_mode(LOW_MODE_VELOCITY, now=2.0)
        selection = self.machine.select(now=2.0)
        self.assertFalse(selection.is_standing_transition)
        self.assertEqual(selection.model_name, MODEL_FREE_WALK)
        self.assertEqual(selection.command, ZERO_COMMAND)

    def test_target_pose_uses_latest_closed_loop_error(self):
        self.machine.set_low_mode(LOW_MODE_TARGET_POSE)
        self.machine.set_high_mode(HIGH_MODE_NAVIGATION, now=0.0)

        self.machine.set_navigation_command((1.116, -0.067, 0.262), now=1.0)
        first_selection = self.machine.select(now=1.0)
        self.assertEqual(first_selection.model_name, MODEL_ACCURATE_ARRIVAL)
        self.assertEqual(first_selection.command, (1.116, -0.067, 0.262))

        self.machine.set_navigation_command((0.664, -0.098, 0.175), now=1.1)
        next_selection = self.machine.select(now=1.1)
        self.assertEqual(next_selection.model_name, MODEL_ACCURATE_ARRIVAL)
        self.assertEqual(next_selection.command, (0.664, -0.098, 0.175))

    def test_invalid_modes_enter_zero_free_walk_fallback(self):
        self.machine.set_high_mode(HIGH_MODE_ARM_WALK, now=1.0)
        self.assertEqual(self.machine.select(now=1.0).model_name, MODEL_ARM_WALK)

        with self.assertRaises(ValueError):
            self.machine.set_high_mode(9, now=2.0)
        high_fallback = self.machine.select(now=2.0)
        self.assertEqual(high_fallback.model_name, MODEL_FREE_WALK)
        self.assertEqual(high_fallback.command, ZERO_COMMAND)

        self.machine.set_high_mode(HIGH_MODE_ARM_WALK, now=2.5)
        with self.assertRaises(ValueError):
            self.machine.set_low_mode(9, now=2.6)
        arm_mode_fallback = self.machine.select(now=2.6)
        self.assertEqual(
            arm_mode_fallback.model_name,
            MODEL_FREE_WALK,
        )
        self.assertEqual(arm_mode_fallback.command, ZERO_COMMAND)

        self.machine.set_high_mode(HIGH_MODE_NAVIGATION, now=3.0)
        self.machine.set_low_mode(LOW_MODE_VELOCITY, now=3.0)
        with self.assertRaises(ValueError):
            self.machine.set_low_mode(9, now=4.0)
        low_fallback = self.machine.select(now=4.0)
        self.assertEqual(low_fallback.model_name, MODEL_FREE_WALK)
        self.assertEqual(low_fallback.command, ZERO_COMMAND)

    def test_inputs_expire_at_their_configured_deadlines(self):
        self.machine.set_low_mode(LOW_MODE_VELOCITY)
        self.machine.set_high_mode(HIGH_MODE_NAVIGATION, now=0.0)
        self.machine.set_navigation_command((0.4, 0.0, 0.0), now=1.0)
        self.assertEqual(self.machine.select(now=1.25).command, (0.4, 0.0, 0.0))
        expired_velocity = self.machine.select(now=1.251)
        self.assertEqual(expired_velocity.model_name, MODEL_FREE_WALK)
        self.assertEqual(expired_velocity.command, ZERO_COMMAND)

        arm = ArmCommand(1, (0.1,) * 14, (0.0,) * 14, 1.0)
        self.machine.set_low_mode(LOW_MODE_VELOCITY, now=2.0)
        self.machine.set_high_mode(HIGH_MODE_ARM_WALK, now=2.0)
        self.machine.set_navigation_command((0.2, 0.0, 0.1), now=2.0)
        self.machine.set_arm_command(arm, now=2.0)
        self.assertEqual(
            self.machine.select(now=2.19).command,
            (0.2, 0.0, 0.1),
        )
        self.assertEqual(self.machine.select(now=2.19).arm_command, arm)
        self.assertIsNone(self.machine.select(now=2.201).arm_command)
        self.assertEqual(self.machine.select(now=2.251).command, ZERO_COMMAND)

    def test_identical_arm_sequence_is_allowed_only_as_a_heartbeat(self):
        self.machine.set_high_mode(HIGH_MODE_ARM_WALK, now=0.0)
        command = ArmCommand(2, (0.0,) * 14, (0.0,) * 14, 0.5)
        self.machine.set_arm_command(command, now=1.0)
        self.machine.set_arm_command(command, now=1.1)
        self.assertEqual(self.machine.select(now=1.25).arm_command, command)

        changed = ArmCommand(2, (0.1,) * 14, (0.0,) * 14, 0.5)
        with self.assertRaisesRegex(ValueError, "payload changed"):
            self.machine.set_arm_command(changed, now=1.2)

        older = ArmCommand(1, (0.0,) * 14, (0.0,) * 14, 0.5)
        with self.assertRaisesRegex(ValueError, "must not decrease"):
            self.machine.set_arm_command(older, now=1.2)


if __name__ == "__main__":
    unittest.main()
