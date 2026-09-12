/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

// Target-attitude maths for step_rig_att. Free of uORB, parameters and timers
// so it can be unit-tested directly (StepAttMathTest.cpp). Self-contained on
// purpose: step_rig_att does not depend on any other rig module.

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

namespace step_rig_att
{

enum class TargetMode : int32_t {
	TiltDirection = 0,
	EulerRollPitch = 1,
};

/**
 * Target attitude from a total lean angle and the direction to lean in.
 *
 * STEPATT_DIR names the direction the airframe LEANS, measured in the body
 * xy-plane from body x (nose) toward body y (right wing). With yaw held at zero
 * the nose points north, so the parameter reads as a compass bearing:
 *
 *     0 deg   -> leans toward the nose  (north), i.e. nose-down pitch
 *    90 deg   -> leans toward the right wing (east), i.e. right roll
 *   180 deg   -> leans toward the tail  (south), i.e. nose-up pitch
 *   270 deg   -> leans toward the left wing (west), i.e. left roll
 *
 * The rotation AXIS is 90 degrees away from the lean direction - lean right and
 * you have rotated about the nose. Hence axis = [sin(dir), -cos(dir), 0], which
 * is why this differs by 90 degrees from hold_rig_att's HRATT_DIR, where the
 * parameter names the axis rather than the lean.
 *
 * The axis is a UNIT vector, so the rotation vector axis*tilt has magnitude
 * exactly tilt whatever the direction - that is what keeps STEPATT_TILT meaning
 * "total degrees of lean" rather than something that grows on the diagonals.
 * Feeding the two components in independently (AxisAnglef(tilt, tilt, 0) for a
 * 45 degree direction) would give sqrt(2)*tilt instead.
 *
 * Crucially this rotation is about a HORIZONTAL axis, so it contains no yaw
 * component at all - see decompose(). That is the whole reason this mode
 * exists: the attitude controller can deliver it on the two strong axes alone.
 */
inline matrix::Quatf target_tilt_direction(float tilt_deg, float dir_deg)
{
	const float dir = math::radians(dir_deg);
	const matrix::Vector3f axis(sinf(dir), -cosf(dir), 0.f);
	const matrix::Vector3f rotvec = axis * math::radians(tilt_deg);

	return matrix::Quatf(matrix::AxisAnglef(rotvec(0), rotvec(1), rotvec(2)));
}

/**
 * Target attitude from Euler roll and pitch, with yaw at zero.
 *
 * PX4's intrinsic Z-Y-X, exactly as the failure detector and any Euler readback
 * mean it. Past roughly 60 degrees on both axes at once this stops describing
 * the attitude an operator has in mind: the pitch is applied about the
 * already-rolled axis, so part of the command turns into a rotation about the
 * body z axis that the controller executes at MC_YAW_WEIGHT on the weakest axis
 * available. decompose() reports exactly how much.
 */
inline matrix::Quatf target_euler(float roll_deg, float pitch_deg)
{
	return matrix::Quatf(matrix::Eulerf(math::radians(roll_deg), math::radians(pitch_deg), 0.f));
}

/**
 * Split a target the way AttitudeControl::update() will: into a total tilt, the
 * direction of that tilt, and the residual delta-yaw that gets scaled by
 * MC_YAW_WEIGHT.
 *
 * Mirroring the controller's own split rather than approximating it separately
 * is the point: the delta-yaw figure is then exactly the part of a command that
 * will be executed at reduced weight on the axis with the least torque. For
 * roll 65 / pitch 60 this reports tilt 77.8, dir 22.0, delta-yaw -40.4 - i.e.
 * 40 degrees of that command are never really going to arrive.
 *
 * All outputs in degrees; dir is wrapped to [0, 360) and uses the same lean
 * convention as STEPATT_DIR, so a decomposition can be fed straight back in as
 * a mode-0 command.
 */
inline void decompose(const matrix::Quatf &q, float &tilt_deg, float &dir_deg, float &dyaw_deg)
{
	const matrix::Vector3f e_z(0.f, 0.f, 1.f);      // level reference
	const matrix::Vector3f e_z_d = q.dcm_z();       // where the target points body z

	// atan2(sin, cos) rather than acos(cos): acos' derivative is unbounded at
	// +-1, which is exactly where a near-level or near-inverted target sits,
	// and the two-argument form is well conditioned over the whole 0-180 range
	// for no extra cost.
	const float sin_tilt = sqrtf(e_z_d(0) * e_z_d(0) + e_z_d(1) * e_z_d(1));
	tilt_deg = math::degrees(atan2f(sin_tilt, e_z_d(2)));

	// Body z is the DOWN axis in FRD, so the airframe leans opposite to its
	// horizontal part - lean right and the belly faces left.
	if (sin_tilt > 1e-6f) {
		dir_deg = math::degrees(atan2f(-e_z_d(1), -e_z_d(0)));

		if (dir_deg < 0.f) {
			dir_deg += 360.f;
		}

	} else {
		// No tilt at all: the direction is undetermined rather than zero.
		dir_deg = 0.f;
	}

	// The reduced (tilt-only) rotation the controller builds: shortest arc from
	// level's body z onto the target's.
	const matrix::Quatf qd_red(e_z, e_z_d);

	// Whatever remains once the tilt is accounted for can only be a rotation
	// about the shared body z axis - the delta-yaw channel.
	matrix::Quatf qd_dyaw = qd_red.inversed() * q;
	qd_dyaw.canonicalize();
	dyaw_deg = math::degrees(2.f * atan2f(qd_dyaw(3), qd_dyaw(0)));
}

} // namespace step_rig_att
