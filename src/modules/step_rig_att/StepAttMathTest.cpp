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

#include <gtest/gtest.h>

#include "StepAttMath.hpp"

using namespace step_rig_att;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

// The direction convention is the thing that actually went wrong in the field -
// a combined Euler command did not lean where the operator expected - so these
// tests pin the physical meaning of every parameter value, not just the algebra.

TEST(StepAttMath, tiltDirectionLeansWhereItSays)
{
	// Each direction, checked against the body-z axis the airframe ends up
	// with. Body z is the DOWN axis in FRD, so a lean toward +x (north) tips
	// the belly toward -x. Getting this backwards is a 180 degree error, which
	// is precisely the mistake worth a test.
	struct Case {
		float dir_deg;
		float expect_x;   // sign of the horizontal body-z components
		float expect_y;
	};

	const Case cases[] = {
		{0.f,   -1.f,  0.f},   // lean north / nose-down
		{90.f,   0.f, -1.f},   // lean east / right roll
		{180.f,  1.f,  0.f},   // lean south / nose-up
		{270.f,  0.f,  1.f},   // lean west / left roll
	};

	for (const Case &c : cases) {
		const Vector3f e_z_d = target_tilt_direction(30.f, c.dir_deg).dcm_z();
		const float sin30 = 0.5f;

		EXPECT_NEAR(e_z_d(0), c.expect_x * sin30, 1e-4f) << "dir " << (double)c.dir_deg;
		EXPECT_NEAR(e_z_d(1), c.expect_y * sin30, 1e-4f) << "dir " << (double)c.dir_deg;
	}
}

TEST(StepAttMath, tiltDirectionMatchesSingleAxisCommands)
{
	// Direction 90 must be exactly a right roll, and direction 0 exactly a
	// nose-down pitch - i.e. the new mode reproduces the single-axis commands
	// that were already known to work, rather than being a separate animal.
	const Quatf roll_right = target_tilt_direction(40.f, 90.f);
	const Quatf euler_roll = target_euler(40.f, 0.f);
	EXPECT_NEAR(fabsf(roll_right.dot(euler_roll)), 1.f, 1e-5f);

	const Quatf nose_down = target_tilt_direction(40.f, 0.f);
	const Quatf euler_pitch = target_euler(0.f, -40.f);
	EXPECT_NEAR(fabsf(nose_down.dot(euler_pitch)), 1.f, 1e-5f);
}

TEST(StepAttMath, documentedEulerReadbackIsCorrect)
{
	// The table printed in STEPATT_DIR's docs and in the module help. These
	// numbers were wrong once already - written with the signs of the axis
	// convention rather than the lean convention - so they are pinned here
	// rather than left to prose. Verified against SIH: a commanded
	// tilt 45 / dir 45 flies to roll 35.1, pitch -30.0, yaw -10.0.
	struct Case {
		float dir_deg;
		float roll_deg;
		float pitch_deg;
		float yaw_deg;
	};

	const Case cases[] = {
		{0.f,     0.0f, -45.0f,  0.0f},   // north / nose-down
		{45.f,   35.3f, -30.0f, -9.7f},   // north-east
		{90.f,   45.0f,   0.0f,  0.0f},   // east / right roll
		{135.f,  35.3f,  30.0f,  9.7f},   // south-east
		{180.f,   0.0f,  45.0f,  0.0f},   // south / nose-up
		{270.f, -45.0f,   0.0f,  0.0f},   // west / left roll
	};

	for (const Case &c : cases) {
		const Eulerf euler(target_tilt_direction(45.f, c.dir_deg));

		EXPECT_NEAR(math::degrees(euler.phi()), c.roll_deg, 0.1f) << "dir " << (double)c.dir_deg;
		EXPECT_NEAR(math::degrees(euler.theta()), c.pitch_deg, 0.1f) << "dir " << (double)c.dir_deg;
		EXPECT_NEAR(math::degrees(euler.psi()), c.yaw_deg, 0.1f) << "dir " << (double)c.dir_deg;
	}
}

TEST(StepAttMath, tiltMagnitudeIsIndependentOfDirection)
{
	// The tilt parameter must mean "total degrees of lean" in every direction.
	// Building the rotation vector from a non-unit axis would make a diagonal
	// command sqrt(2) times larger than an on-axis one.
	for (float dir = 0.f; dir < 360.f; dir += 15.f) {
		float tilt_deg = 0.f;
		float dir_out = 0.f;
		float dyaw_deg = 0.f;
		decompose(target_tilt_direction(55.f, dir), tilt_deg, dir_out, dyaw_deg);

		EXPECT_NEAR(tilt_deg, 55.f, 1e-3f) << "dir " << (double)dir;
	}
}

TEST(StepAttMath, decomposeRoundTripsTheDirection)
{
	for (float dir = 0.f; dir < 360.f; dir += 15.f) {
		float tilt_deg = 0.f;
		float dir_out = 0.f;
		float dyaw_deg = 0.f;
		decompose(target_tilt_direction(35.f, dir), tilt_deg, dir_out, dyaw_deg);

		EXPECT_NEAR(dir_out, dir, 1e-2f);
	}
}

TEST(StepAttMath, tiltDirectionHasNoDeltaYaw)
{
	// The reason this mode exists. A rotation about a horizontal axis leaves
	// nothing in the yaw channel, so the controller delivers all of it on the
	// two strong axes.
	for (float dir = 0.f; dir < 360.f; dir += 15.f) {
		for (float tilt : {10.f, 45.f, 90.f, 135.f}) {
			float tilt_deg = 0.f;
			float dir_out = 0.f;
			float dyaw_deg = 0.f;
			decompose(target_tilt_direction(tilt, dir), tilt_deg, dir_out, dyaw_deg);

			EXPECT_NEAR(dyaw_deg, 0.f, 1e-2f)
					<< "tilt " << (double)tilt << " dir " << (double)dir;
		}
	}
}

TEST(StepAttMath, singleAxisEulerHasNoDeltaYaw)
{
	// Why roll-only and pitch-only commands always arrived: one axis at a time
	// is degenerate-free however large the angle.
	for (float angle : {20.f, 56.f, 60.f, 90.f}) {
		float tilt_deg = 0.f;
		float dir_deg = 0.f;
		float dyaw_deg = 0.f;

		decompose(target_euler(angle, 0.f), tilt_deg, dir_deg, dyaw_deg);
		EXPECT_NEAR(dyaw_deg, 0.f, 1e-2f) << "roll " << (double)angle;
		EXPECT_NEAR(tilt_deg, angle, 1e-2f) << "roll " << (double)angle;

		decompose(target_euler(0.f, angle), tilt_deg, dir_deg, dyaw_deg);
		EXPECT_NEAR(dyaw_deg, 0.f, 1e-2f) << "pitch " << (double)angle;
		EXPECT_NEAR(tilt_deg, angle, 1e-2f) << "pitch " << (double)angle;
	}
}

TEST(StepAttMath, combinedEulerHidesDeltaYaw)
{
	// The reported failure: roll 65 + pitch 60 is not a 65/60 lean at all. It
	// is a 77.8 degree lean plus 40.4 degrees dumped into the weakest axis.
	float tilt_deg = 0.f;
	float dir_deg = 0.f;
	float dyaw_deg = 0.f;
	decompose(target_euler(65.f, 60.f), tilt_deg, dir_deg, dyaw_deg);

	EXPECT_NEAR(tilt_deg, 77.8f, 0.1f);
	EXPECT_NEAR(dyaw_deg, -40.4f, 0.1f);
}

TEST(StepAttMath, equivalentTiltCommandRecoversTheSameLean)
{
	// Feeding a decomposition back in as a mode-0 command must reproduce the
	// lean the operator actually got - this is what the runtime diagnostic
	// tells them to do.
	float tilt_deg = 0.f;
	float dir_deg = 0.f;
	float dyaw_deg = 0.f;
	decompose(target_euler(65.f, 60.f), tilt_deg, dir_deg, dyaw_deg);

	const Vector3f from_euler = target_euler(65.f, 60.f).dcm_z();
	const Vector3f from_tilt = target_tilt_direction(tilt_deg, dir_deg).dcm_z();

	// Same body-z axis: the lean is identical, only the delta-yaw is dropped.
	EXPECT_NEAR(from_tilt(0), from_euler(0), 1e-3f);
	EXPECT_NEAR(from_tilt(1), from_euler(1), 1e-3f);
	EXPECT_NEAR(from_tilt(2), from_euler(2), 1e-3f);
}

TEST(StepAttMath, levelTargetIsIdentity)
{
	float tilt_deg = 0.f;
	float dir_deg = 0.f;
	float dyaw_deg = 0.f;
	decompose(target_tilt_direction(0.f, 137.f), tilt_deg, dir_deg, dyaw_deg);

	EXPECT_NEAR(tilt_deg, 0.f, 1e-4f);
	EXPECT_NEAR(dyaw_deg, 0.f, 1e-4f);
	// Direction is meaningless with no tilt and must not produce a NaN.
	EXPECT_TRUE(std::isfinite(dir_deg));
}
