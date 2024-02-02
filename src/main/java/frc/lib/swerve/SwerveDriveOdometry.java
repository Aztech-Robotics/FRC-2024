// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.swerve;

import java.util.Arrays;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Objects;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Class for swerve drive odometry. Odometry allows you to track the robot's
 * position on the field
 * over a course of a match using readings from your swerve drive encoders and
 * swerve azimuth
 * encoders.
 *
 * <p>
 * Teams can use odometry during the autonomous period for complex tasks like
 * path following.
 * Furthermore, odometry can be used for latency compensation when using
 * computer-vision systems.
 */
public class SwerveDriveOdometry {
	private final SwerveDriveKinematics m_kinematics;
	private Pose2d m_poseMeters;
	private Rotation2d m_previousAngle;
	private final int m_numModules;
	private SwerveModulePosition[] m_previousModulePositions;
	private final Matrix<N3, N1> m_q = new Matrix<>(Nat.N3(), Nat.N1());
	private Matrix<N3, N3> m_visionK = new Matrix<>(Nat.N3(), Nat.N3());
	private static final double kBufferDuration = 1.5;
	private final TimeInterpolatableBuffer<InterpolationRecord> m_poseBuffer = TimeInterpolatableBuffer.createBuffer(kBufferDuration);

	/**
	 * Constructs a SwerveDriveOdometry object.
	 *
	 * @param kinematics      The swerve drive kinematics for your drivetrain.
	 * @param modulePositions The wheel positions reported by each module.
	 */
	public SwerveDriveOdometry (SwerveDriveKinematics kinematics, SwerveModulePosition[] modulePositions) {
		this(kinematics, modulePositions, new Pose2d(), VecBuilder.fill(0.0001, 0.0001, 0.0001), VecBuilder.fill(0.00001, 0.00001, 0.00001)); 
	}

	/**
	 * Constructs a SwerveDriveOdometry object.
	 *
	 * @param kinematics      The swerve drive kinematics for your drivetrain.
	 * @param modulePositions The wheel positions reported by each module.
	 * @param initialPose     The starting position of the robot on the field.
	 * @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
     * in meters, and heading in radians). Increase these numbers to trust your state estimate less.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     * in meters, y position in meters, and heading in radians). Increase these numbers to trust
     * the vision pose measurement less.
	 */
	public SwerveDriveOdometry(SwerveDriveKinematics kinematics, SwerveModulePosition[] modulePositions, Pose2d initialPose, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs) {
		m_kinematics = kinematics;
		m_poseMeters = initialPose;
		m_previousAngle = initialPose.getRotation();
		m_numModules = modulePositions.length;
		m_previousModulePositions = new SwerveModulePosition[m_numModules];
		for (int index = 0; index < m_numModules; index++) {
			m_previousModulePositions[index] = new SwerveModulePosition(modulePositions[index].distanceMeters, modulePositions[index].angle);
		}
		for (int i = 0; i < 3; ++i) {
			m_q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
		}
		setVisionMeasurementStdDevs(visionMeasurementStdDevs);
	}

	/**
	 * Sets the pose estimator's trust of global measurements. This might be used to change trust in
	 * vision measurements after the autonomous period, or to change trust as distance to a vision
	 * target increases.
	 *
	 * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
	 * numbers to trust global measurements from vision less. This matrix is in the form [x, y,
	 * theta]ᵀ, with units in meters and radians.
	 */
	public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
		var r = new double[3];
		for (int i = 0; i < 3; ++i) {
			r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
		}
		// Solve for closed form Kalman gain for continuous Kalman filter with A = 0
		// and C = I. See wpimath/algorithms.md.
		for (int row = 0; row < 3; ++row) {
			if (m_q.get(row, 0) == 0.0) {
				m_visionK.set(row, row, 0.0);
			} else {
				m_visionK.set(row, row, m_q.get(row, 0) / (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r[row])));
			}
		}
	}

	/**
	 * Resets the robot's position on the field.
	 *
	 * <p>
	 * Module positions do not need to be reset in user code.
	 *
	 * @param modulePositions The wheel positions reported by each module.,
	 * @param pose            The position on the field that your robot is at.
	 */
	public void resetPosition(SwerveModulePosition[] modulePositions, Pose2d pose) {
		m_poseMeters = pose;
		m_previousAngle = pose.getRotation();
		for (int index = 0; index < m_numModules; index++) {
			m_previousModulePositions[index] = new SwerveModulePosition(modulePositions[index].distanceMeters, modulePositions[index].angle);
		}
		m_poseBuffer.clear();
	}

	/**
	 * Returns the position of the robot on the field.
	 *
	 * @return The pose of the robot (x and y are in meters).
	 */
	public Pose2d getPoseMeters() {
		return m_poseMeters;
	}

	/**
	 * Updates the robot's position on the field using forward kinematics and
	 * integration of the pose
	 * over time. This method automatically calculates the current time to calculate
	 * period
	 * (difference between two timestamps). The period is used to calculate the
	 * change in distance
	 * from a velocity. This also takes in an angle parameter which is used instead
	 * of the angular
	 * rate that is calculated from forward kinematics.
	 * @param currentTimeSeconds Time at which this method was called, in seconds.
	 * @param gyroAngle       The angle reported by the gyroscope.
	 * @param modulePositions The current position of all swerve modules. Please
	 *                        provide the positions
	 *                        in the same order in which you instantiated your
	 *                        SwerveDriveKinematics.
	 * @return The new pose of the robot.
	 */
	public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
		if (modulePositions.length != m_numModules) {
			throw new IllegalArgumentException(
			"Number of modules is not consistent with number of wheel locations provided in " + "constructor");
		}

		var moduleDeltas = new SwerveModulePosition[m_numModules];
		for (int index = 0; index < m_numModules; index++) {
			var current = modulePositions[index];
			var previous = m_previousModulePositions[index];

			moduleDeltas[index] = new SwerveModulePosition(current.distanceMeters - previous.distanceMeters, current.angle);
			previous.distanceMeters = current.distanceMeters;
		}

		var twist = m_kinematics.toTwist2d(moduleDeltas);
		twist.dtheta = gyroAngle.minus(m_previousAngle).getRadians();
		var newPose = m_poseMeters.exp(twist);
		m_previousAngle = gyroAngle;
		m_poseMeters = new Pose2d(newPose.getTranslation(), gyroAngle);

		var internalModulePositions = new SwerveModulePosition[m_numModules];
		for (int i = 0; i < m_numModules; i++) {
		internalModulePositions[i] =
			new SwerveModulePosition(modulePositions[i].distanceMeters, modulePositions[i].angle);
		}
		m_poseBuffer.addSample(currentTimeSeconds, new InterpolationRecord(m_poseMeters, gyroAngle, internalModulePositions));

		return m_poseMeters;
	}

	/**
	 * Updates the pose estimator with wheel encoder and gyro information. This should be called every
	 * loop.
	 *
	 * @param gyroAngle The current gyro angle.
	 * @param modulePositions The current distance measurements and rotations of the swerve modules.
	 * @return The estimated pose of the robot in meters.
	 */
	public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
		return updateWithTime(MathSharedStore.getTimestamp(), gyroAngle, modulePositions);
	}

	/**
	 * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
	 * while still accounting for measurement noise.
	 *
	 * <p>This method can be called as infrequently as you want, as long as you are calling {@link
	 * SwerveDrivePoseEstimator#update} every loop.
	 *
	 * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
	 * recommend only adding vision measurements that are already within one meter or so of the
	 * current pose estimate.
	 *
	 * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
	 * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
	 *     don't use your own time source by calling {@link
	 *     SwerveDrivePoseEstimator#updateWithTime(double,Rotation2d,SwerveModulePosition[])} then you
	 *     must use a timestamp with an epoch since FPGA startup (i.e., the epoch of this timestamp is
	 *     the same epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.) This means that
	 *     you should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as your time source
	 *     or sync the epochs.
	 */
	public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
		// Step 0: If this measurement is old enough to be outside the pose buffer's timespan, skip.
		try {
			if (m_poseBuffer.getInternalBuffer().lastKey() - kBufferDuration > timestampSeconds) {
				return;
			}
		} catch (NoSuchElementException ex) {
			return;
		}

		// Step 1: Get the pose odometry measured at the moment the vision measurement was made.
		var sample = m_poseBuffer.getSample(timestampSeconds); 

		if (sample.isEmpty()) {
			return;
		}

		// Step 2: Measure the twist between the odometry pose and the vision pose.
		var twist = sample.get().poseMeters.log(visionRobotPoseMeters);

		// Step 3: We should not trust the twist entirely, so instead we scale this twist by a Kalman
		// gain matrix representing how much we trust vision measurements compared to our current pose.
		var k_times_twist = m_visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

		// Step 4: Convert back to Twist2d.
		var scaledTwist = new Twist2d(k_times_twist.get(0, 0), k_times_twist.get(1, 0), k_times_twist.get(2, 0));

		// Step 5: Reset Odometry to state at sample with vision adjustment.
		resetPosition(sample.get().modulePositions, sample.get().poseMeters.exp(scaledTwist));

		// Step 6: Record the current pose to allow multiple measurements from the same timestamp
		m_poseBuffer.addSample(timestampSeconds, new InterpolationRecord(m_poseMeters, sample.get().gyroAngle, sample.get().modulePositions));

		// Step 7: Replay odometry inputs between sample time and latest recorded sample to update the
		// pose buffer and correct odometry.
		for (Map.Entry<Double, InterpolationRecord> entry : m_poseBuffer.getInternalBuffer().tailMap(timestampSeconds).entrySet()) {
			updateWithTime(entry.getKey(), entry.getValue().gyroAngle, entry.getValue().modulePositions);
		}
	}

	/*
	* Represents an odometry record. The record contains the inputs provided as well as the pose that
	* was observed based on these inputs, as well as the previous record and its inputs.
	*/
	private class InterpolationRecord implements Interpolatable<InterpolationRecord> {
		// The pose observed given the current sensor inputs and the previous pose.
		private final Pose2d poseMeters;

		// The current gyro angle.
		private final Rotation2d gyroAngle;

		// The distances and rotations measured at each module.
		private final SwerveModulePosition[] modulePositions;

		/**
		 * Constructs an Interpolation Record with the specified parameters.
		 *
		 * @param pose The pose observed given the current sensor inputs and the previous pose.
		 * @param gyro The current gyro angle.
		 * @param wheelPositions The distances and rotations measured at each wheel.
		 */
		private InterpolationRecord(Pose2d poseMeters, Rotation2d gyro, SwerveModulePosition[] modulePositions) {
			this.poseMeters = poseMeters;
			this.gyroAngle = gyro;
			this.modulePositions = modulePositions;
		}

		/**
		 * Return the interpolated record. This object is assumed to be the starting position, or lower
		 * bound.
		 *
		 * @param endValue The upper bound, or end.
		 * @param t How far between the lower and upper bound we are. This should be bounded in [0, 1].
		 * @return The interpolated value.
		 */
		@Override
		public InterpolationRecord interpolate(InterpolationRecord endValue, double t) {
			if (t < 0) {
				return this;
			} else if (t >= 1) {
				return endValue;
			} else {
				// Find the new wheel distances.
				var modulePositions = new SwerveModulePosition[m_numModules];
				// Find the distance travelled between this measurement and the interpolated measurement.
				var moduleDeltas = new SwerveModulePosition[m_numModules];
				for (int i = 0; i < m_numModules; i++) {
					double ds = MathUtil.interpolate(this.modulePositions[i].distanceMeters, endValue.modulePositions[i].distanceMeters, t);
					Rotation2d theta = this.modulePositions[i].angle.interpolate(endValue.modulePositions[i].angle, t);
					modulePositions[i] = new SwerveModulePosition(ds, theta);
					moduleDeltas[i] = new SwerveModulePosition(ds - this.modulePositions[i].distanceMeters, theta);
				}
				// Find the new gyro angle.
				var gyro_lerp = gyroAngle.interpolate(endValue.gyroAngle, t);
				// Create a twist to represent this change based on the interpolated sensor inputs.
				Twist2d twist = m_kinematics.toTwist2d(moduleDeltas);
				twist.dtheta = gyro_lerp.minus(gyroAngle).getRadians();
				return new InterpolationRecord(poseMeters.exp(twist), gyro_lerp, modulePositions);
			}
		}

		@Override
		public boolean equals(Object obj) {
			if (this == obj) {
				return true;
			}
			if (!(obj instanceof InterpolationRecord)) {
				return false;
			}
			InterpolationRecord record = (InterpolationRecord) obj;
			return Objects.equals(gyroAngle, record.gyroAngle) && Arrays.equals(modulePositions, record.modulePositions) && Objects.equals(poseMeters, record.poseMeters);
		}

		@Override
		public int hashCode() {
			return Objects.hash(gyroAngle, Arrays.hashCode(modulePositions), poseMeters);
		}
	}
}
