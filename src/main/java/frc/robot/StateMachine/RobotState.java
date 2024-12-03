package frc.robot.StateMachine;

import com.studica.frc.AHRS;
import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import com.ninjas4744.NinjasLib.RobotStateIO;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Constants;
import frc.robot.Constants.VisionConstants;

public class RobotState extends RobotStateIO<RobotStates> {
	private final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI);
	private final DigitalInput _indexerNote = new DigitalInput(Constants.kIndexerBeamBreakerId);

	private SwerveDrivePoseEstimator poseEstimator;
	private final StructPublisher<Pose2d> _robotPosePublisher =
		NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();

	public RobotState(){
		robotState = RobotStates.IDLE;
	}

	public static RobotState getInstance() {
		return (RobotState)RobotStateIO.getInstance();
	}

	/**
	 * @return Whether there's a note in the indexer according to its beam breaker
	 */
	public boolean getNoteInIndexer() {
		return !_indexerNote.get();
	}

	/**
	 * @return position of the robot
	 */
	public Pose2d getRobotPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/**
	 * Set where the code thinks the robot is
	 *
	 * @param pose - the pose to set the robot pose to
	 */
	public void setRobotPose(Pose2d pose) {
		_robotPosePublisher.set(pose);
//		poseEstimator.resetPosition(getGyroYaw(), Swerve.getInstance().getModulePositions(), pose);
	}

	/**
	 * Updates the robot pose according to odometry parameters
	 *
	 * @param modulePositions - The current position of the swerve modules.
	 */
	public void updateRobotPose(SwerveModulePosition[] modulePositions) {
		poseEstimator.update(getGyroYaw(), modulePositions);

		_robotPosePublisher.set(getRobotPose());
	}

	/**
	 * Updates the robot pose according to the given vision estimation
	 *
	 * @param visionEstimation - the estimation
	 */
	public void updateRobotPose(VisionOutput visionEstimation) {
		if (visionEstimation.hasTargets){
			poseEstimator.addVisionMeasurement(
				visionEstimation.robotPose,
				visionEstimation.timestamp,
				new Matrix<>(Nat.N3(), Nat.N1(), new double[] {
					VisionConstants.calculateFOM(visionEstimation),
					VisionConstants.calculateFOM(visionEstimation),
					VisionConstants.calculateFOM(visionEstimation),
				})
			);
		}

		_robotPosePublisher.set(getRobotPose());
	}

	/**
	 * @return yaw angle of the robot according to gyro
	 */
	public Rotation2d getGyroYaw() {
		if (!isSimulated())
			return Rotation2d.fromDegrees(navX.getAngle()/*SwerveConstants.kInvertGyro ? -navX.getAngle() : navX.getAngle()*/);
		else
			return getRobotPose().getRotation()/*SwerveConstants.kInvertGyro
					? getRobotPose().getRotation().unaryMinus()
					: getRobotPose().getRotation()*/;
	}

	public Translation3d getRobotVelocity() {
		return new Translation3d(navX.getVelocityX(), navX.getVelocityY(), navX.getVelocityZ());
	}

	/**
	 * Resets the gyro angle, sets it to the given angle
	 *
	 * @param angle - the angle to set the gyro to
	 */
	public void resetGyro(Rotation2d angle) {
		if (!isSimulated()) {
			System.out.print("Gyro: " + navX.getAngle() + " -> " + angle.getDegrees());
			navX.reset();
			navX.setAngleAdjustment(angle.getDegrees());
		} else {
			System.out.print("Gyro: " + getRobotPose().getRotation().getDegrees() + " -> " + angle.getDegrees());
			setRobotPose(new Pose2d(getRobotPose().getTranslation(), angle));
		}
	}

	/**
	 * Initialize and create the pose estimator for knowing robot's pose according to vision and
	 * odometry
	 */
	public void initPoseEstimator() {
//		poseEstimator = !isSimulated()
//				? new SwerveDrivePoseEstimator(
//						SwerveConstants.kSwerveKinematics,
//						getGyroYaw(),
//						SwerveIO.getInstance().getModulePositions(),
//						new Pose2d())
//				: new SwerveDrivePoseEstimator(
//						SwerveConstants.kSwerveKinematics,
//						new Rotation2d(),
//						SwerveIO.getInstance().getModulePositions(),
//						new Pose2d());
	}
}
