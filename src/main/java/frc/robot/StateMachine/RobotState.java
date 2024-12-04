package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
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

public class RobotState extends RobotStateWithSwerve<RobotStates> {
	private final DigitalInput _indexerNote = new DigitalInput(Constants.kIndexerBeamBreakerId);

	public RobotState(){
		_robotState = RobotStates.IDLE;
	}

	public static RobotState getInstance() {
		return (RobotState)RobotStateWithSwerve.getInstance();
	}

	/**
	 * @return Whether there's a note in the indexer according to its beam breaker
	 */
	public boolean getNoteInIndexer() {
		return !_indexerNote.get();
	}
}
