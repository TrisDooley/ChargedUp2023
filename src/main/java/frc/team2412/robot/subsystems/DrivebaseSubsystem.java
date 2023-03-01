package frc.team2412.robot.subsystems;

import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;
import frc.team2412.robot.Robot;
import frc.team2412.robot.sim.PhysicsSim;
import frc.team2412.robot.util.ModuleUtil;
import frc.team2412.robot.util.PFFController;
import frc.team2412.robot.util.gyroscope.Gyroscope;
import frc.team2412.robot.util.gyroscope.NavXGyro;
import frc.team2412.robot.util.gyroscope.Pigeon2Gyro;
import frc.team2412.robot.util.motorcontroller.BrushlessSparkMaxController;
import frc.team2412.robot.util.motorcontroller.MotorController;
import frc.team2412.robot.util.motorcontroller.MotorController.MotorControlMode;
import frc.team2412.robot.util.motorcontroller.MotorController.MotorNeutralMode;
import frc.team2412.robot.util.motorcontroller.TalonFXController;

public class DrivebaseSubsystem extends SubsystemBase {

	private static final boolean IS_COMP = Robot.getInstance().isCompetition();

	// ordered from front left, front right, back left, back right
	private static final Rotation2d[] PRACTICE_DRIVEBASE_ENCODER_OFFSETS = {
		Rotation2d.fromDegrees(337.236),
		Rotation2d.fromDegrees(251.982),
		Rotation2d.fromDegrees(205.839),
		Rotation2d.fromDegrees(311.396)
	};
	private static final Rotation2d[] COMP_DRIVEBASE_ENCODER_OFFSETS = {
		// ALIGNMENT WITH BEVELS FACING RIGHT
		Rotation2d.fromDegrees(248.1),
		Rotation2d.fromDegrees(298.62),
		Rotation2d.fromDegrees(316.7),
		Rotation2d.fromDegrees(27.5)
		// Rotation2d.fromDegrees(-111.796),
		// Rotation2d.fromDegrees(-28 + 90),
		// // OLD Rotation2d.fromDegrees(-343.388),
		// Rotation2d.fromDegrees(-41),
		// // OLDRotation2d.fromDegrees(-21.796),
		// Rotation2d.fromDegrees(-332.841)[\]
	};

	// max drive speed is from SDS website and not calculated with robot weight
	public static final double MAX_DRIVE_SPEED_METERS_PER_SEC = IS_COMP ? 4.4196 : 4.1148;
	// this is calculated as rotations_per_sec = velocity/(2*pi*turning radius (diagonal diameter))
	public static final Rotation2d MAX_ROTATIONS_PER_SEC =
			Rotation2d.fromRotations(IS_COMP ? 0.8574 : 1.0724);

	// magic number found by trial and error (aka informal characterization)
	private static final double ODOMETRY_ADJUSTMENT = 0.957;
	// 4 inches * odemetry adjustment
	private static final double WHEEL_DIAMETER_METERS =
			IS_COMP ? 0.1016 * ODOMETRY_ADJUSTMENT : 0.1016;
	private static final double DRIVE_REDUCTION = IS_COMP ? 6.75 : 8.14;
	// steer reduction is the conversion from rotations of motor to rotations of the wheel
	// module rotation * STEER_REDUCTION = motor rotation
	public static final double STEER_REDUCTION =
			IS_COMP ? 150.0 / 7.0 : (32.0 / 15.0) * (60.0 / 10.0);

	private static final double TIP_F = 0.02;
	private static final double TIP_P = 0.1;
	private static final double TIP_TOLERANCE = 2.5;

	private static final double DRIVE_VELOCITY_COEFFICIENT =
			DRIVE_REDUCTION / (Math.PI * WHEEL_DIAMETER_METERS); // meters to motor rotations

	// Balance controller is in degrees
	private final PFFController<Double> balanceController;

	private SwerveModuleState[] currentStates;

	private final MotorController[] moduleDriveMotors =
			IS_COMP
					? new BrushlessSparkMaxController[] {
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_FRONT_LEFT_DRIVE_MOTOR),
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_FRONT_RIGHT_DRIVE_MOTOR),
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_BACK_LEFT_DRIVE_MOTOR),
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_BACK_RIGHT_DRIVE_MOTOR)
					}
					: new TalonFXController[] {
						new TalonFXController(Hardware.DRIVEBASE_FRONT_LEFT_DRIVE_MOTOR),
						new TalonFXController(Hardware.DRIVEBASE_FRONT_RIGHT_DRIVE_MOTOR),
						new TalonFXController(Hardware.DRIVEBASE_BACK_LEFT_DRIVE_MOTOR),
						new TalonFXController(Hardware.DRIVEBASE_BACK_RIGHT_DRIVE_MOTOR)
					};

	private final MotorController[] moduleAngleMotors =
			IS_COMP
					? new BrushlessSparkMaxController[] {
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_FRONT_LEFT_ANGLE_MOTOR),
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_FRONT_RIGHT_ANGLE_MOTOR),
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_BACK_LEFT_ANGLE_MOTOR),
						new BrushlessSparkMaxController(Hardware.DRIVEBASE_BACK_RIGHT_ANGLE_MOTOR)
					}
					: new TalonFXController[] {
						new TalonFXController(Hardware.DRIVEBASE_FRONT_LEFT_ANGLE_MOTOR),
						new TalonFXController(Hardware.DRIVEBASE_FRONT_RIGHT_ANGLE_MOTOR),
						new TalonFXController(Hardware.DRIVEBASE_BACK_LEFT_ANGLE_MOTOR),
						new TalonFXController(Hardware.DRIVEBASE_BACK_RIGHT_ANGLE_MOTOR)
					};

	private final WPI_CANCoder[] moduleEncoders = {
		new WPI_CANCoder(Hardware.DRIVEBASE_FRONT_LEFT_ENCODER_PORT),
		new WPI_CANCoder(Hardware.DRIVEBASE_FRONT_RIGHT_ENCODER_PORT),
		new WPI_CANCoder(Hardware.DRIVEBASE_BACK_LEFT_ENCODER_PORT),
		new WPI_CANCoder(Hardware.DRIVEBASE_BACK_RIGHT_ENCODER_PORT)
	};

	private final Rotation2d[] moduleOffsets =
			IS_COMP ? COMP_DRIVEBASE_ENCODER_OFFSETS : PRACTICE_DRIVEBASE_ENCODER_OFFSETS;

	// 2ft x 2ft for practice bot
	private static final Translation2d[] moduleLocations =
			IS_COMP
					? new Translation2d[] {
						new Translation2d(
								Units.inchesToMeters(12.375), Units.inchesToMeters(10.375)), // front left
						new Translation2d(
								Units.inchesToMeters(12.375), Units.inchesToMeters(-10.375)), // front right
						new Translation2d(
								Units.inchesToMeters(-12.375), Units.inchesToMeters(10.375)), // back left
						new Translation2d(
								Units.inchesToMeters(-12.375), Units.inchesToMeters(-10.375)) // back right
					}
					: new Translation2d[] {
						new Translation2d(Units.inchesToMeters(8.5), Units.inchesToMeters(8.5)), // front left
						new Translation2d(Units.inchesToMeters(8.5), Units.inchesToMeters(-8.5)), // front right
						new Translation2d(Units.inchesToMeters(-8.5), Units.inchesToMeters(8.5)), // back left
						new Translation2d(Units.inchesToMeters(-8.5), Units.inchesToMeters(-8.5)) // back right
					};

	public static final SwerveDriveKinematics kinematics =
			new SwerveDriveKinematics(
					moduleLocations[0], moduleLocations[1], moduleLocations[2], moduleLocations[3]);

	private Gyroscope gyroscope;

	private SwerveDrivePoseEstimator poseEstimator;
	private Pose2d pose;

	private Field2d field = new Field2d();

	private DoublePublisher frontLeftActualVelocityPublisher;
	private DoublePublisher frontRightActualVelocityPublisher;
	private DoublePublisher backLeftActualVelocityPublisher;
	private DoublePublisher backRightActualVelocityPublisher;

	private DoublePublisher frontLeftTargetVelocityPublisher;
	private DoublePublisher frontRightTargetVelocityPublisher;
	private DoublePublisher backLeftTargetVelocityPublisher;
	private DoublePublisher backRightTargetVelocityPublisher;

	private DoublePublisher frontLeftActualAnglePublisher;
	private DoublePublisher frontRightActualAnglePublisher;
	private DoublePublisher backLeftActualAnglePublisher;
	private DoublePublisher backRightActualAnglePublisher;

	private DoublePublisher frontLeftTargetAnglePublisher;
	private DoublePublisher frontRightTargetAnglePublisher;
	private DoublePublisher backLeftTargetAnglePublisher;
	private DoublePublisher backRightTargetAnglePublisher;

	private PIDController compTranslationalPID = new PIDController(0.0007, 0, 0);
	private PIDController compRotationalPID = new PIDController(0.1, 0, 0.5);

	private NetworkTableInstance networkTableInstance;
	private NetworkTable networkTableDrivebase;

	public DrivebaseSubsystem(SwerveDrivePoseEstimator initialPoseEstimator) {
		gyroscope = IS_COMP ? new Pigeon2Gyro(Hardware.GYRO_PORT) : new NavXGyro(SerialPort.Port.kMXP);
		// Bonk's gyro has positive as counter-clockwise
		if (!IS_COMP) {
			gyroscope.setInverted(true);
		}

		poseEstimator = initialPoseEstimator;
		poseEstimator.resetPosition(gyroscope.getRawYaw(), getModulePositions(), new Pose2d());

		pose = poseEstimator.getEstimatedPosition();

		balanceController =
				PFFController.ofDouble(TIP_F, TIP_P)
						.setTargetPosition(gyroscope.getRawRoll().getDegrees())
						.setTargetPositionTolerance(TIP_TOLERANCE);

		// configure encoders offsets
		for (int i = 0; i < moduleEncoders.length; i++) {
			moduleEncoders[i].configFactoryDefault();
			moduleEncoders[i].configSensorInitializationStrategy(
					SensorInitializationStrategy.BootToAbsolutePosition);
		}

		// configure drive motors
		for (int i = 0; i < moduleDriveMotors.length; i++) {
			MotorController driveMotor = moduleDriveMotors[i];
			driveMotor.setNeutralMode(MotorController.MotorNeutralMode.BRAKE);

			if (IS_COMP) {
				driveMotor.setControlMode(MotorControlMode.VELOCITY);
				driveMotor.setPID(
						compTranslationalPID.getP(), compTranslationalPID.getI(), compTranslationalPID.getD());
				driveMotor.setMeasurementPeriod(8);
			} else {
				driveMotor.setControlMode(MotorControlMode.VELOCITY);
				driveMotor.setPID(0.1, 0.001, 1023.0 / 20660.0);
			}
		}

		// configure angle motors
		for (int i = 0; i < moduleAngleMotors.length; i++) {
			MotorController steeringMotor = moduleAngleMotors[i];
			steeringMotor.configFactoryDefault();
			steeringMotor.setNeutralMode(MotorNeutralMode.BRAKE);
			// Configure PID values
			if (IS_COMP) {
				steeringMotor.setPID(
						compRotationalPID.getP(), compRotationalPID.getI(), compRotationalPID.getD());
			} else {
				steeringMotor.setPID(0.15, 0.00, 1.0);
			}

			if (IS_COMP) {
				steeringMotor.setInverted(true);
			}

			steeringMotor.useIntegratedEncoder();
			steeringMotor.setIntegratedEncoderPosition(
					getModuleAngles()[i].getRotations() * STEER_REDUCTION);
			steeringMotor.configureOptimization();

			steeringMotor.setControlMode(MotorControlMode.POSITION);
		}

		// configure network tables
		configureNetworkTables();
	}

	/** Drives the robot using forward, strafe, and rotation. Units in meters */
	public void drive(
			double forward,
			double strafe,
			Rotation2d rotation,
			boolean fieldOriented,
			boolean autoBalance) {
		// Auto balancing will only be used in autonomous
		if (autoBalance) {
			forward += balanceController.update(gyroscope.getRawRoll().getDegrees());
		}

		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

		if (fieldOriented) {
			chassisSpeeds =
					ChassisSpeeds.fromFieldRelativeSpeeds(
							forward, -strafe, rotation.getRadians(), gyroscope.getAngle());
		} else {
			chassisSpeeds = new ChassisSpeeds(forward, -strafe, rotation.getRadians());
		}

		drive(chassisSpeeds);
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		SwerveModuleState[] moduleStates = getModuleStates(chassisSpeeds);
		// if (Math.abs(chassisSpeeds.vxMetersPerSecond) <= 0.01
		// 		&& Math.abs(chassisSpeeds.vyMetersPerSecond) <= 0.01
		// 		&& Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= 0.01) {
		// 	moduleStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
		// 	moduleStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
		// 	moduleStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
		// 	moduleStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
		// }
		drive(moduleStates);
	}

	/** Drives the robot using states */
	public void drive(SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_DRIVE_SPEED_METERS_PER_SEC);

		for (int i = 0; i < states.length; i++) {
			if (IS_COMP) {
				states[i] = SwerveModuleState.optimize(states[i], getModuleAngles()[i]);
			} else {
				// this optimize assumes that PID loop is not continuous
				states[i] = ModuleUtil.optimize(states[i], getModuleAngles()[i]);
			}
		}

		// Set motor speeds and angles
		for (int i = 0; i < moduleDriveMotors.length; i++) {
			// if (IS_COMP) {
			// 	moduleDriveMotors[i].set(
			// 			states[i].speedMetersPerSecond / MAX_DRIVE_SPEED_METERS_PER_SEC * 12); // set voltage
			// } else {
			moduleDriveMotors[i].set(
					states[i].speedMetersPerSecond * DRIVE_VELOCITY_COEFFICIENT); // set velocity
			// }
		}
		for (int i = 0; i < moduleAngleMotors.length; i++) {
			moduleAngleMotors[i].set(states[i].angle.getRotations() * STEER_REDUCTION);
		}

		currentStates = states;

		frontLeftActualVelocityPublisher.set(
				moduleDriveMotors[0].getVelocity() / DRIVE_VELOCITY_COEFFICIENT);
		frontRightActualVelocityPublisher.set(
				moduleDriveMotors[1].getVelocity() / DRIVE_VELOCITY_COEFFICIENT);
		backLeftActualVelocityPublisher.set(
				moduleDriveMotors[2].getVelocity() / DRIVE_VELOCITY_COEFFICIENT);
		backRightActualVelocityPublisher.set(
				moduleDriveMotors[3].getVelocity() / DRIVE_VELOCITY_COEFFICIENT);

		frontLeftTargetVelocityPublisher.set(
				states[0].speedMetersPerSecond * DRIVE_VELOCITY_COEFFICIENT);
		frontRightTargetVelocityPublisher.set(
				states[1].speedMetersPerSecond * DRIVE_VELOCITY_COEFFICIENT);
		backLeftTargetVelocityPublisher.set(
				states[2].speedMetersPerSecond * DRIVE_VELOCITY_COEFFICIENT);
		backRightTargetVelocityPublisher.set(
				states[3].speedMetersPerSecond * DRIVE_VELOCITY_COEFFICIENT);

		frontLeftTargetAnglePublisher.set(states[0].angle.getDegrees());
		frontRightTargetAnglePublisher.set(states[1].angle.getDegrees());
		backLeftTargetAnglePublisher.set(states[2].angle.getDegrees());
		backRightTargetAnglePublisher.set(states[3].angle.getDegrees());
	}

	/**
	 * Array with modules with front left at [0], front right at [1], back left at [2], back right at
	 * [3]
	 */
	public SwerveModuleState[] getModuleStates(ChassisSpeeds speeds) {
		return kinematics.toSwerveModuleStates(speeds);
	}

	public SwerveModuleState[] getCurrentStates() {
		return currentStates;
	}

	public double getVelocity() {
		if (currentStates == null) {
			return 0.0;
		}
		return Math.sqrt(
				Math.pow(kinematics.toChassisSpeeds(getCurrentStates()).vxMetersPerSecond, 2)
						+ Math.pow(kinematics.toChassisSpeeds(getCurrentStates()).vyMetersPerSecond, 2));
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];

		for (int i = 0; i < moduleDriveMotors.length; i++) {
			positions[i] =
					new SwerveModulePosition(
							moduleDriveMotors[i].getIntegratedEncoderPosition() / DRIVE_VELOCITY_COEFFICIENT,
							Rotation2d.fromRotations(
									moduleAngleMotors[i].getIntegratedEncoderPosition() / STEER_REDUCTION));
		}

		return positions;
	}

	/** Returns the module angles using encoders */
	public Rotation2d[] getModuleAngles() {
		Rotation2d[] rotations = new Rotation2d[4];
		for (int i = 0; i < moduleAngleMotors.length; i++) {
			rotations[i] =
					Rotation2d.fromDegrees(
							(moduleEncoders[i].getAbsolutePosition() - moduleOffsets[i].getDegrees()));
		}
		return rotations;
	}

	/** Returns the kinematics */
	public SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	/**
	 * Resets the gyroscope's angle to 0 After this is called, the radio (on bonk) or the intake (on
	 * comp) will be the robot's new global forward
	 */
	public void resetGyroAngle() {
		resetGyroAngle(gyroscope.getRawYaw());
	}

	/**
	 * Resets the robot's forward to the new angle relative to the radio (on bonk)
	 *
	 * @param angle The new forward
	 */
	public void resetGyroAngle(Rotation2d angle) {
		gyroscope.setAngleAdjustment(angle.unaryMinus());
	}

	/** Returns the robot's pose */
	public Pose2d getPose() {
		return pose;
	}

	/**
	 * Set's the robot's pose to the provided pose
	 *
	 * @param pose the new pose
	 */
	public void resetPose(Pose2d pose) {
		poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
		this.pose = pose;
	}

	/**
	 * Reset's the robot's pose to (0, 0) with rotation of 0. <br>
	 * Also resets the gyroscope
	 */
	public void resetPose() {
		resetGyroAngle();
		resetPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
	}

	public void simInit(PhysicsSim sim) {
		for (int i = 0; i < moduleDriveMotors.length; i++) {
			// sim.addTalonFX(moduleDriveMotors[i], 2, 20000, true);
			// sim.addTalonFX(moduleAngleMotors[i], 2, 20000);
		}
	}

	private void configureNetworkTables() {
		SmartDashboard.putData("Field", field);

		networkTableInstance = NetworkTableInstance.getDefault();
		networkTableDrivebase = networkTableInstance.getTable("Drivebase");

		frontLeftActualVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Front left actual velocity").publish();
		frontRightActualVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Front right actual velocity").publish();
		backLeftActualVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Back left actual velocity").publish();
		backRightActualVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Back right actual velocity").publish();

		frontLeftTargetVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Front left target velocity").publish();
		frontRightTargetVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Front right target velocity").publish();
		backLeftTargetVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Back left target velocity").publish();
		backRightTargetVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Back right target velocity").publish();

		frontLeftActualAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Front left actual angle").publish();
		frontRightActualAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Front right actual angle").publish();
		backLeftActualAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Back left actual angle").publish();
		backRightActualAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Back right actual angle").publish();

		frontLeftTargetAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Front left target angle").publish();
		frontRightTargetAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Front right target angle").publish();
		backLeftTargetAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Back left target angle").publish();
		backRightTargetAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Back right target angle").publish();

		frontLeftActualVelocityPublisher.set(0.0);
		frontRightActualVelocityPublisher.set(0.0);
		backLeftActualVelocityPublisher.set(0.0);
		backRightActualVelocityPublisher.set(0.0);

		frontLeftTargetVelocityPublisher.set(0.0);
		frontRightTargetVelocityPublisher.set(0.0);
		backLeftTargetVelocityPublisher.set(0.0);
		backRightTargetVelocityPublisher.set(0.0);

		frontLeftActualAnglePublisher.set(0.0);
		frontRightActualAnglePublisher.set(0.0);
		backLeftActualAnglePublisher.set(0.0);
		backRightActualAnglePublisher.set(0.0);

		frontLeftTargetAnglePublisher.set(0.0);
		frontRightTargetAnglePublisher.set(0.0);
		backLeftTargetAnglePublisher.set(0.0);
		backRightTargetAnglePublisher.set(0.0);

		SmartDashboard.putData("Translational PID", compTranslationalPID);
		SmartDashboard.putData("Rotational PID", compRotationalPID);
	}

	private double oldTranslationalSetpoint = 0.0;
	private double oldRotationalSetpoint = 0.0;

	@Override
	public void periodic() {
		pose = poseEstimator.update(gyroscope.getAngle(), getModulePositions());
		field.setRobotPose(pose);

		for (MotorController motor : moduleDriveMotors) {
			if (compTranslationalPID.getSetpoint() != oldTranslationalSetpoint) {
				motor.setPID(
						compTranslationalPID.getP(), compTranslationalPID.getI(), compTranslationalPID.getD());
				oldTranslationalSetpoint = compTranslationalPID.getSetpoint();
			}
		}
		for (MotorController motor : moduleAngleMotors) {
			if (compRotationalPID.getSetpoint() != oldRotationalSetpoint) {
				motor.setPID(compRotationalPID.getP(), compRotationalPID.getI(), compRotationalPID.getD());
				oldRotationalSetpoint = compRotationalPID.getSetpoint();
			}
		}

		frontLeftActualAnglePublisher.set(getModuleAngles()[0].getDegrees());
		frontRightActualAnglePublisher.set(getModuleAngles()[1].getDegrees());
		backLeftActualAnglePublisher.set(getModuleAngles()[2].getDegrees());
		backRightActualAnglePublisher.set(getModuleAngles()[3].getDegrees());
	}
}
