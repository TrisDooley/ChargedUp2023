package frc.team2412.robot;

import static frc.team2412.robot.Controls.ControlConstants.*;
import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.WRIST_PRESCORE;
import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.WRIST_RETRACT;
import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.WRIST_SCORE;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.ARM_HIGH_POSITION;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.ARM_LOW_POSITION;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.ARM_MIDDLE_POSITION;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.ARM_SUBSTATION_POSITION;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team2412.robot.commands.arm.ManualArmOverrideOffCommand;
import frc.team2412.robot.commands.arm.ManualArmOverrideOnCommand;
import frc.team2412.robot.commands.arm.SetFullArmCommand;
import frc.team2412.robot.commands.arm.SetWristCommand;
import frc.team2412.robot.commands.arm.SetArmPrePositionCommand;
import frc.team2412.robot.commands.drivebase.DriveCommand;
import frc.team2412.robot.commands.intake.IntakeDefaultCommand;
import frc.team2412.robot.commands.intake.IntakeSetInCommand;
import frc.team2412.robot.commands.intake.IntakeSetOutCommand;
import frc.team2412.robot.commands.led.LEDPurpleCommand;
import frc.team2412.robot.commands.led.LEDYellowCommand;

public class Controls {
	public static class ControlConstants {
		public static final int CONTROLLER_PORT = 0;
		public static final int CODRIVER_CONTROLLER_PORT = 1;

		public static final double FAST_WRIST_EXTEND_TOLERANCE = 0.5;
	}

	private final CommandXboxController driveController;
	private final CommandXboxController codriveController;

	//sticks: drive
	//left trigger while held: intake 
	//left trigger when released: hold
	//right trigger: hold to prescore
	//right trigger: release to wrist score
	//right bumper: hold to extake
	//right bumper: release to lower arm

	//Codriver controls should be the same

	//Driver
	
	public final Trigger intakeButton;
	public final Trigger prescoreButton;
	public final Trigger scoreButton;

	public final Trigger triggerDriverAssist;

	// Codriver

	public final Trigger armManualControlOn;
	public final Trigger armManualControlOff;

	public final Trigger positionLow;
	public final Trigger positionMiddle;
	public final Trigger positionHigh;
	public final Trigger positionSubstation;

	public final Trigger ledPurple;
	public final Trigger ledYellow;

	private final Subsystems s;

	public Controls(Subsystems s) {
		driveController = new CommandXboxController(CONTROLLER_PORT);
		codriveController = new CommandXboxController(CODRIVER_CONTROLLER_PORT);
		this.s = s;

		//Driver Bindings
		triggerDriverAssist = driveController.leftBumper();
		intakeButton = driveController.leftTrigger();
		prescoreButton = driveController.rightTrigger();
		scoreButton = driveController.rightBumper();

		//Codriver Bindings
		armManualControlOn = codriveController.start();
		armManualControlOff = codriveController.back();

		positionLow = codriveController.y();
		positionMiddle = codriveController.x();
		positionHigh = codriveController.a();
		positionSubstation = codriveController.b();

		ledPurple = codriveController.rightBumper();
		ledYellow = codriveController.leftBumper();

		if (Subsystems.SubsystemConstants.DRIVEBASE_ENABLED) {
			bindDrivebaseControls();
		}
		if (Subsystems.SubsystemConstants.INTAKE_ENABLED) {
			bindIntakeControls();
		}
		if (Subsystems.SubsystemConstants.LED_ENABLED) {
			bindLEDControls();
		}
		if (Subsystems.SubsystemConstants.ARM_ENABLED) {
			bindArmControls();
		}
	}

	public void bindDrivebaseControls() {
		CommandScheduler.getInstance()
				.setDefaultCommand(
						s.drivebaseSubsystem,
						new DriveCommand(
								s.drivebaseSubsystem,
								driveController::getLeftY,
								driveController::getLeftX,
								driveController::getRightX));
		driveController.start().onTrue(new InstantCommand(s.drivebaseSubsystem::resetGyroAngle));
		driveController.rightStick().onTrue(new InstantCommand(s.drivebaseSubsystem::resetPose));
		driveController.leftStick().onTrue(new InstantCommand(s.drivebaseSubsystem::toggleXWheels));
	}

	public void bindArmControls() {
		//Manual control
		s.armSubsystem.setPresetAdjustJoysticks(
				codriveController::getRightY, codriveController::getLeftY);

		armManualControlOn.onTrue(
				new ManualArmOverrideOnCommand(
						s.armSubsystem, codriveController::getRightY, codriveController::getLeftY));
		armManualControlOff.onTrue(new ManualArmOverrideOffCommand(s.armSubsystem));


		//Codriver setting position controls
		positionLow.onTrue(new SetArmPrePositionCommand(s.armSubsystem, ARM_LOW_POSITION));
		positionMiddle.onTrue(new SetArmPrePositionCommand(s.armSubsystem, ARM_MIDDLE_POSITION));
		positionHigh.onTrue(new SetArmPrePositionCommand(s.armSubsystem, ARM_HIGH_POSITION));
		positionSubstation.onTrue(new SetArmPrePositionCommand(s.armSubsystem, ARM_SUBSTATION_POSITION));


		//Driver Activating Arm Controls
		//Prescore
		prescoreButton.onTrue(new SetFullArmCommand(s.armSubsystem, s.armSubsystem.getPosition(), WRIST_PRESCORE));
		prescoreButton.onFalse(new SetWristCommand(s.armSubsystem, WRIST_SCORE));
		//Score
		scoreButton.onTrue(new IntakeSetOutCommand(s.intakeSubsystem));
		scoreButton.onFalse(new SetFullArmCommand(s.armSubsystem, ARM_LOW_POSITION, WRIST_RETRACT));
		//Retract when done scoring
		scoreButton.onFalse(new SetFullArmCommand(s.armSubsystem, ARM_LOW_POSITION, WRIST_RETRACT));
	}

	public void bindIntakeControls() {
		CommandScheduler.getInstance()
				.setDefaultCommand(s.intakeSubsystem, new IntakeDefaultCommand(s.intakeSubsystem));

		// Driver Buttons
		//Turn the motors on once
		intakeButton.onTrue(new IntakeSetInCommand(s.intakeSubsystem));
		//Set new position if codriver changes the arm position while intaking
		intakeButton.whileTrue(new SetFullArmCommand(s.armSubsystem, s.armSubsystem.getPosition(), WRIST_PRESCORE));
		//Carry position when done intaking(the intake motors stall to hold a piece, no need to stop them)
		intakeButton.onFalse(new SetFullArmCommand(s.armSubsystem, ARM_LOW_POSITION, WRIST_RETRACT));
	}

	public void bindLEDControls() {
		ledPurple.onTrue(new LEDPurpleCommand(s.ledSubsystem));
		ledYellow.onTrue(new LEDYellowCommand(s.ledSubsystem));
	}
}
