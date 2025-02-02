package frc.team2412.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.ArmSubsystem;
import frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType;

public class SetArmPrePositionCommand extends CommandBase {

	private ArmSubsystem armSubsystem;
	private PositionType positionType;

	public SetArmPrePositionCommand(ArmSubsystem armSubsystem, PositionType positionType) {
		this.armSubsystem = armSubsystem;
		this.positionType = positionType;
	}

	@Override
	public void initialize() {
		if (!armSubsystem.getManualOverride()) {
			armSubsystem.setPosition(positionType);
		}
	}

	@Override
	public void end(boolean interrupted) {
		// armSubsystem.stopArm();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
