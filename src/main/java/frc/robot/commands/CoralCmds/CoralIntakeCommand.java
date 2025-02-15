package frc.robot.commands.CoralCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeCommand extends Command {

    private CoralIntakeSubsystem coralIntakeSub;
  
    public CoralIntakeCommand(CoralIntakeSubsystem coralIntakeSub) {
      this.coralIntakeSub = coralIntakeSub;
      addRequirements(this.coralIntakeSub);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      coralIntakeSub.setPIDStatus(true);
      coralIntakeSub.setCoralPivotPIDSetpoint(-340);
      coralIntakeSub.setIntakeSpeed(CoralConstants.CORAL_INTAKE_SPEED);
    }

  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      coralIntakeSub.setIntakeSpeed(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return coralIntakeSub.getOpticalSensor();
    }
  }
  
