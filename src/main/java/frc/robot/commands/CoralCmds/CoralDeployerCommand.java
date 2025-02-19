package frc.robot.commands.CoralCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.Constants.CoralConstants;


public class CoralDeployerCommand extends Command {
    private CoralIntakeSubsystem coralIntakeSub; 
  
    public CoralDeployerCommand(CoralIntakeSubsystem coralIntakeSub) {
      this.coralIntakeSub = coralIntakeSub;
      addRequirements(this.coralIntakeSub);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      coralIntakeSub.setIntakeSpeed(CoralConstants.CORAL_DEPLOY_SPEED);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      coralIntakeSub.setIntakeSpeed(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return !coralIntakeSub.getOpticalSensor();
    }
}
