package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeCommand extends Command {

    private CoralIntakeSubsystem coralIntakeSub; 
  
    public CoralIntakeCommand(CoralIntakeSubsystem coralIntakeSub) {
      this.coralIntakeSub = coralIntakeSub;
      addRequirements(this.coralIntakeSub);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      coralIntakeSub.setIntakeSpeed(0.1);
    }
  
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
  
