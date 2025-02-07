package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class DriveCommand extends Command {
  private SwerveDriveSubsystem s_Subsystem;

  private DoubleSupplier xSupplier, ySupplier, zSupplier;
  private boolean fieldOriented;

  public DriveCommand(SwerveDriveSubsystem s_Subsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier, boolean fieldOriented) {
    this.s_Subsystem = s_Subsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.zSupplier = zSupplier;
    this.fieldOriented = fieldOriented;
    addRequirements(s_Subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SwerveModuleState[] states;

    double xSpeed = xSupplier.getAsDouble();
    double ySpeed = ySupplier.getAsDouble();
    double zSpeed = zSupplier.getAsDouble();

    xSpeed = deadzone(xSpeed);
    ySpeed = deadzone(ySpeed);
    zSpeed = deadzone(zSpeed);

    xSpeed = modifyAxis(xSpeed);
    ySpeed = modifyAxis(ySpeed);
    zSpeed = modifyAxis(zSpeed);

    if(fieldOriented){
      states = SwerveConstants.kKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, s_Subsystem.getRotation2d()));
    }
    else{
      states = SwerveConstants.kKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
    }

    s_Subsystem.setModuleStates(states);
  }

  @Override
  public void end(boolean interrupted) {
    s_Subsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public double deadzone(double speed){
    if(Math.abs(speed) < 0.1){
      return 0;
    }
    else{
      return speed;
    }
  }

  public static double modifyAxis(double num){
    num = Math.copySign(num * num, num);
    return num;
  } 
}