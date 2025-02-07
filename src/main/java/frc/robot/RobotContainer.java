package frc.robot;

import frc.robot.commands.DriveCommand;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  
  SwerveDriveSubsystem s_Subsystem = new SwerveDriveSubsystem();


  XboxController xboxController = new XboxController(0);

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    s_Subsystem.setDefaultCommand(new DriveCommand(s_Subsystem, () -> xboxController.getLeftY() * 0.3, () -> xboxController.getLeftX() * 0.3, () -> xboxController.getRightX() * 0.6, false));
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(xboxController, XboxController.Button.kA.value).onTrue(new InstantCommand(s_Subsystem::resetPigeon2));
  }

 
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}