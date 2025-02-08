package frc.robot;

import frc.robot.commands.CoralDepolyerCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.AlgaeIntakeCmds.IntakeCmd;
import frc.robot.commands.AlgaeIntakeCmds.OuttakeCmd;
import frc.robot.commands.AlgaeIntakeCmds.StoragePositionCmd;
import frc.robot.commands.ElevatorCmds.TestPIDCmd;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  
  SwerveDriveSubsystem s_Subsystem = new SwerveDriveSubsystem();
  private final CoralIntakeSubsystem coralIntakeSub = new CoralIntakeSubsystem();
  public final AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  
  XboxController xboxDriver = new XboxController(0);
  XboxController xboxOperator = new XboxController(1);

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    s_Subsystem.setDefaultCommand(new DriveCommand(s_Subsystem, () -> xboxDriver.getLeftY() * 0.3, () -> xboxDriver.getLeftX() * 0.3, () -> xboxDriver.getRightX() * 0.6, false));
    configureBindings();
  }

  private void configureBindings() {

    /// SWERVE 
    
    new JoystickButton(xboxDriver, XboxController.Button.kA.value).onTrue(new InstantCommand(s_Subsystem::resetPigeon2));

    /// CORAL

    // new JoystickButton(xboxOperator, XboxController.Button.kA.value).onTrue(new CoralIntakeCommand(coralIntakeSub));
    // new JoystickButton(xboxOperator, XboxController.Button.kB.value).onTrue(new CoralDepolyerCommand(coralIntakeSub));
    // new JoystickButton(xboxOperator, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(() -> coralIntakeSub.setCoralPivotPIDSetpoint(0)));
    // new JoystickButton(xboxOperator, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> coralIntakeSub.setCoralPivotPIDSetpoint(45)));
    // new JoystickButton(xboxOperator, XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(() -> coralIntakeSub.setCoralPivotPIDSetpoint(-45)));
    
    /// ALGAE

    //button control to move the algae pivot to the storage position
    // new JoystickButton(xboxOperator, XboxController.Button.kB.value).onTrue(new StoragePositionCmd(algaeIntakeSubsystem));

    // new JoystickButton(xboxOperator, XboxController.Button.kY.value).onTrue(new IntakeCmd(algaeIntakeSubsystem));
    // new JoystickButton(xboxOperator, XboxController.Button.kX.value).whileTrue(new OuttakeCmd(algaeIntakeSubsystem));
    
    /// ELEVATOR

    // new JoystickButton(xboxOperator, XboxController.Button.kA.value).onTrue(new TestPIDCmd(elevatorSubsystem, 100));
    // new JoystickButton(xboxOperator, XboxController.Button.kB.value).onTrue(new TestPIDCmd(elevatorSubsystem, 0));
    
  }

 
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}