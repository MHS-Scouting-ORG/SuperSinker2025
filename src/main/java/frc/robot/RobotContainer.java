package frc.robot;

import frc.robot.commands.CoralDepolyerCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.AlgaeIntakeCmds.IntakeCmd;
import frc.robot.commands.AlgaeIntakeCmds.OuttakeCmd;
import frc.robot.commands.AlgaePivotCmds.StoragePositionCmd;
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

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

private final Telemetry logger = new Telemetry(MaxSpeed);

private final CommandXboxController joystick = new CommandXboxController(0);

public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


  private final SendableChooser<Command> autoChooser;
  
  
  private final CoralIntakeSubsystem coralIntakeSub = new CoralIntakeSubsystem();
  public final AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  
  XboxController xboxDriver = new XboxController(0);
  XboxController xboxOperator = new XboxController(1);

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  private void configureBindings() {

    /// SWERVE 
    // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.4) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.4) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.6) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // new JoystickButton(joystick, XboxController.Button.kA.value).onTrue(new InstantCommand(drivetrain::))

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