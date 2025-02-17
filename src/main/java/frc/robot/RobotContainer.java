package frc.robot;

import frc.robot.commands.AlgaeIntakeCmds.IntakeCmd;
import frc.robot.commands.AlgaeIntakeCmds.OuttakeCmd;
import frc.robot.commands.CoralCmds.CoralDeployerCommand;
import frc.robot.commands.CoralCmds.CoralIntakeCommand;
import frc.robot.commands.CoralCmds.PivotLeftCommand;
import frc.robot.commands.CoralCmds.PivotMiddleCommand;
import frc.robot.commands.CoralCmds.PivotRightCommand;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import frc.robot.LimelightHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  /* * * CONTROLLERS * * */
  private final CommandXboxController d_xbox = new CommandXboxController(0);
  private final CommandXboxController o_xbox = new CommandXboxController(1); 

  /* * * AUTO CHOOSER * * */
  private final SendableChooser<Command> autoChooser;

  private PIDController limelightPID = new PIDController(0.001, 0, 0);

  /* * * SUBSYSTEMS * * */
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final CoralIntakeSubsystem coralIntakeSub = new CoralIntakeSubsystem();
  public final AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {

    //////////////////////////
    //        DRIVER        //
    //////////////////////////

    // SWERVE 
    // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() ->
              drive.withVelocityX(-d_xbox.getLeftY() * MaxSpeed * 0.4) // Drive forward with negative Y (forward)
                  .withVelocityY(-d_xbox.getLeftX() * MaxSpeed * 0.4) // Drive left with negative X (left)
                  .withRotationalRate(-d_xbox.getRightX() * MaxAngularRate * 0.6) // Drive counterclockwise with negative X (left)
          )
      );  

      // RESET HEADING 
      // reset the field-centric heading on left bumper press
      d_xbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
      drivetrain.registerTelemetry(logger::telemeterize);

      // OUTPUT CORAL 
      d_xbox.a().onTrue(new CoralDeployerCommand(coralIntakeSub)); 
      // INTAKE CORAL 
      d_xbox.b().whileTrue(new ParallelCommandGroup(
          new CoralIntakeCommand(coralIntakeSub), 
          new PivotMiddleCommand(coralIntakeSub)
      ));


      /* * * CTRE STUFF * * */

      // EMERGENCY STOP DRIVETRAIN 
      // d_xbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
      /// ROBOT ORIENTED?? 
      // d_xbox.b().whileTrue(drivetrain.applyRequest(() ->
      //     point.withModuleDirection(new Rotation2d(-d_xbox.getLeftY(), -d_xbox.getLeftX()))
      // ));

      // SYSID 
      // Run SysId routines when holding back/start and X/Y.
      // Note that each routine should be run exactly once in a single log.
      // d_xbox.back().and(d_xbox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      // d_xbox.back().and(d_xbox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      // d_xbox.start().and(d_xbox.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      // d_xbox.start().and(d_xbox.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    //////////////////////////
    //       OPERATOR       //
    //////////////////////////
    
    // CORAL LEFT/RIGHT 
    o_xbox.leftBumper().onTrue(new PivotLeftCommand(coralIntakeSub)); 
    o_xbox.rightBumper().onTrue(new PivotRightCommand(coralIntakeSub));

    //////////////////////////
    //        TESTING       //
    //////////////////////////

    // LIMELIGHT TESTING 
      // new JoystickButton(joystick, XboxController.Button.kA.value).onTrue(new InstantCommand(drivetrain::))
      d_xbox.x().whileTrue(
          drivetrain.applyRequest(() ->
          drive.withVelocityX(-d_xbox.getLeftY() * MaxSpeed * 0.4) // Drive forward with negative Y (forward)
              .withVelocityY(-d_xbox.getLeftX() * MaxSpeed * 0.4) // Drive left with negative X (left)
              .withRotationalRate(limelightPID.calculate(limelightPID.calculate(LimelightHelpers.getTX("limelight"), 0))) // Drive counterclockwise with negative X (left)
          )
      );
  }

 
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}