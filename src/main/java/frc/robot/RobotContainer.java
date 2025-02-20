package frc.robot;

import frc.robot.commands.AlgaeIntakeCmds.IntakeCmd;
import frc.robot.commands.AlgaeIntakeCmds.OuttakeCmd;
import frc.robot.commands.AlgaePivotCmds.DealgifyL3PositionCmd;
import frc.robot.commands.AlgaePivotCmds.NoAlgaeTuckCmd;
import frc.robot.commands.CoralCmds.CoralDeployerCommand;
import frc.robot.commands.CoralCmds.CoralIntakeCommand;
import frc.robot.commands.CoralCmds.PivotLeftCommand;
import frc.robot.commands.CoralCmds.PivotMiddleCommand;
import frc.robot.commands.CoralCmds.PivotRightCommand;
import frc.robot.commands.ElevatorCmds.L2ElevPos;
import frc.robot.commands.ElevatorCmds.L3ElevPos;
import frc.robot.commands.IntegratedCmds.AlgaeGroundPickup;
import frc.robot.commands.IntegratedCmds.L2Dealgify;
import frc.robot.commands.IntegratedCmds.ProcessorScoring;
import frc.robot.commands.IntegratedCmds.TuckCmd;
import frc.robot.commands.IntegratedCmds.TuckWithAlgae;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.subsystems.CoralPivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
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

  private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  private final Telemetry logger = new Telemetry(MaxSpeed);

  /* * * CONTROLLERS * * */
  private final CommandXboxController d_xbox = new CommandXboxController(0);
  // private final CommandXboxController o_xbox = new CommandXboxController(1); 
  private final Joystick o_joystick = new Joystick(1); 

  /* * * AUTO CHOOSER * * */
  private final SendableChooser<Command> autoChooser;

  private PIDController limelightPID = new PIDController(0.001, 0, 0);

  /* * * SUBSYSTEMS * * */
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final CoralIntakeSubsystem coralIntakeSub = new CoralIntakeSubsystem();
  private final CoralPivotSubsystem coralPivotSub = new CoralPivotSubsystem(); 
  public final AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  public final Command elevInit = new InstantCommand(() -> elevatorSubsystem.turnPIDOff(), elevatorSubsystem);
  public final Command disableAlgaeIntakePID = new InstantCommand(() -> algaeIntakeSubsystem.disablePID(), algaeIntakeSubsystem);


  public RobotContainer() {

    registerNamedCommands();

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
              drive.withVelocityX(-d_xbox.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                  .withVelocityY(-d_xbox.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                  .withRotationalRate(-d_xbox.getRightX() * MaxAngularRate * 0.7) // Drive counterclockwise with negative X (left)
          )
      );  

      // ROBOT CENTRIC 
      d_xbox.x().whileTrue(
          drivetrain.applyRequest(() -> 
              driveRobotCentric.withVelocityX(-d_xbox.getLeftY() * MaxSpeed * 0.5)
              .withVelocityY(-d_xbox.getLeftX() * MaxSpeed * 0.5)
              .withRotationalRate(-d_xbox.getRightX() * MaxAngularRate * 0.7))
      );

      // RESET HEADING 
      // reset the field-centric heading 
      d_xbox.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
      drivetrain.registerTelemetry(logger::telemeterize);

      // OUTPUT CORAL 
      d_xbox.leftBumper().whileTrue(new CoralDeployerCommand(coralIntakeSub)); 
      // INTAKE CORAL 
      d_xbox.rightBumper().whileTrue(new CoralIntakeCommand(coralIntakeSub, coralPivotSub)); 

      // // IMTAKE ALGAE FROM GROUND
      // d_xbox.a().onTrue(new AlgaeGroundPickup(elevatorSubsystem, algaeIntakeSubsystem)); 
      // d_xbox.b().whileTrue(new OuttakeCmd(algaeIntakeSubsystem)); 

      /* * * CTRE STUFF * * */

      // EMERGENCY STOP DRIVETRAIN 
      d_xbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
      /// POINT WHEELS  
      d_xbox.b().whileTrue(drivetrain.applyRequest(() ->
          point.withModuleDirection(new Rotation2d(-d_xbox.getLeftY(), -d_xbox.getLeftX()))
      ));

      // SYSID 
      // Run SysId routines when holding back/start and X/Y.
      // Note that each routine should be run exactly once in a single log.
      d_xbox.back().and(d_xbox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      d_xbox.back().and(d_xbox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      d_xbox.start().and(d_xbox.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      d_xbox.start().and(d_xbox.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    //////////////////////////
    //       OPERATOR       //
    //////////////////////////
    /// 7- L3 DEALG       8- L2 ELEV 
    /// 9- L2 DEALG       10- PROCESSOR 
    /// 11- TUCK W ALG    12- TUCK WO ALG 
    
    // CORAL LEFT/RIGHT 
    new JoystickButton(o_joystick, 3).onTrue(new PivotLeftCommand(coralPivotSub)); 
    new JoystickButton(o_joystick, 4).onTrue(new PivotRightCommand(coralPivotSub));

    // ALGAE INTAKE OVERRIDE 
    new JoystickButton(o_joystick, 2).whileTrue(new IntakeCmd(algaeIntakeSubsystem)); 

    // TUCKS 
    new JoystickButton(o_joystick, 11).onTrue(new TuckWithAlgae(algaeIntakeSubsystem, elevatorSubsystem)); 
    new JoystickButton(o_joystick, 12).onTrue(new TuckCmd(algaeIntakeSubsystem, elevatorSubsystem)); 
    new JoystickButton(o_joystick, 5).onTrue(new NoAlgaeTuckCmd(algaeIntakeSubsystem)); 

    // PROCESSOR 
    new JoystickButton(o_joystick, 10).onTrue(new ProcessorScoring(algaeIntakeSubsystem, elevatorSubsystem)); 

    // SCORING 
    new JoystickButton(o_joystick, 9).onTrue(new L2Dealgify(elevatorSubsystem, algaeIntakeSubsystem)); 
    new JoystickButton(o_joystick, 7).onTrue(new L3ElevPos(elevatorSubsystem)); 
    // new JoystickButton(o_joystick, 6).onTrue(new DealgifyL3PositionCmd(algaeIntakeSubsystem)); 
    new JoystickButton(o_joystick, 8).onTrue(new L2ElevPos(elevatorSubsystem)); 

    //////////////////////////
    //        TESTING       //
    //////////////////////////

    // LIMELIGHT TESTING 
      // new JoystickButton(joystick, XboxController.Button.kA.value).onTrue(new InstantCommand(drivetrain::))
      // d_xbox.x().whileTrue(
      //     drivetrain.applyRequest(() ->
      //     drive.withVelocityX(-d_xbox.getLeftY() * MaxSpeed * 0.4) // Drive forward with negative Y (forward)
      //         .withVelocityY(-d_xbox.getLeftX() * MaxSpeed * 0.4) // Drive left with negative X (left)
      //         .withRotationalRate(limelightPID.calculate(limelightPID.calculate(LimelightHelpers.getTX("limelight"), 0))) // Drive counterclockwise with negative X (left)
      //     )
      // );
  }

  public Command ElevInit() {
    return elevInit;
  }

  public Command disableAlgaeIntakePID(){
    return disableAlgaeIntakePID;
  }
 
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return null; 
  }

  public void registerNamedCommands() {
    // ELEV L2, PIV RIGHT 
    NamedCommands.registerCommand("L2Right", new ParallelCommandGroup(
      new L2ElevPos(elevatorSubsystem), 
      new PivotRightCommand(coralPivotSub)
    ));

    // ELEV L2, PIV RIGHT 
    NamedCommands.registerCommand("L2Left", new ParallelCommandGroup(
      new L2ElevPos(elevatorSubsystem), 
      new PivotLeftCommand(coralPivotSub)
    ));

    // TUCK 
    NamedCommands.registerCommand("tuck", new TuckCmd(algaeIntakeSubsystem, elevatorSubsystem));

    // OUTTAKE CORAL 
    NamedCommands.registerCommand("outtake", new CoralDeployerCommand(coralIntakeSub));

    // INTAKE CORAL 
    NamedCommands.registerCommand("intake", new CoralIntakeCommand(coralIntakeSub, coralPivotSub));

  }


}