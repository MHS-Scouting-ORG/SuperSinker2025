// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command coralInnitCommand;
  private Command coralSetpointCommand;
  private final RobotContainer m_robotContainer;

  private final Command disableAlgaePivotPID;
  private Command elevInit;

  public Robot() {
    m_robotContainer = new RobotContainer();

    elevInit = m_robotContainer.ElevInit();
    coralInnitCommand = m_robotContainer.coralInnit();
    coralSetpointCommand = m_robotContainer.coralSetpoint();
    // algaeInit = m_robotContainer.algaeInit();
    disableAlgaePivotPID = m_robotContainer.disableAlgaeIntakePID();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    elevInit.schedule();
    elevInit.cancel();

    disableAlgaePivotPID.schedule();
    disableAlgaePivotPID.cancel();
    
    coralInnitCommand.schedule();
    coralInnitCommand.cancel();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    elevInit.schedule();
    elevInit.cancel();

    disableAlgaePivotPID.schedule();
    disableAlgaePivotPID.cancel();
    
    coralInnitCommand.schedule();
    coralInnitCommand.cancel();
    // coralSetpointCommand.schedule();
    // coralSetpointCommand.cancel();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
