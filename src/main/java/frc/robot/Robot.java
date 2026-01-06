// Copyright (c) FIRST
// Open Source Software; WPILib BSD license file in the root directory.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Robot extends TimedRobot {
  private final CommandXboxController driverController = new CommandXboxController(0);
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final int LEFT_LEADER_ID = 1;
  private final int RIGHT_LEADER_ID = 2;

  private final SparkMax leftFront = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
  private final SparkMax leftRear = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax rightFront = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
  private final SparkMax rightRear = new SparkMax(4, MotorType.kBrushed);

  private SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
  private SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
  private SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
  private SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

  private final DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);

  public Robot() {
    m_robotContainer = new RobotContainer();

    rightLeaderConfig
      .inverted(true)
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);

    leftLeaderConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(50);
    
    rightFollowerConfig
      .follow(RIGHT_LEADER_ID)
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);
    
    leftFollowerConfig
      .follow(LEFT_LEADER_ID)
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);
    
    leftFront.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftRear.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFront.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightRear.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void disabledInit() {
    drive.stopMotor();
  }

  @Override
  public void disabledPeriodic() {
    drive.stopMotor();
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  @Override
  public void teleopExit() {
    drive.stopMotor();
  }

  @Override
  public void testExit() {
    drive.stopMotor();
  }


  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }


  @Override
  public void teleopPeriodic() {
    double leftY = -driverController.getLeftY();
    double rightY = -driverController.getRightY();

    
    drive.tankDrive(-leftY, -rightY);

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}