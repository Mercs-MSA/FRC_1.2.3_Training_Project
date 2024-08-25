// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.math.controller.PIDController;

// All of this is related to the Kraken X60
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  // We define 3 constants so that we can easily adjust our controller gain constants
  final double kP = 20.0;
  final double kI = 0.0;
  final double kD = 0.001;

  // We define two constants for the commanded positions; this is in units of shaft rotations
  final double kPosition1 = 0.8;
  final double kPosition2 = 0.0;

  // We define two constants for the commanded velocity; this is in units of shaft rotations per minute
  final double kVelocity1 = 10.0;
  final double kVelocity2 = 0.0;

  // These variables store our feedback data
  double feedbackKrakenX60_Position;
  double feedbackKrakenX60_Velocity;

  // We define a PID controller that we utilizes the gain constants
  PIDController myPIDController_Position = new PIDController(kP, kI, kD);
  PIDController myPIDController_Velocity = new PIDController(kP, kI, kD);

  // We define a TalonFX motor controller that can be used to move the Kraken X60 and generate position feedback data
  final TalonFX myKrakenX60 = new TalonFX(20);

  // The Configuration object is something the KrakenX60 requires to be configured
  final TalonFXConfiguration myKrakenX60Config = new TalonFXConfiguration();

  // These VoltageOut object is needed by the KrakenX60 to control the motors
  final VoltageOut myVoltage = new VoltageOut(0.0);

  @Override
  public void robotInit() {

    // These modify and then set basic motor control parameters
    myKrakenX60Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    myKrakenX60Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    myKrakenX60.getConfigurator().apply(myKrakenX60Config, 1.0);

    // This provides an initial set point for the controller, which is the starting position or 0.0
    myPIDController_Position.setSetpoint(0.0);
    myPIDController_Velocity.setSetpoint(0.0);
  }

  @Override
  public void robotPeriodic() {

    // This retrieves the rotational position and velocity of the motor shaft in # of rotations and rotations per minute
    feedbackKrakenX60_Position = myKrakenX60.getPosition().getValueAsDouble();
    feedbackKrakenX60_Velocity = myKrakenX60.getVelocity().getValueAsDouble();

    // This method performs the controller calculations using a feedback measurement we give it and outputs a voltage to actually control the motor
    myVoltage.withOutput(myPIDController_Position.calculate(feedbackKrakenX60_Position));

    // The version below is the same as above except it controls using velocity; uncomment to use it instead of the above
    myVoltage.withOutput(myPIDController_Velocity.calculate(feedbackKrakenX60_Velocity));
  }

  @Override
  public void autonomousInit() {

    // This will set the motor setpoint to position/velocity 2 whenever you switch to Autonomous; but doesn't actually command it move
    myPIDController_Position.setSetpoint(kPosition2);
    myPIDController_Velocity.setSetpoint(kVelocity2);
  }

  @Override
  public void autonomousPeriodic() {

    // actually commands the motor with desired voltage
    myKrakenX60.setControl(myVoltage);
  }

  @Override
  public void teleopInit() {
    
    // These will set the motor setpoint to position/velocity 1 whenever you switch to TeleOp; but doesn't actually command it move
    myPIDController_Position.setSetpoint(kPosition1);
    myPIDController_Velocity.setSetpoint(kVelocity1);
  }

  @Override
  public void teleopPeriodic() {

    // actually commands the motor with desired voltage
    myKrakenX60.setControl(myVoltage);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}