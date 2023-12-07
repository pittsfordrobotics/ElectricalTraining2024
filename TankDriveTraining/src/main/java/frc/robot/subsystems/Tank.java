// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.TankDriveXbox;

public class Tank extends SubsystemBase {
  //Create CAN Spark Max objects
  private final CANSparkMax leftMotor1 = new CANSparkMax(Constants.CAN_LEFT_MOTOR_1, MotorType.kBrushless);
  private final CANSparkMax leftMotor2 = new CANSparkMax(Constants.CAN_LEFT_MOTOR_2, MotorType.kBrushless);
  private final CANSparkMax rightMotor1 = new CANSparkMax(Constants.CAN_RIGHT_MOTOR_1, MotorType.kBrushless);
  private final CANSparkMax rightMotor2 = new CANSparkMax(Constants.CAN_RIGHT_MOTOR_2, MotorType.kBrushless);
  //Set up kinematics object, chassis speeds, pid, and encoders
  private ChassisSpeeds targetSpeeds;
  private final DifferentialDriveKinematics kinematics = Constants.kinematics;
  private DifferentialDriveWheelSpeeds wheelSpeeds;
  private SparkMaxPIDController leftPID;
  private SparkMaxPIDController rightPID;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  /** Creates a new Tank Drive with 4 motors and 6 wheels. */
  public Tank() {
    this.setDefaultCommand(new TankDriveXbox(this));
    //Restore factory defaults and set current limits
    leftMotor1.restoreFactoryDefaults();
    leftMotor2.restoreFactoryDefaults();
    rightMotor1.restoreFactoryDefaults();
    rightMotor2.restoreFactoryDefaults();
    leftMotor1.setSmartCurrentLimit(30);
    leftMotor2.setSmartCurrentLimit(30);
    rightMotor1.setSmartCurrentLimit(30);
    rightMotor2.setSmartCurrentLimit(30);
    leftMotor1.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kBrake);

    //Each side of tank has 2 motors. One will follow the other.
    leftMotor2.follow(leftMotor1, true);
    rightMotor2.follow(rightMotor1, true);

    //Encoders and PID control
    leftEncoder = leftMotor1.getEncoder();
    rightEncoder = rightMotor1.getEncoder();
    leftEncoder.setVelocityConversionFactor(Math.PI * Constants.WHEEL_DIAMETER_METERS / Constants.GEAR_RATIO / 60);
    rightEncoder.setVelocityConversionFactor(Math.PI * Constants.WHEEL_DIAMETER_METERS / Constants.GEAR_RATIO / 60);
    leftPID = leftMotor1.getPIDController();
    rightPID = rightMotor1.getPIDController();
    leftPID.setFeedbackDevice(leftEncoder);
    rightPID.setFeedbackDevice(rightEncoder);
    leftPID.setP(Constants.TANK_P);
    leftPID.setI(Constants.TANK_I);
    leftPID.setD(Constants.TANK_D);
    rightPID.setP(Constants.TANK_P);
    rightPID.setI(Constants.TANK_I);
    rightPID.setD(Constants.TANK_D);
  }
  /**
   * Drive the tank drive
   * @param forward how far forward the left stick is pushed
   * @param turn how far the right stick is pushed sideways, left positive
   */
  public void drive(double forward, double turn){
    targetSpeeds = new ChassisSpeeds(forward * Constants.MAX_LINEAR_SPEED_METER_PER_SECOND, 0, turn * Constants.MAX_ANGULAR_SPEED_RAD_PER_SECOND);
    wheelSpeeds = kinematics.toWheelSpeeds(targetSpeeds);
    wheelSpeeds.desaturate(Constants.MAX_WHEEL_SPEED_METER_PER_SECOND);
    leftPID.setReference(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity);
    rightPID.setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity);
  }
  public void stopMotors() {
    leftMotor1.set(0);
    rightMotor1.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
