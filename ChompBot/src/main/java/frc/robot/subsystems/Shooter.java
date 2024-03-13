// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.lang.Math;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Shooter extends SubsystemBase {
   /** Creates a new Arm. */
   private final CANSparkMax m_shooterMotorLeader = new CANSparkMax(Constants.ArmConstants.CANID_SHOOTER_SPARKMAX_1,
         MotorType.kBrushless);
   
   // Take out follower motor for now
   //private final CANSparkMax m_shooterMotorFollower = new CANSparkMax(Constants.ArmConstants.CANID_SHOOTER_SPARKMAX_2,
   //      MotorType.kBrushless);
   private final WPI_VictorSPX m_intakeMotor = new WPI_VictorSPX(Constants.ArmConstants.CANID_INTAKE_VICTOR);
   
   private final WPI_TalonSRX m_armMotor = new WPI_TalonSRX(ArmConstants.CANID_ARM_TALON_1);
   private final WPI_VictorSPX m_armMotor2 = new WPI_VictorSPX(ArmConstants.CANID_ARM_VICTOR_2);

   private final SparkPIDController m_shooterPID = m_shooterMotorLeader.getPIDController();
   private final RelativeEncoder m_shooterEnc = m_shooterMotorLeader.getEncoder();   
   
   private final LaserCan m_laser = new LaserCan(21);
   private boolean m_laserValidMeasurement = false;
   private double m_shooterSetpoint = 0.0;
   private boolean m_isShooterAtSetpoint = false;
   private double m_laserDistInches = 0.0;
   private double m_armSetpoint = 0.0;

   private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
   private GenericEntry sbtLaserEntry = tab.add("Laser Distance", 0).getEntry();
   private GenericEntry sbtShooterSpeed = tab.add("Shooter RPM", 0).getEntry();
   private GenericEntry sbtShooterMotorOutput = tab.add("Shooter Output", 0).getEntry();
   private GenericEntry sbtShooterCurrent = tab.add("Shooter Amps", 0).getEntry();
   
   private GenericEntry sbtShooterSetpoint = tab.add("Shooter Setpt", 0).getEntry();
   private GenericEntry sbtNoteInsideIntake = tab.add("Is Note Inside", 0).getEntry();
   private GenericEntry sbtArmSpeed = tab.add("Arm Speed", 0).getEntry();
   private GenericEntry sbtArmPosition = tab.add("Arm Position", 0).getEntry();
   private GenericEntry stbArmXboxInput = tab.add("Arm Xbox Input", 0).getEntry();
   private GenericEntry stbArmXboxOutput = tab.add("Arm Xbox Scaled", 0).getEntry();
   private GenericEntry stbArmClosedLoopEnabled = tab.add("ArmClosedLoopEnabled", 0).getEntry();
   private GenericEntry stbArmSetpoint = tab.add("Arm Setpoint", 0).getEntry();
   private GenericEntry stbArmError = tab.add("Arm Error", 0).getEntry();

   private GenericEntry stbShootOpenVal = tab.add("Shoot Cmd Value", 0).getEntry();
   public Shooter() {
      m_shooterMotorLeader.restoreFactoryDefaults();
      //m_shooterMotorFollower.restoreFactoryDefaults();
      m_intakeMotor.setNeutralMode(NeutralMode.Brake);
      m_intakeMotor.setInverted(false);
      m_shooterMotorLeader.setIdleMode(IdleMode.kCoast);
      m_shooterMotorLeader.setInverted(true);
      
      //m_shooterMotorFollower.setIdleMode(IdleMode.kCoast);
      //m_shooterMotorFollower.follow(m_shooterMotorLeader, true);
      initLaserCAN();
      initShooterMotorSettings();
      initArmMotorSettings();

   }

   public boolean CheckIfNoteInIntake()
   {
      return (m_laserDistInches < Constants.ArmConstants.LASER_BEAM_BREAK_THRESHOLD);    
   }

   public boolean IsShooterUpToSpeed()
   {
      return m_isShooterAtSetpoint;
   }

   public boolean IsShooterSpinningTooFastForIntake()
   {
      return (Math.abs(m_shooterEnc.getVelocity()) > ArmConstants.SHOOTER_TOO_FAST_FOR_INTAKE_SPEED ? true : false);
   }

   private void armByXbox(double armValue)
   {
      double armOutput = 0.0;
      stbArmXboxInput.setDouble(armValue);
      m_armSetpoint = m_armMotor.getSelectedSensorPosition();
      if (armValue < 0.0)
      {
         armOutput = Math.abs(ArmConstants.ARM_PEAK_DOWN_POWER) * armValue;
         stbArmClosedLoopEnabled.setDouble(0.0);
         m_armMotor.set(armOutput);
      }
      else if (armValue > 0.0)
      {
         armOutput = Math.abs(ArmConstants.ARM_PEAK_UP_POWER) * armValue;
         stbArmClosedLoopEnabled.setDouble(0.0);
         m_armMotor.set(armOutput);
      }
     
      stbArmXboxOutput.setDouble(armOutput);
   }

   public void setArmAtCurrentPosition()
   {
      setArmPosition(m_armMotor.getSelectedSensorPosition());
   }

   public void setArmPosition(double position)
   {
      


      m_armSetpoint = position;
      double currentPos = m_armMotor.getSelectedSensorPosition();
      if ((position < currentPos) && (position == ArmConstants.ARM_CLOSE_SHOT_SETPOINT))
         position += 20;

      double degrees = (currentPos - ArmConstants.ARM_DOWN_ENCODER_VALUE-50) / (4096 / 360.0);
      double radians = java.lang.Math.toRadians(degrees);
      double cosineScalar = java.lang.Math.cos(radians);
      SmartDashboard.putNumber("cosine scalar", cosineScalar);

      m_armMotor.set(ControlMode.Position
      , position
      , DemandType.ArbitraryFeedForward
      , ArmConstants.ARM_ARBITRARY_FF_MAX*cosineScalar);
   }


   private void shooterByXbox(double intake, double shooter)
   {
      //m_intakeMotor.set(intake);
      //m_shooterMotorLeader.set(shooter);
   }

   public void intakeNote()
   {
      // run intake motors at intake speed
      // if beam is broken, stop
      if (!CheckIfNoteInIntake())
         intakeMotorSet(Constants.ArmConstants.INTAKE_MOTOR_SPEED);
      else
         intakeIdle();
   }

   public void intakeNote(double speed)
   {
      // run intake motors at intake speed
      // if beam is broken, stop
      if (!CheckIfNoteInIntake())
         intakeMotorSet(speed);
      else
         intakeIdle();
   }


   private void intakeIdle()
   {
      m_intakeMotor.stopMotor();
      shooterIdle();
      //m_armMotor.stopMotor();
      stbArmClosedLoopEnabled.setDouble(1.0);
      //setArmAtCurrentPosition();
      //m_armMotor.set(ControlMode.Position, m_armSetpoint);
    

   }

   public void intakeBackwardSlow(boolean enable)
   {
      if (enable)
      {
         m_intakeMotor.set(Constants.ArmConstants.INTAKE_BACK_SLOW_SPEED);
      }
      else
      {
         m_intakeMotor.set(0.0);
      }
   }

   public void shooterIdle()
   {
      m_shooterMotorLeader.set(0.0);
      //m_shooterPID.setReference(0.0, ControlType.kVelocity);
   }

   public void intakeMotorSet(double val)
   {
      m_intakeMotor.set(val);

   }

   public void intakeMotorAndShooterSet(double intakeSpeed, double shooterSpeed)
   {
      m_intakeMotor.set(intakeSpeed);
      m_shooterMotorLeader.set(shooterSpeed);
   }

   public void shooterEnablePID(double setpoint)
   {
      m_shooterSetpoint = setpoint;
      m_shooterPID.setReference(setpoint, ControlType.kVelocity);
   }

   private void shootNote(double targetRPM)
   {
      // 1) Set shooter wheel to target RPM
      // 2) IF shooter at target RPM, shoot the note

      // TODO: if beam is broken, drive the intake backwards slowly-ish
      // Dont do anything else until beam is no longer broken
      
      m_shooterSetpoint = targetRPM;
      shooterEnablePID(targetRPM);
      if (m_isShooterAtSetpoint)
         intakeMotorSet(Constants.ArmConstants.INTAKE_MOTOR_SHOOT_SPEED);
   }

   private void shootNoteOpenLoop(double speed)
   {
      stbShootOpenVal.setDouble(speed);
      SmartDashboard.putNumber("Shooter OpenLoop Spd", speed);
      m_shooterMotorLeader.set(speed);
      intakeMotorSet(Constants.ArmConstants.INTAKE_MOTOR_SHOOT_SPEED);
   }

   public Command armByXboxCommand(DoubleSupplier armVal)
   {
      //return run(() -> armByXbox(armVal*Constants.ArmConstants.ARM_MAX_OUTPUT_POWER)).withName("armByXbox");
      return run(() -> armByXbox(armVal.getAsDouble())
      ).withName("ArmByXbox2"); 
   }

   public Command setArmPositionCmd(double pos)
   {
      return new InstantCommand(() -> setArmPosition(pos)).withName("SetArmPosCmd");
   }

   public Command shooterByXboxCommand(DoubleSupplier intake, DoubleSupplier shooter) {
      return run(() -> shooterByXbox(intake.getAsDouble(), shooter.getAsDouble()))
            .withName("shooterByXbox");
   }

   public Command intakeNoteCommand()
   {
      return startEnd(() -> intakeNote()
                     ,() -> intakeIdleCommand()
      ).withName("intakeNote");
   }

   public Command shootNoteCommand(double rpm)
   {
      return run(() -> shootNote(rpm)).withName("shootNote");
   }

   public Command shooteNotePIDTest(double rpm)
   {
      return startEnd(
         () -> shooterEnablePID(rpm)
         ,() -> shooterIdle()
         ).withName("shootPidTest");
   }

   public Command shootNoteCommandOpenLoop(double speed)
   {
      return startEnd(() -> shootNoteOpenLoop(speed)
                  , ()-> shooterIdle()
      ).withName("shootOpenLoop");
   }

   public Command shootNoteCommandOpenLoop2(DoubleSupplier speed)
   {

      return run(() -> shootNoteOpenLoop(speed.getAsDouble())
      ).withName("shootOpenLoop2");
   }

   public Command intakeManualSpeed(DoubleSupplier speed, DoubleSupplier shooterSpeed)
   {
      return run(() -> intakeMotorAndShooterSet(speed.getAsDouble(), shooterSpeed.getAsDouble())).withName("intakeManualSpd");
   }

   public Command intakeIdleCommand()
   {
      return run(() -> intakeIdle()).withName("idle");
   }


   private void UpdateLaserDistance()
   {
      LaserCan.Measurement measurement = m_laser.getMeasurement();
      if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) 
      {
         m_laserValidMeasurement = true;
         m_laserDistInches = (double)measurement.distance_mm / 25.4;
      }
      else
      {
         m_laserValidMeasurement = false;
      }
    
   }

   public double GetLaserDistance()
   {
      return m_laserDistInches;
   }

   private void initLaserCAN()
   {
      // TODO: set defaults here if necessary
      /*
      try
      {
        // m_laser.setRangingMode(LaserCan.RangingMode.SHORT);
         //m_laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
         //m_laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
      }
      catch (ConfigurationFailedException e)
      {
         System.out.println("LaserCAN configuration failed! " + e);
      }
      */
   }
   private void UpdateShuffleboardValues()
   {
      sbtLaserEntry.setDouble(m_laserDistInches);
      sbtShooterSpeed.setDouble(m_shooterEnc.getVelocity());
      sbtNoteInsideIntake.setBoolean(CheckIfNoteInIntake());
      sbtShooterMotorOutput.setDouble(m_shooterMotorLeader.getAppliedOutput());
      sbtShooterCurrent.setDouble(m_shooterMotorLeader.getOutputCurrent());
      sbtShooterSetpoint.setDouble(m_shooterSetpoint);
      sbtArmSpeed.setDouble((m_armMotor.get()));
      sbtArmPosition.setDouble(m_armMotor.getSelectedSensorPosition());
      stbArmSetpoint.setDouble(m_armSetpoint);
      stbArmError.setDouble(m_armMotor.getClosedLoopError());
      
   }

   private void initShooterMotorSettings()
   {
      m_shooterMotorLeader.setSmartCurrentLimit(40);
      //m_shooterMotorFollower.setSmartCurrentLimit(40);
      m_shooterMotorLeader.setClosedLoopRampRate(0.5);
      //m_shooterMotorFollower.setClosedLoopRampRate(0.5);
      m_shooterPID.setP(ArmConstants.SHOOTER_PID_P);
      m_shooterPID.setI(ArmConstants.SHOOTER_PID_I);
      m_shooterPID.setD(ArmConstants.SHOOTER_PID_D);
      m_shooterPID.setFF(ArmConstants.SHOOTER_PID_F);

      m_shooterPID.setOutputRange(0.0, 1.0);

   }
   private void initArmMotorSettings()
   {
      m_armMotor.configFactoryDefault();
      m_armMotor2.configFactoryDefault();
      m_armMotor.setInverted(true);
      m_armMotor2.setInverted(true);

      m_armMotor.setNeutralMode(NeutralMode.Brake);
      m_armMotor2.setNeutralMode(NeutralMode.Brake);
      m_armMotor2.follow(m_armMotor);

      m_armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
      m_armMotor.setSensorPhase(true);
      m_armSetpoint = m_armMotor.getSelectedSensorPosition();
      m_armMotor.configForwardSoftLimitEnable(true);
      m_armMotor.configForwardSoftLimitThreshold(ArmConstants.ARM_SOFT_LIMIT_UPPER);

      m_armMotor.configReverseSoftLimitEnable(true);
      m_armMotor.configReverseSoftLimitThreshold(ArmConstants.ARM_SOFT_LIMIT_LOWER);

      //m_armMotor.configPeakOutputForward(ArmConstants.ARM_PEAK_UP_POWER);
      m_armMotor.configPeakOutputReverse(ArmConstants.ARM_PEAK_DOWN_POWER);



      //m_armMotor.SetSensorPhase(true);
      //m_armMotor.configAllowableClosedloopError(0, ArmConstants.ARM_MAX_PID_ERROR);
      //m_armMotor.configClosedLoopPeakOutput(0, ArmConstants.ARM_MAX_PID_POWER);

      m_armMotor.config_kP(0, ArmConstants.ARM_PID_P);
      m_armMotor.config_kI(0, ArmConstants.ARM_PID_I);
      m_armMotor.config_kD(0, ArmConstants.ARM_PID_D);
      m_armMotor.config_kF(0, ArmConstants.ARM_PID_F);

      //https://v5.docs.ctr-electronics.com/en/stable/ch16_ClosedLoop.html#arbitrary-feed-forward
   }

   @Override
   public void periodic() {
      UpdateLaserDistance();
      UpdateShuffleboardValues();

      if ( (Math.abs(m_shooterEnc.getVelocity() - m_shooterSetpoint) < Constants.ArmConstants.SHOOTER_VELOCITY_RANGE)
            && m_shooterSetpoint > 0 )
         m_isShooterAtSetpoint = true;
      else
         m_isShooterAtSetpoint = false;

      SmartDashboard.putNumber("ArmAbsEnc", m_armMotor.getSelectedSensorPosition());
      SmartDashboard.putBoolean("shooterReady?", m_isShooterAtSetpoint);
      // This method will be called once per scheduler run
   }

}
