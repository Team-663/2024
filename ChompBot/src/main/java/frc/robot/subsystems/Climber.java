// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
   // two victors:
   private final WPI_VictorSPX m_climberMotorL = new WPI_VictorSPX(ClimberConstants.CANID_CLIMBER_VICTOR_L);
   private final WPI_VictorSPX m_climberMotorR = new WPI_VictorSPX(ClimberConstants.CANID_CLIMBER_VICTOR_R);

   // two SPIKE relay
   private final Relay m_climberRatchetL = new Relay(ClimberConstants.CHANNEL_CLIMBER_RELAY_L);
   private final Relay m_climberRatchetR = new Relay(ClimberConstants.CHANNEL_CLIMBER_RELAY_R);

   // two digitalInputs
   private final DigitalInput m_climberLimitSwitchL = new DigitalInput(ClimberConstants.CHANNEL_CLIMBER_DIGITALINPUT_L);
   private final DigitalInput m_climberLimitSwitchR = new DigitalInput(ClimberConstants.CHANNEL_CLIMBER_DIGITALINPUT_R);

   private boolean m_isClimberLockedL = true;
   private boolean m_isClimberLockedR = true;

   /** Creates a new Climber. */
   public Climber() 
   {
      m_climberRatchetL.setDirection(Relay.Direction.kForward);
      m_climberRatchetR.setDirection(Relay.Direction.kForward);

      m_climberMotorL.configFactoryDefault();
      m_climberMotorR.configFactoryDefault();

      m_climberMotorL.setNeutralMode(NeutralMode.Brake);
      m_climberMotorR.setNeutralMode(NeutralMode.Brake);

      m_climberMotorR.setInverted(true);

   }

   public Command climberUnlockCommand()
   {
      return run(()-> unlockBothClimbers());
   }

   public Command moveClimberUpCommand(DoubleSupplier speed, boolean r)
   {
      return run(() -> moveClimber(speed.getAsDouble(), r));
   }

   public Command moveClimberDownCommand(boolean r)
   {
      return run(() -> moveClimber(-0.25, r));
   }

   public Command climberIdleCommand()
   {
      return run(()-> climberIdle());
   }

   public void moveClimber(double speed, boolean r) {

      double lSpeed = 0.0;
      double rSpeed = 0.0;
      // Move climber UP
      if (speed > 0.0)
      {
         if (r) 
         {
            if (m_isClimberLockedR)
               rSpeed = 0.0;
               //m_climberMotorR.set(0.0);
            else
               rSpeed = speed;
               //m_climberMotorR.set(speed);
         } 
         else 
         {
            if (m_isClimberLockedL)
               lSpeed = 0.0;
               //m_climberMotorL.set(0.0);
            else
               lSpeed = speed;
               //m_climberMotorL.set(speed);
         }
      }
      // Move climber DOWN
      else if (speed < 0.0) {
         if (r) {
            if (isClimberDown(r))
               rSpeed = 0.0;
               //m_climberMotorR.set(0.0);
            else
               rSpeed = speed;
               //m_climberMotorR.set(speed);
         } else {
            if (isClimberDown(r))
               lSpeed = 0.0;
               //m_climberMotorL.set(0.0);
            else
               lSpeed = speed;
               //m_climberMotorL.set(speed);
         }
      }

      SmartDashboard.putNumber("climbRPut", rSpeed);
      SmartDashboard.putNumber("climbLPut", lSpeed);

      if (r)
         m_climberMotorR.set(rSpeed);
      else
         m_climberMotorL.set(lSpeed);
      // do something
   }

   private void climberLock(boolean r)
   {
      SmartDashboard.putString("Climber Status", "Locked");
      if (r) {
         // lock right climber
         m_climberRatchetR.set(Relay.Value.kOff);
         m_isClimberLockedR = true;
      } else {
         // lock left climber
         m_climberRatchetL.set(Relay.Value.kOff);
         m_isClimberLockedL = true;
      }
   }

   private void climberUnlock(boolean r) {
      SmartDashboard.putString("Climber Status", "Unlocked");
      if (r) {
         // lock right climber
         m_climberRatchetR.set(Relay.Value.kForward);
         m_isClimberLockedR = false;
      } else {
         // lock left climber
         m_climberRatchetL.set(Relay.Value.kForward);
         m_isClimberLockedL = false;
      }
   }

   public void unlockBothClimbers()
   {
      climberUnlock(true);
      climberUnlock(false);
   }

   private boolean isClimberLocked(boolean r) {
      if (r)
         return m_isClimberLockedR;
      else
         return m_isClimberLockedL;
   }

   private boolean isClimberDown(boolean r) {

      if (r)
         return !m_climberLimitSwitchR.get();
      else
         return !m_climberLimitSwitchL.get();

   }

   // Climbers speed to zero and lock both
   private void climberIdle()
   {
      m_climberMotorL.set(0.0);
      m_climberMotorR.set(0.0);
      climberLock(true);
      climberLock(false);
   }

   @Override
   public void periodic() {
      if (m_isClimberLockedL)
         SmartDashboard.putString("ClimberL Ratchet", "Locked");
      else
         SmartDashboard.putString("ClimberL Ratchet", "Unlocked");

      if (m_isClimberLockedR)
         SmartDashboard.putString("ClimberR Ratchet", "Locked");
      else
         SmartDashboard.putString("ClimberR Ratchet", "Unlocked");


      SmartDashboard.putNumber("ClimbLActual", m_climberMotorL.get());
      SmartDashboard.putNumber("ClimbRActual", m_climberMotorR.get());
      // This method will be called once per scheduler run
   }
}
