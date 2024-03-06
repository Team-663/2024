// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class shootNote extends Command {
   Shooter m_shooter;
   BooleanSupplier m_shoot;
   double m_speed;
   /** Creates a new shootNote. */
   public shootNote(Shooter shooter, double speed, BooleanSupplier shoot)
   {
      m_shooter = shooter;
      m_shoot = shoot;
      m_speed = speed;
      addRequirements(m_shooter);
      // Use addRequirements() here to declare subsystem dependencies.
   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize()
   {
      m_shooter.shooterEnablePID(m_speed);
   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
      double intakeSpeed = 0.0;

      if (m_shoot.getAsBoolean())
      {
         intakeSpeed = Constants.ArmConstants.INTAKE_MOTOR_SHOOT_SPEED;
         SmartDashboard.putBoolean("ShootNow", true);
      }
      else
      {
         SmartDashboard.putBoolean("ShootNow", false);
      }
      SmartDashboard.putNumber("ShootNote Intk", intakeSpeed);
      m_shooter.intakeMotorSet(intakeSpeed);

   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
      m_shooter.intakeMotorSet(0);
      m_shooter.shooterIdle();
   }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
      return false;
   }
}
