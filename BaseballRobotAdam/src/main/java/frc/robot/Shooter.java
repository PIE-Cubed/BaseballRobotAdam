// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Shooter {
    private final int CURRENT_LIMIT_AMPS = 80;

    private final int LEFT_FLYWHEEL_CAN_ID = 11;
    private final int RIGHT_FLYWHEEL_CAN_ID = 12;

    private final double MAX_WHEEL_SPEED = 1;

    private SparkFlex leftFlywheelMotor;
    private SparkFlex rightFlywheelMotor;
    private SparkBaseConfig leftFlywheelConfig;
    private SparkBaseConfig rightFlywheelConfig;
    private DoubleSolenoid solenoid;

    public Shooter() {
        leftFlywheelMotor = new SparkFlex(LEFT_FLYWHEEL_CAN_ID, MotorType.kBrushless);
        rightFlywheelMotor = new SparkFlex(RIGHT_FLYWHEEL_CAN_ID, MotorType.kBrushless);

        leftFlywheelConfig = new SparkFlexConfig();
        leftFlywheelConfig.idleMode(IdleMode.kBrake);
        leftFlywheelConfig.inverted(false);
        leftFlywheelConfig.smartCurrentLimit(CURRENT_LIMIT_AMPS);

        rightFlywheelConfig = new SparkFlexConfig();
        rightFlywheelConfig.idleMode(IdleMode.kBrake);
        rightFlywheelConfig.inverted(true);
        rightFlywheelConfig.smartCurrentLimit(CURRENT_LIMIT_AMPS);

        leftFlywheelMotor.configure(leftFlywheelConfig, 
                                    ResetMode.kNoResetSafeParameters, 
                                    PersistMode.kPersistParameters);

        rightFlywheelMotor.configure(rightFlywheelConfig, 
                                    ResetMode.kNoResetSafeParameters, 
                                    PersistMode.kPersistParameters);

        stopWheels();

        solenoid = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 1, 0);
        reverseShooter();
    }

    public void extendShooter() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void reverseShooter() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void spinUpWheels() {
        leftFlywheelMotor.set(MAX_WHEEL_SPEED);
        rightFlywheelMotor.set(MAX_WHEEL_SPEED);
    }

    public void stopWheels() {
        leftFlywheelMotor.set(0);
        rightFlywheelMotor.set(0);
    }
}