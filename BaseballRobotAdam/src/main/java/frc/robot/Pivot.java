// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Pivot {
    private final int CURRENT_LIMIT_AMPS = 80;

    private final int PIVOT_CAN_ID = 10;

    private final double MAX_PIVOT_POWER = 0.075; // 0.075

    private final double PIVOT_P = 4; // 1.6
    private final double PIVOT_I = 2; // 130
    private final double PIVOT_D = 0;   // ^^^ with 0.003 added to the target value
    private final double MIN_I_RANGE = -0.05;
    private final double MAX_I_RANGE = 0.05;
    private final double I_ZONE = 0.02;
    private final double PID_TOLERANCE = 0.002;
    
    private SparkFlex pivotMotor;
    private SparkBaseConfig pivotMotorConfig;
    private PIDController pivotPID;
    private DutyCycleEncoder absEncoder;

    public Pivot() {
        pivotMotor = new SparkFlex(PIVOT_CAN_ID, MotorType.kBrushless);

        pivotMotorConfig = new SparkFlexConfig();
        pivotMotorConfig.inverted(false);
        pivotMotorConfig.smartCurrentLimit(CURRENT_LIMIT_AMPS);
        pivotMotor.configure(
            pivotMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );

        pivotPID = new PIDController(PIVOT_P, PIVOT_I, PIVOT_D);
        pivotPID.setTolerance(PID_TOLERANCE);
        pivotPID.setIZone(I_ZONE);
        pivotPID.setIntegratorRange(MIN_I_RANGE, MAX_I_RANGE);

        absEncoder = new DutyCycleEncoder(0);
    }

    public void pivotTo(double targetValue) {
        double power = pivotPID.calculate((absEncoder.get()), MathUtil.clamp(targetValue, 0.73, 0.81));

        System.out.println("Current encoder value: " + absEncoder.get());
        System.out.println("Current PID output: " + power);

        pivotMotor.set(MathUtil.clamp(power, (MAX_PIVOT_POWER * -1), MAX_PIVOT_POWER));
    }

    // + Values lower the shooter and - values raise the shooter
    public void setMotor(double value) {
        pivotMotor.set(value);
    }

    /*
     * minimum angle of 0.73
     * maximum angle of 0.81
     */
    public void printEncoderValue() {
        System.out.println("Current encoder value: " + absEncoder.get());
    }
}
