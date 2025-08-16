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

    private final double PIVOT_P = 1.6; // 1.6
    private final double PIVOT_I = 2; // 130
    private final double PIVOT_D = 0;   // ^^^ with 0.003 added to the target value
    private final double MIN_I_RANGE = 0.0025;
    private final double MAX_I_RANGE = 0.01;
    private final double I_ZONE = 0.01;
    private final double PID_TOLERANCE = 0.002;

    private boolean pivotFirstTime = true;

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

    public int pivotTo(double targetValue) {
        if (pivotFirstTime == true) {
            pivotPID.reset();
            pivotFirstTime = false;
        }

        targetValue += 0.003;

        double power = pivotPID.calculate((absEncoder.get()), targetValue);

        pivotMotor.set(MathUtil.clamp(power, (MAX_PIVOT_POWER * -1), MAX_PIVOT_POWER));

        if (pivotPID.atSetpoint()) {
            pivotFirstTime = true;
            return Robot.DONE;
        }

        return Robot.CONT;
    }

    public void printPidOutput(double pidTarget) {
        pidTarget += 0.003;

        System.out.println("Current encoder value: " + absEncoder.get());
        System.out.println("Current PID output: " + pivotPID.calculate((absEncoder.get()), pidTarget));
        System.out.println("Accumulated integrator error: " + pivotPID.getAccumulatedError());
    }

    // + Values lower the shooter and - values raise the shooter
    public void setMotor(double value) {
        pivotMotor.set(value);
    }
}
