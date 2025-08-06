// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;

public class Pivot {
    private final int CURRENT_LIMIT_AMPS = 40;

    private final int PIVOT_CAN_ID = 10;

    private final double MAX_PIVOT_POWER = 0.3;

    private final double PIVOT_P = 0.005;
    private final double PIVOT_I = 0;
    private final double PIVOT_D = 0;

    private boolean pivotFirstTime = true;

    private SparkFlex pivotMotor;
    private SparkBaseConfig pivotMotorConfig;
    private SparkAbsoluteEncoder pivotEncoder;
    private AbsoluteEncoderConfig pivotEncoderConfig;
    private PIDController pivotPID;

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
        pivotPID.setTolerance(0.15); // Degrees

        pivotEncoder = pivotMotor.getAbsoluteEncoder();
    }

    public int pivotTo(double targetAngle) {
        if (pivotFirstTime == true) {
            pivotPID.reset();
            pivotFirstTime = false;
        }

        double power = pivotPID.calculate(pivotEncoder.getPosition(), targetAngle);

        if (pivotPID.atSetpoint()) {
            pivotFirstTime = true;
            return Robot.DONE;
        }

        return Robot.CONT;
    }
}
