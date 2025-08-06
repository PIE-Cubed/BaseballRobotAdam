// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Controller {
    private final double TRIGGER_DEADZONE = 0.8;

    private XboxController controller;

    public Controller() {
        controller = new XboxController(0);
    }

    public boolean extendShooter() {
        return (controller.getRightTriggerAxis() >= TRIGGER_DEADZONE);
    }

    public boolean spinWheelButton() {
        return controller.getAButton();
    }

    public boolean pivotUpDPad() {
        if (controller.getPOV() == -1) {
            return false;
        }

        return (controller.getPOV() >= 315) && (controller.getPOV() <= 45);
    }

    public boolean pivotDownDPad() {
        if (controller.getPOV() == -1) {
            return false;
        }

        return (controller.getPOV() >= 135) && (controller.getPOV() <= 225);
    }
}
