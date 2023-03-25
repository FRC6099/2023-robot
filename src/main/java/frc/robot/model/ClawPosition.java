// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.model;

import java.util.Arrays;

/** Add your docs here. */
public enum ClawPosition {
    OPEN,
    CUBE,
    CONE,
    CLOSED;

    public static String[] valuesAsString() {
        return Arrays.asList(values())
                .stream()
                .map(it -> it.name())
                .toArray(String[]::new);
    }
}
