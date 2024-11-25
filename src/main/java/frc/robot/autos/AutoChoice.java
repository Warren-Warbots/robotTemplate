// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

public enum AutoChoice {
    TESTS("Tests"),
    DO_NOTHING("DoNothing");

    public final String pathName;

    private AutoChoice(String pathName) {
        this.pathName = pathName;
    }
}