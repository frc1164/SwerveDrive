// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Class to define setpoints for arm and clasp.*/
public class Setpoint {
    public double X_Cone;
    public double Y_Cone;
    public ClaspState claspCone;
    public double X_Cube;
    public double Y_Cube;
    public ClaspState claspCube;
    public ArmState state;
   
    public Setpoint(double X_Cone, double Y_Cone, ClaspState claspCone, double X_Cube, double Y_Cube, ClaspState claspCube, ArmState state) {
        this.X_Cone = X_Cone;
        this.Y_Cone = Y_Cone;
        this.claspCone = claspCone;
        this.X_Cube = X_Cube;
        this.Y_Cube = Y_Cube;
        this.claspCube = claspCube;
        this.state = state;
    }

    // Enumerated type ArmState
    public enum ArmState{
        STOWED, FLOOR, MID_NODE, MID_NODE_PLACED, TOP_NODE, TOP_NODE_PLACED, SUBSTATION, INTERMEDIATE, OTHER
    }

    // Enumerated type ClaspState
    public enum ClaspState{
        IN, OUT
    }
}