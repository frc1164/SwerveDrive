// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Class to define setpoints for arm and clasp.*/
public class Setpoint {
    public double shoulderCone;
    public double extensionCone;
    public ClaspState claspCone;
    public double shoulderCube;
    public double extensionCube;
    public ClaspState claspCube;
    public ArmState state;
   
    public Setpoint(double shoulderCone, double extensionCone, ClaspState claspCone, double shoulderCube, double extensionCube, ClaspState claspCube, ArmState state) {
        this.shoulderCone = shoulderCone;
        this.extensionCone = extensionCone;
        this.claspCone = claspCone;
        this.shoulderCube = shoulderCube;
        this.extensionCube = extensionCube;
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
