package frc.robot.subsystems;

public abstract class Subsystem {
    // This is a super awesome way to put data from Subsystems to SmartDashboard
    public abstract void outputTelemetry();
    public abstract void init();
    public abstract void doRun();
}