package frc.robot.dashboard;

public class PIDList {
    public final double P;
    public final double I;
    public final double D;
    public final double F;
    public PIDList(double P, double I, double D, double F) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = F;
    }

}