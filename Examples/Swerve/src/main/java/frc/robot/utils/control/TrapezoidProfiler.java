package frc.robot.utils.control;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

// This class simplifies the use of WPILib trapezoidal motion profiles.
public class TrapezoidProfiler {
    private TrapezoidProfile.Constraints constraints;
    private double period;
    private TrapezoidProfile m_profile;
    private TrapezoidProfile.State m_setpoint;
    private TrapezoidProfile.State m_goal;
    public TrapezoidProfiler(TrapezoidProfile.Constraints constraints, double period) {
        this.constraints = constraints;
        this.period = period;
        m_profile = new TrapezoidProfile(constraints);
        m_setpoint = new TrapezoidProfile.State();
        m_goal = new TrapezoidProfile.State();
    }

    public TrapezoidProfiler(TrapezoidProfile.Constraints constraints) {
        this(constraints, 0.02); // Default period of 20ms
    }

    public TrapezoidProfiler() {
        this(new TrapezoidProfile.Constraints(0,0)); // Default constraints with zero velocity and acceleration
    }

    public TrapezoidProfile.State calculate() {
        m_setpoint = m_profile.calculate(period, m_setpoint, m_goal);
        return m_setpoint;
    }

    public TrapezoidProfile.State getSetpoint() {
        return m_setpoint;
    }

    public TrapezoidProfile.State getGoal() {
        return m_goal;
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return constraints;
    }

    public TrapezoidProfiler withConstraints(TrapezoidProfile.Constraints constraints) {
        this.constraints = constraints;
        m_profile = new TrapezoidProfile(constraints);
        return this;
    }

    public TrapezoidProfiler withPeriod(double period) {
        this.period = period;
        return this;
    }

    public void setInital(TrapezoidProfile.State initialState) {
        this.m_setpoint = initialState;
    }

    public void setGoal(TrapezoidProfile.State goal) {
        this.m_goal = goal;
    }

    public void setGoal(double goal) {
        setGoal(new TrapezoidProfile.State(goal, 0.0));
    }
}
