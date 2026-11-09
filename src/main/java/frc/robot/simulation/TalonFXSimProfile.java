package frc.robot.simulation;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.robot.simulation.PhysicsSim.SimProfile;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DCMotorSim;

/** Holds information about a simulated TalonFX. */
public class TalonFXSimProfile extends SimProfile {
    private static final double kMotorResistance = 0.002; // Assume 2mOhm resistance for voltage drop calculation

    private final DCMotorSim _motorSim;
    private final TalonFXSimState _talonFXSimState;

    /*
     * @param talonFX
     * The TalonFX device
     *
     * @param rotorInertia
     * Rotational Inertia of the mechanism at the rotor
     */
    public TalonFXSimProfile(final TalonFX talonFX, final double rotorInertia) {
        var gearbox = DCMotor.getKrakenX60Foc(1);
        this._motorSim = new DCMotorSim(
                Models.singleJointedArmFromPhysicalConstants(gearbox, rotorInertia, 1.0), gearbox);
        this._talonFXSimState = talonFX.getSimState();
    }

    public void run() {
        /// DEVICE SPEED SIMULATION
        _motorSim.setInputVoltage(_talonFXSimState.getMotorVoltage());

        _motorSim.update(getPeriod());

        /// SET SIM PHYSICS INPUTS
        final double pivot_position_rot = _motorSim.getAngularPosition();
        final double pivot_velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocity());

        // pivot
        _talonFXSimState.setRawRotorPosition(pivot_position_rot);
        _talonFXSimState.setRotorVelocity(pivot_velocity_rps);

        _talonFXSimState.setSupplyVoltage(12 - _talonFXSimState.getSupplyCurrent() * kMotorResistance);
    }
}