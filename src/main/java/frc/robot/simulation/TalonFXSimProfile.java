package frc.robot.simulation;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.simulation.PhysicsSim.SimProfile;

/**
 * Holds information about a simulated TalonFX.
 */
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
        this._motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, rotorInertia, 1.0), gearbox);
        this._talonFXSimState = talonFX.getSimState();
    }

    public void run() {
        /// DEVICE SPEED SIMULATION
        _motorSim.setInputVoltage(_talonFXSimState.getMotorVoltage());

        _motorSim.update(getPeriod());

        /// SET SIM PHYSICS INPUTS
        final double pivot_position_rot = _motorSim.getAngularPositionRotations();
        final double pivot_velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec());

        // pivot
        _talonFXSimState.setRawRotorPosition(pivot_position_rot);
        _talonFXSimState.setRotorVelocity(pivot_velocity_rps);

        _talonFXSimState.setSupplyVoltage(12 - _talonFXSimState.getSupplyCurrent() * kMotorResistance);

    }

}