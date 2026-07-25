package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;

public class TalonFxUtils {
  public static StatusCode configureTalon(TalonFX motor, TalonFXConfiguration config) {
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) {
        break;
      }
    }
    if (!status.isOK()) {
      // reportError shows up red in the Driver Station messages, where drivers
      // actually look - println only goes to the console log
      DriverStation.reportError(
          "Could not configure TalonFX with CAN ID " + motor.getDeviceID() + ", error code: " + status.toString(),
          false);
    }
    return status;
  }
}
