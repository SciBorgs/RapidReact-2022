package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.BangBangController;

public class SafeBangBangController extends BangBangController {

    public SafeBangBangController(CANSparkMax... motors) {
        for (CANSparkMax i : motors) {
            if (i.getIdleMode() != IdleMode.kCoast) {
                throw new IllegalStateException("Motors must be in coast mode (SparkMax CAN ID " + i.getDeviceId() + ")");
            }
        }
    }

}
