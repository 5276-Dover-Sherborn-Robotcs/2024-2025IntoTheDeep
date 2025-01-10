package org.firstinspires.ftc.teamcode.PedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {

        TwoWheelConstants.forwardTicksToInches = (4.8*2*Math.PI/2.54)/2000;
        TwoWheelConstants.strafeTicksToInches = (4.8*2*Math.PI/2.54)/2000;
        TwoWheelConstants.forwardY = 4.4/25.4;
        TwoWheelConstants.strafeX = -0.5;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "fl";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "fr";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    }
}




