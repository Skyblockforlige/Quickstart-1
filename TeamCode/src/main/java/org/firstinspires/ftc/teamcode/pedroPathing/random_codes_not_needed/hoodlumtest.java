package org.firstinspires.ftc.teamcode.pedroPathing.random_codes_not_needed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
@Disabled
@Configurable
public class hoodlumtest extends LinearOpMode {
    public static Servo hood;
    public static double pos=0.4;//0-0.4 ONLY STRICTLY
    //bottom pos 0.61
    //top pos 0.targ
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
        hood=hardwareMap.servo.get("hood");
        hood.setPosition(0);
        waitForStart();
        while(opModeIsActive()){
            hood.setPosition(pos);
            telemetry.addData("hood pos", hood.getPosition());
            telemetry.update();
        }
    }
}
