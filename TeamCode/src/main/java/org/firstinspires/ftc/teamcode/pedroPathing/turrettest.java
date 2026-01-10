package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@Config
@TeleOp
public class turrettest extends LinearOpMode {
    Timer goonTimer;
    DcMotorEx lf;
    ControlSystem cs1;
    CRServo turretL;
    public static double p=0.00035,i=0.0000000005,d=0.0000000002;
    public static double gearratio = 50.0/9.0;
    public static double tickstodegree = (50.0/9.0) *(8192.0/360.0);
    public static double ticksperdegree = 126.42;
    public static int target = 0;


    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
        lf=hardwareMap.get(DcMotorEx.class,"lf");
        turretL=hardwareMap.crservo.get("turretL");
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            KineticState current2 = new KineticState(lf.getCurrentPosition(),lf.getVelocity());
            cs1 = ControlSystem.builder()
                    .posPid(p,i,d)
                    .build();
            cs1.setGoal(new KineticState(target));
            if(gamepad2.dpad_right){
                target+=5*ticksperdegree;
                sleep(300);
            }
            if(gamepad2.dpad_left){
                target-=5*ticksperdegree;
                sleep(300);
            }
            if(Math.abs(gamepad2.left_stick_x)==0) {
                turretL.setPower(-cs1.calculate(current2));
            } else{
                turretL.setPower(-gamepad2.left_stick_x);
                target=lf.getCurrentPosition();

            }
            telemetry.addData("Turret Power", -cs1.calculate(current2));

            telemetry.addData("Turret Current Position: " , lf.getCurrentPosition());
            telemetry.addData("Turret Target Position: ", target);
            telemetry.update();
        }
    }

}
