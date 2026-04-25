package org.firstinspires.ftc.teamcode.pedroPathing.miscelenous_important_codes;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@TeleOp
@Configurable
@Config
public class velpid2 extends LinearOpMode {
    public static double p=0.019,i=0,d=3;
    public static double v=0.4,a=0.1,s=0.3;
    private ControlSystem cs;
    private DcMotorEx flywheel;
    private DcMotorEx flywheel2;
    private DcMotorEx intake;
    private Timer currentTimer;
    public static double targetTicksPerSecond=2200;

    @Override
    public void runOpMode(){
        currentTimer = new Timer();
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        rconstants.initHardware(hardwareMap);

        flywheel = hardwareMap.get(DcMotorEx.class,"shooter");
        flywheel2=hardwareMap.get(DcMotorEx.class,"shooter2");
        intake=rconstants.intake;
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while(opModeIsActive()){
            if(intake.getCurrent(CurrentUnit.AMPS)<6.5) {
                intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                currentTimer.resetTimer();
            } else if(currentTimer.getElapsedTimeSeconds()>1.2){
                intake.setPower(-1);
            } else{
                intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }
            cs =  ControlSystem.builder()
                    .velPid(p, i, d)
                    .basicFF(v,a,s)
                    .build();
            cs.setGoal(new KineticState(0,targetTicksPerSecond));
            KineticState current = new KineticState(flywheel2.getCurrentPosition(), Math.abs(flywheel2.getVelocity()));
            double output2 = cs.calculate(current);
            flywheel.setPower(output2);
            flywheel2.setPower(output2);
            telemetry.addData("motor current speed",Math.abs(flywheel2.getVelocity()));
            telemetry.addData("target",targetTicksPerSecond);
            telemetry.addData("output",output2);
            telemetry.update();
        }

    }

}
