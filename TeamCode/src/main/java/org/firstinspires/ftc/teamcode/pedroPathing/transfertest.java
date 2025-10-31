package org.firstinspires.ftc.teamcode.pedroPathing;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@TeleOp
public class transfertest extends LinearOpMode {
    public static double p1=0.019,i1=0,d1=3;
    public static double v1=0.4,a1=0.1,s1=0.3;
    private DcMotorEx spindexer;
    private CRServoImplEx transfer;
    private DcMotorEx flywheel;
    private DcMotorEx rb;
    private ControlSystem cs1;
    private ControlSystem cs;
    public static double p=0.019,i=0,d=3;
    public static double v=0.4,a=0.1,s=0.3;
    private int targetpos;
    public static double targetTicksPerSecond=2200;


    @Override
    public void runOpMode(){

        rb = hardwareMap.get(DcMotorEx.class,"rb");
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel = hardwareMap.get(DcMotorEx.class,"shooter");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        transfer=hardwareMap.get(CRServoImplEx.class, "transfer");
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            cs =  ControlSystem.builder()
                    .velPid(p, i, d)
                    .basicFF(v,a,s)
                    .build();
            cs.setGoal(new KineticState(0,targetTicksPerSecond));
            cs1 =  ControlSystem.builder()
                    .posPid(p1, i1, d1)
                    .basicFF(v1,a1,s1)
                    .build();
            cs1.setGoal(new KineticState(targetpos));
            KineticState current = new KineticState(rb.getCurrentPosition(),rb.getVelocity());
            double output = cs1.calculate(current);
            transfer.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
            spindexer.setPower(-gamepad2.left_stick_y);

            KineticState current1 = new KineticState(flywheel.getCurrentPosition(), Math.abs(flywheel.getVelocity()));
            double output2 = cs.calculate(current1);
            //flywheel.setPower(output2);
            flywheel.setPower(-gamepad2.right_stick_y);
            telemetry.addData("motor current pos",rb.getCurrentPosition());
            telemetry.addData("target",targetpos);
            telemetry.addData("motor current speed",Math.abs(flywheel.getVelocity()));
            telemetry.addData("target",targetTicksPerSecond);
            telemetry.addData("output",output2);
            telemetry.addData("output",output);
            telemetry.update();
        }
    }
}
