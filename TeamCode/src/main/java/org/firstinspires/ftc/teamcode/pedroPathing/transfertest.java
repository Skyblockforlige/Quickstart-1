package org.firstinspires.ftc.teamcode.pedroPathing;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class transfertest extends LinearOpMode {
    private DcMotorEx spindexer;
    private CRServoImplEx transfer;
    private DcMotorEx flywheel;


    @Override
    public void runOpMode(){
        flywheel = hardwareMap.get(DcMotorEx.class,"shooter");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        transfer=hardwareMap.get(CRServoImplEx.class, "transfer");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            transfer.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
            spindexer.setPower(-gamepad2.left_stick_y);
            flywheel.setPower(-gamepad2.right_stick_y);
        }
    }
}
