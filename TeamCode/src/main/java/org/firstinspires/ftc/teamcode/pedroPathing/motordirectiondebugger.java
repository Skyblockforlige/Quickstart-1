package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class motordirectiondebugger extends LinearOpMode {
    public DcMotorEx lf;
    public DcMotorEx lb;
    public DcMotorEx rf;
    public DcMotorEx rb;
    @Override
    public void runOpMode(){
        lf = hardwareMap.get(DcMotorEx.class,"lf");
        lb=hardwareMap.get(DcMotorEx.class,"lb");
        rf = hardwareMap.get(DcMotorEx.class,"rf");
        rb=hardwareMap.get(DcMotorEx.class,"rb");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.cross){
                lf.setPower(1);
            } else{
                lf.setPower(0);
            }
            if(gamepad1.circle){
                lb.setPower(1);
            } else{
                lb.setPower(0);
            }
            if(gamepad1.square){
                rf.setPower(1);
            } else{
                rf.setPower(0);
            }
            if(gamepad1.triangle){
                rb.setPower(1);
            } else{
                rb.setPower(0);
            }
        }
    }
}
