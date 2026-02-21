package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@TeleOp
@Disabled
public class spindexerrevtest extends LinearOpMode {
    Timer goonTimer;
    DcMotorEx spindexer;
    ControlSystem cs1;
    public static double p=0.0009,i=0,d=0;
    public static int movespindexer = 2731;

    @Override
    public void runOpMode(){
        spindexer=hardwareMap.get(DcMotorEx.class,"spindexer");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int target = 0;
        waitForStart();
        while(opModeIsActive()){
            KineticState current2 = new KineticState(spindexer.getCurrentPosition(),spindexer.getVelocity());
            cs1 = ControlSystem.builder()
                    .posPid(p,i,d)
                    .build();
            cs1.setGoal(new KineticState(target));
            if(gamepad2.a){
                target+=movespindexer;
                sleep(300);
            }
            if(Math.abs(gamepad2.left_stick_y)==0) {
                spindexer.setPower(cs1.calculate(current2));
            } else{
                spindexer.setPower(-gamepad2.left_stick_y);
                target=spindexer.getCurrentPosition();

            }

        }
    }
    public void moveSpindexer(int pos, double speed){
        spindexer.setTargetPosition((spindexer.getCurrentPosition()+pos));
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setPower(speed);
    }
}
