package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

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
    //target=12500
    public static int target = 0;
    private Limelight3A limelight;


    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
        lf=hardwareMap.get(DcMotorEx.class,"turret_enc");
        turretL=hardwareMap.crservo.get("turretL");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int detected = 0;
        int actual= 0;
        limelight.start();

        waitForStart();
        while(opModeIsActive()){
            LLResult llResult = limelight.getLatestResult();
            if (llResult.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                detected=fiducialResults.get(0).getFiducialId();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("ID:", fr.getFiducialId());
                    detected=fr.getFiducialId();
                    if(detected==23){
                        actual=22;
                    } else if(detected==22){
                        actual=21;
                    } else if(detected==21){
                        actual=23;
                    }
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

            }
            switch(actual){
                case 21:

                    //fake intakes PGP, case 21 is GPP,

                    break;

                case 22:



                    break;

                case 23:


                    break;

            }
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
            telemetry.addData("April Tag:", actual);
            telemetry.addData("Detected:" , detected);
            telemetry.update();
        }
    }

}
