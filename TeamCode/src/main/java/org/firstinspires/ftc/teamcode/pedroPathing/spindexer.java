package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
@TeleOp
@Disabled
public class spindexer extends LinearOpMode {
    private DcMotorEx spindexer;
    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;

    private NormalizedColorSensor colorSensor3;
    private float[] hsvValues1 = new float[3];
    private float[] hsvValues2 = new float[3];

    private float[] hsvValues3 = new float[3];
    private boolean targetPurple = true;
    private boolean idle = false;
    private int[] classifier = new int[9];
    private CRServoImplEx transfer;
    private ServoImplEx transfermover;
    private String motif;
    private double transfermoverpos = 0.5;

    @Override
    public void runOpMode(){
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        colorSensor1=hardwareMap.get(NormalizedColorSensor.class,"cs1");
        colorSensor2=hardwareMap.get(NormalizedColorSensor.class,"cs2");
        colorSensor3=hardwareMap.get(NormalizedColorSensor.class,"cs3");
        transfer=hardwareMap.get(CRServoImplEx.class, "transfer");
        transfermover=hardwareMap.get(ServoImplEx.class,"transfermover");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motif="PGP";
        waitForStart();
        while(opModeIsActive()){
            NormalizedRGBA cs1= colorSensor1.getNormalizedColors();
            Color.colorToHSV(cs1.toColor(),hsvValues1);
            NormalizedRGBA cs2= colorSensor2.getNormalizedColors();
            Color.colorToHSV(cs2.toColor(),hsvValues2);
            NormalizedRGBA cs3= colorSensor3.getNormalizedColors();
            Color.colorToHSV(cs3.toColor(),hsvValues3);
            boolean idle = true;

            for (int i = 0; i < classifier.length; i++) {
                if (classifier[i] == 0)
                {break;}
                int next = (i + 1 < classifier.length) ? classifier[i + 1] : 0;
                int next2 = (i + 2 < classifier.length) ? classifier[i + 2] : 0;

                switch (motif) {
                    case "PGP":
                        if (classifier[i] == 1 && next == 2 && next2 == 1) {
                            targetPurple = true;
                            idle = false;
                        } else if (classifier[i] == 1 && next == 2 && next2 == 0) {
                            targetPurple = true;
                            idle = false;
                        } else if (classifier[i] == 2 && next == 1) {
                            targetPurple = true;
                            idle = false;
                        }
                        break;

                    case "PPG":
                        if (classifier[i] == 1 && next == 1 && next2 == 2) {
                            targetPurple = false;
                            idle = false;
                        } else if (classifier[i] == 1 && next == 2) {
                            targetPurple = false;
                            idle = false;
                        }
                        break;

                    case "GPP":
                        if (classifier[i] == 2 && next == 1 && next2 == 1) {
                            targetPurple = true;
                            idle = false;
                        } else if (classifier[i] == 2 && next == 1) {
                            targetPurple = true;
                            idle = false;
                        }
                        break;

                    default:
                        idle = true;
                        break;
                }

                if (!idle) break;
            }


        }
            if(targetPurple&&!idle){
                if (hsvValues1[0] <= 290 && hsvValues1[0] >= 270) {
                    spindexer.setPower(0);
                    transfer.setPower(1);
                    transfermover.setPosition(transfermoverpos);
                } else {
                    if ((hsvValues2[0] >= 270 && hsvValues2[0] <= 290)) {
                        spindexer.setPower(1);
                        transfer.setPower(0);
                        transfermover.setPosition(0);
                    } else if (hsvValues3[0] >= 270 && hsvValues3[0] <= 290) {
                        spindexer.setPower(-1);
                        transfer.setPower(0);
                        transfermover.setPosition(0);
                    } else{
                        spindexer.setPower(0);
                        transfer.setPower(0);
                        transfermover.setPosition(0);
                    }
                }
            }else if(!targetPurple && !idle) {
                if (hsvValues1[0] <= 110 && hsvValues1[0] >= 90) {
                    spindexer.setPower(0);
                    transfer.setPower(1);
                    transfermover.setPosition(transfermoverpos);
                } else {
                    if ((hsvValues2[0] >= 90 && hsvValues2[0] <= 110)) {
                        spindexer.setPower(1);
                        transfer.setPower(0);
                        transfermover.setPosition(0);
                    } else if (hsvValues3[0] >= 90 && hsvValues3[0] <= 110) {
                        spindexer.setPower(-1);
                        transfer.setPower(0);
                        transfermover.setPosition(0);
                    } else{
                        spindexer.setPower(0);
                        transfer.setPower(0);
                        transfermover.setPosition(0);
                    }
                }
            } else if(idle){
                spindexer.setPower(0);
                transfer.setPower(0);
                transfermover.setPosition(0);
            }

        }
    }

