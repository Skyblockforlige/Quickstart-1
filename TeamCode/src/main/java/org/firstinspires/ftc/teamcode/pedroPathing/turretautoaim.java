package org.firstinspires.ftc.teamcode.pedroPathing;





import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp

public class turretautoaim extends OpMode {



    //Max rad the turret can go w/o tangling but idk the actual values
    private Follower follower;

    //TODO: find the actual values for these

    public static final double turret_max = 110;
    public static Pose startingPose; //See ExampleAuto to understand how to use this

    public static final double turret_min = -110;



    //the turret offset (so like if .setPos(0) is to the left instead of straight add an offset here)

    //TODO: test and finalize offset!

    public static final double turret_offset = 0;



    //allowed tolerance between actual pos and goal

    public static final double angle_tolerance = Math.toRadians(5);

    public DcMotorEx turretEnc;





   /*

   put like thrubore.getCurrentPos here or something idk

   also update this every loop

   */

    private double currentTurretRad = 67;



    public double targetRad = 0.0;



    public void update(double robotX, double robotY, double robotHeading, double targetX, double targetY){



        double thetaField = Math.atan2(targetY-robotY, targetX-robotX);



        double desired = wrapAngle(thetaField-robotHeading - turret_offset);



        Double best = bestAngle(desired, currentTurretRad, turret_min, turret_max);



        if (best==null){

            targetRad = currentTurretRad;

        }else{

            targetRad = best;

        }



    }



    public void setCurrentTurretRad(double turretAngleRad){

        this.currentTurretRad = turretAngleRad;

    }



    private static double wrapAngle(double a){

        while (a>Math.PI) a-=2.0*Math.PI;

        while (a<Math.PI) a+=2.0*Math.PI;

        return a;

    }



    private static double bestAngle(double desired, double current, double min, double max){

        Double best = null;

        double bestErr = Double.POSITIVE_INFINITY;



        for (int k = -1; k<=1; k++){

            double candidate = desired + k * 2.0 * Math.PI;



            if(candidate<min || candidate>max) continue;



            double err = Math.abs(candidate-current);

            if(err<bestErr){

                bestErr = err;

                best = candidate;

            }



        }

        return best;

    }



    private static double clamp(double v, double min, double max){

        return Math.max(min, Math.min(max, v));

    }



    private boolean nearEdge(double turretRad){

        return (turretRad<turret_min+angle_tolerance) || (turretRad>turret_max+angle_tolerance);

    }


    @Override
    public void init(){
        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");
        follower.update();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        turretautoaim aim = new turretautoaim();


    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        // 1) Get pose from Pinpoint

        // pose.x, pose.y, pose.headingRad

        double x = follower.getPose().getX();

        double y = follower.getPose().getY();

        double heading = follower.getPose().getHeading();



        // 2) Update turret current angle from encoder (you implement this conversion)

        aim.setCurrentTurretRad(currentTurretRad);



        // 3) Choose target field point idk what that is

        double targetX = ...;

        double targetY = ...;



        // 4) Compute turret setpoint

        aim.update(x, y, heading, targetX, targetY);



        // 5) Turret PID to aim.turretSetpointRad

        // turretController.setTarget(aimer.turretSetpointRad);






    }
}