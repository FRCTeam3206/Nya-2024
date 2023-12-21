package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;

public class SwerveSimulation extends SubsystemBase{
    private Pose2d pose=new Pose2d(2.0433433724010306,4.709550419100839,new Rotation2d());
    private ChassisSpeeds speeds=new ChassisSpeeds();
    Field2d simField2d=new Field2d();
    public SwerveSimulation(){
        System.out.println("Simulating Swerve");
        SmartDashboard.putData("Simulated Field", simField2d);
        AutoBuilder.configureHolonomic(this::getPose, this::resetPose, this::getSpeeds, this::setSpeeds,AutoConstants.pathFollowerConfig, this);
    }
    public Pose2d getPose(){
        return pose;
    }
    public void resetPose(Pose2d pose){
        this.pose=pose;
    }
    public ChassisSpeeds getSpeeds(){
        return speeds;
    }
    public void setSpeeds(ChassisSpeeds speeds){
        this.speeds=speeds;
    }
    //Very simple update
    @Override
    public void periodic(){
        double dx=speeds.vxMetersPerSecond;
        double dy=speeds.vyMetersPerSecond;
        double dt=speeds.omegaRadiansPerSecond;
        Pose2d change=new Pose2d(dx, dy, new Rotation2d(dt)).times(1.0/50);//50 Hz clock on WPILib (m/s)*(s)=m
        pose=pose.plus(new Transform2d(new Pose2d(), change));
        simField2d.setRobotPose(pose);
    }
}
