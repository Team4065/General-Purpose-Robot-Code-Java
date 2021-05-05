// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

/**
 * A loader for path weaver trajectories
 */
public class PathLoader {

    private Trajectory trajectory;

    /**
     * Loads the data from a path weaver file.
     * @param pathName The name of the pathweaver path.
     */
    public PathLoader(String pathName){
        try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/".concat(pathName.concat(".wpilib.json")));
          System.out.println(trajectoryPath.toFile().exists());
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
          DriverStation.reportError("Unable to open trajectory: " + "paths/".concat(pathName.concat(".wpilib.json")), ex.getStackTrace());
        }
    }

    public Trajectory getTrajectory(){
        return trajectory;
    }
}