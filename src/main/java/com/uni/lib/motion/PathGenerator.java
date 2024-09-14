package com.uni.lib.motion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.Constants;
import com.uni.lib.geometry.HeavilyInspired.Edge;
import com.uni.lib.geometry.HeavilyInspired.Node;
import com.uni.lib.geometry.HeavilyInspired.Obstacle;
import com.uni.lib.geometry.HeavilyInspired.VisGraph;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

/** Custom PathPlanner version of SwerveControllerCommand */
public class PathGenerator {
    private double grain = 2;
    private List<Node> fullPath;
    private Node start;
    private Node endTarget;
    private VisGraph aStar;
    private List<Node> usedNodes;
    private List<Pose2d> fullPathPoses;
    private List<Obstacle> totalObstacles = Constants.FieldConstants.obstacles;
    private static PathGenerator instance;

    public static PathGenerator getInstance(){
        if(instance == null)
            instance = new PathGenerator();
        return instance;
    }

    public PathGenerator() {
        aStar = VisGraph.getInstance();
        fullPath = new ArrayList<Node>();
        fullPathPoses = new ArrayList<Pose2d>();
        usedNodes = new ArrayList<>();
        for (double i = 2 * grain; i < 15 * grain; i++) {
            for (double j = 0; j < 9 * grain; j++) {
                Node newNode = new Node(i / grain, j / grain);
                aStar.addNode(newNode);
                usedNodes.add(newNode);
            }
        }
        for (int i = 0; i < aStar.getNodeSize(); i++) {
            Node startNode = aStar.getNode(i);
            for (int j = i; j < aStar.getNodeSize(); j++) {
                aStar.addEdge(new Edge(startNode, aStar.getNode(j)), totalObstacles);
            }
        }
    }

    public PathPlannerTrajectory generatePath(com.uni.lib.geometry.Pose2d currentPose, PathConstraints constraints, com.uni.lib.geometry.Pose2d endPose, com.uni.lib.swerve.ChassisSpeeds currentVel) {
        start = new Node(currentPose);
        endTarget = new Node(endPose);
        aStar.addNode(endTarget);
        fullPathPoses = new ArrayList<Pose2d>();
        fullPath = new ArrayList<Node>();
        if(!usedNodes.contains(endTarget)){
            for (int i = 0; i < aStar.getNodeSize(); i++) {
                Node end = aStar.getNode(i);
                aStar.addEdge(new Edge(endTarget, end), totalObstacles);
            }
            usedNodes.add(endTarget);
        }


        if (aStar.addEdge(new Edge(start, endTarget), totalObstacles)) {
            fullPath.add(0, start);
            fullPath.add(1, endTarget);

        } else {
            for (int i = 0; i < aStar.getNodeSize(); i++) {
                Node end = aStar.getNode(i);
                aStar.addEdge(new Edge(start, end), totalObstacles);
            }
            fullPath = aStar.findPath(start, endTarget);
        }
        for(Node p : fullPath){
            Logger.recordOutput("p " + fullPath.indexOf(p), p.toPose());
            fullPathPoses.add(p.toPose());
        }

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(fullPathPoses);

        PathPlannerPath path = new PathPlannerPath(
            bezierPoints, 
            constraints, 
            new GoalEndState(0, endTarget.getHolRot().toWPI()));

        return path.getTrajectory(new ChassisSpeeds(-2,-2,0 ), currentPose.getRotation().toWPI());
    }
}   
