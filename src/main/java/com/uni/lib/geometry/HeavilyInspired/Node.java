package com.uni.lib.geometry.HeavilyInspired;

import java.util.ArrayList;
import java.util.List;

import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;


public class Node {
    public double x;
    public double y;
    Rotation2d holonomicRotation;
    public List < Node > neighbors;
  
    public Node(double x, double y) {
        this.x = x;
        this.y = y;
        holonomicRotation = Rotation2d.fromDegrees(0);
        this.neighbors = new ArrayList < > ();
    }
  
    public Node(double x, double y, Rotation2d holonomicRotation) {
        this.x = x;
        this.y = y;
        this.holonomicRotation = holonomicRotation;
        this.neighbors = new ArrayList < > ();
    }
    public Node(Pose2d pose) {
        this.x = pose.getTranslation().x();
        this.y = pose.getTranslation().y();
        this.holonomicRotation = pose.getRotation();
        this.neighbors = new ArrayList < > ();
    }

    public Node(edu.wpi.first.math.geometry.Pose2d pose) {
        this.x = pose.getTranslation().getX();
        this.y = pose.getTranslation().getY();
        this.holonomicRotation = Rotation2d.fromDegrees(pose.getRotation().getDegrees());
        this.neighbors = new ArrayList < > ();
    }

    public Node(Translation2d coordinates, Rotation2d holonomicRotation) {
      this.x = coordinates.x();
      this.y = coordinates.y(); 
      this.holonomicRotation = holonomicRotation;
      this.neighbors = new ArrayList < > ();
    }


  
	  public void addNeighbor(Node neighbor) {
        this.neighbors.add(neighbor);
    }
    public double getX(){
      return x;
    }
    public double getY(){
      return y;
    }
    public Translation2d getTranslation(){
      return new Translation2d(x,y);
    }
    public Rotation2d getHolRot(){
      return holonomicRotation;
    }
    
    public edu.wpi.first.math.geometry.Pose2d toPose(){
      return new edu.wpi.first.math.geometry.Pose2d(x,y, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(holonomicRotation.getDegrees()));
    }
    public void setHolRot(double degree){
      this.holonomicRotation = Rotation2d.fromDegrees(degree);
    } 
  }
