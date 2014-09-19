#!/bin/bash          

# run cop-slam on all data sets
./copslam ../data/KITTI_00_vo.g2o     ../data/KITTI_00_cs.g2o
./copslam ../data/KITTI_02_vo.g2o     ../data/KITTI_02_cs.g2o
./copslam ../data/NewCollege_vo.g2o   ../data/NewCollege_cs.g2o
./copslam ../data/TheHague_02_vo.g2o  ../data/TheHague_02_cs.g2o
./copslam ../data/Pittsburgh_A_vo.g2o ../data/Pittsburgh_A_cs.g2o
./copslam ../data/Pittsburgh_B_vo.g2o ../data/Pittsburgh_B_cs.g2o
./copslam ../data/Pittsburgh_C_vo.g2o ../data/Pittsburgh_C_cs.g2o
