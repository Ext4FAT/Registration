#pragma once

/************************************************************************/
/* namespace std                                                        */
/************************************************************************/
#include "Common.hpp"

/************************************************************************/
/* namespace pcl                                                        */
/************************************************************************/
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/print.h>

#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/visualization/pcl_visualizer.h>

using Eigen::Matrix4f;

enum METHOD {
	ICP_CLASSIC,
	ICP_NOLINEAR,
	ICP_WITHNORMLS,
	RANSACPLUSICP,
	SAC_IA
};

enum FEATYPE {
	FPFH,
	PFH,
	VFH,
	SIFT
};


struct RegisterParameter {
	//Downsample
	float leaf = 0.01f;
	//RANSAC
	int MaximumIterationsRANSAC = 50000; // Number of RANSAC iterations
	int NumberOfSamples = 5; // Number of points to sample for generating/prerejecting a pose
	int CorrespondenceRandomness = 5; // Number of nearest features to use
	float SimilarityThreshold = 0.7f; // Polygonal edge length similarity threshold
	float MaxCorrespondence = 2.5f; // Inlier threshold
	float InlierFraction = 0.2f;
	//ICP
	double EuclideanEpsilon = 2e-8;
	int MaximumIterationsICP = 1000;
	//Method
	METHOD Method = ICP_NOLINEAR;
	FEATYPE FeaType;
};

/************************************************************************/
/* Typedef                                                              */
/************************************************************************/
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;

typedef pcl::NormalEstimationOMP<PointNT, PointNT> NormalEstimationNT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerNT;

/************************************************************************/
/* Output                                                               */
/************************************************************************/
using pcl::console::print_info;
using pcl::console::print_warn;
using pcl::console::print_error;
using pcl::console::print_value;
using pcl::console::print_debug;
using pcl::console::print_highlight;

/************************************************************************/
/* Load model and mesh                                                  */
/************************************************************************/
/**
*  @brief LoadModel: load .pcd file to program, either model or mesh
*  @param model_path   .pcd filepath
*  @param model        load PointCloudT/PointCloudNT to memory
*/
bool  LoadModel(const string model_path, PointCloudT::Ptr &model); //XYZ
bool  LoadModel(const string model_path, PointCloudNT::Ptr &model); //Normal

/************************************************************************/
/* Load grasping region point cloud                                     */
/************************************************************************/
bool LoadGraspPcd(const string model_path, PointCloudT::Ptr &grasp);
bool LoadGrasp(const string model_path, PointCloudT::Ptr &grasp);

/************************************************************************/
/* Downsample model point cloud                                         */
/************************************************************************/
void Downsample(PointCloudNT::Ptr &model, float leaf);


/************************************************************************/
/* Estimate model curvatures                                            */
/************************************************************************/
void EstimateCurvatures(PointCloudNT::Ptr &model, float radius);

/************************************************************************/
/* Estimate FPFH features                                               */
/************************************************************************/
void EstimateFPFH(PointCloudNT::Ptr &model, FeatureCloudT::Ptr &model_features, float leaf);

/************************************************************************/
/* Output Transformation Matrix                                         */
/************************************************************************/
void Print4x4Matrix(const Matrix4f & matrix);


/************************************************************************/
/* Registration with RANSAC and ICP                                     */
/************************************************************************/
/**
*  @brief Registration: register model and mesh with RANSAC+ICP, out Transformation matrix
*  @param model        input 3D points cloud
*  @param mesh         generated by Depth camera and convert to Point Cloud Normal Point
*  @param model_align  output aligned model, which used to reflect to 2D
*  @param para		   register parameter
*  @param showGraphic  show graphic result or not
*/
Matrix4f Registration(	PointCloudNT::Ptr &model,
						PointCloudNT::Ptr &mesh,
						PointCloudNT::Ptr &model_align,
						RegisterParameter &para,
						bool showGraphic = true	);

/**
*  @brief RegistrationNoShow: register model and mesh with RANSAC+ICP, out Transformation matrix, but not show 3D Points Cloud
*  @param model        input 3D points cloud
*  @param mesh         generated by Depth camera and convert to Point Cloud Normal Point
*  @param model_align  output aligned model, which used to reflect to 2D
*  @param para		   register parameter
*/
Matrix4f RegistrationNoShow(	PointCloudNT::Ptr &model,
								PointCloudNT::Ptr &mesh,
								PointCloudNT::Ptr &model_align,
								RegisterParameter &para );
/**
*  @brief RegistrationNoShow_ICP: register model and mesh with ICP, ICP-WITHNORMAL, ICP-NONLINEAR, out Transformation matrix
*  @param model        input 3D points cloud
*  @param mesh         generated by Depth camera and convert to Point Cloud Normal Point
*  @param model_align  output aligned model, which used to reflect to 2D
*  @param para		   register parameter
*/
Matrix4f RegistrationNoShow_ICP(	PointCloudNT::Ptr &model,
									PointCloudNT::Ptr &mesh,
									PointCloudNT::Ptr &model_align,
									RegisterParameter &para);
