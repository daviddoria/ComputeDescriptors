// STL
#include <iostream>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

typedef pcl::PointCloud<pcl::PointXYZ> InputCloud;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> OutputCloud;

static void ComputePrincipalCurvatures(InputCloud::Ptr input, OutputCloud::Ptr output);
static void AddToPolyData(OutputCloud::Ptr outputCloud, vtkPolyData* const polyData);

int
main (int argc, char** argv)
{
  if(argc < 4)
    {
    throw std::runtime_error("Required arguments: PCLFileName.pcd VTKFileName.vtp OutputFileName.vtp");
    }

  std::string pclFileName = argv[1];
  std::string vtkFileName = argv[2];
  std::string outputFileName = argv[3];
  std::cout << "Reading " << pclFileName << " and " << vtkFileName << std::endl;

  InputCloud::Ptr cloud (new InputCloud);

  if (pcl::io::loadPCDFile<InputCloud::PointType> (pclFileName, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }

  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  // Add the descriptors to the VTK data
  std::cout << "Adding features to vtp file..." << std::endl;
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(vtkFileName.c_str());
  reader->Update();

  OutputCloud::Ptr outputCloud (new OutputCloud);
  ComputePrincipalCurvatures(cloud, outputCloud);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->DeepCopy(reader->GetOutput());
  AddToPolyData(outputCloud, polyData);

  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(outputFileName.c_str());
  writer->SetInputConnection(polyData->GetProducerPort());
  writer->Write();

  return 0;
}

void ComputePrincipalCurvatures(InputCloud::Ptr input, OutputCloud::Ptr output)
{
  // Compute the normals
  pcl::NormalEstimation<InputCloud::PointType, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (input);

  pcl::search::KdTree<InputCloud::PointType>::Ptr tree (new pcl::search::KdTree<InputCloud::PointType>);
  normalEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  normalEstimation.setRadiusSearch (0.1);

  normalEstimation.compute (*cloudWithNormals);

  // Only compute the descriptor on a subset of the points
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  //for(unsigned int pointId = 0; pointId < 1000; ++pointId)
  for(unsigned int pointId = 0; pointId < input->points.size(); ++pointId)
  {
    indices->indices.push_back(pointId);
  }
  
  // Setup the principal curvatures computation
  pcl::PrincipalCurvaturesEstimation<InputCloud::PointType, pcl::Normal, OutputCloud::PointType> principalCurvaturesEstimation;

  principalCurvaturesEstimation.setIndices(indices);

  // Provide the original point cloud (without normals)
  principalCurvaturesEstimation.setInputCloud (input);
  
  principalCurvaturesEstimation.setRadiusSearch(.1);

  // Provide the point cloud with normals
  principalCurvaturesEstimation.setInputNormals(cloudWithNormals);

  // Use the same KdTree from the normal estimation
  principalCurvaturesEstimation.setSearchMethod (tree);

  // Actually compute the principal curvatures
  OutputCloud::Ptr principalCurvatures (new OutputCloud);
  principalCurvaturesEstimation.compute (*principalCurvatures);

}

void AddToPolyData(OutputCloud::Ptr outputCloud, vtkPolyData* const polyData)
{
  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName("PrincipalCurvatures");
  descriptors->SetNumberOfComponents(3);
  descriptors->SetNumberOfTuples(polyData->GetNumberOfPoints());

  // Zero all of the descriptors, we may not have one to assign for every point.
  std::vector<float> zeroVector(3, 0);

  for(vtkIdType pointId = 0; pointId < polyData->GetNumberOfPoints(); ++pointId)
    {
    descriptors->SetTupleValue(pointId, zeroVector.data());
    }

  std::cout << "Attaching features to VTK data..." << std::endl;
  for(size_t pointId = 0; pointId < outputCloud->points.size(); ++pointId)
    {
    OutputCloud::PointType descriptor = outputCloud->points[pointId];
    descriptors->SetTupleValue(pointId, descriptor.principal_curvature);
    }

  polyData->GetPointData()->AddArray(descriptors);

}
