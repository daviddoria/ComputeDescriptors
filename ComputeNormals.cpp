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
typedef pcl::PointCloud<pcl::Normal> OutputCloud;

static void ComputeNormals(InputCloud::Ptr input, OutputCloud::Ptr output);
static void AddNormalsToPolyData(OutputCloud::Ptr cloudWithNormals, vtkPolyData* const polyData);

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

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pclFileName, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }

  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(vtkFileName.c_str());
  reader->Update();

  OutputCloud::Ptr cloudWithNormals (new OutputCloud);
  ComputeNormals(cloud, cloudWithNormals);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->DeepCopy(reader->GetOutput());
  AddNormalsToPolyData(cloudWithNormals, polyData);

  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(outputFileName.c_str());
  writer->SetInputConnection(polyData->GetProducerPort());
  writer->Write();
  
  return 0;
}

void ComputeNormals(InputCloud::Ptr input, OutputCloud::Ptr output)
{
  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (input);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod (tree);

  normalEstimation.setRadiusSearch (0.1);

  normalEstimation.compute (*output);
}

void AddNormalsToPolyData(OutputCloud::Ptr cloudWithNormals, vtkPolyData* const polyData)
{
  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName("Normals");
  descriptors->SetNumberOfComponents(3);
  descriptors->SetNumberOfTuples(cloudWithNormals->points.size());

  std::cout << "Attaching features to VTK data..." << std::endl;
  for(size_t pointId = 0; pointId < cloudWithNormals->points.size(); ++pointId)
    {
    pcl::Normal descriptor = cloudWithNormals->points[pointId];
    descriptors->SetTupleValue(pointId, descriptor.data_n); // Is this the right member?
    }

  polyData->GetPointData()->AddArray(descriptors);

}
