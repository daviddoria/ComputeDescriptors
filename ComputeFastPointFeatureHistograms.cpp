// STL
#include <iostream>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

typedef pcl::PointCloud<pcl::PointXYZ> InputCloud;
typedef pcl::PointCloud<pcl::FPFHSignature33> OutputCloud;

static void ComputeFastPointFeatureHistograms(InputCloud::Ptr input, OutputCloud::Ptr output);
static void AddToPolyData(OutputCloud::Ptr outputCloud, vtkPolyData* const polyData);

int main (int argc, char** argv)
{
  if(argc < 4)
  {
    throw std::runtime_error("Required arguments: filename.pcd  VTKFileName.vtp OutputFileName.vtp");
  }

  std::string fileName = argv[1];
  std::string vtkFileName = argv[2];
  std::string outputFileName = argv[3];
  std::cout << "Reading " << fileName << std::endl;

  InputCloud::Ptr cloud (new InputCloud);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }

  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(vtkFileName.c_str());
  reader->Update();

  OutputCloud::Ptr outputCloud (new OutputCloud);
  ComputeFastPointFeatureHistograms(cloud, outputCloud);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->DeepCopy(reader->GetOutput());
  AddToPolyData(outputCloud, polyData);

  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(outputFileName.c_str());
  writer->SetInputConnection(polyData->GetProducerPort());
  writer->Write();

  return 0;
}

void ComputeFastPointFeatureHistograms(InputCloud::Ptr input, OutputCloud::Ptr output)
{
  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (input);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  normalEstimation.setRadiusSearch (0.03);

  normalEstimation.compute (*cloudWithNormals);

  // Setup the feature computation
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimation;
  // Provide the original point cloud (without normals)
  fpfhEstimation.setInputCloud (input);
  // Provide the point cloud with normals
  fpfhEstimation.setInputNormals(cloudWithNormals);

  // fpfhEstimation.setInputWithNormals(cloud, cloudWithNormals); PFHEstimation does not have this function
  // Use the same KdTree from the normal estimation
  fpfhEstimation.setSearchMethod (tree);

  OutputCloud::Ptr pfhFeatures(new OutputCloud);

  fpfhEstimation.setRadiusSearch (0.2);

  // Actually compute the spin images
  fpfhEstimation.compute (*pfhFeatures);

  std::cout << "output points.size (): " << pfhFeatures->points.size () << std::endl;

  // Display and retrieve the shape context descriptor vector for the 0th point.
  pcl::FPFHSignature33 descriptor = pfhFeatures->points[0];
  std::cout << descriptor << std::endl;
}

void AddToPolyData(OutputCloud::Ptr outputCloud, vtkPolyData* const polyData)
{
  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName("Normals");
  descriptors->SetNumberOfComponents(3);
  descriptors->SetNumberOfTuples(outputCloud->points.size());

  std::cout << "Attaching features to VTK data..." << std::endl;
  for(size_t pointId = 0; pointId < outputCloud->points.size(); ++pointId)
    {
    OutputCloud::PointType descriptor = outputCloud->points[pointId];
    descriptors->SetTupleValue(pointId, descriptor.histogram);
    }

  polyData->GetPointData()->AddArray(descriptors);
}
