// STL
#include <iostream>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/normal_3d.h>

// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

typedef pcl::PointCloud<pcl::PointXYZ> InputCloud;
const unsigned int numberOfBins = 153;
typedef pcl::PointCloud<pcl::Histogram<numberOfBins> > OutputCloud;

static void ComputeSpinImages(InputCloud::Ptr input, OutputCloud::Ptr output);
static void AddToPolyData(OutputCloud::Ptr outputCloud, vtkPolyData* const polyData);

int main (int argc, char** argv)
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

  // Display and retrieve the shape context descriptor vector for the 0th point.
  //SpinImage descriptor = spinImages->points[0];
  //std::cout << descriptor << std::endl;

  OutputCloud::Ptr outputCloud (new OutputCloud);
  ComputeSpinImages(cloud, outputCloud);

  // Add the descriptors to the VTK data
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(vtkFileName.c_str());
  reader->Update();

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->DeepCopy(reader->GetOutput());
  AddToPolyData(outputCloud, polyData);

  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(outputFileName.c_str());
  writer->SetInputConnection(polyData->GetProducerPort());
  writer->Write();

  return EXIT_SUCCESS;
}

void ComputeSpinImages(InputCloud::Ptr input, OutputCloud::Ptr output)
{
  // Compute the normals
  std::cout << "Computing normals..." << std::endl;
  pcl::NormalEstimation<InputCloud::PointType, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (input);

  pcl::search::KdTree<InputCloud::PointType>::Ptr tree (new pcl::search::KdTree<InputCloud::PointType>);
  normalEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  normalEstimation.setRadiusSearch (0.03);

  normalEstimation.compute (*cloudWithNormals);

  pcl::SpinImageEstimation<InputCloud::PointType, pcl::Normal, OutputCloud::PointType> spinImageEstimation(8, 0.5, 16);
  // Provide the original point cloud (without normals)
  //spinImageEstimation.setInputCloud (cloud);
  // Provide the point cloud with normals
  //spinImageEstimation.setInputNormals(cloudWithNormals);
  spinImageEstimation.setInputWithNormals(input, cloudWithNormals);
  // Use the same KdTree from the normal estimation
  spinImageEstimation.setSearchMethod (tree);

  OutputCloud::Ptr spinImages(new OutputCloud);

  //spinImageEstimation.setRadiusSearch (0.2);
  //spinImageEstimation.setRadiusSearch (0.4);
  //spinImageEstimation.setMinPointCountInNeighbourhood(5); // If less than this many points is found in the specified radius, an error results.

  spinImageEstimation.setKSearch(20);

  // Actually compute the spin images
  std::cout << "Computing spin images..." << std::endl;
  spinImageEstimation.compute (*spinImages);

  std::cout << "output points.size (): " << spinImages->points.size () << std::endl;

}

void AddToPolyData(OutputCloud::Ptr outputCloud, vtkPolyData* const polyData)
{
  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName("SpinImage");
  descriptors->SetNumberOfComponents(numberOfBins);
  descriptors->SetNumberOfTuples(polyData->GetNumberOfPoints());

  std::cout << "Attaching shape contexts to VTK data..." << std::endl;
  for(size_t pointId = 0; pointId < outputCloud->points.size(); ++pointId)
    {
      descriptors->SetTupleValue(pointId, outputCloud->points[pointId].histogram);
    }

  polyData->GetPointData()->AddArray(descriptors);
}
