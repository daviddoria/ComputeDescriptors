// STL
#include <iostream>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/normal_3d.h>

// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

typedef pcl::PointCloud<pcl::PointXYZ> InputCloud;
typedef pcl::PointCloud<pcl::SHOT> OutputCloud;

static void ComputeShapeContexts(InputCloud::Ptr input, OutputCloud::Ptr output);
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

  InputCloud::Ptr cloud (new InputCloud);

  if (pcl::io::loadPCDFile<InputCloud::PointType> (pclFileName, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }

  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  OutputCloud::Ptr outputCloud (new OutputCloud);
  ComputeShapeContexts(cloud, outputCloud);

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

void ComputeShapeContexts(InputCloud::Ptr input, OutputCloud::Ptr output)
{
  // Compute the normals
  std::cout << "Computing normals..." << std::endl;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (input);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  normalEstimation.setRadiusSearch (0.03);

  normalEstimation.compute (*cloudWithNormals);

  // Setup the shape context computation
  typedef pcl::SHOT ShapeContext; // Signature of Histograms of OrienTations (SHOT).
  pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, ShapeContext > shapeContext;
  // Provide the original point cloud (without normals)
  shapeContext.setInputCloud (input);
  // Provide the point cloud with normals
  shapeContext.setInputNormals(cloudWithNormals);
  // Use the same KdTree from the normal estimation
  shapeContext.setSearchMethod (tree);

  OutputCloud::Ptr shapeContextFeatures(new OutputCloud);

  // The search radius must be set to above the minimal search radius
  std::cout << "min radius: " << shapeContext.getMinimalRadius() << std::endl;
  float radius = .01; // originally .2

  // Search radius must be greater than min radius
//   shapeContext.setMinimalRadius(radius); // If minimal radius is the same as radius search, get INF errors.
//   shapeContext.setRadiusSearch(radius/2);
  
  // Like this, memory is quickly filled
//   shapeContext.setMinimalRadius(radius/2); // If minimal radius is the same as radius search, get INF errors.
//   shapeContext.setRadiusSearch(radius);

  // The minimal radius is generally set to approx. 1/10 of the search radius, while the pt. density radius is generally set to 1/5
  shapeContext.setRadiusSearch (radius);
  shapeContext.setPointDensityRadius(radius/5.0f);
  shapeContext.setMinimalRadius(radius/10.0f);

  // If minimal radius is the same as radius search, get INF errors.
//   shapeContext.setMinimalRadius(radius);
//   shapeContext.setRadiusSearch(radius);

  // Actually compute the shape contexts
  std::cout << "Computing shape contexts..." << std::endl;
  shapeContext.compute (*shapeContextFeatures);

}

void AddToPolyData(OutputCloud::Ptr outputCloud, vtkPolyData* const polyData)
{
  if(outputCloud->points.size() != polyData->GetNumberOfPoints())
  {
    throw std::runtime_error("Number of points in outputCloud does not match number of points in polyData!");
  }
  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName("ShapeContext");
  descriptors->SetNumberOfComponents(outputCloud->points[0].descriptor.size());
  descriptors->SetNumberOfTuples(polyData->GetNumberOfPoints());

  std::cout << "Attaching shape contexts to VTK data..." << std::endl;
  for(size_t pointId = 0; pointId < outputCloud->points.size(); ++pointId)
    {
    std::vector<float> descriptor = outputCloud->points[pointId].descriptor;
    descriptors->SetTupleValue(pointId, descriptor.data());
    }

  polyData->GetPointData()->AddArray(descriptors);

}
