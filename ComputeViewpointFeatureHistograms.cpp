// STL
#include <iostream>
#include <map>
#include <vector>

// Boost
#include <boost/make_shared.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>

// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

// ITK
#include "itkImageFileReader.h"
#include "itkImageRegionConstIteratorWithIndex.h"

typedef pcl::PointCloud<pcl::PointXYZ> InputCloud;
typedef pcl::PointCloud<pcl::VFHSignature308> OutputCloud;

typedef itk::Image<bool, 2> MaskImageType;

static void ComputeViewpointFeatureHistograms(InputCloud::Ptr input, MaskImageType* mask, OutputCloud::Ptr output);
static void ComputeViewpointFeatureHistogram(InputCloud::Ptr input, MaskImageType* mask, itk::Index<2>& index, OutputCloud::Ptr output);
static void AddToPolyData(OutputCloud::Ptr outputCloud, vtkPolyData* const polyData);

itk::ImageRegion<2> GetRegionInRadiusAroundPixel(const itk::Index<2>& pixel, const unsigned int radius);

int main (int argc, char** argv)
{
  if(argc < 5)
    {
    throw std::runtime_error("Required arguments: PCLInputFileName.pcd VTKInputFileName.vtp mask.mha OutputFileName.vtp");
    }

  std::string pclInputFileName = argv[1];
  std::string vtkInputFileName = argv[2];
  std::string maskFileName = argv[3];
  std::string outputFileName = argv[4];
  std::cout << "Reading " << pclInputFileName << " and " << vtkInputFileName << " and " << maskFileName << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pclInputFileName, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }

  
  // Read the VTK data
  std::cout << "Reading vtp file..." << std::endl;
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(vtkInputFileName.c_str());
  reader->Update();
  
  typedef itk::ImageFileReader<MaskImageType> MaskReaderType;
  MaskReaderType::Pointer maskReader = MaskReaderType::New();
  maskReader->SetFileName(maskFileName);
  maskReader->Update();

  OutputCloud::Ptr outputCloud (new OutputCloud);
  ComputeViewpointFeatureHistograms(cloud, maskReader->GetOutput(), outputCloud);

  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->DeepCopy(reader->GetOutput());
  AddToPolyData(outputCloud, polyData);

  std::cout << "Writing output..." << std::endl;
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(outputFileName.c_str());
  writer->SetInputConnection(polyData->GetProducerPort());
  writer->Write();

  return 0;
}

itk::ImageRegion<2> GetRegionInRadiusAroundPixel(const itk::Index<2>& pixel, const unsigned int radius)
{
  // This function returns a Region with the specified 'radius' centered at 'pixel'. By the definition of the radius of a square patch, the output region is (radius*2 + 1)x(radius*2 + 1).
  // Note: This region is not necessarily entirely inside the image!

  // The "index" is the lower left corner, so we need to subtract the radius from the center to obtain it
  itk::Index<2> lowerLeft;
  lowerLeft[0] = pixel[0] - radius;
  lowerLeft[1] = pixel[1] - radius;

  itk::ImageRegion<2> region;
  region.SetIndex(lowerLeft);
  itk::Size<2> size;
  size[0] = radius*2 + 1;
  size[1] = radius*2 + 1;
  region.SetSize(size);

  return region;
}

void ComputeViewpointFeatureHistogram(InputCloud::Ptr input, MaskImageType* mask, itk::Index<2>& index, OutputCloud::Ptr output)
{
  
}

void ComputeViewpointFeatureHistograms(InputCloud::Ptr input, MaskImageType* mask, OutputCloud::Ptr output)
{
  // Only compute the descriptor on a subset of the points
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  //for(unsigned int pointId = 0; pointId < 1000; ++pointId)
  for(unsigned int pointId = 0; pointId < input->points.size(); ++pointId) // compute for all points in the same style
  {
    indices->indices.push_back(pointId);
  }

  std::cout << "Created " << indices->indices.size() << " indices." << std::endl;

  // Compute the normals
  pcl::NormalEstimation<InputCloud::PointType, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (input);

  pcl::search::KdTree<InputCloud::PointType>::Ptr tree (new pcl::search::KdTree<InputCloud::PointType>);
  normalEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  normalEstimation.setRadiusSearch (0.03);

  normalEstimation.compute (*cloudWithNormals);

  std::map<itk::Index<2>, unsigned int, itk::Index<2>::LexicographicCompare> coordinateMap;



  std::cout << "Creating index map..." << std::endl;
  vtkIntArray* indexArray = vtkIntArray::SafeDownCast(reader->GetOutput()->GetPointData()->GetArray("OriginalPixel"));
  for(vtkIdType pointId = 0; pointId < reader->GetOutput()->GetNumberOfPoints(); ++pointId)
    {
    //int* pixelIndexArray;
    int pixelIndexArray[2];
    indexArray->GetTupleValue(pointId, pixelIndexArray);

    itk::Index<2> pixelIndex;
    pixelIndex[0] = pixelIndexArray[0];
    pixelIndex[1] = pixelIndexArray[1];
    coordinateMap[pixelIndex] = pointId;
    }

  vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
  data->DeepCopy(reader->GetOutput());

  vtkSmartPointer<vtkFloatArray> descriptors = vtkSmartPointer<vtkFloatArray>::New();
  descriptors->SetName("ViewpointFeatureHistograms");
  descriptors->SetNumberOfComponents(308);
  descriptors->SetNumberOfTuples(cloud->points.size());

  // Zero all of the descriptors, we may not have one to assign for every point.
  std::vector<float> zeroVector(308, 0);

  for(size_t pointId = 0; pointId < cloud->points.size(); ++pointId)
    {
    descriptors->SetTupleValue(pointId, zeroVector.data());
    }

  unsigned int patch_half_width = 5;

  std::cout << "Computing descriptors..." << std::endl;
  itk::ImageRegion<2> fullRegion = mask->GetLargestPossibleRegion();
  itk::ImageRegionConstIteratorWithIndex<MaskImageType> imageIterator(mask, fullRegion);
  std::cout << "Full region: " << fullRegion << std::endl;

  while(!imageIterator.IsAtEnd())
    {
    itk::ImageRegion<2> patchRegion = GetRegionInRadiusAroundPixel(imageIterator.GetIndex(), patch_half_width);
    //std::cout << "patchRegion: " << patchRegion << std::endl;
    if(!fullRegion.IsInside(patchRegion))
      {
      ++imageIterator;
      continue;
      }

    // Get a list of the pointIds in the region
    //std::vector<unsigned int> pointIds;
    std::vector<int> pointIds;
  
    itk::ImageRegionConstIteratorWithIndex<MaskImageType> patchIterator(mask, patchRegion);
    while(!patchIterator.IsAtEnd())
      {
      if(!patchIterator.Get())
        {
        pointIds.push_back(coordinateMap[patchIterator.GetIndex()]);
        }
      ++patchIterator;
      }

    //std::cout << "There are " << pointIds.size() << " points in this patch." << std::endl;

    if(pointIds.size() < patchRegion.GetNumberOfPixels())
      {
      ++imageIterator;
      continue;
      }

    // Setup the feature computation
    pcl::VFHEstimation<InputCloud::PointType, pcl::Normal, OutputCloud::PointType> vfhEstimation;

    //vfhEstimation.setIndices(&pointIds);
    vfhEstimation.setIndices(boost::make_shared<std::vector<int> >(pointIds));

    // Provide the original point cloud (without normals)
    vfhEstimation.setInputCloud (input);

    // Provide the point cloud with normals
    vfhEstimation.setInputNormals(cloudWithNormals);

    // vfhEstimation.setInputWithNormals(cloud, cloudWithNormals); VFHEstimation does not have this function
    // Use the same KdTree from the normal estimation
    vfhEstimation.setSearchMethod (tree);

    //vfhEstimation.setRadiusSearch (0.2);

    // Actually compute the VFH
    OutputCloud::Ptr vfhFeatures(new OutputCloud);
    vfhEstimation.compute (*vfhFeatures);

    // Display and retrieve the shape context descriptor vector for the 0th point.
    OutputCloud::PointType descriptor = vfhFeatures->points[0];

    unsigned int currentPointId = coordinateMap[imageIterator.GetIndex()];
    
    //std::cout << "descriptor " << currentPointId << " : " << descriptor.histogram << std::endl;
    descriptors->SetTupleValue(currentPointId, descriptor.histogram);

    ++imageIterator;
    }

  data->GetPointData()->AddArray(descriptors);
}

void AddToPolyData(OutputCloud::Ptr outputCloud, vtkPolyData* const polyData)
{
  
}
