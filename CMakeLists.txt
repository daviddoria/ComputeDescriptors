cmake_minimum_required(VERSION 2.6)

PROJECT(ComputeDescriptors)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET")

#################################
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
INCLUDE_DIRECTORIES(${VTK_INCLUDE_DIRS})

FIND_PACKAGE(PCL 1.5 REQUIRED)
INCLUDE(${VTK_USE_FILE})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

###################
ADD_EXECUTABLE(ComputeShapeContexts ComputeShapeContexts.cpp)

TARGET_LINK_LIBRARIES(ComputeShapeContexts
${VTK_LIBRARIES}
pcl_io
pcl_common
pcl_features
pcl_search
pcl_kdtree
)

ADD_EXECUTABLE(ComputeSpinImages ComputeSpinImages.cpp)

TARGET_LINK_LIBRARIES(ComputeSpinImages
${VTK_LIBRARIES}
pcl_io
pcl_common
pcl_features
pcl_search
pcl_kdtree
)

ADD_EXECUTABLE(ComputePointFeatureHistograms ComputePointFeatureHistograms.cpp)

TARGET_LINK_LIBRARIES(ComputePointFeatureHistograms
${VTK_LIBRARIES}
pcl_io
pcl_common
pcl_features
pcl_search
pcl_kdtree
)

ADD_EXECUTABLE(ComputeFastPointFeatureHistograms ComputeFastPointFeatureHistograms.cpp)

TARGET_LINK_LIBRARIES(ComputeFastPointFeatureHistograms
${VTK_LIBRARIES}
pcl_io
pcl_common
pcl_features
pcl_search
pcl_kdtree
)


ADD_EXECUTABLE(ComputePrincipalCurvatures ComputePrincipalCurvatures.cpp)

TARGET_LINK_LIBRARIES(ComputePrincipalCurvatures
${VTK_LIBRARIES}
pcl_io
pcl_common
pcl_features
pcl_search
pcl_kdtree
)


ADD_EXECUTABLE(ComputeViewpointFeatureHistograms ComputeViewpointFeatureHistograms.cpp)

TARGET_LINK_LIBRARIES(ComputeViewpointFeatureHistograms
${VTK_LIBRARIES} ${ITK_LIBRARIES}
pcl_io
pcl_common
pcl_features
pcl_search
pcl_kdtree
)


ADD_EXECUTABLE(ComputeNormals ComputeNormals.cpp)

TARGET_LINK_LIBRARIES(ComputeNormals
${VTK_LIBRARIES}
pcl_io
pcl_common
pcl_features
pcl_search
pcl_kdtree
)
