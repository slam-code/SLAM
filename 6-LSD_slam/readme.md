```c
建议阅读顺序：

      util/settings.h settings.cpp
      util/SophusUtil.h
      util/IndexThreadReduce.h
      util/Undistorter.h  Undistorter.cpp
      util/EigenCoreInclude.h

IOWrapper/Timestamp.h Timestamp.cpp
IOWrapper/TimestampedObject.h

      util/globalFuncs.h
      util/globalFuncs.h

========================
IOWrapper/ImageDisplay.h
IOWrapper/NotifyBuffer.h
IOWrapper/InputImageStream.h
IOWrapper/Output3DWrapper.h
IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
IOWrapper/ROS/ROSImageStreamThread.h
IOWrapper/ROS/ROSOutput3DWrapper.h

=======================
DataStructures/FrameMemory.h FrameMemory.cpp
DataStructures/FramePoseStruct.h FramePoseStruct.cpp
DataStructures/Frame.h Frame.cpp

Tracking/least_squares.h least_squares.cpp
Tracking/Sim3Tracker.h Sim3Tracker.cpp
Tracking/SE3Tracker.h SE3Tracker.cpp
Tracking/TrackingReference.h TrackingReference.cpp
Tracking/Relocalizer.h Relocalizer.cpp


====非闭环可以不看==========
GlobalMapping/FabMap.h FabMap.cpp
==========================

GlobalMapping/g2oTypeSim3Sophus.h g2oTypeSim3Sophus.cpp
GlobalMapping/KeyFrameGraph.h  KeyFrameGraph.cpp
GlobalMapping/TrackableKeyFrameSearch.h TrackableKeyFrameSearch.cpp


=======================

DepthEstimation/DepthMapPixelHypothesis.h
DepthEstimation/DepthMap.h DepthMap.cpp

========full system=====

LiveSLAMWrapper.h LiveSLAMWrapper.cpp
SlamSystem.h SlamSystem.cpp

跑一个简单的pipeline：
main_on_images.cpp
main_live_odometry.cpp

```
