# setup.py
from setuptools import setup, Extension, find_packages
from Cython.Build import cythonize
# import sys

extensions = [
    Extension("control.LidarAcquisitionProcess", ["control/LidarAcquisitionProcess.py"]),
    Extension("control.LidarAcquisitionStrategies", ["control/LidarAcquisitionStrategies.py"]),
    Extension("model.dataprocess.DataFilter", ["model/dataprocess/DataFilter.py"]),
    Extension("model.dataprocess.DataSplitting", ["model/dataprocess/DataSplitting.py"]),
    Extension("model.dataprocess.complete_workpiece.DataFindBlocks", ["model/dataprocess/complete_workpiece/DataFindBlocks.py"]),
    Extension("model.dataprocess.complete_workpiece.DataPartitionAuto", ["model/dataprocess/complete_workpiece/DataPartitionAuto.py"]),
    Extension("model.dataprocess.complete_workpiece.DataPartitionStatic", ["model/dataprocess/complete_workpiece/DataPartitionStatic.py"]),
    Extension("model.dataprocess.complete_workpiece.GunDistributor", ["model/dataprocess/complete_workpiece/GunDistributor.py"]),
    Extension("model.dataprocess.frame_by_frame.BuildSideFrame", ["model/dataprocess/frame_by_frame/BuildSideFrame.py"]),
    Extension("model.formats.complete_workpiece.BlockDataFormat", ["model/formats/complete_workpiece/BlockDataFormat.py"]),
    Extension("model.formats.complete_workpiece.FrameDataFormat", ["model/formats/complete_workpiece/FrameDataFormat.py"]),
    Extension("model.formats.frame_by_frame.AxisFrameDataFormat", ["model/formats/frame_by_frame/AxisFrameDataFormat.py"]),
    Extension("model.lidar.LidarCommon", ["model/lidar/LidarCommon.py"]),
    Extension("model.lidar.LidarDirectionState", ["model/lidar/LidarDirectionState.py"]),
    Extension("model.plc.PlcCommon", ["model/plc/PlcCommon.py"]),
    Extension("model.plc.PlcFrame", ["model/plc/PlcFrame.py"]),
    Extension("model.plot.Draw", ["model/plot/Draw.py"]),
    Extension("model.plot.PointCloudCanvas", ["model/plot/PointCloudCanvas.py"]),
]

setup(
    name="Package",
    version="0.1",
    packages=find_packages(),
    ext_modules=cythonize(extensions, language_level=3),
    zip_safe=False
)
# Package_DLL
# 打包构建算法dll
# d:/SHHC/.venv/Scripts/python.exe setup.py build_ext --inplace
