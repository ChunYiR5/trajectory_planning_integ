from setuptools import setup
from setuptools import Extension

pyKinematics = Extension(name='pyKinematics',  # 模組名稱
                           sources=['pyKinematics.cpp'],    # 原始碼
                           include_dirs=[r'.\eigen-3.3.8',
                                         r'.\env\Scripts',     # 依賴的第三方庫的標頭檔案
                                         r'.\env\Lib\site-packages\pybind11\include',
                                         "C:\\opt\\ros\\foxy\\x64\\include\\eigen3",
                                         "C:\\opt\\ros\\foxy\\x64\\Lib\\site-packages\\pybind11\\include",
                                         "D:\\ros2_project\\controller_ws\\install\\include"]
                           )

setup(ext_modules=[pyKinematics])