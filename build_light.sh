# echo "Configuring and building Thirdparty/g2o ..."
# cd Thirdparty/g2o
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j4
# cd ../../..

echo "Configuring and building ORB_SLAM2 ..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
