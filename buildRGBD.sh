# third party
cd ORB_SLAM3/Thirdparty/
sudo rm -rf DBoW2/build/* Sophus/build/* g2o/build/* DBoW2/lib/* g2o/lib/*
cd -

# no-gpu -----
cd darknet/build/
sudo rm -rf *
cmake ..
make
cd -

# gpu -----
cd darknet-gpu/build/
sudo rm -rf *
cmake ..
make
cd -

# qt
sudo rm -r build-qrgbd-Desktop_Qt_6_1_3_GCC_64bit-Release

# darklib[no-gpu]
cp darknet/build/libdarknet.so  yolov4-learn/lib/

# ORB-SLAM3
cd ORB_SLAM3
sudo rm -r build
./build.sh
