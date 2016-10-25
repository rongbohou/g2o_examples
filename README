# g2o example
**Authors:** rongbohou, email:rongbohou@163.com

some g2o example of doing BA ICP


# Require: 
g2o, OpenCV 2.4.x

# build
mkdir build ＆＆ cd build 
cmake  .. 
make 

# usage
(1) ./bin/ba_example ./data/1.png ./data/2.png
It uses g2o BA to estimate the relative motion between frames and the 3d positions (under a unknown scale).

(2) ./bin/simple_optimize 
g2o simple optimize example. input graph:./data/sphere_bignoise_vertex3.g2o output optimize graph:./data/sphere_after.g2o
*doubt*: in the CMakeLists.txt,if doesn't link OpenCV_LIBS,this program will exist a error: what():std::bad_alloc when optimizer.optimize(),because the number of vertice and edges is wrong.

(3) ./bin/gicp_demo
It uses g2o ICP to algin two point cloud with noise.

(4)./bin/icp_example
It uses g2o ICP to algin two point cloud  from ./data/ism_test.pcd and the transition of this .pcd file.
Finally,it estimates the transition matrix. As we can see from the program, we must give the macthes and a good initial guess

