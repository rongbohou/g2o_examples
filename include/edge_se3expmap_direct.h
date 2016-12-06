
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/se3_ops.h>
#include <types_sba.h>
#include <Eigen/Geometry>
//                                                                                        template_name<error's dimension,error's type,type of node>
class G2O_TYPES_SBA_API EdgeProjectDirect : public  BaseBinaryEdge<1, double, VertexSE3Expmap>{
public: //method
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeProjectDirect();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError();
//calculate the jocobin matrix
  virtual void linearizeOplus();
public: //variables member

};
