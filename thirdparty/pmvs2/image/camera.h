#ifndef IMAGE_CAMERA_H
#define IMAGE_CAMERA_H

#include <vector>
#include <string>
#include <climits>
#include "../numeric/vec4.h"
#include "../numeric/mat4.h"
#include "../numeric/mat3.h"

namespace Image {

class Ccamera {
 public:
  Ccamera(void);
  virtual ~Ccamera();
  
  // Update projection matrices from intrinsics and extrinsics
  void updateProjection(void);
  // Update all the camera related parameters
  void updateCamera(void);

  virtual void init(const std::string cname, const int maxLevel);
  void write(const std::string file);
  
  inline pmvsVec3f project(const pmvsVec4f& coord, const int level) const;
  inline pmvsVec3f mult(const pmvsVec4f& coord, const int level) const;
  
  static void setProjection(const std::vector<float>& intrinsics,
			    const std::vector<float>& extrinsics,
			    std::vector<pmvsVec4f>& projection,
			    const int txtType);
  
  float getScale(const pmvsVec4f& coord, const int level) const;
  void getPAxes(const pmvsVec4f& coord, const pmvsVec4f& normal,
		pmvsVec4f& pxaxis, pmvsVec4f& pyaxis, const int level = 0) const;

  void setAxesScale(const float axesScale);
  
  static void proj2q(Mat4& mat, double q[6]);
  static void q2proj(const double q[6], Mat4& mat);  
  static void setProjectionSub(double params[], std::vector<pmvsVec4f>& projection,
			       const int level);

  float computeDistance(const pmvsVec4f& point) const;
  float computeDepth(const pmvsVec4f& point) const;  
  float computeDepthDif(const pmvsVec4f& rhs, const pmvsVec4f& lhs) const;

  // Compute where the viewing ray passing through coord intersects
  // with the plane abcd.
  pmvsVec4f intersect(const pmvsVec4f& coord, const pmvsVec4f& abcd) const;
  void intersect(const pmvsVec4f& coord, const pmvsVec4f& abcd,
                 pmvsVec4f& cross, float& distance) const;
  // Computer a 3D coordinate that projects to a given image
  // coordinate. You can specify a different depth by the third
  // component of icoord.
  pmvsVec4f unproject(const pmvsVec3f& icoord, const int m_level) const;
  
  void setK(pmvsMat3f& K) const;
  void setRT(pmvsMat4f& RT) const;

  void getR(pmvsMat3f& R) const;
  
  //----------------------------------------------------------------------
  // txt file name
  std::string m_cname;  
  // Optical center
  pmvsVec4f m_center;
  // Optical axis
  pmvsVec4f m_oaxis;
  
  float m_ipscale;
  // 3x4 projection matrix
  std::vector<std::vector<pmvsVec4f> > m_projection;
  pmvsVec3f m_xaxis;
  pmvsVec3f m_yaxis;
  pmvsVec3f m_zaxis;

  // intrinsic and extrinsic camera parameters. Compact form.
  std::vector<float> m_intrinsics;
  std::vector<float> m_extrinsics;
  // camera parameter type
  int m_txtType;
 protected:
  int m_maxLevel;

  float m_axesScale;

  pmvsVec4f getOpticalCenter(void) const;
};

inline pmvsVec3f Ccamera::project(const pmvsVec4f& coord,
			      const int level) const {
  pmvsVec3f vtmp;    
  for (int i = 0; i < 3; ++i)
    vtmp[i] = m_projection[level][i] * coord;

  if (vtmp[2] <= 0.0) {
    vtmp[0] = -0xffff;
    vtmp[1] = -0xffff;
    vtmp[2] = -1.0f;
    return vtmp;
  }
  else
    vtmp /= vtmp[2];
  
  vtmp[0] = std::max((float)(INT_MIN + 3.0f),
		     std::min((float)(INT_MAX - 3.0f),
			      vtmp[0]));
  vtmp[1] = std::max((float)(INT_MIN + 3.0f),
		     std::min((float)(INT_MAX - 3.0f),
			      vtmp[1]));
  
  return vtmp;
}

inline pmvsVec3f Ccamera::mult(const pmvsVec4f& coord,
			      const int level) const {
  pmvsVec3f vtmp;    
  for (int i = 0; i < 3; ++i)
    vtmp[i] = m_projection[level][i] * coord;
  
  return vtmp;
}
 
template<class T>
float computeEPD(const TMat3<T>& F, const TVec3<T>& p0, const TVec3<T>& p1) {
  TVec3<T> line = F * p1;
  const T ftmp = sqrt(line[0] * line[0] + line[1] * line[1]);
  if (ftmp == 0.0)
    return 0.0;

  line /= ftmp;
  return fabs(line * p0);
}

template<class T>
void setF(const Image::Ccamera& lhs, const Image::Ccamera& rhs,
	  TMat3<T>& F, const int level = 0) {
  const TVec4<T>& p00 = lhs.m_projection[level][0];
  const TVec4<T>& p01 = lhs.m_projection[level][1];
  const TVec4<T>& p02 = lhs.m_projection[level][2];

  const TVec4<T>& p10 = rhs.m_projection[level][0];
  const TVec4<T>& p11 = rhs.m_projection[level][1];
  const TVec4<T>& p12 = rhs.m_projection[level][2];

  F[0][0] = det(TMat4<T>(p01, p02, p11, p12));
  F[0][1] = det(TMat4<T>(p01, p02, p12, p10));
  F[0][2] = det(TMat4<T>(p01, p02, p10, p11));

  F[1][0] = det(TMat4<T>(p02, p00, p11, p12));
  F[1][1] = det(TMat4<T>(p02, p00, p12, p10));
  F[1][2] = det(TMat4<T>(p02, p00, p10, p11));

  F[2][0] = det(TMat4<T>(p00, p01, p11, p12));
  F[2][1] = det(TMat4<T>(p00, p01, p12, p10));
  F[2][2] = det(TMat4<T>(p00, p01, p10, p11));
 }
 
} // namespace image

#endif // CAMERA_H
