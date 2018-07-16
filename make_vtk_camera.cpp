// https://gist.github.com/decrispell/fc4b69f6bedf07a3425b

/**
 * Convert standard camera intrinsic and extrinsic parameters to a vtkCamera instance for rendering
 * Assume square pixels and 0 skew (for now).
 *
 * focal_len : camera focal length (units pixels)
 * nx,ny : image dimensions in pixels
 * principal_pt: camera principal point,
 *    i.e. the intersection of the principal ray with the image plane (units pixels)
 * camera_rot, camera_trans : rotation, translation matrix mapping world points to camera coordinates
 * depth_min, depth_max : needed to set the clipping range
 *
 **/
vtkSmartPointer<vtkCamera> make_vtk_camera(double focal_len,
                                           int nx, int ny,
                                           vgl_point_2d<double> const& principal_pt,
                                           vgl_rotation_3d<double> const& camera_rot,
                                           vgl_vector_3d<double> const& camera_trans,
                                           double depth_min, double depth_max)
{
  // create the camera
  vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();

  // convert camera rotation and translation into a 4x4 homogeneous transformation matrix
  vtkSmartPointer<vtkMatrix4x4> camera_RT = make_transform(camera_rot, camera_trans);
  // apply the transform to scene objects
  camera->SetModelTransformMatrix( camera_RT );

  // the camera can stay at the origin because we are transforming the scene objects
  camera->SetPosition(0, 0, 0);
  // look in the +Z direction of the camera coordinate system
  camera->SetFocalPoint(0, 0, 1);
  // the camera Y axis points down
  camera->SetViewUp(0,-1,0);

  // ensure the relevant range of depths are rendered
  camera->SetClippingRange(depth_min, depth_max);

  // convert the principal point to window center (normalized coordinate system) and set it
  double wcx = -2*(principal_pt.x() - double(nx)/2) / nx;
  double wcy =  2*(principal_pt.y() - double(ny)/2) / ny;
  camera->SetWindowCenter(wcx, wcy);

  // convert the focal length to view angle and set it
  double view_angle = vnl_math::deg_per_rad * (2.0 * std::atan2( ny/2.0, focal_len ));
  std::cout << "view_angle = " << view_angle << std::endl;
  camera->SetViewAngle( view_angle );

  return camera;
}


/** 
 * Helper function: Convert rotation and translation into a vtk 4x4 homogeneous transform
 */
vtkSmartPointer<vtkMatrix4x4> make_transform(vgl_rotation_3d<double> const& R,
                                             vgl_vector_3d<double> const& T)
{
  vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();
  vnl_matrix_fixed<double,3,3> R_mat = R.as_matrix();
  for (int r=0; r<3; ++r) {
    for (int c=0; c<3; ++c) {
      m->SetElement(r,c,R_mat[r][c]);
    }
  }
  m->SetElement(0,3,T.x());
  m->SetElement(1,3,T.y());
  m->SetElement(2,3,T.z());
  m->SetElement(3,3,1);

  return m;
}
