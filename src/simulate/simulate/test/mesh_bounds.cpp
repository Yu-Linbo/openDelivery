// Use the same COLLADA loader as Gazebo, including its asset axis conventions.
#include <iomanip>
#include <iostream>
#include <gazebo/common/Mesh.hh>
#include <gazebo/common/MeshManager.hh>

int main(int argc, char **argv) {
  std::cout << std::setprecision(17);
  for (int i = 1; i < argc; ++i) {
    const auto mesh = gazebo::common::MeshManager::Instance()->Load(argv[i]);
    if (!mesh || mesh->GetVertexCount() == 0) return 1;
    std::cout << mesh->Min() << " " << mesh->Max() << "\n";
  }
}
