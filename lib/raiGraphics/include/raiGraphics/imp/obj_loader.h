#ifndef OBJ_LOADER_H_INCLUDED
#define OBJ_LOADER_H_INCLUDED

#include <glm/glm.hpp>
#include <vector>
#include <string>
#include "raiCommon/rai_utils.hpp"

namespace rai_graphics {

struct OBJIndex {
  unsigned int vertexIndex;
  unsigned int uvIndex;
  unsigned int normalIndex;

  bool operator<(const OBJIndex &r) const { return vertexIndex < r.vertexIndex; }
};

class IndexedModel {
 public:
  std::vector<glm::vec3> positions;
  std::vector<glm::vec2> texCoords;
  std::vector<glm::vec3> normals;
  std::vector<unsigned int> indices;

  void CalcNormals();
};

class OBJModel {
 public:
  std::vector<OBJIndex> OBJIndices;
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec2> uvs;
  std::vector<glm::vec3> normals;
  bool hasUVs;
  bool hasNormals;

  OBJModel(const std::string &fileName);

  IndexedModel ToIndexedModel();
 private:
  unsigned int FindLastVertexIndex(const std::vector<OBJIndex *> &indexLookup,
                                   const OBJIndex *currentIndex,
                                   const IndexedModel &result);
  void CreateOBJFace(const std::string &line);

  glm::vec2 ParseOBJVec2(const std::string &line);
  glm::vec3 ParseOBJVec3(const std::string &line);
  OBJIndex ParseOBJIndex(const std::string &token, bool *hasUVs, bool *hasNormals);
};

} // rai_graphics

#endif // OBJ_LOADER_H_INCLUDED
