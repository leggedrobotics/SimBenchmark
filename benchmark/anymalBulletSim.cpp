//
// Created by kangd on 13.02.18.
//

#include <bulletSim/World_RG.hpp>
#include <bulletSim/importURDF/UrdfParser.h>
#include <Bullet3Common/b3Logging.h>
#include <Bullet3Common/b3FileUtils.h>
#include <bulletSim/importURDF/b3ResourcePath.h>

struct BulletURDFInternalData
{

  UrdfParser m_urdfParser;
  struct GUIHelperInterface* m_guiHelper;
  std::string m_sourceFile;
  char m_pathPrefix[1024];
  int m_bodyId;
  btHashMap<btHashInt,UrdfMaterialColor> m_linkColors;
  btAlignedObjectArray<btCollisionShape*> m_allocatedCollisionShapes;
  mutable btAlignedObjectArray<btTriangleMesh*> m_allocatedMeshInterfaces;
  btHashMap<btHashPtr, UrdfCollision> m_bulletCollisionShape2UrdfCollision;

//  UrdfRenderingInterface* m_customVisualShapesConverter;
  bool m_enableTinyRenderer;
  int m_flags;

  void setSourceFile(const std::string& relativeFileName, const std::string& prefix)
  {
    m_sourceFile = relativeFileName;
    m_urdfParser.setSourceFile(relativeFileName);
    strncpy(m_pathPrefix, prefix.c_str(), sizeof(m_pathPrefix));
    m_pathPrefix[sizeof(m_pathPrefix)-1] = 0; // required, strncpy doesn't write zero on overflow
  }

  BulletURDFInternalData()
  {
    m_enableTinyRenderer = true;
    m_pathPrefix[0] = 0;
    m_flags=0;
  }

  void setGlobalScaling(btScalar scaling)
  {
    m_urdfParser.setGlobalScaling(scaling);
  }

};

struct BulletErrorLogger : public ErrorLogger
{
  int m_numErrors;
  int m_numWarnings;

  BulletErrorLogger()
      :m_numErrors(0),
       m_numWarnings(0)
  {
  }
  virtual void reportError(const char* error)
  {
    m_numErrors++;
    b3Error(error);
  }
  virtual void reportWarning(const char* warning)
  {
    m_numWarnings++;
    b3Warning(warning);
  }

  virtual void printMessage(const char* msg)
  {
    b3Printf(msg);
  }

};

struct BulletURDFInternalData* m_data;


bool loadURDF(const char* fileName, bool forceFixedBase)
{
  if (strlen(fileName)==0)
    return false;

//int argc=0;
  char relativeFileName[1024];

  b3FileUtils fu;

  //bool fileFound = fu.findFile(fileName, relativeFileName, 1024);
  bool fileFound = (b3ResourcePath::findResourcePath(fileName,relativeFileName,1024))>0;

  std::string xml_string;

  if (!fileFound){
    b3Warning("URDF file '%s' not found\n", fileName);
    return false;
  } else
  {

    char path[1024];
    fu.extractPath(relativeFileName, path, sizeof(path));
    m_data->setSourceFile(relativeFileName, path);

    std::fstream xml_file(relativeFileName, std::fstream::in);
    while ( xml_file.good())
    {
      std::string line;
      std::getline( xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();
  }

  BulletErrorLogger loggie;
  m_data->m_urdfParser.setParseSDF(false);
  bool result = m_data->m_urdfParser.loadUrdf(xml_string.c_str(), &loggie, forceFixedBase);

  return result;
}

int main() {

  bullet_sim::World_RG bulletSim(800, 600, 0.5, bullet_sim::NO_BACKGROUND);
  bulletSim.setGravity({0, 0, -9.8});
  bulletSim.setLightPosition(30, 0, 10);

  // add objects
//  auto checkerboard = bulletSim.addCheckerboard(2, 100, 100, 0.1, 1, -1, bullet_sim::GRID);
//
//  m_data = new BulletURDFInternalData;
//  m_data->setGlobalScaling(1.0);
//  m_data->m_flags = 0;
//  m_data->m_guiHelper = 0;

//  loadURDF("../res/r2d2/r2d2.urdf", false);
//  loadURDF("../res/lego/lego.urdf", false);


//  Eigen::VectorXd jointNominalConfig(19);
//  Eigen::VectorXd jointState(18), jointVel(18), jointForce(18);
//
//  jointNominalConfig << 0, 0, 0, 0, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
//  std::vector<bullet_sim::ArticulatedSystemHandle> animals;
//
//  auto anymal = bulletSim.addArticulatedSystem(urdfPath);
//  anymal->setGeneralizedCoordinate({0, 0, 0.54,
//                                    1.0, 0.0, 0.0, 0.0, 0.03, 0.4,
//                                    -0.8, -0.03, 0.4, -0.8, 0.03, -0.4,
//                                    0.8, -0.03, -0.4, 0.8});
//  anymal->setGeneralizedVelocity(Eigen::VectorXd::Zero(anymal->getDOF()));
//  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
//
//  const double kp = 40.0, kd = 1.0;
//
//  bulletSim.cameraFollowObject(checkerboard, {2, 2, 2});
//  StopWatch watch;
//
//  watch.start();
//  while(bulletSim.visualizerLoop(0.01, 1.0)) {
//    bulletSim.integrate1(0.01);
//
//    jointState = anymal->getGeneralizedCoordinate();
//    jointVel = anymal->getGeneralizedVelocity();
//    jointForce = anymal->getGeneralizedForce();
//
//    jointForce = kp * (jointNominalConfig - jointState).tail(18) - kd * jointVel;
//    jointForce.head(6).setZero();
//    anymal->setGeneralizedForce(jointForce);
//
//    bulletSim.integrate2(0.01);
//  }



//  std::cout<<"time taken for 100k steps "<<watch.measure()<<"s \n";
  return 0;
}