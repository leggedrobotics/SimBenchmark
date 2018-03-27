/* Copyright (C) 2015 Google

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "BulletUrdfImporter.h"
#include "URDFImporterInterface.h"
#include "btBulletCollisionCommon.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"//to create a tesselation of a generic btConvexShape
#include "Bullet3Common/b3FileUtils.h"
#include <string>
#include "Utils/b3ResourcePath.h"
#include "URDFToBullet.h"//for flags

static btScalar gUrdfDefaultCollisionMargin = 0.001;

#include <iostream>
#include <fstream>
#include <list>
#include "UrdfParser.h"



ATTRIBUTE_ALIGNED16(struct) BulletURDFInternalData
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	UrdfParser m_urdfParser;
	std::string m_sourceFile;
	char m_pathPrefix[1024];
	int m_bodyId;
	btHashMap<btHashInt,UrdfMaterialColor> m_linkColors;
    btAlignedObjectArray<btCollisionShape*> m_allocatedCollisionShapes;
	btAlignedObjectArray<int> m_allocatedTextures;
	mutable btAlignedObjectArray<btTriangleMesh*> m_allocatedMeshInterfaces;
	btHashMap<btHashPtr, UrdfCollision> m_bulletCollisionShape2UrdfCollision;

	UrdfRenderingInterface* m_customVisualShapesConverter;
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

void BulletURDFImporter::printTree()
{
//	btAssert(0);
}

BulletURDFImporter::BulletURDFImporter(struct GUIHelperInterface* helper, UrdfRenderingInterface* customConverter, double globalScaling, int flags)
{
	m_data = new BulletURDFInternalData;
	m_data->setGlobalScaling(globalScaling);
	m_data->m_flags = flags;
	m_data->m_customVisualShapesConverter = customConverter;

  
}

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

bool BulletURDFImporter::loadURDF(const char* fileName, bool forceFixedBase)
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

int BulletURDFImporter::getNumModels() const
{
    return m_data->m_urdfParser.getNumModels();
}

void BulletURDFImporter::activateModel(int modelIndex)
{
    m_data->m_urdfParser.activateModel(modelIndex);
}


bool BulletURDFImporter::loadSDF(const char* fileName, bool forceFixedBase)
{

    //int argc=0;
    char relativeFileName[1024];
    
    b3FileUtils fu;
    
    //bool fileFound = fu.findFile(fileName, relativeFileName, 1024);
    bool fileFound = (b3ResourcePath::findResourcePath(fileName,relativeFileName,1024))>0;
    
    std::string xml_string;
    
    if (!fileFound){
        b3Warning("SDF file '%s' not found\n", fileName);
        return false;
    } else
    {
        
        char path[1024];
        fu.extractPath(relativeFileName, path, sizeof(path));
        m_data->setSourceFile(relativeFileName, path);
        
        std::fstream xml_file(relativeFileName, std::fstream::in);
        while ( xml_file.good() )
        {
            std::string line;
            std::getline( xml_file, line);
            xml_string += (line + "\n");
        }
        xml_file.close();
    }
    
    BulletErrorLogger loggie;
    //todo: quick test to see if we can re-use the URDF parser for SDF or not
    m_data->m_urdfParser.setParseSDF(true);
    bool result = m_data->m_urdfParser.loadSDF(xml_string.c_str(), &loggie);
    
    return result;
}


const char* BulletURDFImporter::getPathPrefix()
{
	return m_data->m_pathPrefix;
}

    
void BulletURDFImporter::setBodyUniqueId(int bodyId)
{
    m_data->m_bodyId =bodyId;
}


int BulletURDFImporter::getBodyUniqueId() const
{
    return  m_data->m_bodyId;
}


BulletURDFImporter::~BulletURDFImporter()
{
	delete m_data;
}

    
int BulletURDFImporter::getRootLinkIndex() const
{
	if (m_data->m_urdfParser.getModel().m_rootLinks.size()==1)
	{
		return m_data->m_urdfParser.getModel().m_rootLinks[0]->m_linkIndex;
	}
    return -1;
};
    
void BulletURDFImporter::getLinkChildIndices(int linkIndex, btAlignedObjectArray<int>& childLinkIndices) const
{
	childLinkIndices.resize(0);
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	if (linkPtr)
	{
		const UrdfLink* link = *linkPtr;
		//int numChildren = m_data->m_urdfParser->getModel().m_links.getAtIndex(linkIndex)->
		
		for (int i=0;i<link->m_childLinks.size();i++)
		{
			int childIndex =link->m_childLinks[i]->m_linkIndex;
			childLinkIndices.push_back(childIndex);
		}
	}
}


std::string BulletURDFImporter::getLinkName(int linkIndex) const
{
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	btAssert(linkPtr);
	if (linkPtr)
	{
		UrdfLink* link = *linkPtr;
		return link->m_name;
	}
	return "";
}

std::string BulletURDFImporter::getBodyName() const
{
	return m_data->m_urdfParser.getModel().m_name;
}
    
std::string BulletURDFImporter::getJointName(int linkIndex) const
{
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	btAssert(linkPtr);
	if (linkPtr)
	{
		UrdfLink* link = *linkPtr;
		if (link->m_parentJoint)
		{
			return link->m_parentJoint->m_name;
		}
	}
	return "";
}
    

void  BulletURDFImporter::getMassAndInertia(int linkIndex, btScalar& mass,btVector3& localInertiaDiagonal, btTransform& inertialFrame) const
{
	//the link->m_inertia is NOT necessarily aligned with the inertial frame
	//so an additional transform might need to be computed
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	
	
	btAssert(linkPtr);
	if (linkPtr)
	{
		UrdfLink* link = *linkPtr;
		btMatrix3x3 linkInertiaBasis;
		btScalar linkMass, principalInertiaX, principalInertiaY, principalInertiaZ;
		if (link->m_parentJoint==0 && m_data->m_urdfParser.getModel().m_overrideFixedBase)
		{
			linkMass = 0.f;
			principalInertiaX = 0.f;
			principalInertiaY = 0.f;
			principalInertiaZ = 0.f;
			linkInertiaBasis.setIdentity();
		}
		else
		{
			linkMass = link->m_inertia.m_mass;
			if (link->m_inertia.m_ixy == 0.0 &&
			    link->m_inertia.m_ixz == 0.0 &&
			    link->m_inertia.m_iyz == 0.0)
			{
				principalInertiaX = link->m_inertia.m_ixx;
				principalInertiaY = link->m_inertia.m_iyy;
				principalInertiaZ = link->m_inertia.m_izz;
				linkInertiaBasis.setIdentity();
			}
			else
			{
				principalInertiaX = link->m_inertia.m_ixx;
				btMatrix3x3 inertiaTensor(link->m_inertia.m_ixx, link->m_inertia.m_ixy, link->m_inertia.m_ixz,
							  link->m_inertia.m_ixy, link->m_inertia.m_iyy, link->m_inertia.m_iyz,
							  link->m_inertia.m_ixz, link->m_inertia.m_iyz, link->m_inertia.m_izz);
				btScalar threshold = 1.0e-6;
				int numIterations = 30;
				inertiaTensor.diagonalize(linkInertiaBasis, threshold, numIterations);
				principalInertiaX = inertiaTensor[0][0];
				principalInertiaY = inertiaTensor[1][1];
				principalInertiaZ = inertiaTensor[2][2];
			}
		}
		mass = linkMass;
		if (principalInertiaX < 0 ||
		    principalInertiaX > (principalInertiaY + principalInertiaZ) ||
		    principalInertiaY < 0 ||
		    principalInertiaY > (principalInertiaX + principalInertiaZ) ||
		    principalInertiaZ < 0 ||
		    principalInertiaZ > (principalInertiaX + principalInertiaY))
		{
			b3Warning("Bad inertia tensor properties, setting inertia to zero for link: %s\n", link->m_name.c_str());
			principalInertiaX = 0.f;
			principalInertiaY = 0.f;
			principalInertiaZ = 0.f;
			linkInertiaBasis.setIdentity();
		}
		localInertiaDiagonal.setValue(principalInertiaX, principalInertiaY, principalInertiaZ);
		inertialFrame.setOrigin(link->m_inertia.m_linkLocalFrame.getOrigin());
		inertialFrame.setBasis(link->m_inertia.m_linkLocalFrame.getBasis()*linkInertiaBasis);
	}
	else
	{
		mass = 1.f;
		localInertiaDiagonal.setValue(1,1,1);
		inertialFrame.setIdentity();
	}
}
    
bool BulletURDFImporter::getJointInfo2(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction, btScalar& jointMaxForce, btScalar& jointMaxVelocity) const 
{
    jointLowerLimit = 0.f;
    jointUpperLimit = 0.f;
	jointDamping = 0.f;
	jointFriction = 0.f;
	jointMaxForce = 0.f;
	jointMaxVelocity = 0.f;

	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(urdfLinkIndex);
	btAssert(linkPtr);
	if (linkPtr)
	{
		UrdfLink* link = *linkPtr;
		linkTransformInWorld = link->m_linkTransformInWorld;
		
		if (link->m_parentJoint)
		{
			UrdfJoint* pj = link->m_parentJoint;
			parent2joint = pj->m_parentLinkToJointTransform;
			jointType = pj->m_type;
			jointAxisInJointSpace = pj->m_localJointAxis;
			jointLowerLimit = pj->m_lowerLimit;
			jointUpperLimit = pj->m_upperLimit;
			jointDamping = pj->m_jointDamping;
			jointFriction = pj->m_jointFriction;
			jointMaxForce = pj->m_effortLimit;
			jointMaxVelocity = pj->m_velocityLimit;
			return true;
		} else
		{
			parent2joint.setIdentity();
			return false;
		}
	}
	
	return false;

};

bool BulletURDFImporter::getJointInfo(int urdfLinkIndex, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction) const
{
	btScalar jointMaxForce;
	btScalar jointMaxVelocity;
	return getJointInfo2(urdfLinkIndex, parent2joint, linkTransformInWorld, jointAxisInJointSpace, jointType, jointLowerLimit, jointUpperLimit, jointDamping, jointFriction,jointMaxForce,jointMaxVelocity); 
	
}

void BulletURDFImporter::setRootTransformInWorld(const btTransform& rootTransformInWorld)
{
    m_data->m_urdfParser.getModel().m_rootTransformInWorld = rootTransformInWorld ;
}



bool BulletURDFImporter::getRootTransformInWorld(btTransform& rootTransformInWorld) const
{
    rootTransformInWorld = m_data->m_urdfParser.getModel().m_rootTransformInWorld;
    return true;
}

bool findExistingMeshFile(
	const std::string& urdf_path, std::string fn,
	const std::string& error_message_prefix,
	std::string* out_found_filename, int* out_type)
{
	if (fn.size() <= 4)
	{
		b3Warning("%s: invalid mesh filename '%s'\n", error_message_prefix.c_str(), fn.c_str());
		return false;
	}

	std::string ext;
	std::string ext_ = fn.substr(fn.size()-4);
	for (std::string::iterator i=ext_.begin(); i!=ext_.end(); ++i)
	{
		ext += char(tolower(*i));
	}

	if (ext==".dae")
	{
		*out_type = UrdfGeometry::FILE_COLLADA;
	}
	else if (ext==".stl")
	{
		*out_type = UrdfGeometry::FILE_STL;
	}
	else if (ext==".obj")
	{
		*out_type = UrdfGeometry::FILE_OBJ;
	}
	else
	{
		b3Warning("%s: invalid mesh filename extension '%s'\n", error_message_prefix.c_str(), ext.c_str());
		return false;
	}

	std::string drop_it = "package://";
	if (fn.substr(0, drop_it.length())==drop_it)
		fn = fn.substr(drop_it.length());

	std::list<std::string> shorter;
	shorter.push_back("../..");
	shorter.push_back("..");
	shorter.push_back(".");
	int cnt = urdf_path.size();
	for (int i=0; i<cnt; ++i)
	{
		if (urdf_path[i]=='/' || urdf_path[i]=='\\')
		{
			shorter.push_back(urdf_path.substr(0, i));
		}
	}
	shorter.reverse();

	std::string existing_file;

	{
		std::string attempt = fn;
		FILE* f = fopen(attempt.c_str(), "rb");
		if (f)
		{
			existing_file = attempt;
			fclose(f);
		}
	}
	if (existing_file.empty())
	{
		for (std::list<std::string>::iterator x=shorter.begin(); x!=shorter.end(); ++x)
		{
			std::string attempt = *x + "/" + fn;
			FILE* f = fopen(attempt.c_str(), "rb");
			if (!f)
			{
				//b3Printf("%s: tried '%s'", error_message_prefix.c_str(), attempt.c_str());
				continue;
			}
			fclose(f);
			existing_file = attempt;
			//b3Printf("%s: found '%s'", error_message_prefix.c_str(), attempt.c_str());
			break;
		}
	}

	if (existing_file.empty())
	{
		b3Warning("%s: cannot find '%s' in any directory in urdf path\n", error_message_prefix.c_str(), fn.c_str());
		return false;
	}
	else
	{
		*out_found_filename = existing_file;
		return true;
	}
}

int BulletURDFImporter::getUrdfFromCollisionShape(const btCollisionShape* collisionShape, UrdfCollision& collision) const
{
	UrdfCollision* col = m_data->m_bulletCollisionShape2UrdfCollision.find(collisionShape);
	if (col)
	{
		collision = *col;
		return 1;
	}
	return 0;
}

btCollisionShape* BulletURDFImporter::convertURDFToCollisionShape(const UrdfCollision* collision, const char* urdfPathPrefix) const
{
	BT_PROFILE("convertURDFToCollisionShape");

	btCollisionShape* shape = 0;

    switch (collision->m_geometry.m_type)
    {
	case URDF_GEOM_PLANE:
		{
			btVector3 planeNormal = collision->m_geometry.m_planeNormal;
			btScalar planeConstant = 0;//not available?
			btStaticPlaneShape* plane = new btStaticPlaneShape(planeNormal,planeConstant);
			shape = plane;
			shape ->setMargin(gUrdfDefaultCollisionMargin);
			break;
		}
		case URDF_GEOM_CAPSULE:
        {
			btScalar radius = collision->m_geometry.m_capsuleRadius;
			btScalar height = collision->m_geometry.m_capsuleHeight;
			btCapsuleShapeZ* capsuleShape = new btCapsuleShapeZ(radius,height);
			shape = capsuleShape;
			shape ->setMargin(gUrdfDefaultCollisionMargin);
			break;
		}

        case URDF_GEOM_CYLINDER:
        {
			btScalar cylRadius = collision->m_geometry.m_capsuleRadius;
			btScalar cylHalfLength = 0.5*collision->m_geometry.m_capsuleHeight;
			if (m_data->m_flags & CUF_USE_IMPLICIT_CYLINDER)
			{
				btVector3 halfExtents(cylRadius,cylRadius, cylHalfLength);
				btCylinderShapeZ* cylZShape = new btCylinderShapeZ(halfExtents);
				shape = cylZShape;
			} else
			{
				btAlignedObjectArray<btVector3> vertices;
				//int numVerts = sizeof(barrel_vertices)/(9*sizeof(float));
				int numSteps = 32;
				for (int i=0;i<numSteps;i++)
				{

					btVector3 vert(cylRadius*btSin(SIMD_2_PI*(float(i)/numSteps)),cylRadius*btCos(SIMD_2_PI*(float(i)/numSteps)),cylHalfLength);
					vertices.push_back(vert);
					vert[2] = -cylHalfLength;
					vertices.push_back(vert);

				}
				btConvexHullShape* cylZShape = new btConvexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
				cylZShape->setMargin(gUrdfDefaultCollisionMargin);
				cylZShape->recalcLocalAabb();
				cylZShape->initializePolyhedralFeatures();
				cylZShape->optimizeConvexHull();
				shape = cylZShape;
			}
           
            break;
        }
        case URDF_GEOM_BOX:
        {
			btVector3 extents = collision->m_geometry.m_boxSize;
			btBoxShape* boxShape = new btBoxShape(extents*0.5f);
			//btConvexShape* boxShape = new btConeShapeX(extents[2]*0.5,extents[0]*0.5);
            shape = boxShape;
			shape ->setMargin(gUrdfDefaultCollisionMargin);
            break;
        }
        case URDF_GEOM_SPHERE:
        {
			btScalar radius = collision->m_geometry.m_sphereRadius;
			btSphereShape* sphereShape = new btSphereShape(radius);
            shape = sphereShape;
			shape ->setMargin(gUrdfDefaultCollisionMargin);
            break;
	}

	case URDF_GEOM_MESH:
    {
      RAIFATAL("Mesh collision shape is not implemented yet")
    } // mesh case

        default:
		b3Warning("Error: unknown collision geometry type %i\n", collision->m_geometry.m_type);
		
	}
	if (shape && collision->m_geometry.m_type==URDF_GEOM_MESH)
	{
		m_data->m_bulletCollisionShape2UrdfCollision.insert(shape, *collision);
	}
	return shape;
}

bool BulletURDFImporter::getLinkColor(int linkIndex, btVector4& colorRGBA) const
{
	const UrdfMaterialColor* matColPtr = m_data->m_linkColors[linkIndex];
	if (matColPtr)
	{
		colorRGBA = matColPtr->m_rgbaColor;
		return true;
	}
	return false;
}

bool BulletURDFImporter::getLinkColor2(int linkIndex, UrdfMaterialColor& matCol) const
{
	UrdfMaterialColor* matColPtr = m_data->m_linkColors[linkIndex];
	if (matColPtr)
	{
		matCol = *matColPtr;
		return true;
	}
	return false;
}

bool BulletURDFImporter::getLinkContactInfo(int urdflinkIndex, URDFLinkContactInfo& contactInfo ) const
{
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(urdflinkIndex);
	if (linkPtr)
	{
		const UrdfLink* link = *linkPtr;
		contactInfo = link->m_contactInfo;
		return true;
	}
	return false;
}

bool BulletURDFImporter::getLinkAudioSource(int linkIndex, SDFAudioSource& audioSource) const
{
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	if (linkPtr)
	{
		const UrdfLink* link = *linkPtr;
		if (link->m_audioSource.m_flags & SDFAudioSource::SDFAudioSourceValid)
		{
			audioSource = link->m_audioSource;
			return true;
		}
	}
	return false;
}

void BulletURDFImporter::setEnableTinyRenderer(bool enable)
{
	m_data->m_enableTinyRenderer = enable;
}

int BulletURDFImporter::getNumAllocatedCollisionShapes() const
{
    return m_data->m_allocatedCollisionShapes.size();
}


btCollisionShape* BulletURDFImporter::getAllocatedCollisionShape(int index)
{
    return m_data->m_allocatedCollisionShapes[index];
}

int BulletURDFImporter::getNumAllocatedMeshInterfaces() const
{
    return m_data->m_allocatedMeshInterfaces.size();
}



btStridingMeshInterface* BulletURDFImporter::getAllocatedMeshInterface(int index)
{
    return m_data->m_allocatedMeshInterfaces[index];
}

int BulletURDFImporter::getNumAllocatedTextures() const
{
	return m_data->m_allocatedTextures.size();
}

int BulletURDFImporter::getAllocatedTexture(int index) const
{
	return m_data->m_allocatedTextures[index];
}



std::vector<std::string> BulletURDFImporter::getMeshFilePath(int linkIndex) {

  std::vector<std::string> paths;

  int graphicsIndex = -1;
  btAlignedObjectArray<int> indices;
  btTransform startTrans; startTrans.setIdentity();
  btAlignedObjectArray<BulletURDFTexture> textures;

  const UrdfModel& model = m_data->m_urdfParser.getModel();
  UrdfLink* const* linkPtr = model.m_links.getAtIndex(linkIndex);
  if (linkPtr)
  {

    const UrdfLink* link = *linkPtr;

    for (int v = 0; v < link->m_visualArray.size();v++)
    {
      const UrdfVisual& vis = link->m_visualArray[v];
      btTransform childTrans = vis.m_linkLocalFrame;
      btHashString matName(vis.m_materialName.c_str());
      UrdfMaterial *const * matPtr = model.m_materials[matName];
      if (matPtr)
      {
        UrdfMaterial *const  mat = *matPtr;
        //printf("UrdfMaterial %s, rgba = %f,%f,%f,%f\n",mat->m_name.c_str(),mat->m_rgbaColor[0],mat->m_rgbaColor[1],mat->m_rgbaColor[2],mat->m_rgbaColor[3]);
        UrdfMaterialColor matCol;
        matCol.m_rgbaColor = mat->m_matColor.m_rgbaColor;
        matCol.m_specularColor = mat->m_matColor.m_specularColor;
        m_data->m_linkColors.insert(linkIndex,matCol);
      }

      paths.emplace_back(vis.m_geometry.m_meshFileName);
    }
  }

  return paths;
}

 class btCompoundShape* BulletURDFImporter::convertLinkCollisionShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame) const
{

    btCompoundShape* compoundShape = new btCompoundShape();
    m_data->m_allocatedCollisionShapes.push_back(compoundShape);

    compoundShape->setMargin(gUrdfDefaultCollisionMargin);
	UrdfLink* const* linkPtr = m_data->m_urdfParser.getModel().m_links.getAtIndex(linkIndex);
	btAssert(linkPtr);
	if (linkPtr)
	{

		UrdfLink* link = *linkPtr;


		for (int v=0;v<link->m_collisionArray.size();v++)
		{
			const UrdfCollision& col = link->m_collisionArray[v];
			btCollisionShape* childShape = convertURDFToCollisionShape(&col ,pathPrefix);
			m_data->m_allocatedCollisionShapes.push_back(childShape);
			if (childShape->getShapeType()==COMPOUND_SHAPE_PROXYTYPE)
			{
				btCompoundShape* compound = (btCompoundShape*) childShape;
				for (int i=0;i<compound->getNumChildShapes();i++)
				{
					m_data->m_allocatedCollisionShapes.push_back(compound->getChildShape(i));
				}
			}

			if (childShape)
			{
				btTransform childTrans = col.m_linkLocalFrame;

				compoundShape->addChildShape(localInertiaFrame.inverse()*childTrans,childShape);
			}
		}
	}

    return compoundShape;
}
