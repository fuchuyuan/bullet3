//this file is autogenerated using stringify.bat (premake --stringify) in the build folder of this project
static const char* bvhTraversalKernelCL= \
"//keep this enum in sync with the CPU version (in btCollidable.h)\n"
"//written by Erwin Coumans\n"
"\n"
"#define SHAPE_CONVEX_HULL 3\n"
"#define SHAPE_CONCAVE_TRIMESH 5\n"
"#define TRIANGLE_NUM_CONVEX_FACES 5\n"
"#define SHAPE_COMPOUND_OF_CONVEX_HULLS 6\n"
"#define SHAPE_SPHERE 7\n"
"\n"
"typedef unsigned int u32;\n"
"\n"
"#define MAX_NUM_PARTS_IN_BITS 10\n"
"\n"
"///btQuantizedBvhNode is a compressed aabb node, 16 bytes.\n"
"///Node can be used for leafnode or internal node. Leafnodes can point to 32-bit triangle index (non-negative range).\n"
"typedef struct\n"
"{\n"
"	//12 bytes\n"
"	unsigned short int	m_quantizedAabbMin[3];\n"
"	unsigned short int	m_quantizedAabbMax[3];\n"
"	//4 bytes\n"
"	int	m_escapeIndexOrTriangleIndex;\n"
"} btQuantizedBvhNode;\n"
"/*\n"
"	bool isLeafNode() const\n"
"	{\n"
"		//skipindex is negative (internal node), triangleindex >=0 (leafnode)\n"
"		return (m_escapeIndexOrTriangleIndex >= 0);\n"
"	}\n"
"	int getEscapeIndex() const\n"
"	{\n"
"		btAssert(!isLeafNode());\n"
"		return -m_escapeIndexOrTriangleIndex;\n"
"	}\n"
"	int	getTriangleIndex() const\n"
"	{\n"
"		btAssert(isLeafNode());\n"
"		unsigned int x=0;\n"
"		unsigned int y = (~(x&0))<<(31-MAX_NUM_PARTS_IN_BITS);\n"
"		// Get only the lower bits where the triangle index is stored\n"
"		return (m_escapeIndexOrTriangleIndex&~(y));\n"
"	}\n"
"	int	getPartId() const\n"
"	{\n"
"		btAssert(isLeafNode());\n"
"		// Get only the highest bits where the part index is stored\n"
"		return (m_escapeIndexOrTriangleIndex>>(31-MAX_NUM_PARTS_IN_BITS));\n"
"	}\n"
"*/\n"
"\n"
"int	getTriangleIndex(const btQuantizedBvhNode* rootNode)\n"
"{\n"
"	unsigned int x=0;\n"
"	unsigned int y = (~(x&0))<<(31-MAX_NUM_PARTS_IN_BITS);\n"
"	// Get only the lower bits where the triangle index is stored\n"
"	return (rootNode->m_escapeIndexOrTriangleIndex&~(y));\n"
"}\n"
"\n"
"int isLeaf(const btQuantizedBvhNode* rootNode)\n"
"{\n"
"	//skipindex is negative (internal node), triangleindex >=0 (leafnode)\n"
"	return (rootNode->m_escapeIndexOrTriangleIndex >= 0)? 1 : 0;\n"
"}\n"
"	\n"
"int getEscapeIndex(const btQuantizedBvhNode* rootNode)\n"
"{\n"
"	return -rootNode->m_escapeIndexOrTriangleIndex;\n"
"}\n"
"\n"
"typedef struct\n"
"{\n"
"	//12 bytes\n"
"	unsigned short int	m_quantizedAabbMin[3];\n"
"	unsigned short int	m_quantizedAabbMax[3];\n"
"	//4 bytes, points to the root of the subtree\n"
"	int			m_rootNodeIndex;\n"
"	//4 bytes\n"
"	int			m_subtreeSize;\n"
"	int			m_padding[3];\n"
"} btBvhSubtreeInfo;\n"
"\n"
"///keep this in sync with btCollidable.h\n"
"typedef struct\n"
"{\n"
"	int m_numChildShapes;\n"
"	int blaat2;\n"
"	int m_shapeType;\n"
"	int m_shapeIndex;\n"
"	\n"
"} btCollidableGpu;\n"
"\n"
"typedef struct\n"
"{\n"
"	float4	m_childPosition;\n"
"	float4	m_childOrientation;\n"
"	int m_shapeIndex;\n"
"	int m_unused0;\n"
"	int m_unused1;\n"
"	int m_unused2;\n"
"} btGpuChildShape;\n"
"\n"
"\n"
"typedef struct\n"
"{\n"
"	float4 m_pos;\n"
"	float4 m_quat;\n"
"	float4 m_linVel;\n"
"	float4 m_angVel;\n"
"\n"
"	u32 m_collidableIdx;\n"
"	float m_invMass;\n"
"	float m_restituitionCoeff;\n"
"	float m_frictionCoeff;\n"
"} BodyData;\n"
"\n"
"typedef struct \n"
"{\n"
"	union\n"
"	{\n"
"		float4	m_min;\n"
"		float   m_minElems[4];\n"
"		int			m_minIndices[4];\n"
"	};\n"
"	union\n"
"	{\n"
"		float4	m_max;\n"
"		float   m_maxElems[4];\n"
"		int			m_maxIndices[4];\n"
"	};\n"
"} btAabbCL;\n"
"\n"
"\n"
"int testQuantizedAabbAgainstQuantizedAabb(\n"
"								const unsigned short int* aabbMin1,\n"
"								const unsigned short int* aabbMax1,\n"
"								const unsigned short int* aabbMin2,\n"
"								const unsigned short int* aabbMax2)\n"
"{\n"
"	//int overlap = 1;\n"
"	if (aabbMin1[0] > aabbMax2[0])\n"
"		return 0;\n"
"	if (aabbMax1[0] < aabbMin2[0])\n"
"		return 0;\n"
"	if (aabbMin1[1] > aabbMax2[1])\n"
"		return 0;\n"
"	if (aabbMax1[1] < aabbMin2[1])\n"
"		return 0;\n"
"	if (aabbMin1[2] > aabbMax2[2])\n"
"		return 0;\n"
"	if (aabbMax1[2] < aabbMin2[2])\n"
"		return 0;\n"
"	return 1;\n"
"	//overlap = ((aabbMin1[0] > aabbMax2[0]) || (aabbMax1[0] < aabbMin2[0])) ? 0 : overlap;\n"
"	//overlap = ((aabbMin1[2] > aabbMax2[2]) || (aabbMax1[2] < aabbMin2[2])) ? 0 : overlap;\n"
"	//overlap = ((aabbMin1[1] > aabbMax2[1]) || (aabbMax1[1] < aabbMin2[1])) ? 0 : overlap;\n"
"	//return overlap;\n"
"}\n"
"\n"
"\n"
"void quantizeWithClamp(unsigned short* out, float4 point2,int isMax, float4 bvhAabbMin, float4 bvhAabbMax, float4 bvhQuantization)\n"
"{\n"
"	float4 clampedPoint = max(point2,bvhAabbMin);\n"
"	clampedPoint = min (clampedPoint, bvhAabbMax);\n"
"\n"
"	float4 v = (clampedPoint - bvhAabbMin) * bvhQuantization;\n"
"	if (isMax)\n"
"	{\n"
"		out[0] = (unsigned short) (((unsigned short)(v.x+1.f) | 1));\n"
"		out[1] = (unsigned short) (((unsigned short)(v.y+1.f) | 1));\n"
"		out[2] = (unsigned short) (((unsigned short)(v.z+1.f) | 1));\n"
"	} else\n"
"	{\n"
"		out[0] = (unsigned short) (((unsigned short)(v.x) & 0xfffe));\n"
"		out[1] = (unsigned short) (((unsigned short)(v.y) & 0xfffe));\n"
"		out[2] = (unsigned short) (((unsigned short)(v.z) & 0xfffe));\n"
"	}\n"
"\n"
"}\n"
"\n"
"\n"
"// work-in-progress\n"
"__kernel void   bvhTraversalKernel( __global const int2* pairs, \n"
"									__global const BodyData* rigidBodies, \n"
"									__global const btCollidableGpu* collidables,\n"
"									__global btAabbCL* aabbs,\n"
"									__global int4* concavePairsOut,\n"
"									__global volatile int* numConcavePairsOut,\n"
"									__global const btBvhSubtreeInfo* subtreeHeaders,\n"
"									__global const btQuantizedBvhNode* quantizedNodes,\n"
"									float4 bvhAabbMin,\n"
"									float4 bvhAabbMax,\n"
"									float4 bvhQuantization,\n"
"									int numSubtreeHeaders,\n"
"									int numPairs,\n"
"									int maxNumConcavePairsCapacity)\n"
"{\n"
"	int id = get_global_id(0);\n"
"	if (id>=numPairs)\n"
"		return;\n"
"	\n"
"	int bodyIndexA = pairs[id].x;\n"
"	int bodyIndexB = pairs[id].y;\n"
"	int collidableIndexA = rigidBodies[bodyIndexA].m_collidableIdx;\n"
"	int collidableIndexB = rigidBodies[bodyIndexB].m_collidableIdx;\n"
"	\n"
"	//once the broadphase avoids static-static pairs, we can remove this test\n"
"	if ((rigidBodies[bodyIndexA].m_invMass==0) &&(rigidBodies[bodyIndexB].m_invMass==0))\n"
"	{\n"
"		return;\n"
"	}\n"
"		\n"
"	if (collidables[collidableIndexA].m_shapeType!=SHAPE_CONCAVE_TRIMESH)\n"
"		return;\n"
"\n"
"	int shapeTypeB = collidables[collidableIndexB].m_shapeType;\n"
"		\n"
"	if (shapeTypeB!=SHAPE_CONVEX_HULL &&\n"
"		shapeTypeB!=SHAPE_SPHERE	)\n"
"		return;\n"
"\n"
"	\n"
"	unsigned short int quantizedQueryAabbMin[3];\n"
"	unsigned short int quantizedQueryAabbMax[3];\n"
"	quantizeWithClamp(quantizedQueryAabbMin,aabbs[bodyIndexB].m_min,false,bvhAabbMin, bvhAabbMax,bvhQuantization);\n"
"	quantizeWithClamp(quantizedQueryAabbMax,aabbs[bodyIndexB].m_max,true ,bvhAabbMin, bvhAabbMax,bvhQuantization);\n"
"	\n"
"	for (int i=0;i<numSubtreeHeaders;i++)\n"
"	{\n"
"		btBvhSubtreeInfo subtree = subtreeHeaders[i];\n"
"				\n"
"		int overlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,subtree.m_quantizedAabbMin,subtree.m_quantizedAabbMax);\n"
"		if (overlap != 0)\n"
"		{\n"
"			int startNodeIndex = subtree.m_rootNodeIndex;\n"
"			int endNodeIndex = subtree.m_rootNodeIndex+subtree.m_subtreeSize;\n"
"			int curIndex = startNodeIndex;\n"
"			int escapeIndex;\n"
"			int isLeafNode;\n"
"			int aabbOverlap;\n"
"			while (curIndex < endNodeIndex)\n"
"			{\n"
"				btQuantizedBvhNode rootNode = quantizedNodes[curIndex];\n"
"				aabbOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,rootNode.m_quantizedAabbMin,rootNode.m_quantizedAabbMax);\n"
"				isLeafNode = isLeaf(&rootNode);\n"
"				if (aabbOverlap)\n"
"				{\n"
"					if (isLeafNode)\n"
"					{\n"
"						int triangleIndex = getTriangleIndex(&rootNode);\n"
"							\n"
"						int pairIdx = atomic_inc(numConcavePairsOut);\n"
"						if (pairIdx<maxNumConcavePairsCapacity)\n"
"						{\n"
"							int4 newPair = (int4)(bodyIndexA,bodyIndexB,triangleIndex,3);\n"
"							concavePairsOut[pairIdx] = newPair;\n"
"						}\n"
"					} \n"
"					curIndex++;\n"
"				} else\n"
"				{\n"
"					if (isLeafNode)\n"
"					{\n"
"						curIndex++;\n"
"					} else\n"
"					{\n"
"						escapeIndex = getEscapeIndex(&rootNode);\n"
"						curIndex += escapeIndex;\n"
"					}\n"
"				}\n"
"			}\n"
"		}\n"
"	}\n"
"\n"
"}\n"
;
