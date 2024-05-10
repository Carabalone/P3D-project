#include "rayAccelerator.h"
#include "macros.h"
#include "comparePrimitives.h"

using namespace std;

enum Axis {
	X = 0,
	Y = 1,
	Z = 2
};

struct NodeInfo {
	BVH::BVHNode* node;
	float t;
};

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {

		
			BVHNode *root = new BVHNode();

			Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			AABB world_bbox = AABB(min, max);

			for (Object* obj : objs) {
				AABB bbox = obj->GetBoundingBox();
				world_bbox.extend(bbox);
				objects.push_back(obj);
			}
			world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
			world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
			root->setAABB(world_bbox);
			nodes.push_back(root);
			build_recursive(0, objects.size(), root); // -> root node takes all the 
		}

void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {

	// num_obs <= Threshold, make leaf
	if ((right_index - left_index) <= Threshold) {
		node->makeLeaf(left_index, right_index - left_index);
	}
	else {
		Vector dist = node->getAABB().max - node->getAABB().min;
		int axis;

		if (dist.x >= dist.y && dist.x >= dist.z)
			axis = Axis::X;
		else if (dist.y >= dist.x && dist.y >= dist.z)
			axis = Axis::Y;
		else
			axis = Axis::Z;

		// get Largest axis, then calculate the midpoint
		float midPoint = (node->getAABB().max.getAxisValue(axis) + node->getAABB().min.getAxisValue(axis)) / 2.0f;

		// sort the objects based on the axis
		ComparePrimitives cmp = ComparePrimitives(axis);
		std::sort(objects.begin() + left_index, objects.begin() + right_index, cmp);

		unsigned int split_index;
		if (objects[left_index]->getCentroid().getAxisValue(axis) > midPoint ||
			objects[right_index - 1]->getCentroid().getAxisValue(axis) <= midPoint) {
			split_index = left_index + (right_index - left_index) / 2;	
		}
		else {
			for (split_index = left_index; split_index < right_index; split_index++) {
				if (objects[split_index]->getCentroid().getAxisValue(axis) > midPoint) {
					break;
				}
			}
		}

		// build AABBs and call recursively
		Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		AABB leftAABB = AABB(min, max);
		AABB rightAABB = AABB(min, max);

		for (int i = left_index; i < right_index; i++) {
			if (i < split_index) {
				leftAABB.extend(objects[i]->GetBoundingBox());
			}
			else {
				rightAABB.extend(objects[i]->GetBoundingBox());
			}
		}

		BVHNode* leftNode = new BVHNode();
		BVHNode* rightNode = new BVHNode();

		leftNode->setAABB(leftAABB);
		rightNode->setAABB(rightAABB);

		node->makeNode(nodes.size());

		nodes.push_back(leftNode);
		nodes.push_back(rightNode);

		build_recursive(left_index, split_index, leftNode);
		build_recursive(split_index, right_index, rightNode);
	}


	//right_index, left_index and split_index refer to the indices in the objects vector
    // do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
	// node.index can have a index of objects vector or a index of nodes vector
		
	
}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {

	float tmp = 0.0f;
	float tmin = FLT_MAX;  //contains the closest primitive intersection
	bool hit = false;
	Ray localRay = ray;
	float t;

	BVHNode* currentNode = nodes[0];

	std::stack<NodeInfo> stack;

	if (!currentNode->getAABB().intercepts(localRay, t)) {
		return false;
	}

	while (true) {
		if (!currentNode->isLeaf()) {
			NodeInfo leftNodeInfo = { nodes[currentNode->getIndex()], 0.0f };
			NodeInfo rightNodeInfo = { nodes[currentNode->getIndex() + 1], 0.0f };

			bool rightHit = rightNodeInfo.node->getAABB().intercepts(localRay, rightNodeInfo.t);
			bool leftHit = leftNodeInfo.node->getAABB().intercepts(localRay, leftNodeInfo.t);
			if (rightNodeInfo.node->getAABB().isInside(localRay.origin)) {
				rightNodeInfo.t = 0.0f;
			}
			if (leftNodeInfo.node->getAABB().isInside(localRay.origin)) {
				leftNodeInfo.t = 0.0f;
			}

			if (leftHit && rightHit) {
				// both hit, put the farthest on the stack
				if (leftNodeInfo.t > rightNodeInfo.t) {
					stack.push(leftNodeInfo);
					currentNode = rightNodeInfo.node;
					continue;
				}
				else {
					stack.push(rightNodeInfo);
					currentNode = leftNodeInfo.node;
					continue;
				}
			} 
			else if (rightHit) { // only right hits
				stack.push(rightNodeInfo);
				currentNode = rightNodeInfo.node;
				continue;
			}
			else if (leftHit) {
				stack.push(leftNodeInfo);
				currentNode = leftNodeInfo.node;
				continue;
			}
		}
		else { // is leaf
			for (int i = currentNode->getIndex(); i < currentNode->getIndex() + currentNode->getNObjs(); i++) {
				if (objects[i]->intercepts(localRay, tmp) && tmp < tmin) {
					hit = true;
					*hit_obj = objects[i];
					tmin = tmp;
				}
			}
		}

		bool found = false;
		while (!stack.empty()) {
			NodeInfo nodeInfo = stack.top();
			stack.pop();
			
			if (nodeInfo.t < tmin) {
				found = true;
				currentNode = nodeInfo.node;
				break;
			}
		}

		if (found) {
			continue;
		}

		if (hit) {
			hit_point = localRay.origin + localRay.direction * tmin;
		}

		return hit;
	}
}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length

	float tmp;

	double length = ray.direction.length(); //distance between light and intersection point
	ray.direction.normalize();

	float tmin = FLT_MAX;  //contains the closest primitive intersection
	bool hit = false;
	Ray localRay = ray;
	float t;

	BVHNode* currentNode = nodes[0];

	std::stack<NodeInfo> stack;

	if (!currentNode->getAABB().intercepts(localRay, t)) {
		return false;
	}

	while (true) {
		if (!currentNode->isLeaf()) {
			NodeInfo leftNodeInfo = { nodes[currentNode->getIndex()], 0.0f };
			NodeInfo rightNodeInfo = { nodes[currentNode->getIndex() + 1], 0.0f };

			bool rightHit = rightNodeInfo.node->getAABB().intercepts(localRay, rightNodeInfo.t);
			bool leftHit = leftNodeInfo.node->getAABB().intercepts(localRay, leftNodeInfo.t);

			if (rightNodeInfo.node->getAABB().isInside(localRay.origin)) {
				rightNodeInfo.t = 0.0f;
			}
			if (leftNodeInfo.node->getAABB().isInside(localRay.origin)) {
				leftNodeInfo.t = 0.0f;
			}

			if (leftHit && rightHit) {
				// both hit, put the farthest on the stack
				if (leftNodeInfo.t > rightNodeInfo.t) {
					stack.push(leftNodeInfo);
					currentNode = rightNodeInfo.node;
					continue;
				}
				else {
					stack.push(rightNodeInfo);
					currentNode = leftNodeInfo.node;
					continue;
				}
			} 
			else if (leftHit) { // only left hits
				stack.push(leftNodeInfo);
				currentNode = leftNodeInfo.node;
				continue;
			}
			else if (rightHit) { // only right hits
				stack.push(rightNodeInfo);
				currentNode = rightNodeInfo.node;
				continue;
			}
		}
		else { // is leaf
			for (int i = currentNode->getIndex(); i < currentNode->getIndex() + currentNode->getNObjs(); i++) {
				if (objects[i]->intercepts(localRay, tmp)) {
					return true;
				}
			}
		}

		if (stack.empty()) {
			return false;
		}
		auto poppedValue = stack.top();
		stack.pop();
		currentNode = poppedValue.node;

	}

	return(false);
}	

